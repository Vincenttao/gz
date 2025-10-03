# 使用Python的未来特性，支持更现代的类型注解语法
from __future__ import annotations

import json
# 导入日期时间模块，用于生成时间戳
from datetime import datetime, timezone
# 导入类型提示模块，用于函数参数和返回值的类型声明
from typing import Any, Callable, List, Optional

# 导入ROS 2 Python客户端库
import rclpy
# 导入动作消息的状态类型
from action_msgs.msg import GoalStatus
# 导入时间消息类型
from builtin_interfaces.msg import Time
# 导入几何消息类型，用于表示位姿
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# Nav2动作接口在运行时需要；为测试环境提供轻量级存根
# 在可能未安装包的环境中（例如受限的CI沙箱）
try:
    # 尝试导入Nav2的导航到指定位置的动作接口
    from nav2_msgs.action import NavigateToPose
except ModuleNotFoundError:  # pragma: no cover - fallback only exercised in tests
    # 如果找不到模块（如在测试环境中），则创建一个简单的替代类

    class _NavigateGoal:  # 最小替代类，匹配测试中使用的接口
        def __init__(self) -> None:
            # 初始化一个位姿消息
            self.pose = PoseStamped()

    class NavigateToPose:  # type: ignore
        # 为测试环境提供一个简化版本的NavigateToPose类
        Goal = _NavigateGoal


# 导入ROS 2接口消息类型，用于参数声明和配置
from rcl_interfaces.msg import (
    ParameterDescriptor,     # 参数描述符，用于描述参数的属性
    ParameterType,          # 参数类型枚举
    ParameterValue as ParameterValueMsg,  # 参数值消息类型
    SetParametersResult,    # 设置参数结果消息类型
)
# 导入动作客户端，用于与动作服务器通信（如导航动作）
from rclpy.action import ActionClient
# 导入外部关闭异常，用于处理程序被外部中断的情况
from rclpy.executors import ExternalShutdownException
# 导入Node类，ROS 2中所有节点的基类
from rclpy.node import Node
# 导入参数相关类，用于处理节点参数
from rclpy.parameter import Parameter, ParameterValue
# 导入QoS（服务质量）相关类，用于设置消息通信的质量策略
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
# 导入Future类，用于异步操作的结果处理
from rclpy.task import Future
# 导入标准消息类型
from std_msgs.msg import Bool, String


class MissionNode(Node):
    """航路点任务执行器，协调导航和热事件。
    
    这是一个ROS 2节点类，负责：
    1. 管理机器人的巡检任务（按航路点顺序导航）
    2. 监听热警报事件并作出响应
    3. 记录任务执行过程中的各种事件
    """

    def __init__(
        self,
        action_client_factory: Optional[Callable[[Node], ActionClient]] = None,
    ) -> None:
        """初始化任务节点。
        
        Args:
            action_client_factory: 动作客户端工厂函数，用于创建导航动作客户端
                                  如果未提供，则使用默认的Nav2导航客户端
        """
        # 调用父类Node的构造函数，创建名为"mission_node"的节点
        super().__init__("mission_node")

        # 创建QoS（服务质量）配置，确保消息传递的可靠性
        # KEEP_LAST: 保留最新的消息
        # depth=10: 缓冲区大小为10条消息
        # RELIABLE: 可靠传输模式（确保消息送达）
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # 创建发布者，用于发布航路点目标
        # PoseStamped: 带时间戳的位姿消息类型
        # "/inspection/waypoint_goal": 发布的话题名称
        self._waypoint_pub = self.create_publisher(
            PoseStamped, "/inspection/waypoint_goal", qos
        )
        
        # 创建事件发布者，用于发布任务执行过程中的各种事件
        # String: 字符串消息类型
        # "/inspection/event": 事件发布的话题名称
        self._event_pub = self.create_publisher(String, "/inspection/event", qos)
        
        # 创建订阅者，用于监听热警报消息
        # Bool: 布尔值消息类型
        # "/inspection/alarm": 订阅的话题名称
        # self._alarm_callback: 收到消息时调用的回调函数
        self._alarm_sub = self.create_subscription(
            Bool, "/inspection/alarm", self._alarm_callback, qos
        )
        
        # 创建订阅者，用于监听AMCL（自适应蒙特卡洛定位）发布的机器人位姿
        # PoseWithCovarianceStamped: 带协方差的位姿消息类型
        # "/amcl_pose": 订阅的话题名称
        # self._amcl_callback: 收到消息时调用的回调函数
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, qos
        )

        # 声明节点参数，允许在运行时通过参数文件或命令行配置
        # "loop": 参数名称，控制是否循环执行航路点任务
        # True: 默认值，表示默认循环执行
        self.declare_parameter("loop", True)
        
        # "alarm_stop": 参数名称，控制收到警报时是否停止导航
        # True: 默认值，表示默认在收到警报时停止导航
        self.declare_parameter("alarm_stop", True)
        
        # "waypoints": 参数名称，存储航路点列表
        # []: 默认值，空列表
        # descriptor: 参数描述符，指定参数类型为字符串数组
        # TODO 详细检查参数问题
        # Use simpler parameter declaration with explicit type hints
        self.declare_parameter("waypoints", [], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)) #TODO 详细检查参数问题

        # 获取并存储参数值
        # bool(): 确保参数值被转换为布尔类型
        self._loop = bool(self.get_parameter("loop").value)
        self._alarm_stop = bool(self.get_parameter("alarm_stop").value)
        # 获取原始航路点数据
        self._raw_waypoints = self.get_parameter("waypoints").value
        # 解析航路点数据为PoseStamped对象列表
        self._waypoints = self._parse_waypoints(self._raw_waypoints)

        # 如果没有提供动作客户端工厂函数，则使用默认的Nav2导航客户端
        if action_client_factory is None:
            # 定义默认的动作客户端工厂函数
            def _default_action_client(node: Node) -> ActionClient:
                # 创建Nav2导航动作客户端
                # NavigateToPose: 动作类型
                # "navigate_to_pose": 动作服务器名称
                return ActionClient(node, NavigateToPose, "navigate_to_pose")

            # 使用默认工厂函数
            action_client_factory = _default_action_client
        # 创建动作客户端实例
        self._action_client = action_client_factory(self)

        # 初始化任务状态变量
        self._current_index = 0  # 当前执行的航路点索引
        self._current_goal_handle: Optional[Any] = None  # 当前导航目标的句柄
        self._navigation_active = False  # 导航是否正在进行中
        self._alarm_active = False  # 热警报是否激活
        self._mission_started = False  # 任务是否已开始
        self._last_pose: Optional[PoseWithCovarianceStamped] = None  # 最后一次接收到的机器人位姿
        self._event_history: List[str] = []  # 事件历史记录
        self._last_waypoint: Optional[PoseStamped] = None  # 最后发送的航路点

        # 创建定时器，每0.5秒调用一次_mission_step方法
        # 用于推进任务执行流程
        self._mission_timer = self.create_timer(0.5, self._mission_step)
        # 添加参数设置回调函数，当参数被动态修改时会被调用
        self.add_on_set_parameters_callback(self._on_parameters_set)

        # 如果没有配置航路点，则发布任务空闲事件
        if not self._waypoints:
            self._emit_event("MISSION_IDLE", "No waypoints configured")

    # ------------------------------------------------------------------
    # Callbacks

    def _mission_step(self) -> None:
        """任务执行步骤回调函数，由定时器定期调用。
        
        这个方法是任务节点的核心逻辑，负责：
        1. 检查任务是否开始
        2. 验证执行条件
        3. 决定是否发送下一个航路点目标
        """
        # 如果任务尚未开始但已配置航路点，则标记任务开始并发布事件
        if not self._mission_started and self._waypoints:
            self._mission_started = True
            self._emit_event(
                "MISSION_STARTED", f"{len(self._waypoints)} waypoints queued"
            )

        # 如果没有配置航路点，则直接返回
        if not self._waypoints:
            return

        # 如果热警报激活或导航正在进行中，则不执行新任务
        if self._alarm_active or self._navigation_active:
            return

        # 如果动作服务器未准备好，则等待下次调用
        if not self._action_client.server_is_ready():
            return

        # 检查是否已到达航路点列表末尾
        if self._current_index >= len(self._waypoints):
            # 如果启用了循环模式，则重置索引到开头
            if self._loop:
                self._current_index = 0
            else:
                # 如果任务已开始，则标记任务完成并发布事件
                if self._mission_started:
                    self._mission_started = False
                    self._emit_event("MISSION_COMPLETE", "Finished waypoint list")
                return

        # 发送下一个航路点目标
        self._send_next_goal()

    def _send_next_goal(self) -> None:
        """发送下一个航路点目标到导航系统。
        
        这个方法负责：
        1. 获取当前索引对应的航路点
        2. 创建导航目标消息
        3. 发送目标到Nav2导航动作服务器
        4. 设置相关状态标志
        """
        # 获取当前索引对应的航路点
        pose = self._waypoints[self._current_index]
        # 为航路点添加时间戳（使用当前节点时钟时间）
        pose.header.stamp = self.get_clock().now().to_msg()

        # 创建导航到指定位置的动作目标消息
        goal_msg = NavigateToPose.Goal()
        # 设置目标位姿
        goal_msg.pose = pose

        # 标记导航正在进行中
        self._navigation_active = True
        # 发布航路点发送事件
        self._emit_event("WAYPOINT_SENT", f"index={self._current_index}")
        # 发布航路点到指定话题，供其他节点使用
        self._waypoint_pub.publish(pose)
        # 保存最后发送的航路点
        self._last_waypoint = pose

        # 异步发送目标到动作服务器
        # feedback_callback: 导航过程中的反馈回调函数
        goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        # 添加目标响应完成回调函数
        # 当目标被接受或拒绝时会调用此回调
        goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future: Future) -> None:
        """导航目标响应回调函数。
        
        当动作服务器对发送的目标作出响应时调用此函数。
        处理目标被接受或拒绝的情况。
        
        Args:
            future: 异步操作的结果对象
        """
        # 获取目标句柄结果
        goal_handle = future.result()
        # 如果目标句柄为空或目标被拒绝
        if goal_handle is None or not goal_handle.accepted:
            # 标记导航非活跃状态
            self._navigation_active = False
            # 发布航路点被拒绝事件
            self._emit_event("WAYPOINT_REJECTED", f"index={self._current_index}")
            # 推进到下一个航路点
            self._advance_index()
            return

        # 保存当前目标句柄
        self._current_goal_handle = goal_handle
        # 异步获取目标执行结果
        result_future = goal_handle.get_result_async()
        # 添加结果回调函数
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future: Future) -> None:
        """导航目标结果回调函数。
        
        当导航目标执行完成时调用此函数。
        处理导航成功或失败的情况。
        
        Args:
            future: 异步操作的结果对象
        """
        # 标记导航非活跃状态
        self._navigation_active = False
        # 清空当前目标句柄
        self._current_goal_handle = None

        # 获取执行结果
        result = future.result()
        # 获取执行状态（成功、失败等）
        status = getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
        # 如果导航成功
        if status == GoalStatus.STATUS_SUCCEEDED:
            # 发布航路点到达事件
            self._emit_event("WAYPOINT_REACHED", f"index={self._current_index}")
        else:
            # 发布航路点中止事件
            self._emit_event(
                "WAYPOINT_ABORTED", f"index={self._current_index}|status={status}"
            )

        # 推进到下一个航路点
        self._advance_index()

    def _feedback_callback(
        self, feedback
    ) -> None:  # noqa: D401 - Nav2 feedback structure
        """导航反馈回调函数。
        
        在导航过程中定期调用，提供导航进度反馈。
        当前实现中未使用反馈信息，但保留了钩子以备将来使用。
        
        Args:
            feedback: 导航过程中的反馈信息
        """
        # Feedback is not currently used but hook kept for future introspection.
        # 当前未使用反馈信息，但保留了钩子以备将来使用
        return

    def _amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """AMCL位姿回调函数。
        
        当接收到AMCL发布的机器人位姿时调用此函数。
        保存最新的机器人位姿信息。
        
        Args:
            msg: 带协方差的位姿消息
        """
        # 保存最后接收到的位姿消息
        self._last_pose = msg

    def _alarm_callback(self, msg: Bool) -> None:
        """热警报回调函数。
        
        当接收到热警报消息时调用此函数。
        处理警报激活和清除的逻辑。
        
        Args:
            msg: 布尔值消息，True表示警报激活，False表示警报清除
        """
        # 如果收到警报激活消息且当前警报未激活
        if msg.data and not self._alarm_active:
            # 标记警报激活
            self._alarm_active = True
            # 发布热警报进入事件
            self._emit_event("THERMAL_ALARM_ENTER", "Alarm engaged")
            # 如果启用了警报停止功能，且导航正在进行中，且存在当前目标句柄
            if (
                self._alarm_stop
                and self._navigation_active
                and self._current_goal_handle is not None
            ):
                # 异步取消当前导航目标
                cancel_future = self._action_client.cancel_goal_async(
                    self._current_goal_handle
                )
                # 添加取消完成回调函数
                cancel_future.add_done_callback(self._cancel_done_callback)
        # 如果收到警报清除消息且当前警报已激活
        elif not msg.data and self._alarm_active:
            # 标记警报清除
            self._alarm_active = False
            # 发布热警报清除事件
            self._emit_event("THERMAL_ALARM_CLEAR", "Alarm cleared")
            # 如果导航未进行中，则继续任务执行
            if not self._navigation_active:
                self._mission_step()

    def _cancel_done_callback(self, future: Future) -> None:
        """导航取消完成回调函数。
        
        当导航目标取消操作完成时调用此函数。
        
        Args:
            future: 异步操作的结果对象
        """
        # 标记导航非活跃状态
        self._navigation_active = False
        # 发布导航取消事件
        self._emit_event("NAV_CANCELLED", f"index={self._current_index}")

    # ------------------------------------------------------------------
    # Helpers

    def _advance_index(self) -> None:
        """推进航路点索引。
        
        将当前航路点索引加1，并处理循环和任务完成逻辑。
        """
        # 索引加1
        self._current_index += 1
        # 如果索引超出航路点列表范围
        if self._current_index >= len(self._waypoints):
            # 如果启用了循环且存在航路点，则重置索引到开头
            if self._loop and self._waypoints:
                self._current_index = 0
            else:
                # 发布任务完成事件
                self._emit_event("MISSION_COMPLETE", "Finished waypoint list")
                # 标记任务未开始
                self._mission_started = False
                return
        # 继续执行任务步骤
        self._mission_step()

    def _emit_event(self, event_type: str, detail: str) -> None:
        """发布事件消息。
        
        创建并发布事件消息到事件话题，同时记录到历史列表。
        
        Args:
            event_type: 事件类型
            detail: 事件详细信息
        """
        # 获取当前时间戳并转换为字符串
        stamp = self._now_to_string(self.get_clock().now().to_msg())
        # 构造事件条目
        entry = f"{stamp}|{event_type}|{detail}"
        # 添加到事件历史记录
        self._event_history.append(entry)
        # 发布事件消息
        self._event_pub.publish(String(data=entry))

    def _on_parameters_set(self, params: List[Parameter]) -> SetParametersResult:
        """参数设置回调函数。
        
        当节点参数被动态修改时调用此函数。
        
        Args:
            params: 被修改的参数列表
            
        Returns:
            SetParametersResult: 参数设置结果
        """
        # 遍历被修改的参数
        for param in params:
            # 如果是loop参数
            if param.name == "loop":
                # 更新_loop属性
                self._loop = bool(param.value)
            # 如果是alarm_stop参数
            elif param.name == "alarm_stop":
                # 更新_alarm_stop属性
                self._alarm_stop = bool(param.value)
            # 如果是waypoints参数
            elif param.name == "waypoints":
                # 解析并更新航路点列表
                self._waypoints = self._parse_waypoints(param.value)
                # 确保当前索引不超过航路点列表长度
                self._current_index = min(self._current_index, len(self._waypoints))
        # 返回设置成功结果
        return SetParametersResult(successful=True)

    @staticmethod
    def _parse_waypoints(raw_waypoints) -> List[PoseStamped]:
        """解析航路点数据。
        
        将原始航路点数据转换为PoseStamped对象列表。
        
        Args:
            raw_waypoints: 原始航路点数据（可能是字符串、字典或PoseStamped对象）
            
        Returns:
            List[PoseStamped]: 解析后的航路点列表
        """
        # 初始化航路点列表
        waypoints: List[PoseStamped] = []
        # 遍历原始航路点数据
        for entry in raw_waypoints or []:
            # 如果是字符串类型，则尝试解析为JSON
            if isinstance(entry, str):
                try:
                    entry = json.loads(entry)
                except json.JSONDecodeError:
                    # 解析失败则跳过该项
                    continue
            # 如果已经是PoseStamped对象，则直接添加
            if isinstance(entry, PoseStamped):
                waypoints.append(entry)
                continue

            # 如果不是字典类型，则跳过该项
            if not isinstance(entry, dict):
                continue

            # 提取位姿数据和头部信息
            pose_data = entry.get("pose", {})
            header = entry.get("header", {})
            # 创建新的PoseStamped对象
            pose = PoseStamped()
            # 设置坐标系ID，默认为"map"
            pose.header.frame_id = header.get("frame_id", "map")
            # 设置位置坐标
            pose.pose.position.x = float(pose_data.get("position", {}).get("x", 0.0))
            pose.pose.position.y = float(pose_data.get("position", {}).get("y", 0.0))
            pose.pose.position.z = float(pose_data.get("position", {}).get("z", 0.0))

            # 设置方向四元数
            orientation = pose_data.get("orientation", {})
            pose.pose.orientation.x = float(orientation.get("x", 0.0))
            pose.pose.orientation.y = float(orientation.get("y", 0.0))
            pose.pose.orientation.z = float(orientation.get("z", 0.0))
            pose.pose.orientation.w = float(orientation.get("w", 1.0))

            # 添加到航路点列表
            waypoints.append(pose)

        # 返回解析后的航路点列表
        return waypoints

    @staticmethod
    def _now_to_string(stamp: Time) -> str:
        """将时间戳转换为字符串。
        
        Args:
            stamp: ROS 2时间戳
            
        Returns:
            str: ISO格式的时间字符串
        """
        # 将秒和纳秒转换为总秒数
        seconds = stamp.sec + stamp.nanosec * 1e-9
        # 转换为UTC时区的ISO格式时间字符串
        return datetime.fromtimestamp(seconds, tz=timezone.utc).isoformat()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
