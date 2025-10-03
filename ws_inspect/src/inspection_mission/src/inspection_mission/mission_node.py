from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Callable, List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# Nav2 action is required at runtime; provide a lightweight stub for test environments
# where the package may not be installed (e.g., restricted CI sandboxes).
try:
    from nav2_msgs.action import NavigateToPose
except ModuleNotFoundError:  # pragma: no cover - fallback only exercised in tests

    class _NavigateGoal:  # minimal stand-in matching the interface used in tests
        def __init__(self) -> None:
            self.pose = PoseStamped()

    class NavigateToPose:  # type: ignore
        Goal = _NavigateGoal


from rcl_interfaces.msg import ParameterDescriptor, ParameterType, ParameterValue, SetParametersResult
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter, ParameterValue
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, String


class MissionNode(Node):
    """Waypoint mission executor coordinating navigation and thermal events."""

    def __init__(
        self,
        action_client_factory: Optional[Callable[[Node], ActionClient]] = None,
    ) -> None:
        super().__init__("mission_node")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._waypoint_pub = self.create_publisher(
            PoseStamped, "/inspection/waypoint_goal", qos
        )
        self._event_pub = self.create_publisher(String, "/inspection/event", qos)
        self._alarm_sub = self.create_subscription(
            Bool, "/inspection/alarm", self._alarm_callback, qos
        )
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_callback, qos
        )

        self.declare_parameter("loop", True)
        self.declare_parameter("alarm_stop", True)
        # Explicitly type as STRING_ARRAY to avoid empty-list ambiguity
        self.declare_parameter(
            "waypoints",
            ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=[]),
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )

        self._loop = bool(self.get_parameter("loop").value)
        self._alarm_stop = bool(self.get_parameter("alarm_stop").value)
        self._raw_waypoints = self.get_parameter("waypoints").value
        self._waypoints = self._parse_waypoints(self._raw_waypoints)

        if action_client_factory is None:

            def _default_action_client(node: Node) -> ActionClient:
                return ActionClient(node, NavigateToPose, "navigate_to_pose")

            action_client_factory = _default_action_client
        self._action_client = action_client_factory(self)

        self._current_index = 0
        self._current_goal_handle: Optional[Any] = None
        self._navigation_active = False
        self._alarm_active = False
        self._mission_started = False
        self._last_pose: Optional[PoseWithCovarianceStamped] = None
        self._event_history: List[str] = []
        self._last_waypoint: Optional[PoseStamped] = None

        self._mission_timer = self.create_timer(0.5, self._mission_step)
        self.add_on_set_parameters_callback(self._on_parameters_set)

        if not self._waypoints:
            self._emit_event("MISSION_IDLE", "No waypoints configured")

    # ------------------------------------------------------------------
    # Callbacks

    def _mission_step(self) -> None:
        if not self._mission_started and self._waypoints:
            self._mission_started = True
            self._emit_event(
                "MISSION_STARTED", f"{len(self._waypoints)} waypoints queued"
            )

        if not self._waypoints:
            return

        if self._alarm_active or self._navigation_active:
            return

        if not self._action_client.server_is_ready():
            return

        if self._current_index >= len(self._waypoints):
            if self._loop:
                self._current_index = 0
            else:
                if self._mission_started:
                    self._mission_started = False
                    self._emit_event("MISSION_COMPLETE", "Finished waypoint list")
                return

        self._send_next_goal()

    def _send_next_goal(self) -> None:
        pose = self._waypoints[self._current_index]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._navigation_active = True
        self._emit_event("WAYPOINT_SENT", f"index={self._current_index}")
        self._waypoint_pub.publish(pose)
        self._last_waypoint = pose

        goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._navigation_active = False
            self._emit_event("WAYPOINT_REJECTED", f"index={self._current_index}")
            self._advance_index()
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future: Future) -> None:
        self._navigation_active = False
        self._current_goal_handle = None

        result = future.result()
        status = getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._emit_event("WAYPOINT_REACHED", f"index={self._current_index}")
        else:
            self._emit_event(
                "WAYPOINT_ABORTED", f"index={self._current_index}|status={status}"
            )

        self._advance_index()

    def _feedback_callback(
        self, feedback
    ) -> None:  # noqa: D401 - Nav2 feedback structure
        # Feedback is not currently used but hook kept for future introspection.
        return

    def _amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_pose = msg

    def _alarm_callback(self, msg: Bool) -> None:
        if msg.data and not self._alarm_active:
            self._alarm_active = True
            self._emit_event("THERMAL_ALARM_ENTER", "Alarm engaged")
            if (
                self._alarm_stop
                and self._navigation_active
                and self._current_goal_handle is not None
            ):
                cancel_future = self._action_client.cancel_goal_async(
                    self._current_goal_handle
                )
                cancel_future.add_done_callback(self._cancel_done_callback)
        elif not msg.data and self._alarm_active:
            self._alarm_active = False
            self._emit_event("THERMAL_ALARM_CLEAR", "Alarm cleared")
            if not self._navigation_active:
                self._mission_step()

    def _cancel_done_callback(self, future: Future) -> None:
        self._navigation_active = False
        self._emit_event("NAV_CANCELLED", f"index={self._current_index}")

    # ------------------------------------------------------------------
    # Helpers

    def _advance_index(self) -> None:
        self._current_index += 1
        if self._current_index >= len(self._waypoints):
            if self._loop and self._waypoints:
                self._current_index = 0
            else:
                self._emit_event("MISSION_COMPLETE", "Finished waypoint list")
                self._mission_started = False
                return
        self._mission_step()

    def _emit_event(self, event_type: str, detail: str) -> None:
        stamp = self._now_to_string(self.get_clock().now().to_msg())
        entry = f"{stamp}|{event_type}|{detail}"
        self._event_history.append(entry)
        self._event_pub.publish(String(data=entry))

    def _on_parameters_set(self, params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name == "loop":
                self._loop = bool(param.value)
            elif param.name == "alarm_stop":
                self._alarm_stop = bool(param.value)
            elif param.name == "waypoints":
                self._waypoints = self._parse_waypoints(param.value)
                self._current_index = min(self._current_index, len(self._waypoints))
        return SetParametersResult(successful=True)

    @staticmethod
    def _parse_waypoints(raw_waypoints) -> List[PoseStamped]:
        waypoints: List[PoseStamped] = []
        for entry in raw_waypoints or []:
            if isinstance(entry, str):
                try:
                    entry = json.loads(entry)
                except json.JSONDecodeError:
                    continue
            if isinstance(entry, PoseStamped):
                waypoints.append(entry)
                continue

            if not isinstance(entry, dict):
                continue

            pose_data = entry.get("pose", {})
            header = entry.get("header", {})
            pose = PoseStamped()
            pose.header.frame_id = header.get("frame_id", "map")
            pose.pose.position.x = float(pose_data.get("position", {}).get("x", 0.0))
            pose.pose.position.y = float(pose_data.get("position", {}).get("y", 0.0))
            pose.pose.position.z = float(pose_data.get("position", {}).get("z", 0.0))

            orientation = pose_data.get("orientation", {})
            pose.pose.orientation.x = float(orientation.get("x", 0.0))
            pose.pose.orientation.y = float(orientation.get("y", 0.0))
            pose.pose.orientation.z = float(orientation.get("z", 0.0))
            pose.pose.orientation.w = float(orientation.get("w", 1.0))

            waypoints.append(pose)

        return waypoints

    @staticmethod
    def _now_to_string(stamp: Time) -> str:
        seconds = stamp.sec + stamp.nanosec * 1e-9
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
