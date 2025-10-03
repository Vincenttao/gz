from __future__ import annotations

import json
from datetime import datetime, timezone
from math import hypot
from typing import Callable, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import ParameterValue
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time as RclTime
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


try:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
except ImportError:  # pragma: no cover - compatibility shim
    LookupException = ConnectivityException = ExtrapolationException = Exception


class ThermalAlarmNode(Node):
    """Monitor configured hot targets and raise an alarm when approached."""

    def __init__(
        self,
        pose_provider: Optional[Callable[[], Optional[Tuple[float, float, float]]]] = None,
    ) -> None:
        super().__init__("thermal_alarm_node")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._alarm_pub = self.create_publisher(Bool, "/inspection/alarm", qos)
        self._event_pub = self.create_publisher(String, "/inspection/event", qos)

        self.declare_parameter(
            "hot_targets",
            ParameterValue(string_array_value=[]),
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter("trigger_distance", 2.0)
        self.declare_parameter("hysteresis", 0.3)
        self.declare_parameter("check_rate_hz", 10)

        self._hot_targets = self._parse_targets(self.get_parameter("hot_targets").value)
        self._trigger_distance = float(self.get_parameter("trigger_distance").value)
        self._hysteresis = float(self.get_parameter("hysteresis").value)
        self._check_rate = max(1, int(self.get_parameter("check_rate_hz").value))

        self._alarm_state = False
        self._event_history: List[str] = []

        if pose_provider is None:
            self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
            self._pose_provider = self._lookup_robot_position
        else:
            self._tf_buffer = None
            self._tf_listener = None
            self._pose_provider = pose_provider

        self._timer = self.create_timer(1.0 / float(self._check_rate), self._evaluate_alarm)
        self._publish_alarm(False)

    # ------------------------------------------------------------------
    # Evaluation

    def _lookup_robot_position(self) -> Optional[Tuple[float, float, float]]:
        if self._tf_buffer is None:
            return None
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", RclTime())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        translation = transform.transform.translation
        return translation.x, translation.y, translation.z

    def _evaluate_alarm(self) -> None:
        pose = self._pose_provider()
        if pose is None:
            return

        if not self._hot_targets:
            if self._alarm_state:
                self._alarm_state = False
                self._publish_alarm(False)
                self._emit_event("THERMAL_ALARM_CLEAR", "No targets configured")
            return

        min_distance = min(hypot(pose[0] - target[0], pose[1] - target[1]) for target in self._hot_targets)
        if self._alarm_state:
            if min_distance >= self._trigger_distance + self._hysteresis:
                self._alarm_state = False
                self._publish_alarm(False)
                self._emit_event("THERMAL_ALARM_CLEAR", f"distance={min_distance:.2f}")
        else:
            if min_distance <= self._trigger_distance:
                self._alarm_state = True
                self._publish_alarm(True)
                self._emit_event("THERMAL_ALARM_ENTER", f"distance={min_distance:.2f}")

    # ------------------------------------------------------------------
    # Helpers

    def _publish_alarm(self, state: bool) -> None:
        self._alarm_pub.publish(Bool(data=state))

    def _emit_event(self, event_type: str, detail: str) -> None:
        stamp = self._now_to_string(self.get_clock().now().to_msg())
        entry = f"{stamp}|{event_type}|{detail}"
        self._event_history.append(entry)
        self._event_pub.publish(String(data=entry))

    @staticmethod
    def _parse_targets(raw_targets) -> List[Tuple[float, float, float]]:
        targets: List[Tuple[float, float, float]] = []
        for target in raw_targets or []:
            if isinstance(target, str):
                try:
                    target = json.loads(target)
                except json.JSONDecodeError:
                    continue
            if isinstance(target, Pose):
                targets.append((target.position.x, target.position.y, target.position.z))
                continue
            if not isinstance(target, dict):
                continue
            position = target.get("position", {})
            targets.append(
                (
                    float(position.get("x", 0.0)),
                    float(position.get("y", 0.0)),
                    float(position.get("z", 0.0)),
                )
            )
        return targets

    @staticmethod
    def _now_to_string(stamp: Time) -> str:
        seconds = stamp.sec + stamp.nanosec * 1e-9
        return datetime.fromtimestamp(seconds, tz=timezone.utc).isoformat()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ThermalAlarmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
