from __future__ import annotations

import pytest
import rclpy
from rclpy._rclpy_pybind11 import RCLError

from inspection_thermal.thermal_alarm_node import ThermalAlarmNode


class _PoseProvider:
    def __init__(self, initial):
        self.value = initial

    def __call__(self):
        return self.value


@pytest.fixture(scope="module", autouse=True)
def rclpy_runtime():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_alarm_engages_and_clears():
    provider = _PoseProvider((5.0, 0.0, 0.0))
    try:
        node = ThermalAlarmNode(pose_provider=provider)
    except RCLError as exc:
        pytest.skip(f"RMW unavailable: {exc}")
    node._hot_targets = [(0.0, 0.0, 0.0)]
    node._trigger_distance = 2.0
    node._hysteresis = 0.3

    try:
        node._evaluate_alarm()
        assert not node._alarm_state

        provider.value = (1.0, 0.0, 0.0)
        node._evaluate_alarm()
        assert node._alarm_state
        assert any("THERMAL_ALARM_ENTER" in event for event in node._event_history)

        provider.value = (3.0, 0.0, 0.0)
        node._evaluate_alarm()
        assert not node._alarm_state
        assert any("THERMAL_ALARM_CLEAR" in event for event in node._event_history)
    finally:
        node.destroy_node()
