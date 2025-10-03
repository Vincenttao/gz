from __future__ import annotations

from typing import List

import pytest
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from std_msgs.msg import Bool

from inspection_mission.mission_node import MissionNode


class _MockResult:
    def __init__(self, status: int) -> None:
        self.status = status


class _MockGoalHandle:
    def __init__(self) -> None:
        self.accepted = True
        self._result_future = Future()

    def get_result_async(self) -> Future:
        return self._result_future

    def finish(self, status: int) -> None:
        if not self._result_future.done():
            self._result_future.set_result(_MockResult(status))


class _MockActionClient:
    def __init__(self, *_args, **_kwargs) -> None:
        self._server_ready = True
        self.goal_handles: List[_MockGoalHandle] = []
        self.sent_goals: List = []
        self.cancelled_handles: List[_MockGoalHandle] = []

    def server_is_ready(self) -> bool:
        return self._server_ready

    def send_goal_async(self, goal_msg, feedback_callback=None) -> Future:
        self.sent_goals.append(goal_msg.pose)
        goal_handle = _MockGoalHandle()
        self.goal_handles.append(goal_handle)
        future = Future()
        future.set_result(goal_handle)
        return future

    def cancel_goal_async(self, goal_handle) -> Future:
        self.cancelled_handles.append(goal_handle)
        future = Future()
        future.set_result(True)
        return future

    def complete_latest(self, status: int = GoalStatus.STATUS_SUCCEEDED) -> None:
        if self.goal_handles:
            self.goal_handles[-1].finish(status)


@pytest.fixture(scope="module", autouse=True)
def rclpy_runtime():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_waypoint(x: float, y: float) -> dict:
    return {
        "header": {"frame_id": "map"},
        "pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }


def test_waypoint_sequence_advances_and_completes():
    mock_client = _MockActionClient()
    try:
        node = MissionNode(action_client_factory=lambda node: mock_client)
    except RCLError as exc:
        pytest.skip(f"RMW unavailable: {exc}")
    node._waypoints = node._parse_waypoints([_make_waypoint(1.0, 0.0), _make_waypoint(2.0, 0.5)])
    node._loop = False

    try:
        node._mission_step()
        assert len(mock_client.sent_goals) == 1

        mock_client.complete_latest(GoalStatus.STATUS_SUCCEEDED)
        assert any("WAYPOINT_REACHED" in event for event in node._event_history)
        assert len(mock_client.sent_goals) == 2  # next waypoint dispatched automatically

        mock_client.complete_latest(GoalStatus.STATUS_SUCCEEDED)
        assert any("MISSION_COMPLETE" in event for event in node._event_history)
    finally:
        node.destroy_node()


def test_alarm_triggers_cancel_when_requested():
    mock_client = _MockActionClient()
    try:
        node = MissionNode(action_client_factory=lambda node: mock_client)
    except RCLError as exc:
        pytest.skip(f"RMW unavailable: {exc}")
    node._waypoints = node._parse_waypoints([_make_waypoint(0.5, 0.0)])
    node._loop = False

    try:
        node._mission_step()
        assert node._navigation_active

        node._alarm_callback(Bool(data=True))
        assert node._alarm_active
        assert mock_client.cancelled_handles
        assert any("THERMAL_ALARM_ENTER" in event for event in node._event_history)

        node._alarm_callback(Bool(data=False))
        assert not node._alarm_active
        assert any("THERMAL_ALARM_CLEAR" in event for event in node._event_history)
    finally:
        node.destroy_node()
