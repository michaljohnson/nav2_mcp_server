"""Tests for the latched navigation stop."""

from unittest.mock import Mock

import pytest

from nav2_mcp_server import stop_latch


@pytest.fixture(autouse=True)
def clear_latch():
    """Start and end every test without a latched stop."""
    stop_latch.STOP_LATCHED.clear()
    yield
    stop_latch.STOP_LATCHED.clear()


def test_stop_latches_until_resume() -> None:
    """A confirmed stop stays latched until resume_navigation clears it."""
    assert stop_latch.request_navigation_stop() == 'Navigation stop confirmed'
    assert stop_latch.navigation_stopped()
    assert stop_latch.resume_navigation() == 'Navigation resumed'
    assert not stop_latch.navigation_stopped()


def test_stop_is_not_confirmed_while_ros_work_runs() -> None:
    """The stop latches at once but is confirmed only when the lock is free."""
    with stop_latch.ROS_LOCK:
        with pytest.raises(TimeoutError):
            stop_latch.request_navigation_stop(timeout=0.01)
        assert stop_latch.navigation_stopped()
        with pytest.raises(TimeoutError):
            stop_latch.resume_navigation(timeout=0.01)
    assert stop_latch.navigation_stopped()


def test_monitor_cancels_once() -> None:
    """A monitor loop cancels the active task once per latched stop."""
    navigator = Mock()
    assert stop_latch.cancel_if_requested(navigator) is False
    stop_latch.STOP_LATCHED.set()
    sent = stop_latch.cancel_if_requested(navigator)
    sent = stop_latch.cancel_if_requested(navigator, sent)
    assert sent is True
    navigator.cancelTask.assert_called_once()
