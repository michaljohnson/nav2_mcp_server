"""Latched, thread-safe stop signalling; all ROS work stays with the lock owner.

A stop latches: once requested it stays in force until resume_navigation clears it.
While latched, no new goal is sent (navigation.py checks before every goal), so a
navigation request that was already in flight when the stop was confirmed cannot
start a drive afterwards.
"""
import threading

ROS_LOCK = threading.Lock()
STOP_LATCHED = threading.Event()


def navigation_stopped():
    """True while a stop is latched; checked before any goal is sent."""
    return STOP_LATCHED.is_set()


def cancel_if_requested(navigator, already_cancelled=False):
    """Called only by a monitor already holding ROS_LOCK; cancels a task once."""
    if already_cancelled or not STOP_LATCHED.is_set():
        return already_cancelled
    navigator.cancelTask()
    return True


def request_navigation_stop(timeout: float = 4.0) -> str:
    """Latch the stop, then acknowledge only after ROS work releases its lock."""
    STOP_LATCHED.set()
    if not ROS_LOCK.acquire(timeout=timeout):
        raise TimeoutError('Navigation stop not confirmed; ROS worker is still busy')
    ROS_LOCK.release()
    return 'Navigation stop confirmed'


def resume_navigation(timeout: float = 4.0) -> str:
    """Clear a latched stop once no ROS work is running; new goals are accepted again."""
    if not ROS_LOCK.acquire(timeout=timeout):
        raise TimeoutError('Navigation not resumed; ROS worker is still busy')
    try:
        STOP_LATCHED.clear()
        return 'Navigation resumed'
    finally:
        ROS_LOCK.release()
