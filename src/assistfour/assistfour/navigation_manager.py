from __future__ import annotations

from threading import Event, Lock, Thread
from math import atan2, pi
from typing import TYPE_CHECKING
from geometry_msgs.msg import Twist

from .constants import ROBOT_FRAME, SEARCH_SPIN_RATE, MARKER_SEARCH_PERIOD, MINIMUM_ANGLE_THRESHOLD

if TYPE_CHECKING:
    from rclpy.timer import Timer
    from .main import Main


class NavigationManager:
    def __init__(self, node: Main):
        self._node = node

        # Marker searching.
        self._search_spin_loop: Timer | None = None
        self._search_stop_event = Event()
        self._angle_lock = Lock()
        self._angle_to_marker = pi

    def search_for_marker(self, name: str, clockwise: bool):
        # Stow for safety.
        self._node.stow_the_robot()

        # Look down slightly (markers are lower than head).
        self._node.move_to_pose({"joint_head_tilt": -0.3}, blocking=True)

        # Switch to nav mode.
        self._node.switch_to_navigation_mode()

        # Define spin loop.
        def spin():
            # Exit if spin loop is not running or publisher is not ready.
            if self._search_spin_loop is None or self._node.vel_publisher is None:
                return

            # Define velocity command.
            command = Twist()

            # Clamp rate to spin speed and double angle rate to converge faster.
            with self._angle_lock:
                angle_to_marker = self._angle_to_marker
            rate = min(angle_to_marker * 2, SEARCH_SPIN_RATE)

            # Apply direction.
            if clockwise:
                rate = -rate

            # Round to 0 when within threshold.
            if abs(angle_to_marker) < MINIMUM_ANGLE_THRESHOLD:
                rate = 0.0
                self._search_stop_event.set()
                self._search_spin_loop.destroy()
                self._node.get_logger().info("Aligned to marker.")

            # Send command.
            command.angular.z = rate
            self._node.vel_publisher.publish(command)

        # Define search worker. get_tf can block, so this runs outside ROS timer callbacks.
        def search():
            while not self._search_stop_event.is_set():
                # Try to find marker.
                tf = self._node.get_tf(ROBOT_FRAME, name)

                # Set speed as a function of angle to marker.
                with self._angle_lock:
                    if tf is not None:
                        self._angle_to_marker = atan2(
                            tf.transform.translation.y,
                            tf.transform.translation.x,
                        )
                    else:
                        # Set it to be larger than the spin rate.
                        self._angle_to_marker = pi

        # Reset search state for a fresh run.
        self._search_stop_event.clear()

        # Do spin.
        self._search_spin_loop = self._node.create_timer(MARKER_SEARCH_PERIOD, spin)

        # Do search (non-blocking for executor/timers).
        Thread(target=search, daemon=True).start()
