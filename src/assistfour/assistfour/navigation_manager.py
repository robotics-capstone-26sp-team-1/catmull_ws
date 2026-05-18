from __future__ import annotations

from threading import Event, Lock, Thread
from math import atan2, pi
from typing import TYPE_CHECKING
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion, quaternion_matrix
from numpy import array, matmul

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

    def point_at_marker(self, name: str, clockwise: bool, forward_offset: float):
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

            # Clamp rate to spin speed (with direction).
            with self._angle_lock:
                angle_to_marker = self._angle_to_marker

            if abs(angle_to_marker) > SEARCH_SPIN_RATE:
                rate = -SEARCH_SPIN_RATE if clockwise else SEARCH_SPIN_RATE
            else:
                rate = angle_to_marker

            # Round to 0 when within threshold.
            if abs(angle_to_marker) < MINIMUM_ANGLE_THRESHOLD:
                rate = 0.0
                self._search_stop_event.set()
                self._search_spin_loop.destroy()
                self._node.get_logger().info("Aligned to marker.")

            # Send command.
            command.angular.z = rate
            self._node.get_logger().info(f"Setting rate: {rate} rad/sec.")
            self._node.vel_publisher.publish(command)

        # Define search worker. get_tf can block, so this runs outside ROS timer callbacks.
        def search():
            while not self._search_stop_event.is_set():
                # Try to find marker.
                tf = self._node.get_tf(ROBOT_FRAME, name)

                # Compute angle to marker.
                with self._angle_lock:
                    if tf is not None:
                        # Directly set angle to marker if no offset.
                        if forward_offset == 0.0:
                            self._angle_to_marker = atan2(
                                tf.transform.translation.y,
                                tf.transform.translation.x,
                            )
                        else:
                            # Compute offset from marker.
                            rotation_matrix = quaternion_matrix((
                                tf.transform.rotation.x,
                                tf.transform.rotation.y,
                                tf.transform.rotation.z,
                                tf.transform.rotation.w,
                            ))
                            offset_vector = array([[0], [0], [forward_offset], [1]])
                            marker_vector = array(
                                [[tf.transform.translation.x], [tf.transform.translation.y], [0], [1]])
                            offset_direction = matmul(rotation_matrix, offset_vector)
                            final_location = offset_direction + marker_vector
                            final_x = final_location[0, 0]
                            final_y = final_location[1, 0]

                            # Set angle to offset position.
                            self._angle_to_marker = atan2(final_y, final_x)

                    else:
                        # Set it to be larger than the spin rate.
                        self._angle_to_marker = pi

        # Reset search state for a fresh run.
        self._search_stop_event.clear()

        # Do spin.
        self._search_spin_loop = self._node.create_timer(MARKER_SEARCH_PERIOD, spin)

        # Do search (non-blocking for executor/timers).
        search_thread = Thread(target=search, daemon=True)
        search_thread.start()

        # Wait for search to complete.
        search_thread.join()
