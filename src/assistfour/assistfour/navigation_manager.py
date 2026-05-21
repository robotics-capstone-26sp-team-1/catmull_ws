from __future__ import annotations

from threading import Thread
from math import atan2, inf, sqrt
from typing import TYPE_CHECKING
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_matrix
from numpy import array, matmul

from .constants import (
    ROBOT_FRAME,
    SEARCH_SPIN_RATE,
    MARKER_SEARCH_PERIOD,
    MINIMUM_ANGLE_THRESHOLD,
    MINIMUM_FORWARD_DISTANCE_THRESHOLD,
)

if TYPE_CHECKING:
    from rclpy.timer import Timer
    from .main import Main


class NavigationManager:
    def __init__(self, node: Main):
        self._node = node

        # Marker searching.
        self._search_spin_loop: Timer | None = None
        self._marker_found = False

    def point_at_marker(self, name: str, clockwise: bool, forward_offset: float):
        # Enter travel pose.
        self.enter_travel_pose()

        # Look down slightly (markers are lower than head).
        self._node.move_to_pose({"joint_head_tilt": -0.3}, blocking=True)

        # Switch to navigation mode.
        self._node.switch_to_navigation_mode()

        # Define spin loop.
        def spin():
            # Exit if spin loop is not running or publisher is not ready.
            if self._search_spin_loop is None or self._node.vel_publisher is None:
                return

            # Not found yet.
            if self._marker_found:
                self._search_spin_loop.destroy()
                self._node.stop_the_robot()
                self._node.get_logger().info("Stopping search spin.")
            else:
                # Continue spinning.
                command = Twist()
                command.angular.z = -SEARCH_SPIN_RATE if clockwise else SEARCH_SPIN_RATE
                self._node.vel_publisher.publish(command)

        # Define search worker. get_tf can block, so this runs outside ROS timer callbacks.
        def search():
            while True:
                # Try to find marker.
                tf = self._node.get_tf(ROBOT_FRAME, name)

                if tf is not None:
                    self._marker_found = True
                    break

        # Reset search state for the next search.
        self._marker_found = False

        # Do spin.
        self._search_spin_loop = self._node.create_timer(MARKER_SEARCH_PERIOD, spin)

        # Do search (non-blocking for executor/timers).
        search_thread = Thread(target=search, daemon=True)
        search_thread.start()

        # Wait for search to complete.
        search_thread.join()

        # Switch back to position mode.
        self._node.switch_to_position_mode()

        # Iterative refinement.
        angle_to_marker = inf
        while abs(angle_to_marker) > MINIMUM_ANGLE_THRESHOLD:
            # Compute current angle.
            tf = self._node.get_tf(ROBOT_FRAME, name)
            target_point_x, target_point_y = self._target_xy_from_tf(tf, forward_offset)
            angle_to_marker = atan2(target_point_y, target_point_x)

            # Rotate.
            self._node.get_logger().info(f"Correcting by {angle_to_marker}...")
            self._node.move_to_pose({"rotate_mobile_base": angle_to_marker}, blocking=True)

        self._node.get_logger().info("Pointing at marker!")

    def drive_to_marker(self, name: str, forward_offset: float):
        # Enter travel pose.
        self.enter_travel_pose()

        # Look down slightly.
        self._node.move_to_pose({"joint_head_tilt": -0.3, "joint_head_pan": 0}, blocking=True)

        # Verify marker can be found.
        if self._node.get_tf(ROBOT_FRAME, name) is None:
            raise ValueError("Marker not found. Unable to drive to point.")

        # Approach.
        distance_to_point = inf
        while abs(distance_to_point) > MINIMUM_FORWARD_DISTANCE_THRESHOLD:
            # Compute current distance.
            tf = self._node.get_tf(ROBOT_FRAME, name)
            target_point_x, target_point_y = self._target_xy_from_tf(tf, forward_offset)
            distance_to_point = sqrt(target_point_x * target_point_x + target_point_y * target_point_y)

            # Drive.
            self._node.get_logger().info(f"Moving by {distance_to_point}...")
            self._node.move_to_pose({"translate_mobile_base": distance_to_point}, blocking=True)

        self._node.get_logger().info("At marker!")

    def enter_travel_pose(self):
        self._node.move_to_pose(
            {
                "joint_wrist_pitch": -1.6,
                "joint_wrist_roll": 0.0,
                "joint_wrist_yaw": 0.0,
                "joint_lift": 0.45,
                "joint_arm": 0.0,
            },
            blocking=True,
        )

    @staticmethod
    def _target_xy_from_tf(tf, forward_offset: float) -> tuple[float, float]:
        # Compute the target point in the robot frame after applying the marker offset.
        if forward_offset == 0:
            return tf.transform.translation.x, tf.transform.translation.y

        rotation_matrix = quaternion_matrix(
            (
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            )
        )
        offset_vector = array([[0], [0], [forward_offset], [1]])
        marker_vector = array(
            [[tf.transform.translation.x], [tf.transform.translation.y], [0], [1]]
        )
        offset_direction = matmul(rotation_matrix, offset_vector)
        final_location = offset_direction + marker_vector
        return float(final_location[0, 0]), float(final_location[1, 0])
