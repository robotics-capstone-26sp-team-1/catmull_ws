from __future__ import annotations

from threading import Thread
from math import atan2
from typing import TYPE_CHECKING
from geometry_msgs.msg import Twist, TransformStamped
from tf_transformations import quaternion_matrix
from numpy import array, matmul
from time import sleep
from rclpy.time import Time

from .constants import (
    ROBOT_FRAME,
    SEARCH_SPIN_RATE,
    MARKER_SEARCH_PERIOD,
    MINIMUM_ANGLE_THRESHOLD,
    MAX_TF_AGE,
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
                tf = self._get_recent_tf(ROBOT_FRAME, name)

                if tf is not None:
                    self._marker_found = True
                    break

        # Reset search state.
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

        def compute_angle_to_marker() -> float:
            tf = self._get_recent_tf(ROBOT_FRAME, name)
            if tf is None:
                raise ValueError("Marker not found. Unable to compute angle to marker.")
            target_point_x, target_point_y = self._target_xy_from_tf(tf, forward_offset)
            angle = atan2(target_point_y, target_point_x)
            self._node.get_logger().info(f"Angle to marker: {angle}")
            return angle

        # Iterative refinement.
        angle_to_marker = compute_angle_to_marker()
        while abs(angle_to_marker) > MINIMUM_ANGLE_THRESHOLD:
            # Rotate.
            self._node.get_logger().info(f"Correcting by {angle_to_marker}...")
            self._node.move_to_pose({"rotate_mobile_base": angle_to_marker}, blocking=True)

            # Wait for marker to refresh.
            sleep(1)

            # Compute current angle.
            angle_to_marker = compute_angle_to_marker()

        self._node.get_logger().info("Pointing at marker!")

    def drive_to_marker(self, name: str, forward_offset: float):
        # Enter travel pose.
        self.enter_travel_pose()

        # Look down slightly and align to drive direction.
        self._node.move_to_pose({"joint_head_tilt": -0.3, "joint_head_pan": 0}, blocking=True)

        def compute_distance_to_marker() -> float:
            tf = self._get_recent_tf(ROBOT_FRAME, name)
            if tf is None:
                raise ValueError("Marker not found. Unable to drive to point.")

            target_point_x, _ = self._target_xy_from_tf(tf, forward_offset)
            self._node.get_logger().info(f"Distance to marker: {target_point_x}")
            return target_point_x

        # Approach.
        distance_to_marker = compute_distance_to_marker()
        self._node.move_to_pose({"translate_mobile_base": distance_to_marker}, blocking=True)
        # while abs(distance_to_marker) > MINIMUM_FORWARD_DISTANCE_THRESHOLD:
        #     # Compute current distance.
        #     distance_to_marker = compute_distance_to_marker()
        #
        #     # Drive.
        #     self._node.get_logger().info(f"Moving by {distance_to_marker}...")
        #     self._node.move_to_pose({"translate_mobile_base": distance_to_marker}, blocking=True)
        #     sleep(1)
        #
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

    def _get_recent_tf(self, source: str, target: str) -> TransformStamped | None:
        tf = self._node.get_tf(source, target)
        if tf is not None:
            current_time = self._node.get_clock().now()
            tf_age = (current_time - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            self._node.get_logger().info(f"TF age: {tf_age:.3f} seconds")
            if tf_age <= MAX_TF_AGE:
                return tf

        return None

    @staticmethod
    def _target_xy_from_tf(tf: TransformStamped, forward_offset: float) -> tuple[float, float]:
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
