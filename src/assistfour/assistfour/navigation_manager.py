from __future__ import annotations

from threading import Event, Lock, Thread
from math import atan2, pi, inf
from typing import TYPE_CHECKING
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_matrix
from numpy import array, matmul

from .constants import (
    ROBOT_FRAME,
    SEARCH_SPIN_RATE,
    MARKER_SEARCH_PERIOD,
    MINIMUM_ANGLE_THRESHOLD,
    MAX_FORWARD_SPEED,
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
        self._search_stop_event = Event()
        self._angle_lock = Lock()
        self._angle_to_marker = pi

        # Drive to point.
        self._point_drive_loop: Timer | None = None
        self._point_stop_event = Event()
        self._point_lock = Lock()
        self._point_distance_to_target = inf
        self._in_drive_recovery = False

    def point_at_marker(self, name: str, clockwise: bool, forward_offset: float):
        # Enter travel pose.
        self.enter_travel_pose()

        # Look down slightly (markers are lower than head).
        self._node.move_to_pose({"joint_head_tilt": -0.3}, blocking=True)

        # Switch to navigation mode.
        self._node.switch_to_navigation_mode()

        # Assume marker is not found.
        self._angle_to_marker = pi

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
                self._search_stop_event.set()
                self._search_spin_loop.destroy()
                self._node.stop_the_robot()
                self._node.get_logger().info("Aligned to marker.")
                return

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
                        # Compute offset from marker.
                        final_x, final_y = self._target_xy_from_tf(
                            tf, forward_offset
                        )

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

        # Switch back to position mode.
        self._node.switch_to_position_mode()

    def drive_to_point(self, name: str, forward_offset: float):
        # Enter travel pose.
        self.enter_travel_pose()

        # Look down slightly (markers are lower than head).
        self._node.move_to_pose({"joint_head_tilt": -0.6}, blocking=True)

        # Verify marker can be found.
        if self._node.get_tf(ROBOT_FRAME, name) is None:
            raise ValueError("Marker not found. Unable to drive to point.")

        # Switch to navigation mode.
        self._node.switch_to_navigation_mode()

        # Assume we are far away from target.
        self._point_distance_to_target = inf

        # Define drive loop.
        def drive():
            # Exit if drive loop is not running or publisher is not ready.
            if self._point_drive_loop is None or self._node.vel_publisher is None:
                return

            # Define velocity command.
            command = Twist()

            # Pull last known distance to target.
            with self._point_lock:
                point_distance_to_target = self._point_distance_to_target

            # Switch drive based on recovery.
            if self._in_drive_recovery:
                command.linear.x = -MAX_FORWARD_SPEED
            else:
                # Clamp rate to drive speed.
                rate = min(MAX_FORWARD_SPEED, point_distance_to_target)

                # Round to 0 when within threshold.
                if point_distance_to_target < MINIMUM_FORWARD_DISTANCE_THRESHOLD:
                    self._point_stop_event.set()
                    self._point_drive_loop.destroy()
                    self._node.stop_the_robot()
                    self._node.get_logger().info("Reached target point.")
                    return

                command.linear.x = rate

            # Send command.
            self._node.get_logger().info(
                f"Setting rate: {command.linear.x} m/sec. Last known distance to target: {point_distance_to_target}")
            self._node.vel_publisher.publish(command)

        # Define search worker. get_tf can block, so this runs outside ROS timer callbacks.
        def search():
            while not self._point_stop_event.is_set():
                # Try to find marker.
                tf = self._node.get_tf(ROBOT_FRAME, name)

                # Switch to recovery if marker is lost during forward drive.
                if tf is None and not self._in_drive_recovery:
                    self._node.get_logger().error("Marker has been lost.")

                    # Enable recovery mode.
                    self._in_drive_recovery = True
                # Stop if marker is found during recovery.
                elif tf is not None and self._in_drive_recovery:
                    self._node.get_logger().info("Recovered marker!")
                    self._point_stop_event.set()
                    if self._point_drive_loop is not None:
                        self._point_drive_loop.destroy()
                    self._node.stop_the_robot()

                # Update distance to target.
                if tf is not None:
                    target_x, target_y = self._target_xy_from_tf(tf, forward_offset)
                    with self._point_lock:
                        self._point_distance_to_target = max(0.0, target_x)

        # Reset search state for a fresh run.
        self._point_stop_event.clear()

        # Do drive.
        self._point_drive_loop = self._node.create_timer(MARKER_SEARCH_PERIOD, drive)

        # Do search (non-blocking for executor/timers).
        search_thread = Thread(target=search, daemon=True)
        search_thread.start()

        # Wait for search to complete.
        search_thread.join()

        # Switch back to pos mode.
        self._node.switch_to_position_mode()

        # If finished in recovery mode, complete rest of drive using absolute positioning.
        if self._in_drive_recovery:
            self._node.get_logger().info("Completing recovery drive.")
            self._node.move_to_pose({"translate_mobile_base": self._point_distance_to_target}, blocking=True)
            self._in_drive_recovery = False

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
