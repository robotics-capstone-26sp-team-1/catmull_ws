from __future__ import annotations

from sympy import false
from threading import Thread, Event
from math import atan2, radians, sqrt
from typing import TYPE_CHECKING
from time import monotonic, sleep
from geometry_msgs.msg import Twist, TransformStamped
from tf_transformations import quaternion_matrix
from numpy import array, matmul
from rclpy.time import Time

from .constants import (
    ROBOT_FRAME,
    SEARCH_SPIN_RATE,
    MARKER_SEARCH_PERIOD,
    MINIMUM_ANGLE_THRESHOLD,
    MAX_TF_AGE,
    RECENT_TF_POLL_TIME,
    OFFSET_FROM_MARKER,
    WRIST_DOWN,
    LIFT_MID_HEIGHT,
    HEAD_SEARCH_TILT,
    RECENT_TF_TIMEOUT,
    COLUMN_MAP,
    WORLD_FRAME,
    FEEDER_FRAME,
)

if TYPE_CHECKING:
    from rclpy.timer import Timer
    from .main import Main


class NavigationManager:
    def __init__(self, node: Main):
        self._node = node

        # Marker searching.
        self._search_spin_loop: Timer | None = None
        self._search_spin_stop_event = Event()

    def move_to_column(self, column_number: int):
        frame_name = COLUMN_MAP[column_number]
        self.move_to_marker(frame_name)

    def move_to_feeder(self):
        self.move_to_marker(FEEDER_FRAME)

    def move_to_marker(self, name: str):
        """High level operation to drive from anywhere to a marker and align arm to face it."""
        self.point_at_marker(name, True, OFFSET_FROM_MARKER, 0)
        self.drive_to_marker(name, OFFSET_FROM_MARKER)
        self.point_at_marker(name, False, 0, -90)

    def point_at_marker(
            self, name: str, clockwise: bool, forward_offset: float, pan_offset_deg: float
    ):
        # Enter travel pose.
        self.enter_travel_pose()

        # Convert pan offset to radian.
        pan_offset_rad = radians(pan_offset_deg)

        # Look down slightly (markers are lower than head).
        self._node.checked_pose_move(
            {"joint_head_tilt": HEAD_SEARCH_TILT, "joint_head_pan": pan_offset_rad}
        )

        # Switch to navigation mode.
        self._node.switch_to_navigation_mode()

        # Define spin loop.
        def spin():
            # Exit if search spin is stopping or publisher is not ready.
            if (
                    self._search_spin_stop_event.is_set()
                    or self._node.vel_publisher is None
            ):
                return

            # Continue spinning.
            command = Twist()
            command.angular.z = -SEARCH_SPIN_RATE if clockwise else SEARCH_SPIN_RATE
            self._node.vel_publisher.publish(command)
            self._node.check_canceled()

        # Define search worker. get_tf can block, so this runs outside ROS timer callbacks.
        def search():
            while not self._search_spin_stop_event.is_set():
                # Try to find marker.
                tf = self._get_recent_tf(ROBOT_FRAME, name)

                if tf is not None:
                    break

                # Wait 1/4 second before trying again.
                sleep(0.25)
                self._node.check_canceled()

        # Reset search state.
        self._search_spin_stop_event.clear()

        # Do spin.
        self._search_spin_loop = self._node.create_timer(MARKER_SEARCH_PERIOD, spin)

        # Do search (non-blocking for executor/timers).
        search_thread = Thread(target=search, daemon=True)
        search_thread.start()

        # Wait for search to complete.
        search_thread.join()

        # Stop search spin cleanly before proceeding.
        self._search_spin_stop_event.set()
        if self._search_spin_loop is not None:
            self._search_spin_loop.destroy()
            self._search_spin_loop = None
        self._node.stop_the_robot()
        self._node.get_logger().info("Stopping search spin.")

        # Switch back to position mode.
        self._node.switch_to_position_mode()

        def compute_angle_to_marker() -> float:
            tf = self.block_until_recent_tf(ROBOT_FRAME, name)

            # Compute angle based on location.
            target_point_x, target_point_y = self._target_xy_from_tf(tf, forward_offset)
            angle = atan2(target_point_y, target_point_x)

            # Adjust for head pan offset.
            angle -= pan_offset_rad

            self._node.get_logger().info(f"Angle to marker: {angle}")
            return angle

        # Iterative refinement.
        angle_to_marker = compute_angle_to_marker()
        while abs(angle_to_marker) > MINIMUM_ANGLE_THRESHOLD:
            # Rotate.
            self._node.get_logger().info(f"Correcting by {angle_to_marker}...")
            self._node.checked_pose_move({"rotate_mobile_base": angle_to_marker})

            # Compute current angle.
            angle_to_marker = compute_angle_to_marker()

        self._node.get_logger().info("Pointing at marker!")

    def drive_to_marker(self, name: str, forward_offset: float):
        # Enter travel pose.
        self.enter_travel_pose()

        # Look down slightly and align to drive direction.
        self._node.checked_pose_move(
            {"joint_head_tilt": HEAD_SEARCH_TILT, "joint_head_pan": 0.0}
        )

        def compute_distance_to_marker() -> float:
            tf = self.block_until_recent_tf(ROBOT_FRAME, name)

            target_point_x, _ = self._target_xy_from_tf(tf, forward_offset)
            self._node.get_logger().info(f"Distance to marker: {target_point_x}")
            return target_point_x

        # Approach.
        distance_to_marker = compute_distance_to_marker()
        self._node.checked_pose_move({"translate_mobile_base": distance_to_marker})
        self._node.get_logger().info("At marker!")

    def enter_travel_pose(self):
        """Move the arm into a safe pose for traveling."""
        self._node.checked_pose_move(
            {
                "joint_wrist_pitch": WRIST_DOWN,
                "joint_wrist_roll": 0.0,
                "joint_wrist_yaw": 0.0,
                "joint_lift": LIFT_MID_HEIGHT,
                "joint_arm": 0.0,
            }
        )

    def return_to_start(self):
        self.enter_travel_pose()

        def compute_angle_to_odom() -> float:
            tf = self.block_until_recent_tf(ROBOT_FRAME, WORLD_FRAME)
            target_point_x = tf.transform.translation.x
            target_point_y = tf.transform.translation.y
            return atan2(target_point_y, target_point_x)

        angle_to_odom = compute_angle_to_odom()
        while abs(angle_to_odom) > MINIMUM_ANGLE_THRESHOLD:
            self._node.get_logger().info(f"Correcting by {angle_to_odom}...")
            self._node.checked_pose_move({"rotate_mobile_base": angle_to_odom})

            # Update angle.
            angle_to_odom = compute_angle_to_odom()

        distance = sqrt(target_point_x * target_point_x + target_point_y * target_point_y)
        self._node.checked_pose_move({"translate_mobile_base": distance})

    def block_until_recent_tf(self, source: str, target: str) -> TransformStamped:
        """Block until a recent TF is found."""
        start_time = monotonic()
        tf = self._get_recent_tf(source, target)
        while tf is None:
            if monotonic() - start_time >= RECENT_TF_TIMEOUT:
                raise ValueError(
                    f"Recent TF for {target} was not available within {RECENT_TF_TIMEOUT} seconds."
                )
            self._node.get_logger().info("Waiting for more recent TF...")
            sleep(RECENT_TF_POLL_TIME)
            tf = self._get_recent_tf(source, target)

        return tf

    def _get_recent_tf(self, source: str, target: str) -> TransformStamped | None:
        """Returns TF if within MAX_TF_AGE seconds old."""
        tf = self._node.get_tf(source, target)
        if tf is not None:
            current_time = self._node.get_clock().now()
            tf_age = (current_time - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            # self._node.get_logger().info(f"{target} TF age: {tf_age:.3f} seconds.")
            if tf_age <= MAX_TF_AGE:
                return tf

        return None

    @staticmethod
    def _target_xy_from_tf(
            tf: TransformStamped, forward_offset: float
    ) -> tuple[float, float]:
        """Compute the target point in the robot frame after applying the marker offset."""
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
