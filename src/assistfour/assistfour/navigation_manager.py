from __future__ import annotations

from threading import Event, Lock, Thread
from math import atan2, pi, inf
from typing import TYPE_CHECKING
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_matrix
from numpy import array, matmul
import time

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

        #
        # Variables used while rotating to face marker.
        #
        self._search_spin_loop: Timer | None = None
        self._search_stop_event = Event()
        self._angle_lock = Lock()
        self._angle_to_marker = pi

        #
        # Variables used while driving toward marker.
        #
        self._point_drive_loop: Timer | None = None
        self._point_stop_event = Event()
        self._point_lock = Lock()
        self._point_distance_to_target = inf

    def point_at_marker(
        self,
        name: str,
        clockwise: bool,
        forward_offset: float,
    ):
        #
        # Move robot into compact/safe driving pose.
        #
        self.enter_travel_pose()

        #
        # Tilt head downward slightly because
        # the ArUco markers are lower than the camera.
        #
        self._node.move_to_pose(
            {
                "joint_head_tilt": -0.3
            },
            blocking=True,
        )

        #
        # Switch Stretch into navigation mode
        # so cmd_vel commands control the base.
        #
        self._node.switch_to_navigation_mode()

        #
        # Reset angle assumption.
        #
        self._angle_to_marker = pi

        #
        # IMPORTANT:
        # Destroy any previous timer if it still exists.
        #
        # Why:
        # ROS timers can sometimes remain alive
        # after interrupted runs.
        #
        if self._search_spin_loop is not None:
            self._search_spin_loop.destroy()

        #
        # Rotation control loop.
        #
        def spin():

            #
            # Exit if timer or publisher vanished.
            #
            if (
                self._search_spin_loop is None
                or self._node.vel_publisher is None
            ):
                return

            command = Twist()

            #
            # Read latest angle safely.
            #
            with self._angle_lock:
                angle_to_marker = self._angle_to_marker

            #
            # Clamp angular velocity.
            #
            if abs(angle_to_marker) > SEARCH_SPIN_RATE:
                rate = (
                    -SEARCH_SPIN_RATE
                    if clockwise
                    else SEARCH_SPIN_RATE
                )
            else:
                rate = angle_to_marker

            #
            # Stop rotating once aligned.
            #
            if abs(angle_to_marker) < MINIMUM_ANGLE_THRESHOLD:

                self._search_stop_event.set()

                self._search_spin_loop.destroy()

                self._node.stop_the_robot()

                self._node.get_logger().info(
                    "Aligned to marker."
                )

                return

            #
            # Publish angular velocity.
            #
            command.angular.z = rate

            self._node.get_logger().info(
                f"Rotating at {rate:.3f} rad/sec"
            )

            self._node.vel_publisher.publish(command)

        #
        # TF lookup thread.
        #
        # Why separate thread?
        #
        # get_tf() can block, and blocking inside
        # ROS timer callbacks is dangerous.
        #
        def search():

            while not self._search_stop_event.is_set():

                #
                # Lookup marker transform.
                #
                tf = self._node.get_tf(
                    ROBOT_FRAME,
                    name,
                )

                with self._angle_lock:

                    if tf is not None:

                        #
                        # Helpful debugging output.
                        #
                        self._node.get_logger().info(
                            f"Marker detected: "
                            f"x={tf.transform.translation.x:.3f}, "
                            f"y={tf.transform.translation.y:.3f}"
                        )

                        #
                        # If no offset requested,
                        # directly face marker center.
                        #
                        if forward_offset == 0.0:

                            self._angle_to_marker = atan2(
                                tf.transform.translation.y,
                                tf.transform.translation.x,
                            )

                        else:

                            #
                            # Compute adjusted target point.
                            #
                            final_x, final_y = (
                                self._target_xy_from_tf(
                                    tf,
                                    forward_offset,
                                )
                            )

                            #
                            # Compute angle robot must rotate.
                            #
                            self._angle_to_marker = atan2(
                                final_y,
                                final_x,
                            )

                    else:

                        #
                        # If marker disappears,
                        # continue spinning/searching.
                        #
                        self._angle_to_marker = pi

        #
        # Reset event state.
        #
        self._search_stop_event.clear()

        #
        # Create spin timer.
        #
        self._search_spin_loop = self._node.create_timer(
            MARKER_SEARCH_PERIOD,
            spin,
        )

        #
        # Start TF thread.
        #
        search_thread = Thread(
            target=search,
            daemon=True,
        )

        search_thread.start()

        #
        # IMPORTANT:
        # Timeout protection.
        #
        # Why:
        # Prevent robot from spinning forever
        # if marker never appears.
        #
        search_thread.join(timeout=30.0)

        if search_thread.is_alive():

            self._node.get_logger().error(
                "Timed out while searching for marker."
            )

            self._search_stop_event.set()

            self._node.stop_the_robot()

        #
        # Switch back to position mode.
        #
        self._node.switch_to_position_mode()

    def drive_to_point(
        self,
        name: str,
        forward_offset: float,
    ):

        #
        # Enter safe driving pose.
        #
        self.enter_travel_pose()

        #
        # Tilt head further downward.
        #
        self._node.move_to_pose(
            {
                "joint_head_tilt": -0.6
            },
            blocking=True,
        )

        #
        # Switch into navigation mode.
        #
        self._node.switch_to_navigation_mode()

        #
        # Assume target is far away initially.
        #
        self._point_distance_to_target = inf

        #
        # Destroy old timer if necessary.
        #
        if self._point_drive_loop is not None:
            self._point_drive_loop.destroy()

        #
        # Forward driving loop.
        #
        def drive():

            if (
                self._point_drive_loop is None
                or self._node.vel_publisher is None
            ):
                return

            command = Twist()

            with self._point_lock:
                point_distance_to_target = (
                    self._point_distance_to_target
                )

            #
            # Clamp speed.
            #
            rate = min(
                MAX_FORWARD_SPEED,
                point_distance_to_target,
            )

            #
            # Stop once close enough.
            #
            if (
                point_distance_to_target
                < MINIMUM_FORWARD_DISTANCE_THRESHOLD
            ):

                self._point_stop_event.set()

                self._point_drive_loop.destroy()

                self._node.stop_the_robot()

                self._node.get_logger().info(
                    "Reached target point."
                )

                return

            #
            # Publish forward velocity.
            #
            command.linear.x = rate

            self._node.get_logger().info(
                f"Driving forward at {rate:.3f} m/sec"
            )

            self._node.vel_publisher.publish(command)

        #
        # TF monitoring thread.
        #
        def search():

            while not self._point_stop_event.is_set():

                tf = self._node.get_tf(
                    ROBOT_FRAME,
                    name,
                )

                #
                # Stop immediately if marker lost.
                #
                if tf is None:

                    self._node.get_logger().error(
                        "Marker lost during driving."
                    )

                    self._point_stop_event.set()

                    if self._point_drive_loop is not None:
                        self._point_drive_loop.destroy()

                    self._node.stop_the_robot()

                    return

                #
                # Compute adjusted target point.
                #
                target_x, target_y = (
                    self._target_xy_from_tf(
                        tf,
                        forward_offset,
                    )
                )

                #
                # Helpful debugging logs.
                #
                self._node.get_logger().info(
                    f"Target point: "
                    f"x={target_x:.3f}, "
                    f"y={target_y:.3f}"
                )

                #
                # Distance robot still needs to travel.
                #
                with self._point_lock:
                    self._point_distance_to_target = max(
                        0.0,
                        target_x,
                    )

        #
        # Reset state.
        #
        self._point_stop_event.clear()

        #
        # Create drive timer.
        #
        self._point_drive_loop = self._node.create_timer(
            MARKER_SEARCH_PERIOD,
            drive,
        )

        #
        # Start TF monitoring thread.
        #
        search_thread = Thread(
            target=search,
            daemon=True,
        )

        search_thread.start()

        #
        # Timeout protection.
        #
        search_thread.join(timeout=30.0)

        if search_thread.is_alive():

            self._node.get_logger().error(
                "Timed out while driving to marker."
            )

            self._point_stop_event.set()

            self._node.stop_the_robot()

        #
        # Return to position mode.
        #
        self._node.switch_to_position_mode()

    def enter_travel_pose(self):

        #
        # Compact safe pose for navigation.
        #
        self._node.move_to_pose(
            {
                "joint_head_tilt": 0.0,
                "joint_wrist_pitch": -1.6,
                "joint_wrist_roll": 0.0,
                "joint_wrist_yaw": 0.0,
                "joint_head_pan": 0.0,
                "joint_lift": 0.45,
                "joint_arm": 0.0,
            },
            blocking=True,
        )

    @staticmethod
    def _target_xy_from_tf(
        tf,
        forward_offset: float,
    ) -> tuple[float, float]:

        #
        # If no offset requested,
        # directly use marker center.
        #
        if forward_offset == 0.0:

            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )

        #
        # Build marker rotation matrix.
        #
        rotation_matrix = quaternion_matrix(
            (
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            )
        )

        #
        # IMPORTANT FIX:
        #
        # Offset should be along marker FORWARD direction (x-axis),
        # not z-axis.
        #
        # Old code incorrectly offset vertically.
        #
        offset_vector = array(
            [
                [forward_offset],
                [0],
                [0],
                [1],
            ]
        )

        #
        # Marker location in robot frame.
        #
        marker_vector = array(
            [
                [tf.transform.translation.x],
                [tf.transform.translation.y],
                [0],
                [1],
            ]
        )

        #
        # Rotate offset into marker frame.
        #
        offset_direction = matmul(
            rotation_matrix,
            offset_vector,
        )

        #
        # Final target point robot should drive toward.
        #
        final_location = offset_direction + marker_vector

        return (
            float(final_location[0, 0]),
            float(final_location[1, 0]),
        )
