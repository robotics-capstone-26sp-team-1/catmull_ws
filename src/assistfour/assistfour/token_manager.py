from __future__ import annotations

from time import sleep
from typing import TYPE_CHECKING

from .constants import COLUMN_MAP

if TYPE_CHECKING:
    from .main import Main


END_EFFECTOR_FRAME = "link_grasp_center"

#
# Tunable placement constants.
#
DESIRED_MARKER_X = 0.0
DESIRED_MARKER_Y = 0.18
DESIRED_MARKER_Z = 0.10

HOME_LIFT = 0.45
HOME_ARM = 0.0


class TokenManager:
    def __init__(self, node: Main):
        self._node = node

    def place_token(self, column_number: int):
        """
        Align gripper above funnel and release token.
        """

        if column_number not in COLUMN_MAP:
            self._node.get_logger().error(
                f"Invalid column number: {column_number}"
            )
            return

        column_frame = COLUMN_MAP[column_number]

        # Get marker → end effector transform.
        tf = self._node.get_tf(
            column_frame,
            END_EFFECTOR_FRAME,
        )

        if tf is None:
            self._node.get_logger().error(
                "Unable to get transform to end effector."
            )
            return

        # Current end effector position relative to marker.
        current_x = tf.transform.translation.x
        current_y = tf.transform.translation.y
        current_z = tf.transform.translation.z


        # Compute correction vector.
        delta_x = DESIRED_MARKER_X - current_x
        delta_y = DESIRED_MARKER_Y - current_y
        delta_z = DESIRED_MARKER_Z - current_z

        self._node.get_logger().info(
            f"Placement correction: "
            f"x={delta_x:.3f}, "
            f"y={delta_y:.3f}, "
            f"z={delta_z:.3f}"
        )

        # Marker frame conventions:
        # x -> base motion
        # y -> vertical lift
        # z -> arm extension
        self._node.move_to_pose(
            {
                "translate_mobile_base": delta_x,
                "joint_lift": HOME_LIFT + delta_y,
                "joint_arm": HOME_ARM + delta_z,
            },
            blocking=True,
        )

        #
        # Release token.
        #
        self._node.move_to_pose(
            {
                "stretch_gripper": 100,
            },
            blocking=True,
        )

        # Allow token time to fall.
        sleep(1.0)


        # Retract arm safely.
        self._node.move_to_pose(
            {
                "joint_arm": HOME_ARM,
            },
            blocking=True,
        )

        self._node.get_logger().info(
            "Token placement complete."
        )
