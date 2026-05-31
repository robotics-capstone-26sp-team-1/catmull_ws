from __future__ import annotations

from math import radians
from typing import TYPE_CHECKING
from time import sleep
from .navigation_manager import NavigationManager

from .constants import (
    FEEDER_FRAME,
    END_FRAME,
    WRIST_UP,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    COLUMN_MAP,
    FEEDER_ARM_OFFSET,
    FEEDER_LIFT_OFFSET,
)

if TYPE_CHECKING:
    from .main import Main


class TokenManager:
    def __init__(self, node: Main, navigation_manager: NavigationManager) -> None:
        self._node = node
        self._navigation_manager = navigation_manager

    def grab_token(self):
        # Prepare gripper.
        self._node.checked_pose_move(
            {"joint_wrist_pitch": WRIST_UP, "gripper_aperture": GRIPPER_OPEN}
        )

        # Force the TF to refresh.
        sleep(1)

        # Get end to feeder.
        end_to_feeder = self._navigation_manager.block_until_recent_tf(
            END_FRAME, FEEDER_FRAME
        )

        # Get current state.
        joint_state = self._node.joint_state
        lift_index = joint_state.name.index("joint_lift")
        lift_height = joint_state.position[lift_index]
        arm_index = joint_state.name.index("wrist_extension")
        arm_extent = joint_state.position[arm_index]

        # Compute target positions
        target_lift_height = (
            lift_height + end_to_feeder.transform.translation.z - FEEDER_LIFT_OFFSET
        )
        target_arm_extent = (
            arm_extent + end_to_feeder.transform.translation.x - FEEDER_ARM_OFFSET
        )

        # Lift arm.
        self._node.checked_pose_move(
            {
                "joint_lift": target_lift_height,
            }
        )

        # Extend arm.
        self._node.checked_pose_move({"joint_arm": target_arm_extent})

        # Close the gripper.
        self._node.checked_pose_move({"gripper_aperture": GRIPPER_CLOSE})

        # Raise to clear the holder.
        self._node.checked_pose_move(
            {"joint_lift": lift_height + end_to_feeder.transform.translation.z}
        )

        # Retract arm.
        self._node.checked_pose_move({"joint_arm": 0.0})

        # Return to travel pose.
        self._navigation_manager.enter_travel_pose()
        self._navigation_manager.return_to_start()

    def place_token(self, column: int):
        # Ensure gripper is closed.
        self._node.checked_pose_move({"gripper_aperture": GRIPPER_CLOSE})

        # Force the TF to refresh.
        sleep(1)

        # Get end to feeder.
        end_to_feeder = self._navigation_manager.block_until_recent_tf(
            END_FRAME, COLUMN_MAP[column]
        )

        # Get current state.
        joint_state = self._node.joint_state
        lift_index = joint_state.name.index("joint_lift")
        lift_height = joint_state.position[lift_index]
        arm_index = joint_state.name.index("wrist_extension")
        arm_extent = joint_state.position[arm_index]

        # Compute target positions
        target_lift_height = lift_height - end_to_feeder.transform.translation.x + 0.35
        target_arm_extent = arm_extent + end_to_feeder.transform.translation.z + 0.05

        # Lift arm.
        self._node.checked_pose_move(
            {
                "joint_lift": target_lift_height,
            }
        )

        # Rotate arm
        self._node.checked_pose_move({"joint_wrist_roll": radians(90)})

        # Extend arm.
        self._node.checked_pose_move({"joint_arm": target_arm_extent})

        # Close the gripper.
        self._node.checked_pose_move({"gripper_aperture": GRIPPER_OPEN})

        sleep(1)

        # Retract arm.
        self._node.checked_pose_move({"joint_arm": 0.0})

        # Return to travel pose.
        self._navigation_manager.enter_travel_pose()
