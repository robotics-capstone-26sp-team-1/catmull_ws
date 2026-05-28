from __future__ import annotations

from time import sleep
from typing import TYPE_CHECKING

from geometry_msgs.msg import TransformStamped

from rclpy.action import ActionClient

from stretch_pose_interfaces.action import SetPose

from .constants import COLUMN_MAP

if TYPE_CHECKING:
    from .main import Main


class TokenManager:
    def __init__(self, node: Main):
        self._node = node

        # Action client for PoseServer
        self._set_pose_client = ActionClient(
            self._node,
            SetPose,
            "set_pose",
        )

    def place_token(self, column_number: int):
        """
        Move gripper above funnel and release token.
        """

        if column_number not in COLUMN_MAP:
            self._node.get_logger().error(
                f"Invalid column number: {column_number}"
            )
            return

        column_frame = COLUMN_MAP[column_number]

        self._node.get_logger().info(
            f"Placing token into column {column_number}"
        )

        # Wait for PoseServer
        self._set_pose_client.wait_for_server()

        # Create target pose relative to marker
        target = TransformStamped()

        target.header.frame_id = column_frame

        # Approximate release point above funnel 
        target.transform.translation.x = 0.0
        target.transform.translation.y = 0.0
        target.transform.translation.z = 0.22

        # Build action goal
        goal = SetPose.Goal()
        goal.target_pose = target

        # Send goal to PoseServer.
        self._node.get_logger().info(
            "Moving gripper above funnel."
        )

        future = self._set_pose_client.send_goal_async(goal)

        # Wait for goal handle
        self._node.executor.spin_until_future_complete(future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self._node.get_logger().error(
                "Pose goal rejected."
            )
            return

        # Wait for motion completion.
        result_future = goal_handle.get_result_async()

        self._node.executor.spin_until_future_complete(
            result_future
        )

        result = result_future.result().result

        if not result.success:
            self._node.get_logger().error(
                f"Pose failed: {result.message}"
            )
            return

        # Open gripper to release token.
        self._node.get_logger().info(
            "Releasing token."
        )

        self._node.move_to_pose(
            {
                "stretch_gripper": 100,
            },
            blocking=True,
        )

        # Allow token time to fall
        sleep(1.0)


        # Retract arm safely
        self._node.move_to_pose(
            {
                "joint_arm": 0.0,
                "joint_lift": 0.45,
            },
            blocking=True,
        )

        self._node.get_logger().info(
            "Token placement complete."
        )
