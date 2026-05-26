from __future__ import annotations

from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode

from .constants import (
    FEEDER_FRAME,
    COLUMN_MAP,
)

from .navigation_manager import NavigationManager
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.publisher import Publisher

class Main(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)

        # ROS components.
        self.vel_publisher: Publisher | None = None

        # Application components.
        self.navigation_manager = NavigationManager(self)

    def _move_to_column(self, column_number: int):
        """
        Navigate robot to the specified Connect Four column.
        """

        if column_number not in COLUMN_MAP:
            self.get_logger().error(
                f"Invalid column: {column_number}"
            )
            return

        column_frame = COLUMN_MAP[column_number]

        self.get_logger().info(
            f"Moving to column {column_number} ({column_frame})"
        )

        # Ensure head is facing forward before navigation.
        self.move_to_pose(
            {
                "joint_head_pan": 0.0
            },
            blocking=True,
        )

        # Step 1:
        # Rotate robot so it faces marker.
        self.navigation_manager.point_at_marker(
            column_frame,
            clockwise=True,
            forward_offset=0.75,
        )

        # Step 2:
        # Drive robot toward marker.
        self.navigation_manager.drive_to_point(
            column_frame,
            forward_offset=0.75,
        )

        # Step 3:
        # Re-align robot so arm faces marker directly.
        self.navigation_manager.point_at_marker(
            column_frame,
            clockwise=True,
            forward_offset=0.0,
        )

        self.get_logger().info(
            f"Arrived at column {column_number}"
        )

    def main(self, **kwargs):
        HelloNode.main(
            self,
            "main",
            "main",
            wait_for_first_pointcloud=False,
        )

        # Ensure in position mode.
        self.switch_to_position_mode()

        # Initialize ROS components.
        self.vel_publisher = self.create_publisher(
            Twist,
            "/stretch/cmd_vel",
            10,
        )

        # EXAMPLE:
        # Move to column 4
        self._move_to_column(4)

        self.get_logger().info("Motion complete.")


def main():
    assistfour = Main()

    try:
        assistfour.main()
        assistfour.new_thread.join()

    except KeyboardInterrupt:
        assistfour.destroy_node()


if __name__ == "__main__":
    main()
