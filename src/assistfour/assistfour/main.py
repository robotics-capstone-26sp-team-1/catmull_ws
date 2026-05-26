from __future__ import annotations

from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode

from .constants import (
    FEEDER_FRAME,
    COLUMN_1_FRAME,
    COLUMN_2_FRAME,
    COLUMN_3_FRAME,
    COLUMN_4_FRAME,
    COLUMN_5_FRAME,
    COLUMN_6_FRAME,
    COLUMN_7_FRAME,
)

from .navigation_manager import NavigationManager
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


COLUMN_MAP = {
    1: COLUMN_1_FRAME,
    2: COLUMN_2_FRAME,
    3: COLUMN_3_FRAME,
    4: COLUMN_4_FRAME,
    5: COLUMN_5_FRAME,
    6: COLUMN_6_FRAME,
    7: COLUMN_7_FRAME,
}


class Main(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)

        # ROS components.
        self.vel_publisher: Publisher | None = None

        # Application components.
        self.navigation_manager = NavigationManager(self)

    def move_to_column(self, column_number: int):
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

        #
        # Step 1:
        # Rotate robot so it faces marker.
        #
        self.navigation_manager.point_at_marker(
            column_frame,
            clockwise=True,
            forward_offset=0.75,
        )

        #
        # Step 2:
        # Drive robot toward marker.
        #
        self.navigation_manager.drive_to_point(
            column_frame,
            forward_offset=0.75,
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

        #
        # EXAMPLE:
        # Move to column 4
        #
        self.move_to_column(4)

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
