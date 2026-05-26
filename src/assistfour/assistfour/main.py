from __future__ import annotations

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode

from .constants import FEEDER_FRAME
from .navigation_manager import NavigationManager

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


class Main(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)

        # ROS components.
        self.vel_publisher: Publisher | None = None

        # Application components.
        self.navigation_manager = NavigationManager(self)

    def main(self, **kwargs):
        HelloNode.main(self, "main", "main", wait_for_first_pointcloud=False)

        # Ensure in position mode.
        self.switch_to_position_mode()

        # Initialize ROS components.
        self.vel_publisher = self.create_publisher(Twist, "/stretch/cmd_vel", 10)

        # Demo finding feeder and orienting to it.
        self.navigation_manager.point_at_marker(FEEDER_FRAME, True, 0.75, 0)

        self.navigation_manager.drive_to_marker(FEEDER_FRAME, 0.75)

        # Final point to.
        self.navigation_manager.point_at_marker(FEEDER_FRAME, False, 0, -90)

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
