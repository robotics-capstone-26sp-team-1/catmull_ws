from __future__ import annotations

from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode

from .constants import COLUMN_4_FRAME
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

    def main(self, **kwargs):
        HelloNode.main(self, "main", "main", wait_for_first_pointcloud=False)

        # Initialize ROS components.
        self.vel_publisher = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        self.navigation_manager.search_for_marker(COLUMN_4_FRAME, False)


def main():
    assistfour = Main()

    try:
        assistfour.main()
        assistfour.new_thread.join()
    except KeyboardInterrupt:
        assistfour.destroy_node()


if __name__ == '__main__':
    main()
