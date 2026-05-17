from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode

from .navigation_manager import NavigationManager


class Main(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        self.vel_publisher = None
        self.navigation_manager = NavigationManager(self)

    def main(self, **kwargs):
        HelloNode.main(self, "main", "main", wait_for_first_pointcloud=False)

        # Initialize components.
        self.vel_publisher = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        self.navigation_manager.search_for_marker("hello")


def main():
    assistfour = Main()

    try:
        assistfour.main()
        assistfour.new_thread.join()
    except KeyboardInterrupt:
        assistfour.destroy_node()


if __name__ == '__main__':
    main()
