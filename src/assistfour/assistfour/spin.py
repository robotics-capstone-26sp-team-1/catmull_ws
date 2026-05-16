from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode


class Spin(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        self.vel_publisher = None
        self.timer = None

    def main(self, **kwargs):
        HelloNode.main(self, "spin", "spin", wait_for_first_pointcloud=False)
        self.vel_publisher = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self._spin)
        self.get_logger().info("Starting to move in circle...")

    def _spin(self):
        command = Twist()
        command.angular.z = 0.5
        self.get_logger().info("Published 0.5")
        self.vel_publisher.publish(command)


def main():
    node = Spin()
    try:
        node.main()
        node.new_thread.join()
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == '__main__':
    main()
