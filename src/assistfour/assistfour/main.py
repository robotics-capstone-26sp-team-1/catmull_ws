from __future__ import annotations

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# noinspection PyUnresolvedReferences
from assistfour_interfaces.action import GetToken, GotoColumn

from .constants import FEEDER_FRAME
from .navigation_manager import NavigationManager

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


class Main(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)

        # ROS components.
        self.vel_publisher: Publisher | None = None
        self._callback_group: ReentrantCallbackGroup | None = None
        self.get_token_action: ActionServer | None = None
        self.goto_column_action: ActionServer | None = None

        # Application components.
        self.navigation_manager = NavigationManager(self)

    def main(self, **kwargs):
        HelloNode.main(self, "main", "main", wait_for_first_pointcloud=False)

        # Ensure in position mode.
        self.switch_to_position_mode()

        # Initialize ROS components.
        self.vel_publisher = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        self._callback_group = ReentrantCallbackGroup()
        self.get_token_action = ActionServer(
            self,
            GetToken,
            "get_token",
            execute_callback=lambda: None,
            goal_callback=lambda: None,
            cancel_callback=lambda: None,
            callback_group=self._callback_group,
        )
        self.goto_column_action = ActionServer(
            self,
            GotoColumn,
            "goto_column",
            execute_callback=lambda: None,
            goal_callback=lambda: None,
            cancel_callback=lambda: None,
            callback_group=self._callback_group,
        )

        # Demo: move to column 4.
        # self.navigation_manager.move_to_column(4)

        self.get_logger().info("Motion complete.")


def main():
    assistfour = Main()

    try:
        assistfour.main()
        executor = MultiThreadedExecutor()
        executor.add_node(assistfour)
        executor.spin()
    except KeyboardInterrupt:
        assistfour.destroy_node()


if __name__ == "__main__":
    main()
