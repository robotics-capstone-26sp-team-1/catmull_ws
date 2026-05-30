from __future__ import annotations

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# noinspection PyUnresolvedReferences
from assistfour_interfaces.action import GotoColumn, GotoMarker, ReturnToStart

from .navigation_manager import NavigationManager

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


class Main(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)

        # ROS components.
        self.vel_publisher: Publisher | None = None
        self._callback_group: ReentrantCallbackGroup | None = None
        self.goto_marker_action: ActionServer | None = None
        self.return_to_start_action: ActionServer | None = None
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
        self.goto_marker_action = ActionServer(
            self,
            GotoMarker,
            "goto_marker",
            execute_callback=self._goto_marker_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self.return_to_start_action = ActionServer(
            self,
            ReturnToStart,
            "return_to_start",
            execute_callback=self._return_to_start_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self.goto_column_action = ActionServer(
            self,
            GotoColumn,
            "goto_column",
            execute_callback=self._goto_column_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )

        # Demo: move to column 4.
        # self.navigation_manager.move_to_column(4)
        # self.navigation_manager.move_to_feeder()

        self.get_logger().info("Motion complete.")

    @staticmethod
    def _goto_marker_execute(goal_handle):
        result = GotoMarker.Result()
        result.result = "Not implemented."
        goal_handle.succeed()
        return result

    @staticmethod
    def _return_to_start_execute(goal_handle):
        result = ReturnToStart.Result()
        result.result = "Not implemented."
        goal_handle.succeed()
        return result

    @staticmethod
    def _goto_column_execute(goal_handle):
        result = GotoColumn.Result()
        result.result = "Not implemented."
        goal_handle.succeed()
        return result


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
