from __future__ import annotations

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Event

# noinspection PyUnresolvedReferences
from assistfour_interfaces.action import GotoMarker, PlayColumn

from .navigation_manager import NavigationManager
from .token_manager import TokenManager

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


class Main(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)

        # ROS components.
        self.vel_publisher: Publisher | None = None
        self._callback_group: ReentrantCallbackGroup | None = None
        self.get_token_action: ActionServer | None = None
        self.play_column_action: ActionServer | None = None
        self.return_to_start_action: ActionServer | None = None

        # Application components.
        self.navigation_manager = NavigationManager(self)
        self.token_manager = TokenManager(self, self.navigation_manager)
        self.cancel_event = Event()

    def main(self, **kwargs):
        HelloNode.main(self, "main", "main", wait_for_first_pointcloud=False)

        # Ensure in position mode.
        self.switch_to_position_mode()

        # Initialize ROS components.
        self.vel_publisher = self.create_publisher(Twist, "/stretch/cmd_vel", 10)
        self._callback_group = ReentrantCallbackGroup()
        self.get_token_action = ActionServer(
            self,
            GotoMarker,
            "get_token",
            execute_callback=self._get_token_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self.play_column_action = ActionServer(
            self,
            PlayColumn,
            "play_column",
            execute_callback=self._play_column_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self.return_to_start_action = ActionServer(
            self,
            GotoMarker,
            "return_to_start",
            execute_callback=self._return_to_start_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )

        # Demo: move to column 4.
        # self.navigation_manager.move_to_feeder()
        # self.token_manager.grab_token()
        self.navigation_manager.move_to_column(4)
        self.token_manager.place_token(4)
        self.navigation_manager.return_to_start()
        self.get_logger().info("Motion complete.")

    def check_canceled(self):
        """Raise an exception if canceled."""
        if self.cancel_event.is_set():
            raise CancelGoalException()

    def checked_pose_move(self, trajectory: dict):
        """Wrapper for move_to_pose that is blocking and will check for goal cancel state after."""
        self.move_to_pose(trajectory, blocking=True)
        self.check_canceled()

    def _get_token_execute(self, goal_handle):
        # Reset cancel event for goal.
        self.cancel_event.clear()

        result = GotoMarker.Result()
        try:
            self.navigation_manager.move_to_feeder()
            self.token_manager.grab_token()
            self.navigation_manager.return_to_start()

            goal_handle.succeed()
        except CancelGoalException:
            msg = "Get Token Canceled by User."
            self.get_logger().info(msg)
            result.result = msg
            goal_handle.canceled()

        return result

    def _return_to_start_execute(self, goal_handle):
        # Reset cancel event for goal.
        self.cancel_event.clear()

        result = GotoMarker.Result()

        try:
            self.navigation_manager.return_to_start()

            goal_handle.succeed()
        except CancelGoalException:
            msg = "Return to Start Canceled by User."
            self.get_logger().info(msg)
            result.result = msg
            goal_handle.canceled()

        return result

    def _play_column_execute(self, goal_handle):
        # Reset cancel event for goal.
        self.cancel_event.clear()

        result = PlayColumn.Result()
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
