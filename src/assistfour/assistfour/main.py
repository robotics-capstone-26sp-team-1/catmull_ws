from __future__ import annotations

from typing import TYPE_CHECKING

from geometry_msgs.msg import Twist
from hello_helpers.hello_misc import HelloNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Event
from .constants import CancelGoalException
from time import sleep

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
            cancel_callback=self._cancel_execute,
            callback_group=self._callback_group,
        )
        self.play_column_action = ActionServer(
            self,
            PlayColumn,
            "play_column",
            execute_callback=self._play_column_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=self._cancel_execute,
            callback_group=self._callback_group,
        )
        self.return_to_start_action = ActionServer(
            self,
            GotoMarker,
            "return_to_start",
            execute_callback=self._return_to_start_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=self._cancel_execute,
            callback_group=self._callback_group,
        )

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
            sleep(5)
            self.navigation_manager.return_to_start()
        except CancelGoalException:
            msg = "Get Token canceled by user."
            self.get_logger().info(msg)
            self._set_result(result, msg)
            goal_handle.canceled()
            return result
        except ValueError as exc:
            msg = f"Get Token failed: {exc}"
            self.get_logger().error(msg)
            self._set_result(result, msg)
            goal_handle.abort()
            return result

        self.get_logger().info("Get Token Finished.")
        self._set_result(result, "")
        goal_handle.succeed()
        return result

    def _return_to_start_execute(self, goal_handle):
        # Reset cancel event for goal.
        self.cancel_event.clear()

        result = GotoMarker.Result()
        try:
            self.navigation_manager.return_to_start()
        except CancelGoalException:
            msg = "Return to Start canceled by user."
            self.get_logger().info(msg)
            self._set_result(result, msg)
            goal_handle.canceled()
            return result
        except ValueError as exc:
            msg = f"Return to Start failed: {exc}"
            self.get_logger().error(msg)
            self._set_result(result, msg)
            goal_handle.abort()
            return result

        self.get_logger().info("Return to Start Finished.")
        self._set_result(result, "")
        goal_handle.succeed()
        return result

    def _play_column_execute(self, goal_handle):
        # Reset cancel event for goal.
        self.cancel_event.clear()

        result = PlayColumn.Result()
        try:
            target_column = goal_handle.request.column
            self.navigation_manager.move_to_column(target_column)
            self.token_manager.place_token(target_column)
            sleep(5)
            self.navigation_manager.return_to_start()
        except CancelGoalException:
            msg = "Play Column canceled by user."
            self.get_logger().info(msg)
            self._set_result(result, msg)
            goal_handle.canceled()
            return result
        except ValueError as exc:
            msg = f"Play Column failed: {exc}"
            self.get_logger().error(msg)
            self._set_result(result, msg)
            goal_handle.abort()
            return result

        self.get_logger().info("Play Column Finished.")
        self._set_result(result, "")
        goal_handle.succeed()
        return result

    def _cancel_execute(self, _):
        self.cancel_event.set()
        self.stop_the_robot()
        self.switch_to_position_mode()
        return CancelResponse.ACCEPT

    @staticmethod
    def _set_result(result, message: str):
        result.result = message
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
