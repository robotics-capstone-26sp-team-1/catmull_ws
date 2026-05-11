from geometry_msgs.msg import TransformStamped, PointStamped
from hello_helpers.hello_misc import HelloNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.service import Service
import tf2_geometry_msgs  # noqa: F401  (registers geometry msg transforms for tf2)

# noinspection PyUnresolvedReferences
from stretch_pose_interfaces.srv import GetPose

# noinspection PyUnresolvedReferences
from stretch_pose_interfaces.action import SetPose

# Constants.
END_FRAME = "link_grasp_center"
WORLD_FRAME = "odom"
ROBOT_FRAME = "base_link"
ARUCO_FRAME = "column_4"
VALID_FRAMES = {WORLD_FRAME, ROBOT_FRAME, ARUCO_FRAME}

HOME_Y = -0.5
HOME_ARM = 0.1


class PoseServer(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        self._callback_group: ReentrantCallbackGroup | None = None
        self._get_pose_service: Service | None = None
        self._set_pose_service: ActionServer | None = None

    # noinspection PyMethodOverriding
    def main(self):
        HelloNode.main(
            self, "pose_server", "pose_server", wait_for_first_pointcloud=False
        )

        self._callback_group = ReentrantCallbackGroup()

        # Define GetPose service.
        self._get_pose_service = self.create_service(
            GetPose,
            "get_pose",
            self._get_pose_callback,
            callback_group=self._callback_group,
        )

        # Define SetPose action.
        self._set_pose_service = ActionServer(
            self,
            SetPose,
            "set_pose",
            execute_callback=self._set_pose_execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )

        # Ready.
        self.get_logger().info("PoseServer is ready.")

    def _get_pose_callback(self, _, response):
        """
        Get pose service callback.

        :param _: Ignored request params (none).
        :param response: Response the current end effector pose relative to the three frames.
        :return: Completed response.
        """
        try:
            tf_world = self.get_tf(WORLD_FRAME, END_FRAME)
            tf_robot = self.get_tf(ROBOT_FRAME, END_FRAME)
            tf_aruco = self.get_tf(ARUCO_FRAME, END_FRAME)

            if any(t is None for t in [tf_world, tf_robot, tf_aruco]):
                response.success = False
                response.message = "One or more TF lookups failed."
                return response

            # Manually remove x-axis from robot frame.
            tf_robot.transform.translation.x = 0

            response.world_pose = tf_world
            response.robot_pose = tf_robot
            response.aruco_pose = tf_aruco
            response.success = True
            response.message = "OK"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def _set_pose_execute(self, goal_handle):
        # Build result and feedback objects.
        result = SetPose.Result()
        feedback = SetPose.Feedback()

        # Extract target.
        target: TransformStamped = goal_handle.request.target_pose
        ref_frame: str = target.header.frame_id

        self.get_logger().info(f"Going to target {target}...")

        # Validate the frame
        if ref_frame not in VALID_FRAMES:
            result.success = False
            result.message = f"Unknown frame: {ref_frame}."
            goal_handle.abort()
            return result

        # Convert target position into robot frame.
        feedback.status = "Looking up target pose in robot frame."
        goal_handle.publish_feedback(feedback)

        try:
            target_point = PointStamped()
            target_point.header.frame_id = ref_frame
            target_point.header.stamp = self.get_clock().now().to_msg()
            target_point.point.x = target.transform.translation.x
            target_point.point.y = target.transform.translation.y
            target_point.point.z = target.transform.translation.z

            target_in_robot_frame = self.tf2_buffer.transform(
                target_point, ROBOT_FRAME, timeout=Duration(seconds=1)
            )

            self.get_logger().info(f"Target in robot_frame: {target_in_robot_frame}")
        except Exception as e:
            feedback.status = "Unable to get target pose."
            goal_handle.publish_feedback(feedback)
            result.success = False
            result.message = f"Unable to get target pose: {e}."
            goal_handle.abort()
            self.get_logger().error(f"Unable to get target pose: {e}.")
            return result

        # Compute joint command.
        feedback.status = "Computing joint values."
        goal_handle.publish_feedback(feedback)

        target_x = target_in_robot_frame.point.x
        target_y = target_in_robot_frame.point.y
        target_z = target_in_robot_frame.point.z

        joints = self._transform_to_joints(target_x, target_y, target_z)

        self.get_logger().info(
            f"Joint commands → "
            f"translate_mobile_base={joints['translate_mobile_base']:.3f} "
            f"joint_arm={joints['joint_arm']:.3f} "
            f"joint_lift={joints['joint_lift']:.3f}"
        )

        # Check for cancellation before moving.
        if goal_handle.is_cancel_requested:
            feedback.status = "Cancelled."
            goal_handle.publish_feedback(feedback)
            result.success = False
            result.message = "Goal cancelled before motion started."
            goal_handle.canceled()
            return result

        # Execute joint command.
        feedback.status = "Moving to pose..."
        goal_handle.publish_feedback(feedback)

        try:
            self.move_to_pose(joints, blocking=True)
        except Exception as e:
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            return result

        # Check for cancellation after motion completes.
        if goal_handle.is_cancel_requested:
            feedback.status = "Cancelled."
            goal_handle.publish_feedback(feedback)
            result.success = False
            result.message = "Goal cancelled after motion completed."
            goal_handle.canceled()
            return result

        self.get_logger().info("Target reached!")

        feedback.status = "Done!"
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.success = True
        result.message = "OK"

        print("===")

        return result

    @staticmethod
    def _transform_to_joints(transform_x, transform_y, transform_z):
        arm = HOME_ARM + (-(transform_y - HOME_Y))
        lift = transform_z - 0.1
        return {
            "translate_mobile_base": transform_x,
            "joint_arm": arm,
            "joint_lift": lift,
        }


def main():
    node = PoseServer()
    node.main()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
