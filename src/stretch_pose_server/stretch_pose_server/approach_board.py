import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import TransformStamped

# noinspection PyUnresolvedReferences
from stretch_pose_interfaces.action import SetPose


ARUCO_FRAME = "column_4"


class ApproachBoardClient(Node):

    def __init__(self):
        super().__init__("approach_board_client")

        self._client = ActionClient(
            self,
            SetPose,
            "set_pose"
        )

        self.get_logger().info("Waiting for set_pose action server...")
        self._client.wait_for_server()

        self.get_logger().info("Connected to set_pose server.")

    def send_goal(self):

        goal_msg = SetPose.Goal()

        target = TransformStamped()

        #
        # IMPORTANT:
        # Goal is specified RELATIVE to the Connect Four marker.
        #
        target.header.frame_id = ARUCO_FRAME

        #
        # Desired offset from board.
        #
        # Adjust these experimentally.
        #

        # Move robot ~0.7m in front of board
        target.transform.translation.x = -0.7

        # Center laterally
        target.transform.translation.y = 0.0

        # Keep roughly table height
        target.transform.translation.z = 0.3

        #
        # Rotation currently unused by PoseServer,
        # but initialize safely.
        #
        target.transform.rotation.w = 1.0

        goal_msg.target_pose = target

        self.get_logger().info(
            "Sending goal to approach Connect Four board..."
        )

        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f"Feedback: {feedback.status}"
        )

    def result_callback(self, future):

        result = future.result().result

        self.get_logger().info(
            f"Result: success={result.success}, "
            f"message='{result.message}'"
        )

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = ApproachBoardClient()

    node.send_goal()

    rclpy.spin(node)


if __name__ == "__main__":
    main()