import time

import rclpy
import rclpy.time

from rclpy.node import Node
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener

from control_msgs.action import FollowJointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

from builtin_interfaces.msg import Duration


ARUCO_FRAME = "column_4"


class FindBoard(Node):

    def __init__(self):
        super().__init__("find_board")

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Trajectory action client
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory"
        )

        self.get_logger().info(
            "Waiting for trajectory server..."
        )

        self.trajectory_client.wait_for_server()

        self.get_logger().info(
            "Connected to trajectory server."
        )

    def move_head_pan(self, angle):

        goal_msg = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            "joint_head_pan"
        ]

        point = JointTrajectoryPoint()

        point.positions = [angle]

        point.time_from_start = Duration(
            sec=2
        )

        trajectory.points.append(point)

        goal_msg.trajectory = trajectory

        self.get_logger().info(
            f"Moving head pan to {angle:.2f} radians"
        )

        future = self.trajectory_client.send_goal_async(
            goal_msg
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "Trajectory goal rejected."
            )
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        return True

    def marker_found(self):

        try:
            self.tf_buffer.lookup_transform(
                "base_link",
                ARUCO_FRAME,
                rclpy.time.Time()
            )

            return True

        except Exception:
            return False

    def search_for_board(self):

        search_angles = [
            -1.2,
            -0.8,
            -0.4,
            0.0,
            0.4,
            0.8,
            1.2,
            0.0
        ]

        self.get_logger().info(
            "Searching for Connect Four board..."
        )

        while rclpy.ok():

            for angle in search_angles:

                self.move_head_pan(angle)

                time.sleep(2.0)

                if self.marker_found():

                    self.get_logger().info(
                        "Found Connect Four board!"
                    )

                    return

                else:

                    self.get_logger().info(
                        "Marker not found yet..."
                    )

        self.get_logger().info(
            "Search stopped."
        )


def main():

    rclpy.init()

    node = FindBoard()

    try:
        node.search_for_board()

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
