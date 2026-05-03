from hello_helpers.hello_misc import HelloNode
from rclpy.callback_groups import ReentrantCallbackGroup

# noinspection PyUnresolvedReferences
from stretch_pose_interface.srv import GetPose

# Constants.
END_FRAME = "link_grasp_center"
WORLD_FRAME = "odom"
ROBOT_FRAME = "base_link"
ARUCO_FRAME = "column_4"


class PoseServer(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)

    # noinspection PyMethodOverriding
    def main(self):
        HelloNode.main(
            self, "pose_server", "pose_server", wait_for_first_pointcloud=False
        )

        callback_group = ReentrantCallbackGroup()

        # Define GetPose service.
        _get_pose_service = self.create_service(
            GetPose, "get_pose", self._get_pose_callback, callback_group=callback_group
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

            response.world_pose = tf_world
            response.robot_pose = tf_robot
            response.aruco_pose = tf_aruco
            response.success = True
            response.message = "OK"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response


def main():
    node = PoseServer()
    node.main()


if __name__ == "__main__":
    main()
