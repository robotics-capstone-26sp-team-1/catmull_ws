import time

import rclpy

from rclpy import spin

from hello_helpers.hello_misc import HelloNode


ARUCO_FRAME = "column_4"


class FindBoard(HelloNode):

    def __init__(self):
        HelloNode.__init__(self)

    def main(self):

        HelloNode.main(
            self,
            "find_board",
            "find_board",
            wait_for_first_pointcloud=False
        )

        self.get_logger().info(
            "Starting Connect Four board search..."
        )

        #
        # Tilt head slightly downward since
        # the board will likely be below
        # the robot's head height.
        #
        self.move_to_pose(
            {
                "joint_head_tilt": -0.6
            },
            blocking=True
        )

        #
        # Full sweep across a large range
        # using smaller increments.
        #
        search_angles = [
            -3.0,
            -2.5,
            -2.0,
            -1.5,
            -1.0,
            -0.5,
            0.0,
            0.5,
            1.0,
            1.5,
            2.0,
            2.5,
            3.0,
            2.5,
            2.0,
            1.5,
            1.0,
            0.5,
            0.0,
            -0.5,
            -1.0,
            -1.5,
            -2.0,
            -2.5,
        ]

        while rclpy.ok():

            for angle in search_angles:

                self.get_logger().info(
                    f"Scanning at pan angle {angle:.2f}"
                )

                self.move_to_pose(
                    {
                        "joint_head_pan": angle
                    },
                    blocking=True
                )

                #
                # Allow TF time to update.
                #
                time.sleep(1.0)

                #
                # Check whether the board
                # marker exists.
                #
                tf = self.get_tf(
                    "base_link",
                    ARUCO_FRAME
                )

                if tf is not None:

                    self.get_logger().info(
                        "Found Connect Four board!"
                    )

                    #
                    # Center head slightly after detection.
                    #
                    self.move_to_pose(
                        {
                            "joint_head_pan": angle
                        },
                        blocking=True
                    )

                    self.get_logger().info(
                        "Stopping search node."
                    )

                    rclpy.shutdown()

                    return

                self.get_logger().info(
                    "Board not found yet..."
                )


def main():

    rclpy.init()

    node = FindBoard()

    node.main()

    try:
        spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
