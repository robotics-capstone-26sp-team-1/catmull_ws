import time

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
                "joint_head_tilt": -0.3
            },
            blocking=True
        )

        #
        # Full sweep across the full range
        # using smaller increments.
        #
        search_angles = [-4 + i * (6 / 32) for i in range(32)]
        found = False

        while not found:
            self.get_logger().info(
                "Board not found yet..."
            )

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

                    found = True
                    break


def main():
    node = FindBoard()
    node.main()


if __name__ == "__main__":
    main()
