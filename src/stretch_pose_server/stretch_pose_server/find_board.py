import time
from hello_helpers.hello_misc import HelloNode
from math import sqrt, atan2
from tf_transformations import euler_from_quaternion, quaternion_matrix
import numpy as np

ROBOT_FRAME = "base_link"
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

        # Stow the robot for safe movement.
        self.stow_the_robot()

        # Find the board.
        board_transform = self._find_board()

        # Move to it and rotate back.
        self._nav_to_board(board_transform)

        # Home the robot to be ready.
        self.home_the_robot()

    def _find_board(self):
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

        while True:
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

                    return tf

    def _nav_to_board(self, board_transform):
        self.get_logger().info("Navigating to board")

        # Extract quaternion and rotation matrix of marker in base_link frame
        x, y, z, w = (
            board_transform.transform.rotation.x,
            board_transform.transform.rotation.y,
            board_transform.transform.rotation.z,
            board_transform.transform.rotation.w,
        )
        R = quaternion_matrix((x, y, z, w))

        # Apply rotation to the offset vector
        P_dash = np.array([[0], [0], [0.75], [1]])
        P = np.array(
            [
                [board_transform.transform.translation.x],
                [board_transform.transform.translation.y],
                [0],
                [1],
            ]
        )
        X = np.matmul(R, P_dash)

        # Compute the marker position with offset in base_link frame
        P_base = X + P
        P_base[3, 0] = 1  # Homogeneous coordinate

        # Extract adjusted position
        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]

        # Compute rotation and translation needed
        phi = atan2(base_position_y, base_position_x)
        dist = sqrt(base_position_x ** 2 + base_position_y ** 2)

        _, _, z_rot_base = euler_from_quaternion([x, y, z, w])
        # Calculate final rotation: -phi (cancel rotation needed to align),
        # + z_rot_base (original marker rotation),
        # + pi (such that the base and the marker axis are aligned as shown in tutorial)
        z_rot_base = -phi + z_rot_base + np.pi

        # Rotate to board.
        self.move_to_pose({"rotate_mobile_base": phi}, blocking=True)

        # Drive to within 0.75m of it.
        self.move_to_pose({"translate_mobile_base": dist}, blocking=True)

        # Counter Rotate to face board.
        self.move_to_pose({"rotate_mobile_base": z_rot_base}, blocking=True)


def main():
    node = FindBoard()
    node.main()


if __name__ == "__main__":
    main()
