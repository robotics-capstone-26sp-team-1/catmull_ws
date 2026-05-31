from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    stretch_core_share = FindPackageShare("stretch_core")
    rosbridge_share = FindPackageShare("rosbridge_server")

    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [stretch_core_share, "launch", "stretch_driver.launch.py"]
                )
            ]
        ),
        launch_arguments={"broadcast_odom_tf": "True"}.items(),
    )

    d435i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [stretch_core_share, "launch", "d435i_high_resolution.launch.py"]
                )
            ]
        ),
        launch_arguments={"initial_reset": "false"}.items(),
    )

    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [stretch_core_share, "launch", "stretch_aruco.launch.py"]
                )
            ]
        ),
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [rosbridge_share, "launch", "rosbridge_websocket_launch.xml"]
                )
            ]
        ),
        launch_arguments={"send_action_goals_in_new_thread": "true"}.items(),
    )

    return LaunchDescription(
        [
            LogInfo(msg="[startup] Launching assistfour startup..."),
            stretch_driver,
            d435i,
            aruco,
            rosbridge,
            LogInfo(msg="[startup] All launch files started."),
        ]
    )
