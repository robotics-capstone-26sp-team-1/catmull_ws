from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    stretch_core_share = FindPackageShare('stretch_core')

    # --- 1. stretch_driver ---
    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([stretch_core_share, 'launch', 'stretch_driver.launch.py'])
        ]),
        launch_arguments={'broadcast_odom_tf': 'True'}.items(),
    )

    # --- 2. D435i ---
    d435i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([stretch_core_share, 'launch', 'd435i_high_resolution.launch.py'])
        ]),
        launch_arguments={'initial_reset': 'false'}.items(),
    )

    # --- 3. ArUco detector ---
    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([stretch_core_share, 'launch', 'stretch_aruco.launch.py'])
        ]),
    )

    DRIVER_READY = b'[stretch_driver]: Changed to mode = position'
    D435I_READY  = b'[camera]: RealSense Node Is Up!'
    ARUCO_READY  = b'[detect_aruco_node]: detect_aruco_node started'

    on_driver_ready = RegisterEventHandler(
        OnProcessIO(
            on_stdout=lambda event: (
                [
                    LogInfo(msg='[startup] stretch_driver ready — launching D435i...'),
                    d435i,
                ]
                if DRIVER_READY in event.text else []
            )
        )
    )

    on_d435i_ready = RegisterEventHandler(
        OnProcessIO(
            on_stdout=lambda event: (
                [
                    LogInfo(msg='[startup] D435i ready — launching ArUco detector...'),
                    aruco,
                ]
                if D435I_READY in event.text else []
            )
        )
    )

    on_aruco_ready = RegisterEventHandler(
        OnProcessIO(
            on_stdout=lambda event: (
                [LogInfo(msg='[startup] All systems ready — assistfour startup complete!')]
                if ARUCO_READY in event.text else []
            )
        )
    )

    return LaunchDescription([
        LogInfo(msg='[startup] Launching stretch_driver...'),
        stretch_driver,
        on_driver_ready,
        on_d435i_ready,
        on_aruco_ready,
    ])