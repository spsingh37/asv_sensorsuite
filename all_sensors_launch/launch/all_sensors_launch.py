from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define paths to the launch files of each package
    lidar_launch_path = os.path.join(get_package_share_directory('rslidar_sdk'), 'launch', 'start.py')
    camera_launch_path = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    imu_launch_path = os.path.join(get_package_share_directory('vectornav'), 'launch', 'vectornav.launch.py')

    # Include the individual launch files
    lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch_path),
            launch_arguments={'namespace': ''}.items()
            )
    camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch_path),
                        launch_arguments={'namespace': ''}.items()
            )
    imu_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_launch_path),
                        launch_arguments={'namespace': ''}.items()
            )

    # Define transformations from each sensor frame to 'basic' frame
    lidar_to_basic = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "basic", "rslidar"]
    )

    camera_to_basic = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.1", "0.2", "0", "0", "0", "0", "basic", "camera_color_optical_frame"]
    )

    imu_to_basic = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.1", "0", "0.1", "0", "0", "0", "basic", "imu_frame"]
    )

    # Return LaunchDescription with all included launch files and transformations
    return LaunchDescription([
        lidar_launch,
        camera_launch,
        imu_launch,
        lidar_to_basic,
        camera_to_basic,
        imu_to_basic
    ])

