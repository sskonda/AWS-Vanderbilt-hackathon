from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '/home/kapow/Development/AWS-Vanderbilt-hackathon/config/world.sdf'
        }.items()
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': '/home/kapow/Development/AWS-Vanderbilt-hackathon/config/ros_gz_bridge.yaml'}]
    )

    uuv_pose_node = Node(
        package='uuv_sim',
        executable='uuv_pose',
        name='uuv_pose',
        output='screen',
    )

    return LaunchDescription([gz_sim_launch, gz_bridge_node, uuv_pose_node])
