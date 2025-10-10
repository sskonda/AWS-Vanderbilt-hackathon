# config/launch_sim_headless.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    world = '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/config/world.sdf'

    env = [
        # Quiet QML chatter; safe to keep even headless
        SetEnvironmentVariable('QT_LOGGING_RULES', 'qt.qml.connections.warning=false'),
        # If you ever turn GUI back on under X11:
        SetEnvironmentVariable('GDK_BACKEND', 'x11'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('XDG_SESSION_TYPE', 'x11'),
    ]

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # -s = server only (headless), -r = run, -v 3 = verbose
        launch_arguments={'gz_args': f'-s -r -v 3 {world}'}.items()
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': '/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon/config/ros_gz_bridge.yaml'}],
        output='screen'
    )

    uuv_pose_node = Node(
        package='uuv_sim',
        executable='uuv_pose',
        name='uuv_pose',
        output='screen',
    )

    return LaunchDescription(env + [gz_sim_launch, gz_bridge_node, uuv_pose_node])
