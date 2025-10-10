from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def _prepend_env(var_name: str, new_path: str):
    prev = os.environ.get(var_name)
    return new_path if not prev else f"{new_path}:{prev}"

def generate_launch_description():
    # Paths (edit if your repo lives elsewhere)
    repo_root   = "/home/katherine/Code/aws-hackathon/AWS-Vanderbilt-hackathon"
    world_sdf   = os.path.join(repo_root, "config", "world.sdf")
    models_dir  = os.path.join(repo_root, "models")
    bridge_yaml = os.path.join(repo_root, "config", "ros_gz_bridge.yaml")

    # Environment for Fortress GUI on X11 and for model resources
    env = [
        # X11 GUI (you can drop these now that you switched to lightdm/Xorg, but keeping is harmless)
        SetEnvironmentVariable("GDK_BACKEND", "x11"),
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        SetEnvironmentVariable("XDG_SESSION_TYPE", "x11"),

        # Fortress resource path so model:// URIs resolve
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            _prepend_env("IGN_GAZEBO_RESOURCE_PATH", models_dir)
        ),
        # (Optional) also set new var name used by Garden+; harmless on Fortress
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            _prepend_env("GZ_SIM_RESOURCE_PATH", models_dir)
        ),
    ]

    # Start Ignition Gazebo (Fortress) with GUI; -r = start running; -v 3 = verbose
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v 3 {world_sdf}"
        }.items(),
    )

    # Bridge (Fortress naming still uses ignition.msgs.* types in your YAML)
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )

    # Your UUV pose node
    uuv_pose_node = Node(
        package="uuv_sim",
        executable="uuv_pose",
        name="uuv_pose",
        output="screen",
    )

    return LaunchDescription(env + [gz_sim_launch, gz_bridge_node, uuv_pose_node])
