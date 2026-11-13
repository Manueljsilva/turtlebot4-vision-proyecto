"""Launch unificado de SLAM (slam_toolbox) + Nav2 para TurtleBot 4 Lite en ROS 2 Jazzy.

Uso dentro del contenedor Docker:
  ros2 launch /tmp/proyecto2-autonomo/launch/slam_nav2_launch.py use_sim_time:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    raiz = Path(__file__).resolve().parent.parent
    slam_params_default = str(raiz / "config" / "slam_params.yaml")
    nav2_params_default = str(raiz / "config" / "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    slam_dir = get_package_share_directory("slam_toolbox")
    nav2_dir = get_package_share_directory("nav2_bringup")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(slam_dir) / "launch" / "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(nav2_dir) / "launch" / "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="true para Gazebo, false para TurtleBot 4 real",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=slam_params_default,
            description="Ruta al YAML de slam_toolbox",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=nav2_params_default,
            description="Ruta al YAML de Nav2",
        ),
        slam_launch,
        nav2_launch,
    ])
