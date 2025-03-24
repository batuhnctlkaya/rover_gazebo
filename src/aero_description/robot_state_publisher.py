import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Robot paketinin adını değiştir
    pkg_name = "aero_description"

    # Xacro dosyanın yolunu ayarla
    xacro_file = os.path.join(get_package_share_directory(pkg_name), "urdf", "rover_base.urdf.xacro")

    # Xacro dosyasını işleyerek URDF'e çevir
    robot_description = Command(["xacro", xacro_file])

    # robot_state_publisher node'u
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Joint State Publisher GUI (Sadece fiziksel sensör kullanmıyorsan)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=LaunchConfiguration("use_gui"),
    )

    # RViz2'yi açmak için (Opsiyonel)
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), "rviz", "robot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(name="use_gui", default_value="True", description="Joint State Publisher GUI kullan"),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
