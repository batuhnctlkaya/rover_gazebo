import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Robotun Xacro dosyasının yolu
    pkg_name = "aero_description"  # Burayı kendi paket adınla değiştir
    xacro_file = os.path.join(get_package_share_directory(pkg_name), "urdf", "rover_base.urdf.xacro")
    urdf_file = os.path.join("/tmp", "rover_base.urdf")  # Geçici bir dosya oluştur

    # Xacro dosyasını URDF'e dönüştür
    xacro_to_urdf = ExecuteProcess(
        cmd=["xacro", xacro_file, "-o", urdf_file],
        output="screen",
    )

    # Gazebo World dosyasının yolu (eğer özel world kullanıyorsan)
    world_file = os.path.join(get_package_share_directory(pkg_name), "worlds", "empty.world")

    # Gazebo'yu başlatmak için
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_file}.items(),
    )

    # Robotu spawn etmek için
    spawn_entity = ExecuteProcess(
        cmd=[
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", "my_robot",
            "-file", urdf_file,
            "-x", "0", "-y", "0", "-z", "0.5"
        ],
        output="screen",
    )

    return LaunchDescription([
        xacro_to_urdf,
        DeclareLaunchArgument(name="world", default_value=world_file, description="Gazebo world file"),
        gazebo_launch,
        spawn_entity,
    ])
