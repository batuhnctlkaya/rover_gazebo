import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    # Gazebo'yu başlatmak için (boş dünya)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
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
        gazebo_launch,
        spawn_entity,
    ])
