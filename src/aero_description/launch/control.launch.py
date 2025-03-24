from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Xacro dosyasının yolu
    pkg_name = 'aero_description'
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'rover_base.urdf.xacro'
    )
    urdf_file = os.path.join('/tmp', 'rover_base.urdf')  # Geçici bir URDF dosyası oluştur

    # Xacro dosyasını URDF'e dönüştür
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )

    return LaunchDescription([
        # Xacro'dan URDF'e dönüştürme işlemi
        xacro_to_urdf,

        # Gazebo'da robotu spawn etme
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'rover', '-file', urdf_file],
            output='screen'
        ),

        # Kontrolcü düğümü
        Node(
            package='aero_description',
            executable='controller',
            output='screen'
        )
    ])