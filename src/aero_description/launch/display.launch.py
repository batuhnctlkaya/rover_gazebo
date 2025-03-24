import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    file_path = os.path.join(get_package_share_directory('aero_description'), 'urdf', 'rover_base.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_directory('aero_description'), 'config', 'rviz_config.rviz')
    
    # Read the URDF file content
    with open(file_path, 'r') as file:
        robot_description_content = file.read()

    # If using xacro:
    xacro_file_path = os.path.join(get_package_share_directory('aero_description'), 'urdf', 'rover_base.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file_path).toxml()


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])