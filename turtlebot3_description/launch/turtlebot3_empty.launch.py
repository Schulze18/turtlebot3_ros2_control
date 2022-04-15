
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments = {'pause': 'false'}.items(),
             )
    
    # Load URDF
    robot_description_path = os.path.join(
        get_package_share_directory('turtlebot3_description'))

    xacro_file = os.path.join(robot_description_path,
                              'urdf',
                              'turtlebot3_burger.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        remappings=[
            ("/turtlebot3_diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        parameters=[robot_description]
    )

    # Spawn Robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'turtlebot3',
                                   '-z', '0.00'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])