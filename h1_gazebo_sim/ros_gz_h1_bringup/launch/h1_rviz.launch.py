from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    
    urdf = os.path.join(get_package_share_directory('ros_gz_h1_description'), 'models/h1_ign', 'h1_2_handless.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }],
            arguments=[urdf]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('ros_gz_h1_bringup'), 'config', 'check_joints.rviz')]
        )
    ])
