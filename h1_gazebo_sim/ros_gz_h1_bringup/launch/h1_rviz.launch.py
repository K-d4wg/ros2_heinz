from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import math

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value='goal',
        description='Top-level namespace'
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value='/home/k-d4wg/ros2_ws/src/ros2_heinz/h1_gazebo_sim/ros_gz_h1_description/models/h1_ign/h1_2_handless.urdf',
        description='Absolute path to the robot URDF file'
    )

    namespace = LaunchConfiguration('namespace')
    model = LaunchConfiguration('model')

    return LaunchDescription([
        namespace_arg,
        model_arg,

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=namespace,
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': Command(['gz sdf -p ', model])
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_L_hand',
            namespace=namespace,
            arguments=[
                '0.054', '0', '0',          # x y z
                str(math.pi/2), '0', '0',   # roll pitch yaw (in radians)
                'left_wrist_yaw_link',      # parent frame
                'L_hand_base_link'          # child frame
            ]
        ),

        # Similarly for the right hand (example values, replace with actual)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_R_hand',
            namespace=namespace,
            arguments=[
                '0.054', '0', '0',
                str(math.pi/2), '0', '0',
                'right_wrist_yaw_link',
                'R_hand_base_link'
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('ros_gz_h1_bringup'), 'config', 'check_joints.rviz')]
        )
    ])
