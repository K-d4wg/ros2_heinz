import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

  
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_h1_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_h1_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_h1_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the URDF instead of SDF, since the robot state publisher works better with URDF
    urdf_file = os.path.join(pkg_project_description, 'models/h1_ign', 'h1_2_handless.urdf')
    with open(urdf_file, 'r') as infp:
        urdf_description = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'empty_h1.sdf'
        ])}.items(),
    )

    set_gpu_env = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_env = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': urdf_description}
        ]
    )    

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_h1_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'check_joints_gz.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        set_gpu_env,
        set_glx_env,
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'), 
        bridge,
        robot_state_publisher,
        rviz
    ])
