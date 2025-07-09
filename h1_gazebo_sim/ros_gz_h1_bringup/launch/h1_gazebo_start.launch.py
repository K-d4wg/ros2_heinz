import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command

def generate_launch_description():
    # 1. Unpause the simulation
    unpause_gz = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/demo/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--req', 'pause: false'
        ],
        output='screen'
    )

    # 2. Wait 3 seconds before teleporting
    teleport_robot = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/demo/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--req',
            'name: "h1_ign" position: {x: 0.0, y: 0.0, z: 0.6} orientation: {x: 0.0, y: 0.208, z: 0.0, w: 0.978}'
        ],
        output='screen'
    )

    return LaunchDescription([
        unpause_gz,
        TimerAction(
            period=3.0,
            actions=[teleport_robot]
        )
    ])
