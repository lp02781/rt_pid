"""Start both programs.

    ros2 launch rt_pid rt_pid.launch.py cpu:=3 setpoint:=25.0 kp:=0.8
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("cpu", default_value="-1"),        # -1 = any core
        DeclareLaunchArgument("period_us", default_value="1000"),
        DeclareLaunchArgument("prio", default_value="80"),
        DeclareLaunchArgument("setpoint", default_value="10.0"),
        DeclareLaunchArgument("kp", default_value="0.5"),
        DeclareLaunchArgument("ki", default_value="4.0"),
        DeclareLaunchArgument("kd", default_value="0.01"),
    ]

    # The fast loop: real-time, pinned to one core.
    loop = Node(
        package="rt_pid", executable="rt_pid_loop", name="rt_pid_loop",
        output="screen", emulate_tty=True,
        arguments=[
            "--period-us", LaunchConfiguration("period_us"),
            "--prio", LaunchConfiguration("prio"),
            "--cpu", LaunchConfiguration("cpu"),
        ],
    )

    # The ROS node: normal priority, free to run anywhere. Do not pin it to the
    # reserved core, its network traffic is what reserving a core keeps away.
    node = Node(
        package="rt_pid", executable="pid_node", name="pid",
        output="screen", emulate_tty=True,
        parameters=[{
            "setpoint": LaunchConfiguration("setpoint"),
            "kp": LaunchConfiguration("kp"),
            "ki": LaunchConfiguration("ki"),
            "kd": LaunchConfiguration("kd"),
        }],
    )

    return LaunchDescription(args + [loop, node])
