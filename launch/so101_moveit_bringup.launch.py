"""
so101_moveit_bringup.launch.py
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare("so101_moveit_config")

    declared_args = [
        DeclareLaunchArgument(
            "device_port",
            default_value="/dev/ttyACM0",
            description="Serial port for the SO-101.",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="1000000",
            description="Baud rate for the serial connection.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Launch RViz2 for visualization.",
        ),
    ]

    device_port = LaunchConfiguration("device_port")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution([pkg_share, "config", "so101.urdf.xacro"]),
                " use_fake_hardware:=false",
                " device_port:=",
                device_port,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Using MoveIt controllers
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"]), 
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(   # Arm control
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    hand_controller_spawner = Node(  # Gripper control
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            ],
        )
    )

    delay_arm = RegisterEventHandler(  
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=3.0, actions=[arm_controller_spawner]),
            ],
        )
    )

    delay_hand = RegisterEventHandler(  
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=3.0, actions=[hand_controller_spawner]),
            ],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        declared_args
        + [
            robot_state_publisher,
            controller_manager,
            delay_jsb,
            delay_arm,   
            delay_hand,  
            rviz_node,
        ]
    )