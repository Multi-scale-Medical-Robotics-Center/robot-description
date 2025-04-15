# ros2 launch your_package_name launch_file.py
# ros2 launch launch/ros2.py urdfFile:=/PATH/TO/XXXX.urdf [jointUi:=['True'|'False']] rviz2File:=/PATH/TO/XXXX.rviz

# Example 1: ros2  launch launch/raad.py urdfFile:=configs/urdf/ur5e.urdf rviz2File:=configs/rviz/ur5e.rviz
# Example 2: ros2  launch launch/raad.py urdfFile:=configs/urdf/ur5e.urdf rviz2File:=configs/rviz/ur5e.rviz jointUi:='True'

import os
import sys
import launch
import launch_ros.descriptions

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    actions = []

    urdfFile = LaunchConfiguration('urdfFile')
    urdfFileName = urdfFile.perform(context)
    print('\033[92m' + "robot_state_publisher: " + urdfFileName + '\033[0m')
    with open(urdfFileName, 'r') as infp:
        robot_desc = infp.read()

    robot_desc = robot_desc.replace("file://./", "file://" + os.getcwd() +"/")

    # Parameter for robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
           "robot_description": robot_desc
        }]
    )
    actions.append(robot_state_publisher_node)

    # RViz
    rviz2File = LaunchConfiguration('rviz2File')
    rviz2FileName = rviz2File.perform(context)
    print('\033[92m' + "rivz2: " + rviz2FileName + '\033[0m')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz2FileName]
    )
    actions.append(rviz2_node)

    # Joint State Publisher GUI
    jointUi = LaunchConfiguration("jointUi")
    jointUiEnable = jointUi.perform(context)
    print('\033[92m' + "joint_state_publisher_gui: " + jointUiEnable + '\033[0m')
    if jointUiEnable == 'True':
        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        )
        actions.append(joint_state_publisher_gui_node)

    return actions

def generate_launch_description():

    ld = LaunchDescription()

    urdfFile = launch.actions.DeclareLaunchArgument(
        'urdfFile',
        default_value = '',
        description='URDF File'
    )
    ld.add_action(urdfFile)

    rviz2File = launch.actions.DeclareLaunchArgument(
        'rviz2File',
        default_value = '',
        description='Rviz2 Configuration File'
    )
    ld.add_action(rviz2File)

    jointUi = launch.actions.DeclareLaunchArgument(
        'jointUi',
        default_value = 'False',
        description='Show Joint State Publisher GUI'
    )
    ld.add_action(jointUi)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
