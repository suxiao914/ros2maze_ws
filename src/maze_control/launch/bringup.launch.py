#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键启动 TurtleBot3 迷宫左墙跟随仿真
    ros2 launch maze_control bringup.launch.py
可选参数:
    world:=/abs/path/xxx.world   世界文件(默认包内 worlds/maze_tb3.world)
    model:=burger                burger / waffle / waffle_pi
    x:=2.4 y:=2.5 yaw:=3.14159   出生位姿
    rviz:=false                  不启动 RViz2
    follower:=false              不启动算法节点
    gzclient:=false              无界面(仅 gzserver)
    use_sim_time:=true
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('maze_control')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_desc = get_package_share_directory('turtlebot3_description')

    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    urdf_path = os.path.join(tb3_desc, 'urdf', 'turtlebot3_%s.urdf' % model)
    sdf_path = os.path.join(tb3_gazebo, 'models', 'turtlebot3_%s' % model, 'model.sdf')

    # turtlebot3_burger.urdf 是 xacro 模板(内含 ${namespace})，必须展开
    robot_description = xacro.process_file(
        urdf_path, mappings={'namespace': ''}).toxml()

    actions = [
        # 1. Gazebo 服务端：加载迷宫世界
        ExecuteProcess(
            cmd=['gzserver', '--verbose',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 world],
            name='gzserver',
            output='screen'),

        # 2. Gazebo 客户端：图形界面
        ExecuteProcess(
            cmd=['gzclient'],
            name='gzclient',
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient'))),

        # 3. 机器人描述 -> TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}]),

        # 4. 生成小车实体(等 gazebo 起来)
        TimerAction(period=5.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_burger',
                output='screen',
                arguments=['-entity', 'burger',
                           '-file', sdf_path,
                           '-x', x, '-y', y, '-z', '0.01', '-Y', yaw,
                           '-timeout', '60'])]),

        # 5. 用户算法节点：左墙跟随
        TimerAction(period=8.0, actions=[
            Node(
                package='maze_control',
                executable='tb3_wall_follower',
                name='tb3_wall_follower',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('follower')))]),

        # 6. RViz2 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'tb3_maze.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_control')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'maze_tb3.world'),
            description='Gazebo world 文件绝对路径'),
        DeclareLaunchArgument('model', default_value='burger',
                              description='burger / waffle / waffle_pi'),
        DeclareLaunchArgument('x', default_value='2.4'),
        DeclareLaunchArgument('y', default_value='2.5'),
        DeclareLaunchArgument('yaw', default_value='3.14159'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('follower', default_value='true'),
        DeclareLaunchArgument('gzclient', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
