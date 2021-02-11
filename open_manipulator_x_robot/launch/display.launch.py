#!/usr/bin/env python3
#
# Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    robot_description = os.path.join(get_package_share_directory(
        "open_manipulator_x_robot"), "description", "open_manipulator_x_robot.urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            node_executable="joint_state_publisher_gui",
            node_name="joint_state_publisher_gui",
            parameters=[{"source_list": ["joint_states"]},
                        {"robot_description": robot_description_config.toxml()}],
            output="screen"),

        Node(
            package="robot_state_publisher",
            node_executable="robot_state_publisher",
            node_name="robot_state_publisher",
            arguments=[robot_description],
            output="screen"),

        Node(
            package="rviz2",
            node_executable="rviz2",
            node_name="rviz2",
            output="screen")
    ])
