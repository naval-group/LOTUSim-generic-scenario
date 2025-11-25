"""
@file bridge_nodes.launch.py
@author Naval Group
@brief Launch file for Gazebo-ROS 2 bridge nodes.

@details
This launch file starts bridge nodes between Gazebo and ROS 2 for:
- Wind/environmental data (`wind_ros_to_gz_bridge`)
- Simulation statistics (e.g., simulation time, RTF) (`stats_gz_to_ros_bridge`)

It ensures previous instances are killed before launch and introduces a small
startup delay to avoid race conditions.

@note
Designed to integrate Gazebo simulation data with ROS 2 topics for monitoring,
logging, or control purposes.

@version 0.1
@date 2025-10-08

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches two Gazebo-ROS bridge nodes with proper cleanup:
       - `wind_ros_to_gz_bridge`: Bridges wind/environmental data from Gazebo to ROS 2 topics.
       - `stats_gz_to_ros_bridge`: Bridges simulation statistics (e.g., simulation time, RTF) from Gazebo to ROS 2.
    Delays the startup of these nodes slightly (1 second) to avoid race conditions with the pkill commands.
    """

    # ------------------------------------------------------------------
    # Kill any existing bridge nodes
    # ------------------------------------------------------------------
    # This ensures that if any previous instance of these nodes is running,
    # they are terminated before launching new ones.
    kill_wind_node = ExecuteProcess(
        cmd=["pkill", "-f", "wind_ros_to_gz_bridge"],  # kills any process matching this name
        shell=False,  # do not run in shell, safer
        output="screen",  # log output to screen
    )

    kill_stats_node = ExecuteProcess(cmd=["pkill", "-f", "stats_gz_to_ros_bridge"], shell=False, output="screen")

    # ------------------------------------------------------------------
    # Define bridge nodes
    # ------------------------------------------------------------------
    # Define the ROS 2 nodes that will bridge Gazebo topics to ROS 2
    wind_node = Node(
        package="gz_ros2_bridge",  # ROS 2 package containing the executable
        executable="wind_ros_to_gz_bridge",  # Node executable name
        name="wind_ros_to_gz_bridge",  # ROS 2 node name
        output="screen",  # Log output to screen
    )

    stats_node = Node(
        package="gz_ros2_bridge", executable="stats_gz_to_ros_bridge", name="stats_gz_to_ros_bridge", output="screen"
    )

    # ------------------------------------------------------------------
    # Delay node startup
    # ------------------------------------------------------------------
    # TimerAction is used to delay the launch of the nodes slightly (1 second)
    # to ensure they are not killed by the pkill commands above.
    delayed_wind_node = TimerAction(period=1.0, actions=[wind_node])  # wait 1 second  # then launch wind_node

    delayed_stats_node = TimerAction(period=1.0, actions=[stats_node])

    # ------------------------------------------------------------------
    # Delay node startup
    # ------------------------------------------------------------------
    # All actions (pkill + delayed node launches) are included in the LaunchDescription.
    return LaunchDescription([kill_wind_node, kill_stats_node, delayed_wind_node, delayed_stats_node])
