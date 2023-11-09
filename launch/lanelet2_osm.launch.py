# Copyright 2023 Maximilian Leitenstern
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
#
# ========================================== //
# Author: Maximilian Leitenstern (TUM)
# Date: 21.03.2023
# ========================================== //
#
#
import os
import os.path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Define path to config folder
    config = os.path.join(get_package_share_directory("flexmap_fusion"), "config")

    # Define command line args
    traj_path = DeclareLaunchArgument(
        "traj_path", default_value=TextSubstitution(text="test/route_1_GPS.txt")
    )
    poses_path = DeclareLaunchArgument(
        "poses_path", default_value=TextSubstitution(text="test/route1_pose_kitti.txt")
    )
    map_path = DeclareLaunchArgument(
        "map_path", default_value=TextSubstitution(text="test/lanelet2_route_1.osm")
    )
    out_path = DeclareLaunchArgument(
        "out_path", default_value=TextSubstitution(text="lanelet2_map.osm")
    )

    # Start nodes
    rviz2_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d" + os.path.join(config, "rviz_conf_lanelet2_osm.rviz")],
    )

    lanelet2_osm_node = Node(
        package="flexmap_fusion",
        namespace="",
        executable="lanelet2_osm",
        name="lanelet2_osm",
        output="screen",
        parameters=[
            {
                "traj_path": LaunchConfiguration("traj_path"),
                "poses_path": LaunchConfiguration("poses_path"),
                "map_path": LaunchConfiguration("map_path"),
                "out_path": LaunchConfiguration("out_path"),
            },
            os.path.join(config, "lanelet2_osm.param.yaml"),
        ],
    )

    return LaunchDescription(
        [traj_path, poses_path, map_path, out_path, lanelet2_osm_node, rviz2_node]
    )
