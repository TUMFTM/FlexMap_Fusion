// Copyright 2023 Maximilian Leitenstern
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ==========================================
// Author: Maximilian Leitenstern (TUM)
// Date: 12.04.2023
// ==========================================
//
//
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>

/******************************************************************
 * Declare and load parameters from parameter file in /config
 *******************************************************************/
void get_params(
  rclcpp::Node & node, std::string & traj_path, std::string & poses_path, std::string & map_path,
  std::string & osm_path, std::string & out_path, std::string & proj_type, std::string & align_type)
{
  // Get parameters from command line and param file
  // Paths => command line arguments (default in launch file)
  node.declare_parameter<std::string>("traj_path");
  node.declare_parameter<std::string>("poses_path");
  node.declare_parameter<std::string>("map_path");
  node.declare_parameter<std::string>("osm_path", "map_osm.osm");
  node.declare_parameter<std::string>("out_path", "lanelet2_map.osm");
  node.get_parameter("traj_path", traj_path);
  node.get_parameter("poses_path", poses_path);
  node.get_parameter("map_path", map_path);
  node.get_parameter("osm_path", osm_path);
  node.get_parameter("out_path", out_path);

  // Parameters => param file (config/)
  // Projection
  node.declare_parameter<std::string>("proj_type");
  node.get_parameter("proj_type", proj_type);

  // Master
  node.declare_parameter<std::string>("master");
  node.get_parameter("master");

  // Transformation GPS - Poses
  node.declare_parameter<std::string>("align_type");
  node.declare_parameter<int>("align_num_inter_ume");
  node.declare_parameter<int>("rs_num_controlPoints");
  node.declare_parameter<bool>("transform_pcd");
  node.declare_parameter<bool>("save_ascii");
  node.declare_parameter<std::string>("pcd_path");
  node.get_parameter("align_type", align_type);
  node.get_parameter("align_num_inter_ume");
  node.get_parameter("rs_num_controlPoints");
  node.get_parameter("transform_pcd");
  node.get_parameter("save_ascii");
  node.get_parameter("pcd_path");

  // Zero point
  node.declare_parameter<bool>("customZeroPoint");
  node.declare_parameter<double>("zeroLat");
  node.declare_parameter<double>("zeroLong");
  node.get_parameter("customZeroPoint");
  node.get_parameter("zeroLat");
  node.get_parameter("zeroLong");

  // Buffer Growing
  node.declare_parameter<double>("seg_len");
  node.declare_parameter<double>("pline_angle");
  node.declare_parameter<double>("buffer_V");
  node.declare_parameter<double>("buffer_P");
  node.declare_parameter<double>("buffer_rad");
  node.declare_parameter<double>("lim_angle");
  node.declare_parameter<double>("lim_length");
  node.declare_parameter<double>("lim_chord");
  node.declare_parameter<double>("lim_poly");
  node.declare_parameter<double>("w_angle");
  node.declare_parameter<double>("w_length");
  node.declare_parameter<double>("w_chord");
  node.declare_parameter<double>("w_poly");
  node.declare_parameter<double>("lim_tp");
  node.declare_parameter<double>("lim_ref_pline");
  node.get_parameter("seg_len");
  node.get_parameter("pline_angle");
  node.get_parameter("buffer_V");
  node.get_parameter("buffer_P");
  node.get_parameter("buffer_rad");
  node.get_parameter("lim_angle");
  node.get_parameter("lim_length");
  node.get_parameter("lim_chord");
  node.get_parameter("lim_poly");
  node.get_parameter("w_angle");
  node.get_parameter("w_length");
  node.get_parameter("w_chord");
  node.get_parameter("w_poly");
  node.get_parameter("lim_tp");
  node.get_parameter("lim_ref_pline");

  // Visualization
  node.declare_parameter<bool>("viz_lanelet_centerline");
  node.get_parameter("viz_lanelet_centerline");

  // Analysis Output
  node.declare_parameter<std::string>("analysis_output_dir");
  node.declare_parameter<bool>("analysis_traj_matching");
  node.declare_parameter<bool>("analysis_matching");
  node.get_parameter("analysis_output_dir");
  node.get_parameter("analysis_traj_matching");
  node.get_parameter("analysis_matching");
}
