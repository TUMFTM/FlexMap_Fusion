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
  rclcpp::Node & node, std::string & traj_path, std::string & poses_path, std::string & pcd_path,
  std::string & pcd_out_path)
{
  // Get parameters from command line and param file
  // Paths => command line arguments (default in launch file)
  node.declare_parameter<std::string>("traj_path");
  node.declare_parameter<std::string>("poses_path");
  node.declare_parameter<std::string>("pcd_path");
  node.declare_parameter<std::string>("pcd_out_path");
  node.get_parameter("traj_path", traj_path);
  node.get_parameter("poses_path", poses_path);
  node.get_parameter("pcd_path", pcd_path);
  node.get_parameter("pcd_out_path", pcd_out_path);

  // Parameters => param file (config/)
  // Master
  node.declare_parameter<std::string>("master");
  node.get_parameter("master");

  // Transformation GPS - Poses
  node.declare_parameter<int>("align_num_inter_ume");
  node.declare_parameter<int>("rs_num_controlPoints");
  node.declare_parameter<bool>("transform_pcd");
  node.declare_parameter<bool>("save_ascii");
  node.get_parameter("align_num_inter_ume");
  node.get_parameter("rs_num_controlPoints");
  node.get_parameter("transform_pcd");
  node.get_parameter("save_ascii");

  // Zero point
  node.declare_parameter<bool>("customZeroPoint");
  node.declare_parameter<double>("zeroLat");
  node.declare_parameter<double>("zeroLong");
  node.get_parameter("customZeroPoint");
  node.get_parameter("zeroLat");
  node.get_parameter("zeroLong");

  // Analysis Output
  node.declare_parameter<std::string>("analysis_output_dir");
  node.declare_parameter<bool>("analysis_traj_matching");
  node.get_parameter("analysis_output_dir");
  node.get_parameter("analysis_traj_matching");
}
