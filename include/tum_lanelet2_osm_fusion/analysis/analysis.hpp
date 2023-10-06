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
// Date: 20.04.2023
// ==========================================
//
//
#pragma once
//
#include "utility.hpp"

#include <Eigen/Dense>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <string>
#include <utility>
#include <vector>

class canalysis
{
public:
  canalysis();

  /************************************************************************
   * Calculate difference between aligned and rubber-sheeted trajectory
   * and export everything into txt-files for python-visualization
   *************************************************************************/
  bool traj_matching(
    rclcpp::Node & node, const lanelet::ConstLineString3d & src,
    const lanelet::ConstLineString3d & target, const lanelet::ConstLineString3d & target_al,
    const lanelet::ConstLineString3d & target_rs, const lanelet::Areas & tri,
    const std::vector<s_control_point> & cps, std::vector<double> & diff_al,
    std::vector<double> & diff_rs);

  /*************************************************************
   * Export matching data in txt-files for python-visualization
   **************************************************************/
  bool matching(
    rclcpp::Node & node, const std::vector<s_match> & matches, const lanelet::LineStrings3d & osm,
    const lanelet::ConstLanelets & lls, std::vector<std::pair<lanelet::Id, std::string>> & ll_cols,
    const lanelet::ConstLanelets & lls_updated);

private:
  /***************************************************************
   * Calculate difference between source & target trajectory
   * => vertical distance from target to source for each point
   ****************************************************************/
  void calc_diff(
    const lanelet::ConstLineString3d & src, const lanelet::ConstLineString3d & target,
    std::vector<double> & diff);

  /*******************************************************************************
   * Create subdirectory in the desired output-directory if not existing so far
   ********************************************************************************/
  void create_output_dir(rclcpp::Node & node, const std::string & dir_path);

  /**************************************
   * Write linestring to txt-file
   ***************************************/
  void write_ls(
    rclcpp::Node & node, const lanelet::ConstLineString3d & ls, const std::string & dir_path,
    const std::string & file_name);

  /**************************************
   * Write linestrings to txt-file
   ***************************************/
  void write_lss(
    rclcpp::Node & node, const lanelet::LineStrings3d & lss, const std::string & dir_path,
    const std::string & file_name);

  /***************************************
   * Write double-vector to txt-file
   ****************************************/
  void write_double_vec(
    rclcpp::Node & node, const std::vector<double> & vec, const std::string & dir_path,
    const std::string & file_name);

  /********************************
   * Write areas to txt-file
   *********************************/
  void write_areas(
    rclcpp::Node & node, const lanelet::Areas & areas, const std::string & dir_path,
    const std::string & file_name);

  /****************************************
   * Write controlpoints to txt-file
   *****************************************/
  void write_cp(
    rclcpp::Node & node, const std::vector<s_control_point> & cps, const std::string & dir_path,
    const std::string & file_name);

  /**********************************************************************************
   * Write reference or target polyline or match connection of a match to txt-file
   ***********************************************************************************/
  void write_match_pline(
    rclcpp::Node & node, const std::vector<s_match> & matches, const std::string & dir_path,
    const std::string & file_name);

  /***********************************************
   * Write buffers of a match to txt-file
   ************************************************/
  void write_match_areas(
    rclcpp::Node & node, const std::vector<s_match> & matches, const std::string & dir_path,
    const std::string & file_name);

  /***********************************************
   * Write lanelets of a map to txt-file
   ************************************************/
  void write_lanelets(
    rclcpp::Node & node, const lanelet::ConstLanelets & lls, const std::string & dir_path,
    const std::string & file_name);

  /***********************************************
   * Write lanelets of a map to txt-file
   ************************************************/
  void write_lanelets_WGS84(
    rclcpp::Node & node, const lanelet::ConstLanelets & lls, const std::string & dir_path,
    const std::string & file_name);

  /***********************************************
   * Write colors of lanelets of a map to txt-file
   ************************************************/
  void write_lanelets_cols(
    rclcpp::Node & node, const lanelet::ConstLanelets & lls,
    std::vector<std::pair<lanelet::Id, std::string>> & ll_cols, const std::string & dir_path,
    const std::string & file_name);

  /********************************************************************
   * Convert vector of linestrings to vector of constlinestrings
   *********************************************************************/
  lanelet::ConstLineStrings3d to_const(const lanelet::LineStrings3d & lss);
};
