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
// Date: 20.03.2023
// ==========================================
//
//
#pragma once
//
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <vector>

class calign
{
public:
  calign();

  /*********************************************************************
   * Calculate transformation between two linestrings by the selected
   * algorithm
   **********************************************************************/
  bool get_transformation(
    rclcpp::Node & node, const lanelet::ConstLineString3d & src,
    const lanelet::ConstLineString3d & target, Eigen::Matrix3d & trans);

  /*******************************************************************
   * Transform whole map according to transformation matrix
   ********************************************************************/
  bool transform_map(const lanelet::LaneletMapPtr & map_ptr, const Eigen::Matrix3d & trans);

  /*******************************************************************
   * Transform linestring according to given transformation (2D)
   ********************************************************************/
  bool transform_ls(
    lanelet::ConstLineString3d & ls, lanelet::ConstLineString3d & ls_trans,
    const Eigen::Matrix3d & trans);

private:
  /*************************************************************************
   * Calculate transformation matrix according to Umeyama algorithm
   * => interpolate linestrings to get same amount of points
   **************************************************************************/
  bool point_transformation_umeyama(
    const lanelet::ConstLineString3d & src, const lanelet::ConstLineString3d & target,
    Eigen::Matrix3d & trans, rclcpp::Node & node);

  /*************************************************************************************
   * Transforming the coordinates of a point according to transformation matrix (2D)
   **************************************************************************************/
  void transform_pt(lanelet::Point3d & pt, const Eigen::Matrix3d & trans);

  /*********************************************************************************
   * Interpolation of linestring to linestring with defined number of points
   **********************************************************************************/
  Eigen::MatrixXd ls2interp_mat2d(const lanelet::ConstLineString3d & ls, const int & num);

  /************************************************************************************
   * Extract intersection nodes (nodes with valence > 2) from openstreetmap-network
   *************************************************************************************/
  void get_intersection_nodes(
    const lanelet::ConstLineStrings3d & osm_ls, lanelet::ConstPoints3d & pt);
};
