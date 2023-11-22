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
#include "utility.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <string>
#include <vector>

class crubber_sheeting
{
public:
  crubber_sheeting();

  /*****************************************************************
   * Get controlpoints from RVIZ as published point by the user
   ******************************************************************/
  bool select_control_points(
    rclcpp::Node & node, const lanelet::ConstLineString3d & src,
    const lanelet::ConstLineString3d & target, std::vector<s_control_point> & cps);

  /************************************************************************************
   * Calculate triangles and transformation matrices for rubber-sheet transformation
   * according to:
   * White 1985: Piecewise Linear Rubber-Sheet Map Transformation
   *************************************************************************************/
  bool get_transformation(
    rclcpp::Node & node, const lanelet::ConstLineString3d & target,
    std::vector<s_control_point> & cps, lanelet::Areas & tri, std::vector<Eigen::Matrix3d> & trans);

  /*****************************************************************
   * Transform whole map according to rubber-sheet trafo
   ******************************************************************/
  bool transform_map(
    const lanelet::LaneletMapPtr & map_ptr, const lanelet::Areas & tri,
    const std::vector<Eigen::Matrix3d> & trans);

  /***********************************************************************
   * Transfrom linestring according to rubber-sheet transformation
   ************************************************************************/
  bool transform_ls(
    const lanelet::ConstLineString3d & ls, lanelet::ConstLineString3d & ls_trans,
    const lanelet::Areas & tri, const std::vector<Eigen::Matrix3d> & trans);

  /*********************************************************************************
   * Transform corresponding pointcloud map to lanelet map (only x-/y-coordinates)
   **********************************************************************************/
  bool transform_pcd(
    rclcpp::Node & node, const lanelet::Areas & tri, const std::vector<Eigen::Matrix3d> & trans,
    const Eigen::Matrix3d & trans_al);

  /*********************************************************************************
   * Transform corresponding pointcloud map according to SLAM poses - GPS traj
   **********************************************************************************/
  bool transform_pcd(
    rclcpp::Node & node, const lanelet::Areas & tri, const std::vector<Eigen::Matrix3d> & trans,
    const Eigen::Matrix3d & trans_al, pcl::PointCloud<pcl::PointXYZ>::Ptr & pcm);

private:
  /**************************************************************
   * Find closest point on given linestring for given point
   ***************************************************************/
  void closest_on_ls(lanelet::Point3d & pt, const lanelet::ConstLineString3d & ls);

  /************************************************************
   * Define enclosing target rectangle for rubber-sheeting
   *************************************************************/
  lanelet::Area enclosing_rectangle(const lanelet::ConstLineString3d & target);

  /******************************************************************************
   * Compute source rectangle from control points and target rectangle
   *******************************************************************************/
  lanelet::Area src_rectangle(
    const lanelet::Area & target_rec, const std::vector<s_control_point> & cps);

  /**********************************************************************
   * Add points from source and target rectangle to control points
   ***********************************************************************/
  void corner2cp(
    const lanelet::Area & target_rec, const lanelet::Area & src_rec,
    std::vector<s_control_point> & cps);

  /*********************************************************************
   * Calculate triangluation according to
   * White 1985: Piecewise Linear Rubber-Sheet Map Transformation
   **********************************************************************/
  void triangulation(const std::vector<s_control_point> & cps, lanelet::Areas & tri);

  /**********************************************************
   * Calculate transformation matrices for triangles
   * => solve linear equations for three points
   ***********************************************************/
  void tranformation_matrices(
    const std::vector<s_control_point> & cps, const lanelet::Areas & tri,
    std::vector<Eigen::Matrix3d> & trans);

  /****************************************************************
   * Perfrom quadrilateral test to avoid narrow triangles
   *****************************************************************/
  void quadrilateral_test(lanelet::Areas & tri);

  /****************************************************
   * Calculate the height of a triangle
   *****************************************************/
  double triangle_height(const lanelet::Area & ar);

  /***********************************************
   * Calculate the area of a triangle
   ************************************************/
  double triangle_area(const lanelet::Area & ar);

  /************************************************************
   * Transform point according to transformation matrix
   *************************************************************/
  void transform_pt(lanelet::Point3d & pt, const Eigen::Matrix3d & trans);

  /**************************************************************************
   * Solve linear equations defined by three points forming a triangle
   * => calculate rotation matrix
   ***************************************************************************/
  Eigen::Matrix3d solve_linear(
    const lanelet::ConstPoints3d & src, const lanelet::ConstPoints3d & target);
};
