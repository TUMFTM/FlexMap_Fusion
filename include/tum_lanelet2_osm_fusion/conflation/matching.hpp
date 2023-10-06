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
// Date: 24.04.2023
// ==========================================
//
//
#pragma once
//
#include "utility.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <string>
#include <utility>
#include <vector>

class cmatching
{
public:
  cmatching();

  /************************************************************************
   * Collapse adjacent lanelets to a centerline to approximate the level
   * of detail of openstreetmap-data
   *************************************************************************/
  bool collapse_ll_map(const lanelet::LaneletMapPtr & map_ptr, lanelet::LineStrings3d & ls_col);

  /*****************************************************************************************
   * Apply buffer-growing map-matching-algorithm
   * => modified version from "Zhang 2005: A generic matching algorithm for line networks of
   *    different resolutions"
   ******************************************************************************************/
  bool buffer_growing(
    rclcpp::Node & node, lanelet::LineStrings3d & src, lanelet::LineStrings3d & target,
    std::vector<s_match> & matches);

private:
  /********************************************
   * Get centerline of a part of the street
   *********************************************/
  lanelet::LineString3d get_centerline(
    const lanelet::Lanelets & lls, lanelet::Ids & ids, const lanelet::Lanelet & ll);

  /******************************************************************
   * Connect a linestring with all previous/following linestrings
   *******************************************************************/
  void connect_lss(
    const lanelet::Lanelets & lls, lanelet::LineStrings3d & lss,
    std::vector<std::pair<lanelet::Id, lanelet::Id>> & conn, lanelet::LineString3d & ls);

  /****************************************************************************************
   * Split vector of linestrings with plines into vector of segments (copy attributes)
   *****************************************************************************************/
  lanelet::LineStrings3d split_lss(rclcpp::Node & node, lanelet::LineStrings3d & lss);

  /*************************************************************************
   * Instantiate a polyline consisting of linestring segments with 2 points
   **************************************************************************/
  lanelet::LineStrings3d init_pline(
    rclcpp::Node & node, const lanelet::LineString3d & ls, lanelet::LineStrings3d & lss,
    lanelet::Ids & ids);

  /*******************************************************************************
   * Initialize buffers around each line segment based on the given parameters
   ********************************************************************************/
  lanelet::Areas create_buffer(
    const lanelet::LineStrings3d & lss, const double buffer_V, const double buffer_P,
    const double rad);

  /***********************************************************************
   * Find all line segments inside given buffers and return as candidates
   ************************************************************************/
  std::vector<lanelet::LineStrings3d> matching_candidates(
    const lanelet::LineStrings3d & ref_pline, const lanelet::Areas & buffers,
    lanelet::LineStrings3d & lss);

  /************************************************************************
   * Exclude match candidates if one of their geometric measures to
   * the reference polyline exceeds the limits
   *************************************************************************/
  std::vector<lanelet::LineStrings3d> exclude_candidates(
    rclcpp::Node & node, const lanelet::LineStrings3d & ref,
    const std::vector<lanelet::LineStrings3d> & candidates);

  /**********************************************************************
   * Select the best match candidate out of multiple ones by a
   * weighted score of geo-similarity measures
   ***********************************************************************/
  lanelet::LineStrings3d select_candidate(
    rclcpp::Node & node, const lanelet::LineStrings3d & ref,
    const std::vector<lanelet::LineStrings3d> & candidates);

  /*****************************************************************************
   * Set geosimilarity measures for later evaluation of matching result
   ******************************************************************************/
  void calc_geo_measures(rclcpp::Node & node, s_match & match);

  /*****************************************************************************
   * Calculate matching rate
   * => all reference lines for which a match was returned after the algorithm
   ******************************************************************************/
  std::vector<double> matching_stats(rclcpp::Node & node, const std::vector<s_match> & matches);

  /******************************************************************************************
   * Connect centerlines of adjacent lanelets based on the following/previous lanelets in
   * the given direction
   *******************************************************************************************/
  void connect_dir(
    const lanelet::Lanelets & lls, lanelet::LineStrings3d & lss,
    std::vector<std::pair<lanelet::Id, lanelet::Id>> & conn, lanelet::LineString3d & ls,
    const std::string & key);

  /******************************************************************************************
   * Extend reference polyline as long as the valence is < 3 and the angle of the next
   * segment is below the given limit
   *******************************************************************************************/
  void extend_ref_pline(
    lanelet::LineStrings3d & pline, lanelet::LineStrings3d & lss, const double angle_lim,
    lanelet::Ids & ids, const std::string & direction);

  /******************************************************************************
   * Extend the current linestring segment with its previous/following ones
   * as long as they are inside the buffers and add them as match candidates
   *******************************************************************************/
  void extend_candidates(
    std::vector<lanelet::LineStrings3d> & candidates, lanelet::LineStrings3d & pline,
    lanelet::LineStrings3d & lss, const lanelet::LineStrings3d & ref_pline,
    const lanelet::Areas & buffers, lanelet::Ids & ids, const std::string & direction);

  /******************************************************
   * Check if a lanelet or linestring was already used
   *******************************************************/
  bool used_Id(const lanelet::Ids & ids, const lanelet::ConstLanelet & ll);
  bool used_Id(const lanelet::Ids & ids, const lanelet::ConstLineString3d & ls);

  /*********************************************
   * Get lanelets of a given map
   **********************************************/
  lanelet::Lanelets lanelet_layer(const lanelet::LaneletMapPtr & map_ptr);

  /*********************************************************
   * Find a lanelet in a vector of lanelets given its id
   **********************************************************/
  lanelet::ConstLanelet find_ll(const lanelet::Lanelets & lls, const lanelet::Id & id);

  /*********************************************************
   * Find following lanelets to given lanelet
   **********************************************************/
  lanelet::ConstLanelets following_ll(
    const lanelet::Lanelets & lls, const lanelet::ConstLanelet & ll);

  /*****************************************************************************
   * Find centerline-linestring that represents lanelet and get its orientation
   ******************************************************************************/
  lanelet::LineString3d find_ls(
    lanelet::LineStrings3d & lss, const lanelet::ConstLanelet & ll, bool & forward);

  /***************************************************************************
   * Find all linestrings that are connected to either the first or second
   * point of another linestring
   ****************************************************************************/
  void find_ls_from_point(
    lanelet::LineStrings3d & ls_pt, lanelet::LineStrings3d & lss, const lanelet::LineString3d & src,
    const int pos);

  /**************************************************************
   * Connect two linestrings based on their orientation
   ***************************************************************/
  void connect_ls(
    lanelet::LineString3d & ls_pre, lanelet::LineString3d & ls_fol, const bool & pre_forward,
    const bool & fol_forward);

  /**********************************************************
   * Check if two linestrings were already connected
   ***********************************************************/
  bool conn_ls(
    const std::vector<std::pair<lanelet::Id, lanelet::Id>> & conn,
    const lanelet::LineString3d & ls_pre, const lanelet::LineString3d & ls_fol);

  /**********************************************************
   * Check if a linestring segment is inside the buffers
   * => check if first && second point is inside buffers
   ***********************************************************/
  bool ls_inside_buffer(const lanelet::Areas & buf, const lanelet::LineString3d & ls);

  /**************************************************************
   * Calculate angle between first and last point of a polyline
   ***************************************************************/
  double angle_diff_pline(const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2);

  /**************************************************************
   * Calculate length difference between two polylines
   ***************************************************************/
  double len_diff_pline(const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2);

  /**************************************************************
   * Calculate chord difference between two polylines
   ***************************************************************/
  double chord_diff_pline(const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2);

  /************************************************************************************
   * Calculate quotient of area enclosed by two polylines and the sum of their lengths
   *************************************************************************************/
  double poly_area_diff_pline(
    const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2);

  /******************************************************
   * Calculate angle between two linestring segments
   *******************************************************/
  double angle_segment(
    const lanelet::LineString3d & ls1, const lanelet::LineString3d & ls2, const bool align);

  /*******************************************************************
   * Calculate chamfer distance between two linestrings
   ********************************************************************/
  double chamfer_distance(const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2);

  /*****************************************
   * Create rotation matrix from angle
   ******************************************/
  Eigen::Matrix2d rot(double angle);

  /**********************************************
   * Set the z-coordinate of a linestring to 0
   ***********************************************/
  void set_z_zero(lanelet::LineString3d & ls);

  /*******************************************************************
   * Convert vector of linestring segments to a single linestring
   ********************************************************************/
  lanelet::LineString3d ls_seg2string(const lanelet::LineStrings3d & ls);

  /**********************************************************
   * Swap forward/backward in attributes of a linestring
   ***********************************************************/
  void swap_tags(lanelet::LineString3d & ls);
};
