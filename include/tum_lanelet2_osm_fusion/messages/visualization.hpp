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
// Date: 13.10.2023
// ==========================================
//
//
#pragma once

#include "color.hpp"
#include "utility.hpp"
#include "message_conversion.hpp"

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_io/Io.h>

#include <string>
#include <utility>
#include <vector>

class cvisualization
{
public:
  cvisualization();
  
  /*********************************************************************************
   * Convert lanelet boundaries to marker arrays
   **********************************************************************************/
  visualization_msgs::msg::MarkerArray lanelets_boundary_as_marker_array(
  const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA & c,
  const bool viz_centerline, const std::string & additional_namespace);

  /*********************************************************************************
   * Convert lanelet ids to marker array
   **********************************************************************************/
  visualization_msgs::msg::MarkerArray generate_lanelet_id_marker(
  const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA & c,
  const double scale);

  /*********************************************************************************
   * Convert lanelets to triangle marker arrays
   **********************************************************************************/
  visualization_msgs::msg::MarkerArray lanelets2triangle_marker_array(
  const std::string & ns, const lanelet::ConstLanelets & lanelets,
  const std_msgs::msg::ColorRGBA & c);

  /*********************************************************************************
   * Convert lanelets into colored triangle based on color code from conflation
   **********************************************************************************/
  visualization_msgs::msg::MarkerArray colored_lanelets2marker_msg(
    const std::string & ns_base, const lanelet::ConstLanelets & lls,
    const std::vector<std::pair<lanelet::Id, std::string>> & cols);

  /*************************************************************************************
   * Convert areas to markerarray (visualization of the surface but with transparency)
   **************************************************************************************/
  visualization_msgs::msg::MarkerArray areas2marker_msg(
    const lanelet::Areas & areas, const std::string & color, const std::string & ns);

  /*****************************************************************************
   * Convert areas (rubber-sheet triangles) to markerarray (new namespace for
   * each triangle)
   ******************************************************************************/
  visualization_msgs::msg::MarkerArray areas2marker_msg(
    const lanelet::Areas & areas, const std::string color, const std::string ns,
    const float thickness);

  /**********************************************
   * Convert single linestring to markerarray
   ***********************************************/
  visualization_msgs::msg::MarkerArray linestring2marker_msg(
    const lanelet::ConstLineString3d & ls, const std::string & ns,
    const std_msgs::msg::ColorRGBA & c, const float lss);

  /**********************************************
   * Convert multiple linestrings to markerarray
   ***********************************************/
  visualization_msgs::msg::MarkerArray linestrings2marker_msg(
    const lanelet::ConstLineStrings3d & linestrings, const std::string & ns,
    const std_msgs::msg::ColorRGBA & c, const float lss);

  /**************************************************************
   * Insert marker array at the end of another markerarray
   ***************************************************************/
  void insert_marker_array(
    visualization_msgs::msg::MarkerArray * arr_src,
    const visualization_msgs::msg::MarkerArray & arr_in);

  /**********************************************************************************
   * Set color of for a markerarray specified by string defining the color name
   * from "color.hpp" and a defining the intensity
   ***********************************************************************************/
  void set_color(std_msgs::msg::ColorRGBA * color, const std::string & name, double a);

  /********************************************************
   * Convert tags lanelets into markerarray
   * => keys and if they are indexed to be specified
   *********************************************************/
  visualization_msgs::msg::MarkerArray tags2marker_msg(
    const lanelet::ConstLanelets & lanelets, const std::string & color, const double scale,
    const std::string & ns, const bool & id, const std::vector<std::string> & key,
    const bool & index);

  /********************************************************
   * Convert tags linestrings into markerarray
   * => keys and if they are indexed to be specified
   *********************************************************/
  visualization_msgs::msg::MarkerArray tags2marker_msg(
    const lanelet::ConstLineStrings3d & linestrings, const std::string & color, const double scale,
    const std::string & ns, const bool & id, const std::vector<std::string> & key,
    const bool & index);
  
  /************************************************************************
   * Convert areas into vector of the linestrings defining its bounds
   *************************************************************************/
  lanelet::ConstLineStrings3d areas2linestrings(const lanelet::Areas & areas);

  /******************************************************
   * Convert linestring into arrows at its points
   *******************************************************/
  visualization_msgs::msg::MarkerArray linestring_arrows2marker_msg(
    const lanelet::ConstLineStrings3d & lss, const std::string & ns, const std::string & color);

  /*********************************************************************************
   * Convert controlpoints to markerarray (linestring from source to target point)
   **********************************************************************************/
  visualization_msgs::msg::MarkerArray controlpoints2marker_msg(
    const std::vector<s_control_point> & cps, const std::string color, const std::string ns,
    const float thickness);

  /***************************************************************************************
   * Convert reference or target polyline of match to markerarray with changing color
   ****************************************************************************************/
  visualization_msgs::msg::MarkerArray match_pline2marker_msg(
    const std::vector<s_match> & matches, const std::vector<std::string> & colors,
    const std::string ns, const float thickness);

  /***********************************************************************************
   * Convert connection line between reference and target of a match to markerarray
   ************************************************************************************/
  visualization_msgs::msg::MarkerArray match2marker_msg(
    const std::vector<s_match> & matches, const std::string color, const std::string ns,
    const float thickness);

private:
  /***********************************************************************************
   * Initialize linestring marker
   ************************************************************************************/
  void init_linestring_marker(
  visualization_msgs::msg::Marker * marker, const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c);

  /***********************************************************************************
   * Push linestring marker
   ************************************************************************************/
  void push_linestring_marker(
  visualization_msgs::msg::Marker * marker, const lanelet::ConstLineString3d & ls,
  const std_msgs::msg::ColorRGBA & c, const float lss);

  /***********************************************************************************
   * Initialize arrow marker
   ************************************************************************************/
  void init_arrows_marker(
  visualization_msgs::msg::Marker * marker, const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c);

  /***********************************************************************************
   * Push arrow marker
   ************************************************************************************/
  void push_arrows_marker(
  visualization_msgs::msg::Marker * marker, const lanelet::ConstLineString3d & ls,
  const std_msgs::msg::ColorRGBA & c);

  /**********************************************************************
   * Get color code of a lanelet from conflation based on its id
   ***********************************************************************/
  std::string id2col(
    const std::vector<std::pair<lanelet::Id, std::string>> & cols, const lanelet::Id & id);

  /***************************************************************************
   * Convert lanelet to to a triangle for later conversion to markerarray
   ****************************************************************************/
  void lanelet2triangle(
    const lanelet::ConstLanelet & ll, std::vector<geometry_msgs::msg::Polygon> * triangles);

  /***************************************************************************
   * Convert area to to a triangle for later conversion to markerarray
   ****************************************************************************/
  void area2triangle(
    const lanelet::Area & ar, std::vector<geometry_msgs::msg::Polygon> * triangles);

  /***************************************************************************
   * Convert lanelet to polygon
   ****************************************************************************/
  void lanelet2polygon(
    const lanelet::ConstLanelet & ll, geometry_msgs::msg::Polygon * polygon);

  /*****************************************************************************
   * Convert an area to a polygon for later conversion to a triangle
   ******************************************************************************/
  void area2polygon(const lanelet::Area & ar, geometry_msgs::msg::Polygon * polygon);

  /*****************************************************************************
   * Convert an polygon to triangle for later creation of marker array
   ******************************************************************************/
  void polygon2triangle(
    const geometry_msgs::msg::Polygon & polygon,
    std::vector<geometry_msgs::msg::Polygon> * triangles);

  /***********************************************************************************
   * Extract (indexed) keys with corresponding value of a lanelet into a string
   ************************************************************************************/
  std::string ls_attr_key(
    const lanelet::ConstLanelet & ll, const std::vector<std::string> & key, const bool & index);

  /***********************************************************************************
   * Extract (indexed) keys with corresponding value of a linestring into a string
   ************************************************************************************/
  std::string ls_attr_key(
    const lanelet::ConstLineString3d & ls, const std::vector<std::string> & key,
    const bool & index);
  
  bool is_clock_wise(const geometry_msgs::msg::Polygon & polygon);

  bool is_acute_angle(
    const geometry_msgs::msg::Point32 & a, const geometry_msgs::msg::Point32 & o,
    const geometry_msgs::msg::Point32 & b);

  bool is_within_triangle(
    const geometry_msgs::msg::Point32 & a, const geometry_msgs::msg::Point32 & b,
    const geometry_msgs::msg::Point32 & c, const geometry_msgs::msg::Point32 & p);

  void adjacent_points(
    const int i, const int N, const geometry_msgs::msg::Polygon & poly,
    geometry_msgs::msg::Point32 * p0, geometry_msgs::msg::Point32 * p1,
    geometry_msgs::msg::Point32 * p2);
  
  /********************************************************************
   * Convert vector of linestrings to vector of constlinestrings
   *********************************************************************/
  lanelet::ConstLineStrings3d to_const(const lanelet::LineStrings3d & lss);

  // Initialize helper libraries
  cmessage_conversion m_message_conversion;
};
