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
// Date: 23.03.2023
// ==========================================
//
//
#pragma once

#include "color.hpp"
#include "utility.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_io/Io.h>

#include <string>
#include <utility>
#include <vector>

class cmessages
{
public:
  cmessages();

  /********************************************************************
   * Insert a linestring as markerarray into another markerarray
   * with desired color, namespace and linethickness
   *********************************************************************/
  void linestring2marker_msg(
    const lanelet::ConstLineString3d & ls, visualization_msgs::msg::MarkerArray & msg,
    const std::string color, const std::string ns, const float thickness);

  /***************************************************************************************
   * Insert road lanelets and shoulder lanelets into marker array with visualization
   * of centerline, boundary, start, end
   ****************************************************************************************/
  void map_net2marker_msg(
    rclcpp::Node & node, lanelet::ConstLanelets & all, lanelet::ConstLanelets & regular,
    lanelet::ConstLanelets & shoulder, std::vector<lanelet::ConstLineString3d> & stop_lines,
    visualization_msgs::msg::MarkerArray & map_marker_msg);

  /***************************************************************************************
   * Insert road lanelets and shoulder lanelets into marker array with visualization
   * of centerline, boundary, start, end, lanelets are colored based on accordance
   * with osm-lanes tag
   ****************************************************************************************/
  void col_map_net2marker_msg(
    rclcpp::Node & node, lanelet::ConstLanelets & all, lanelet::ConstLanelets & regular,
    lanelet::ConstLanelets & shoulder, std::vector<lanelet::ConstLineString3d> & stop_lines,
    const std::vector<std::pair<lanelet::Id, std::string>> & cols,
    visualization_msgs::msg::MarkerArray & map_marker_msg);

  /*****************************************************************************************
   * Insert linestrings of an openstreetmap-file that represent streets into a markerarray
   ******************************************************************************************/
  void osm_net2marker_msg(
    const lanelet::ConstLineStrings3d & motorways, const lanelet::ConstLineStrings3d & highways,
    const lanelet::ConstLineStrings3d & roads,
    visualization_msgs::msg::MarkerArray & map_marker_msg);

  /*****************************************************************
   * Insert geometries from rubber-sheeting into markerarray
   * => triangles, controlpoints
   ******************************************************************/
  void rs2marker_msg(
    const lanelet::Areas & areas, const std::vector<s_control_point> & cps,
    visualization_msgs::msg::MarkerArray & msg);

  /******************************************************************************
   * Insert geometries from conflation into markerarray
   * => collapsed lanelets with tags of corresponding ids, reference polyline,
   * target polyline, buffers, match-connections
   *******************************************************************************/
  void confl2marker_msg(
    const lanelet::ConstLineStrings3d & ll_coll, const std::vector<s_match> & matches,
    visualization_msgs::msg::MarkerArray & msg);

private:
  /*****************************************************************************
   * Convert areas (rubber-sheet triangles) to markerarray (new namespace for
   * each triangle)
   ******************************************************************************/
  visualization_msgs::msg::MarkerArray areas2marker_msg(
    const lanelet::Areas & areas, const std::string color, const std::string ns,
    const float thickness);

  /*********************************************************************************
   * Convert controlpoints to markerarray (linestring from source to target point)
   **********************************************************************************/
  visualization_msgs::msg::MarkerArray controlpoints2marker_msg(
    const std::vector<s_control_point> & cps, const std::string color, const std::string ns,
    const float thickness);

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

  /***************************************************************************
   * Convert area to to a triangle for later conversion to markerarray
   ****************************************************************************/
  void area2triangle(
    const lanelet::Area & ar, std::vector<geometry_msgs::msg::Polygon> * triangles);

  /*****************************************************************************
   * Convert an area to a polygon for later conversion to a triangle
   ******************************************************************************/
  void area2polygon(const lanelet::Area & ar, geometry_msgs::msg::Polygon * polygon);

  /**********************************************************************************
   * Set color of for a markerarray specified by string defining the color name
   * from "color.hpp" and a defining the intensity
   ***********************************************************************************/
  void set_color(std_msgs::msg::ColorRGBA * color, const std::string & name, double a);

  /**************************************************************
   * Insert marker array at the end of another markerarray
   ***************************************************************/
  void insert_marker_array(
    visualization_msgs::msg::MarkerArray * arr_src,
    const visualization_msgs::msg::MarkerArray & arr_in);

  /**********************************************
   * Convert single linestring to markerarray
   ***********************************************/
  visualization_msgs::msg::MarkerArray linestring2marker_msg(
    const lanelet::ConstLineString3d & ls, const std::string & ns,
    const std_msgs::msg::ColorRGBA & c, const float lss);

  /******************************************************
   * Convert linestring into arrows at its points
   *******************************************************/
  visualization_msgs::msg::MarkerArray linestring_arrows2marker_msg(
    const lanelet::ConstLineStrings3d & lss, const std::string & ns, const std::string & color);

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

  /************************************************************************
   * Convert areas into vector of the linestrings defining its bounds
   *************************************************************************/
  lanelet::ConstLineStrings3d areas2linestrings(const lanelet::Areas & areas);

  /********************************************************************
   * Convert vector of linestrings to vector of constlinestrings
   *********************************************************************/
  lanelet::ConstLineStrings3d to_const(const lanelet::LineStrings3d & lss);

  /**********************************************************************
   * Get color code of a lanelet from conflation based on its id
   ***********************************************************************/
  std::string id2col(
    const std::vector<std::pair<lanelet::Id, std::string>> & cols, const lanelet::Id & id);
};
