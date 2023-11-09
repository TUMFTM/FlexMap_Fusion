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
#include "message_conversion.hpp"
#include "visualization.hpp"

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
    lanelet::ConstLanelets & shoulder, visualization_msgs::msg::MarkerArray & map_marker_msg);

  /***************************************************************************************
   * Insert road lanelets and shoulder lanelets into marker array with visualization
   * of centerline, boundary, start, end, lanelets are colored based on accordance
   * with osm-lanes tag
   ****************************************************************************************/
  void col_map_net2marker_msg(
    rclcpp::Node & node, lanelet::ConstLanelets & all, lanelet::ConstLanelets & regular,
    lanelet::ConstLanelets & shoulder,
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

  void pcd_map2msg(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map, sensor_msgs::msg::PointCloud2 & msg);

private:
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

  // Initialize helper libraries
  cmessage_conversion m_message_conversion;
  cvisualization m_visualization;
};
