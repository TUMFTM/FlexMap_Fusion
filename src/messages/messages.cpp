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
// ========================================== //
// Author: Maximilian Leitenstern (TUM)
// Date: 23.03.2023
// ========================================== //
//
//
#include "messages.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

/**************/
/*Constructors*/
/**************/

cmessages::cmessages()
{
}

/****************/
/*public methods*/
/****************/

/********************************************************************
 * Insert a linestring as markerarray into another markerarray
 * with desired color, namespace and linethickness
 *********************************************************************/
void cmessages::linestring2marker_msg(
  const lanelet::ConstLineString3d & ls, visualization_msgs::msg::MarkerArray & msg,
  const std::string color, const std::string ns, const float thickness)
{
  std_msgs::msg::ColorRGBA col;
  m_visualization.set_color(&col, color, 0.999);

  m_visualization.insert_marker_array(
    &msg, m_visualization.linestring2marker_msg(ls, ns, col, thickness));
}

/***************************************************************************************
 * Insert road lanelets and shoulder lanelets into marker array with visualization
 * of centerline, boundary, start, end
 ****************************************************************************************/
void cmessages::map_net2marker_msg(
  rclcpp::Node & node, lanelet::ConstLanelets & all, lanelet::ConstLanelets & regular,
  lanelet::ConstLanelets & shoulder, visualization_msgs::msg::MarkerArray & map_marker_msg)
{
  (void)all;
  bool viz_lanelet_centerline = node.get_parameter("viz_lanelet_centerline").as_bool();

  // Set colors of elements
  std_msgs::msg::ColorRGBA color_reg, color_shoulder, color_lanelet_id, color_lanelet_borders,
    color_shoulder_borders;
  m_visualization.set_color(&color_reg, "WEBBlueLight", 0.5);
  m_visualization.set_color(&color_shoulder, "Gray1", 0.999);
  m_visualization.set_color(&color_lanelet_id, "", 0.999);
  m_visualization.set_color(&color_lanelet_borders, "Gray3", 0.999);
  m_visualization.set_color(&color_shoulder_borders, "Gray3", 0.999);

  // Get markers and insert them into markerarray

  // Shoulder lanelets
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.generate_lanelet_id_marker(shoulder, color_lanelet_id, 1.0));
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.lanelets_boundary_as_marker_array(
                       shoulder, color_shoulder_borders, viz_lanelet_centerline, "shoulder_"));
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.lanelets2triangle_marker_array(
                       "shoulder_road_lanelets", shoulder, color_shoulder));

  // Regular lanelets
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.lanelets_boundary_as_marker_array(
                       regular, color_lanelet_borders, viz_lanelet_centerline, "lanelets_"));
  m_visualization.insert_marker_array(
    &map_marker_msg,
    m_visualization.lanelets2triangle_marker_array("regular_lanelets", regular, color_reg));
  // Conflated tags of lanelets
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.tags2marker_msg(
                       regular, "Ivory", 0.5, "conflated_attributes", true,
                       {"subtype", "location", "speed_limit", "road_name", "road_surface",
                        "one_way", "lane_markings"},
                       false));
}

/***************************************************************************************
 * Insert road lanelets and shoulder lanelets into marker array with visualization
 * of centerline, boundary, start, end
 ****************************************************************************************/
void cmessages::col_map_net2marker_msg(
  rclcpp::Node & node, lanelet::ConstLanelets & all, lanelet::ConstLanelets & regular,
  lanelet::ConstLanelets & shoulder,
  const std::vector<std::pair<lanelet::Id, std::string>> & cols,
  visualization_msgs::msg::MarkerArray & map_marker_msg)
{
  (void)all;
  bool viz_lanelet_centerline = node.get_parameter("viz_lanelet_centerline").as_bool();

  // Set colors of elements
  std_msgs::msg::ColorRGBA color_reg, color_shoulder, color_lanelet_id, color_lanelet_borders,
    color_shoulder_borders;
  m_visualization.set_color(&color_reg, "WEBBlueLight", 0.5);
  m_visualization.set_color(&color_shoulder, "Gray1", 0.999);
  m_visualization.set_color(&color_lanelet_id, "", 0.999);
  m_visualization.set_color(&color_lanelet_borders, "Gray3", 0.999);
  m_visualization.set_color(&color_shoulder_borders, "Gray3", 0.999);

  // Get markers and insert them into markerarray

  // Shoulder lanelets
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.generate_lanelet_id_marker(shoulder, color_lanelet_id, 1.0));
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.lanelets_boundary_as_marker_array(
                       shoulder, color_shoulder_borders, viz_lanelet_centerline, "shoulder_"));
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.lanelets2triangle_marker_array(
                       "shoulder_road_lanelets", shoulder, color_shoulder));

  // Regular lanelets
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.lanelets_boundary_as_marker_array(
                       regular, color_lanelet_borders, viz_lanelet_centerline, "lanelets_"));
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.colored_lanelets2marker_msg("regular_lanelets", regular, cols));
  // Conflated tags of lanelets
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.tags2marker_msg(
                       regular, "Ivory", 0.5, "conflated_attributes", true,
                       {"subtype", "location", "speed_limit", "road_name", "road_surface",
                        "one_way", "lane_markings"},
                       false));
}

/*****************************************************************************************
 * Insert linestrings of an openstreetmap-file that represent streets into a markerarray
 ******************************************************************************************/
void cmessages::osm_net2marker_msg(
  const lanelet::ConstLineStrings3d & motorways, const lanelet::ConstLineStrings3d & highways,
  const lanelet::ConstLineStrings3d & roads, visualization_msgs::msg::MarkerArray & map_marker_msg)
{
  // Set colors
  std_msgs::msg::ColorRGBA color_motorways, color_highways, color_roads;
  m_visualization.set_color(&color_motorways, "Orange", 0.99);
  m_visualization.set_color(&color_highways, "Orange", 0.99);
  m_visualization.set_color(&color_roads, "Orange", 0.99);

  // Visualize tags that specify the type of highway of the road and the number of lanes
  // if existing
  const std::string ns = "tags";
  const std::vector<std::string> tag{"highway", "lanes"};

  // Motorways
  m_visualization.insert_marker_array(
    &map_marker_msg,
    m_visualization.linestrings2marker_msg(motorways, "motorways", color_motorways, 0.5));
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.tags2marker_msg(motorways, "", 1, ns, false, tag, false));

  // Highways
  m_visualization.insert_marker_array(
    &map_marker_msg,
    m_visualization.linestrings2marker_msg(
      highways, "highways", color_highways, 0.5));
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.tags2marker_msg(highways, "", 1, ns, false, tag, false));

  // Roads
  m_visualization.insert_marker_array(
    &map_marker_msg,
    m_visualization.linestrings2marker_msg(roads, "roads", color_roads, 0.5));
  m_visualization.insert_marker_array(
    &map_marker_msg, m_visualization.tags2marker_msg(roads, "", 1, ns, false, tag, false));
}

/*****************************************************************
 * Insert geometries from rubber-sheeting into markerarray
 * => triangles, controlpoints
 ******************************************************************/
void cmessages::rs2marker_msg(
  const lanelet::Areas & areas, const std::vector<s_control_point> & cps,
  visualization_msgs::msg::MarkerArray & msg)
{
  m_visualization.insert_marker_array(&msg, m_visualization.areas2marker_msg(areas, "Ivory", "Triangles", 1));
  m_visualization.insert_marker_array(
    &msg, m_visualization.controlpoints2marker_msg(cps, "WEBOrange", "controlpoint", 1.5));
}

/******************************************************************************
 * Insert geometries from conflation into markerarray
 * => collapsed lanelets with tags of corresponding ids, reference polyline,
 * target polyline, buffers, match-connections
 *******************************************************************************/
void cmessages::confl2marker_msg(
  const lanelet::ConstLineStrings3d & ll_coll, const std::vector<s_match> & matches,
  visualization_msgs::msg::MarkerArray & msg)
{
  (void)ll_coll;
  std_msgs::msg::ColorRGBA col_coll, col_boundary;
  m_visualization.set_color(&col_coll, "WEBPink", 0.99);
  m_visualization.set_color(&col_boundary, "Ivory", 0.99);
  const std::vector<std::string> cols{"WEBBlueDark", "WEBBlueLight", "WEBYellow", "WEBOrange",
                                      "WEBPink",     "WEBRed",       "WEBGreen"};

  // Put all buffers into a vector
  lanelet::Areas buffers;
  for (const auto & el : matches) {
    lanelet::Areas buf = el.buffers();
    buffers.insert(buffers.end(), buf.begin(), buf.end());
  }
  // Lanelets collapsed
  m_visualization.insert_marker_array(
    &msg,
    m_visualization.linestrings2marker_msg(ll_coll, "Lanelets_coll", col_coll, 0.2));
  m_visualization.insert_marker_array(
    &msg,
    m_visualization.tags2marker_msg(
      ll_coll, "", 1, "Lanelets_coll_tags", false, {"ll_id_forward", "ll_id_backward"}, true));
  m_visualization.insert_marker_array(
    &msg, m_visualization.linestring_arrows2marker_msg(
      ll_coll, "Lanelets_coll_arrows", "WEBPink"));
  // Reference Polylines
  m_visualization.insert_marker_array(
    &msg, m_visualization.match_pline2marker_msg(matches, cols, "Reference_polyline", 0.2));
  // Matched Polylines
  m_visualization.insert_marker_array(
    &msg, m_visualization.match_pline2marker_msg(matches, cols, "Target_polyline", 0.2));
  // Buffers
  m_visualization.insert_marker_array(
    &msg, m_visualization.areas2marker_msg(buffers, "WEBGreen", "buffers"));
  m_visualization.insert_marker_array(
    &msg, m_visualization.linestrings2marker_msg(
      m_visualization.areas2linestrings(buffers), "buffers_boundary", col_boundary, 0.1));
  // Connection between reference and match
  m_visualization.insert_marker_array(
    &msg, m_visualization.match2marker_msg(matches, "Blue", "Matches", 0.2));
}

void cmessages::pcd_map2msg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map, sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::toROSMsg(*pcd_map, msg);
  msg.header.frame_id = "map";
}