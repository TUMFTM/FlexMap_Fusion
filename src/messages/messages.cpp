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
  set_color(&col, color, 0.999);

  insert_marker_array(&msg, linestring2marker_msg(ls, ns, col, thickness));
}

/***************************************************************************************
 * Insert road lanelets and shoulder lanelets into marker array with visualization
 * of centerline, boundary, start, end
 ****************************************************************************************/
void cmessages::map_net2marker_msg(
  rclcpp::Node & node, lanelet::ConstLanelets & all, lanelet::ConstLanelets & regular,
  lanelet::ConstLanelets & shoulder, std::vector<lanelet::ConstLineString3d> & stop_lines,
  visualization_msgs::msg::MarkerArray & map_marker_msg)
{
  (void)all;
  bool viz_lanelet_centerline = node.get_parameter("viz_lanelet_centerline").as_bool();

  // Set colors of elements
  std_msgs::msg::ColorRGBA color_reg, color_shoulder, color_lanelet_id, color_lanelet_borders,
    color_shoulder_borders, color_stop_lines;
  set_color(&color_reg, "WEBBlueLight", 0.5);
  set_color(&color_shoulder, "Gray1", 0.999);
  set_color(&color_lanelet_id, "", 0.999);
  set_color(&color_lanelet_borders, "Gray3", 0.999);
  set_color(&color_shoulder_borders, "Gray3", 0.999);
  set_color(&color_stop_lines, "Black", 0.999);

  // Get markers and insert them into markerarray

  // Stop lines
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::lineStringsAsMarkerArray(
                       stop_lines, "stop_lines", color_stop_lines, 0.5));

  // Shoulder lanelets
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletDirectionAsMarkerArray(shoulder, "shoulder_"));
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::generateLaneletIdMarker(shoulder, color_lanelet_id));
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                       shoulder, color_shoulder_borders, viz_lanelet_centerline, "shoulder_"));
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletsAsTriangleMarkerArray(
                       "shoulder_road_lanelets", shoulder, color_shoulder));

  // Regular lanelets
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletDirectionAsMarkerArray(regular));
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                       regular, color_lanelet_borders, viz_lanelet_centerline));
  insert_marker_array(
    &map_marker_msg,
    lanelet::visualization::laneletsAsTriangleMarkerArray("regular_lanelets", regular, color_reg));
  // Conflated tags of lanelets
  insert_marker_array(
    &map_marker_msg, tags2marker_msg(
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
  lanelet::ConstLanelets & shoulder, std::vector<lanelet::ConstLineString3d> & stop_lines,
  const std::vector<std::pair<lanelet::Id, std::string>> & cols,
  visualization_msgs::msg::MarkerArray & map_marker_msg)
{
  (void)all;
  bool viz_lanelet_centerline = node.get_parameter("viz_lanelet_centerline").as_bool();

  // Set colors of elements
  std_msgs::msg::ColorRGBA color_reg, color_shoulder, color_lanelet_id, color_lanelet_borders,
    color_shoulder_borders, color_stop_lines;
  set_color(&color_reg, "WEBBlueLight", 0.5);
  set_color(&color_shoulder, "Gray1", 0.999);
  set_color(&color_lanelet_id, "", 0.999);
  set_color(&color_lanelet_borders, "Gray3", 0.999);
  set_color(&color_shoulder_borders, "Gray3", 0.999);
  set_color(&color_stop_lines, "Black", 0.999);

  // Get markers and insert them into markerarray

  // Stop lines
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::lineStringsAsMarkerArray(
                       stop_lines, "stop_lines", color_stop_lines, 0.5));

  // Shoulder lanelets
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletDirectionAsMarkerArray(shoulder, "shoulder_"));
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::generateLaneletIdMarker(shoulder, color_lanelet_id));
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                       shoulder, color_shoulder_borders, viz_lanelet_centerline, "shoulder_"));
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletsAsTriangleMarkerArray(
                       "shoulder_road_lanelets", shoulder, color_shoulder));

  // Regular lanelets
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletDirectionAsMarkerArray(regular));
  insert_marker_array(
    &map_marker_msg, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                       regular, color_lanelet_borders, viz_lanelet_centerline));
  insert_marker_array(
    &map_marker_msg, colored_lanelets2marker_msg("regular_lanelets", regular, cols));
  // Conflated tags of lanelets
  insert_marker_array(
    &map_marker_msg, tags2marker_msg(
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
  set_color(&color_motorways, "Orange", 0.99);
  set_color(&color_highways, "Orange", 0.99);
  set_color(&color_roads, "Orange", 0.99);

  // Visualize tags that specify the type of highway of the road and the number of lanes
  // if existing
  const std::string ns = "tags";
  const std::vector<std::string> tag{"highway", "lanes"};

  // Motorways
  insert_marker_array(
    &map_marker_msg,
    lanelet::visualization::lineStringsAsMarkerArray(motorways, "motorways", color_motorways, 0.5));
  insert_marker_array(&map_marker_msg, tags2marker_msg(motorways, "", 1, ns, false, tag, false));

  // Highways
  insert_marker_array(
    &map_marker_msg,
    lanelet::visualization::lineStringsAsMarkerArray(highways, "highways", color_highways, 0.5));
  insert_marker_array(&map_marker_msg, tags2marker_msg(highways, "", 1, ns, false, tag, false));

  // Roads
  insert_marker_array(
    &map_marker_msg,
    lanelet::visualization::lineStringsAsMarkerArray(roads, "roads", color_roads, 0.5));
  insert_marker_array(&map_marker_msg, tags2marker_msg(roads, "", 1, ns, false, tag, false));
}

/*****************************************************************
 * Insert geometries from rubber-sheeting into markerarray
 * => triangles, controlpoints
 ******************************************************************/
void cmessages::rs2marker_msg(
  const lanelet::Areas & areas, const std::vector<s_control_point> & cps,
  visualization_msgs::msg::MarkerArray & msg)
{
  insert_marker_array(&msg, areas2marker_msg(areas, "Ivory", "Triangles", 1));
  insert_marker_array(&msg, controlpoints2marker_msg(cps, "WEBOrange", "controlpoint", 1.5));
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
  set_color(&col_coll, "WEBPink", 0.99);
  set_color(&col_boundary, "Ivory", 0.99);
  const std::vector<std::string> cols{"WEBBlueDark", "WEBBlueLight", "WEBYellow", "WEBOrange",
                                      "WEBPink",     "WEBRed",       "WEBGreen"};

  // Put all buffers into a vector
  lanelet::Areas buffers;
  for (const auto & el : matches) {
    lanelet::Areas buf = el.buffers();
    buffers.insert(buffers.end(), buf.begin(), buf.end());
  }
  // Lanelets collapsed
  insert_marker_array(
    &msg,
    lanelet::visualization::lineStringsAsMarkerArray(ll_coll, "Lanelets_coll", col_coll, 0.2));
  insert_marker_array(
    &msg,
    tags2marker_msg(
      ll_coll, "", 1, "Lanelets_coll_tags", false, {"ll_id_forward", "ll_id_backward"}, true));
  insert_marker_array(
    &msg, linestring_arrows2marker_msg(ll_coll, "Lanelets_coll_arrows", "WEBPink"));
  // Reference Polylines
  insert_marker_array(&msg, match_pline2marker_msg(matches, cols, "Reference_polyline", 0.2));
  // Matched Polylines
  insert_marker_array(&msg, match_pline2marker_msg(matches, cols, "Target_polyline", 0.2));
  // Buffers
  insert_marker_array(&msg, areas2marker_msg(buffers, "WEBGreen", "buffers"));
  insert_marker_array(
    &msg, lanelet::visualization::lineStringsAsMarkerArray(
            areas2linestrings(buffers), "buffers_boundary", col_boundary, 0.1));
  // Connection between reference and match
  insert_marker_array(&msg, match2marker_msg(matches, "Blue", "Matches", 0.2));
}

/*****************/
/*private methods*/
/*****************/

/*****************************************************************************
 * Convert areas (rubber-sheet triangles) to markerarray (new namespace for
 * each triangle)
 ******************************************************************************/
visualization_msgs::msg::MarkerArray cmessages::areas2marker_msg(
  const lanelet::Areas & areas, const std::string color, const std::string ns,
  const float thickness)
{
  visualization_msgs::msg::MarkerArray msg;
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);

  int i = 1;
  std::string ns_ind = ns;
  for (auto & ar : areas) {
    ns_ind = ns + "_" + std::to_string(i);
    lanelet::ConstLineStrings3d ls = ar.outerBound();
    insert_marker_array(
      &msg, lanelet::visualization::lineStringsAsMarkerArray(ls, ns_ind, col, thickness));
    i += 1;
  }
  return msg;
}

/*********************************************************************************
 * Convert controlpoints to markerarray (linestring from source to target point)
 **********************************************************************************/
visualization_msgs::msg::MarkerArray cmessages::controlpoints2marker_msg(
  const std::vector<s_control_point> & cps, const std::string color, const std::string ns,
  const float thickness)
{
  visualization_msgs::msg::MarkerArray msg;
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);

  int i = 1;
  for (auto & cpt : cps) {
    std::string ns_ind = ns + "_" + std::to_string(i);
    lanelet::LineString3d ls(
      lanelet::utils::getId(), {cpt.get_source_point(), cpt.get_target_point()});
    lanelet::ConstLineString3d ls_const = ls;
    insert_marker_array(&msg, linestring2marker_msg(ls, ns_ind, col, thickness));
    i += 1;
  }
  return msg;
}

/*********************************************************************************
 * Convert lanelets into colored triangle based on color code from conflation
 **********************************************************************************/
visualization_msgs::msg::MarkerArray cmessages::colored_lanelets2marker_msg(
  const std::string & ns_base, const lanelet::ConstLanelets & lls,
  const std::vector<std::pair<lanelet::Id, std::string>> & cols)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  if (lls.empty()) {
    return marker_array;
  }

  marker.header.frame_id = "map";
  marker.ns = ns_base;
  marker.header.stamp = rclcpp::Time();
  marker.frame_locked = false;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker.pose.position.x = 0.0;  // p.x();
  marker.pose.position.y = 0.0;  // p.y();
  marker.pose.position.z = 0.0;  // p.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.999;

  int indLl = 0;
  for (const auto & ll : lls) {
    std::vector<geometry_msgs::msg::Polygon> triangles;
    lanelet::visualization::lanelet2Triangle(ll, &triangles);
    std_msgs::msg::ColorRGBA c;
    std::string col_code = id2col(cols, ll.id());
    set_color(&c, col_code, 0.5);

    for (const auto & tri : triangles) {
      geometry_msgs::msg::Point tri0[3];

      for (int i = 0; i < 3; i++) {
        lanelet::utils::conversion::toGeomMsgPt(tri.points[i], &tri0[i]);

        marker.points.push_back(tri0[i]);
        marker.colors.push_back(c);
      }
    }
    ++indLl;
  }
  if (!marker.points.empty()) {
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

/*************************************************************************************
 * Convert areas to markerarray (visualization of the surface but with transparency)
 **************************************************************************************/
visualization_msgs::msg::MarkerArray cmessages::areas2marker_msg(
  const lanelet::Areas & areas, const std::string & color, const std::string & ns)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.2);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  if (areas.empty()) {
    return marker_array;
  }

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time();
  marker.frame_locked = false;
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker.pose.position.x = 0.0;  // p.x();
  marker.pose.position.y = 0.0;  // p.y();
  marker.pose.position.z = 0.0;  // p.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.999;

  for (const auto & ar : areas) {
    std::vector<geometry_msgs::msg::Polygon> triangles;
    area2triangle(ar, &triangles);

    for (const auto & tri : triangles) {
      geometry_msgs::msg::Point tri0[3];

      for (int i = 0; i < 3; i++) {
        lanelet::utils::conversion::toGeomMsgPt(tri.points[i], &tri0[i]);

        marker.points.push_back(tri0[i]);
        marker.colors.push_back(col);
      }
    }
  }
  if (!marker.points.empty()) {
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

/***************************************************************************************
 * Convert reference or target polyline of match to markerarray with changing color
 ****************************************************************************************/
visualization_msgs::msg::MarkerArray cmessages::match_pline2marker_msg(
  const std::vector<s_match> & matches, const std::vector<std::string> & colors,
  const std::string ns, const float thickness)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, colors[0], 0.99);

  visualization_msgs::msg::MarkerArray marker_array;
  if (matches.empty()) {
    return marker_array;
  }
  std::unordered_set<lanelet::Id> added;
  visualization_msgs::msg::Marker plineMarker;
  lanelet::visualization::initArrowsMarker(&plineMarker, "map", ns, col);
  size_t i = 0;
  for (const auto & matchEl : matches) {
    set_color(&col, colors[i], 0.99);
    ++i;
    if (i >= colors.size()) {
      i = 0;
    }
    lanelet::ConstLineStrings3d pline;
    if (ns.find("Ref") != std::string::npos) {
      pline = to_const(matchEl.ref_pline());
    } else if (ns.find("Target") != std::string::npos) {
      pline = to_const(matchEl.target_pline());
    } else {
      std::cerr << __FUNCTION__ << ": specify Ref or Target in the namespace!" << std::endl;
    }
    if (!pline.empty()) {
      for (const auto & ls : pline) {
        if (added.find(ls.id()) == added.end()) {
          lanelet::visualization::pushArrowsMarker(&plineMarker, ls, col);
          lanelet::visualization::pushLineStringMarker(&plineMarker, ls, col, thickness);
          added.insert(ls.id());
        }
      }
    }
  }
  marker_array.markers.push_back(plineMarker);
  return marker_array;
}

/***********************************************************************************
 * Convert connection line between reference and target of a match to markerarray
 ************************************************************************************/
visualization_msgs::msg::MarkerArray cmessages::match2marker_msg(
  const std::vector<s_match> & matches, const std::string color, const std::string ns,
  const float thickness)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.99);

  visualization_msgs::msg::MarkerArray marker_array;
  if (matches.empty()) {
    return marker_array;
  }
  std::unordered_set<lanelet::Id> added;
  visualization_msgs::msg::Marker plineMarker;
  lanelet::visualization::initArrowsMarker(&plineMarker, "map", ns, col);

  for (const auto & matchEl : matches) {
    lanelet::ConstLineStrings3d matchConnections = matchEl.match_conn();
    if (!matchConnections.empty()) {
      for (const auto & ls : matchConnections) {
        if (added.find(ls.id()) == added.end()) {
          lanelet::visualization::pushArrowsMarker(&plineMarker, ls, col);
          lanelet::visualization::pushLineStringMarker(&plineMarker, ls, col, thickness);
          added.insert(ls.id());
        }
      }
    }
  }
  marker_array.markers.push_back(plineMarker);
  return marker_array;
}

/***************************************************************************
 * Convert area to to a triangle for later conversion to markerarray
 ****************************************************************************/
void cmessages::area2triangle(
  const lanelet::Area & ar, std::vector<geometry_msgs::msg::Polygon> * triangles)
{
  if (triangles == nullptr) {
    std::cerr << __FUNCTION__ << ": triangles is null pointer!" << std::endl;
    return;
  }

  triangles->clear();
  geometry_msgs::msg::Polygon ar_poly;
  area2polygon(ar, &ar_poly);
  lanelet::visualization::polygon2Triangle(ar_poly, triangles);
}

/*****************************************************************************
 * Convert an area to a polygon for later conversion to a triangle
 ******************************************************************************/
void cmessages::area2polygon(const lanelet::Area & ar, geometry_msgs::msg::Polygon * polygon)
{
  if (polygon == nullptr) {
    std::cerr << "\033[31m" << __FUNCTION__ << ": polygon is null pointer!\033[0m" << std::endl;
    return;
  }

  lanelet::CompoundPolygon3d ar_poly = ar.outerBoundPolygon();

  polygon->points.clear();
  polygon->points.reserve(ar_poly.size());

  for (const auto & pt : ar_poly) {
    geometry_msgs::msg::Point32 pt32;
    lanelet::utils::conversion::toGeomMsgPt32(pt.basicPoint(), &pt32);
    polygon->points.push_back(pt32);
  }
}

/**********************************************************************************
 * Set color of for a markerarray specified by string defining the color name
 * from "color.hpp"
 ***********************************************************************************/
void cmessages::set_color(std_msgs::msg::ColorRGBA * color, const std::string & name, double a)
{
  s_TUMcolor cl(name);
  color->r = cl.r;
  color->g = cl.g;
  color->b = cl.b;
  color->a = a;
}

/**************************************************************
 * Insert marker array at the end of another markerarray
 ***************************************************************/
void cmessages::insert_marker_array(
  visualization_msgs::msg::MarkerArray * arr_src,
  const visualization_msgs::msg::MarkerArray & arr_in)
{
  arr_src->markers.insert(arr_src->markers.end(), arr_in.markers.begin(), arr_in.markers.end());
}

/**********************************************
 * Convert single linestring to markerarray
 ***********************************************/
visualization_msgs::msg::MarkerArray cmessages::linestring2marker_msg(
  const lanelet::ConstLineString3d & ls, const std::string & ns, const std_msgs::msg::ColorRGBA & c,
  const float lss)
{
  visualization_msgs::msg::MarkerArray marker_array;
  if (ls.empty()) {
    return marker_array;
  }
  visualization_msgs::msg::Marker marker;
  lanelet::visualization::initLineStringMarker(&marker, "map", ns, c);

  lanelet::visualization::pushLineStringMarker(&marker, ls, c, lss);
  marker_array.markers.push_back(marker);
  return marker_array;
}

/******************************************************
 * Convert linestring into arrows at its points
 *******************************************************/
visualization_msgs::msg::MarkerArray cmessages::linestring_arrows2marker_msg(
  const lanelet::ConstLineStrings3d & lss, const std::string & ns, const std::string & color)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);

  visualization_msgs::msg::MarkerArray marker_array;
  if (lss.empty()) {
    return marker_array;
  }
  std::unordered_set<lanelet::Id> added;
  visualization_msgs::msg::Marker marker;
  lanelet::visualization::initArrowsMarker(&marker, "map", ns, col);
  for (const auto & ls : lss) {
    if (added.find(ls.id()) == added.end()) {
      lanelet::visualization::pushArrowsMarker(&marker, ls, col);
      added.insert(ls.id());
    }
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

/********************************************************
 * Convert tags lanelets into markerarray
 * => keys and if they are indexed to be specified
 *********************************************************/
visualization_msgs::msg::MarkerArray cmessages::tags2marker_msg(
  const lanelet::ConstLanelets & lanelets, const std::string & color, const double scale,
  const std::string & ns, const bool & id, const std::vector<std::string> & key, const bool & index)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);

  visualization_msgs::msg::MarkerArray markers;
  for (const auto & ll : lanelets) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = static_cast<int32_t>(ll.id());
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const auto centerline = ll.centerline();
    const size_t target_pos_index = centerline.size() / 2;
    const auto & target_pos = centerline[target_pos_index];
    marker.pose.position.x = target_pos.x();
    marker.pose.position.y = target_pos.y();
    marker.pose.position.z = target_pos.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color = col;
    marker.scale.z = scale;
    marker.frame_locked = false;
    std::string text = ls_attr_key(ll, key, index);
    if (id) {
      text = "lanelet_id -> " + std::to_string(ll.id()) + "\n" + text;
    }
    marker.text = text;
    markers.markers.push_back(marker);
  }
  return markers;
}

/********************************************************
 * Convert tags linestrings into markerarray
 * => keys and if they are indexed to be specified
 *********************************************************/
visualization_msgs::msg::MarkerArray cmessages::tags2marker_msg(
  const lanelet::ConstLineStrings3d & linestrings, const std::string & color, const double scale,
  const std::string & ns, const bool & id, const std::vector<std::string> & key, const bool & index)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);

  visualization_msgs::msg::MarkerArray markers;
  for (const auto & ls : linestrings) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = static_cast<int32_t>(ls.id());
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const size_t target_pos_index = ls.size() / 2;
    const auto & target_pos = ls[target_pos_index];
    marker.pose.position.x = target_pos.x();
    marker.pose.position.y = target_pos.y();
    marker.pose.position.z = target_pos.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color = col;
    marker.scale.z = scale;
    marker.frame_locked = false;
    std::string text = ls_attr_key(ls, key, index);
    if (id) {
      text = "linestring_id -> " + std::to_string(ls.id()) + "\n" + text;
    }
    marker.text = text;
    markers.markers.push_back(marker);
  }
  return markers;
}

/***********************************************************************************
 * Extract (indexed) keys with corresponding value of a lanelet into a string
 ************************************************************************************/
std::string cmessages::ls_attr_key(
  const lanelet::ConstLanelet & ll, const std::vector<std::string> & key, const bool & index)
{
  std::string text;
  if (index) {
    for (const std::string it : key) {
      int i = 1;
      std::string key_ind = it + "_" + std::to_string(i);
      while (ll.hasAttribute(key_ind)) {
        text += key_ind + " -> " + ll.attribute(key_ind).value() + "\n";
        ++i;
        key_ind = it + "_" + std::to_string(i);
      }
    }
  } else {
    for (const std::string it : key) {
      const std::string val = ll.attributeOr(it, "no_tag");
      text += it + " -> " + val + "\n";
    }
  }
  return text;
}

/***********************************************************************************
 * Extract (indexed) keys with corresponding value of a linestring into a string
 ************************************************************************************/
std::string cmessages::ls_attr_key(
  const lanelet::ConstLineString3d & ls, const std::vector<std::string> & key, const bool & index)
{
  std::string text;
  if (index) {
    for (const std::string it : key) {
      int i = 1;
      std::string key_ind = it + "_" + std::to_string(i);
      while (ls.hasAttribute(key_ind)) {
        text += key_ind + " -> " + ls.attribute(key_ind).value() + "\n";
        ++i;
        key_ind = it + "_" + std::to_string(i);
      }
    }
  } else {
    for (const std::string it : key) {
      const std::string val = ls.attributeOr(it, "no_tag");
      text += it + " -> " + val + "\n";
    }
  }
  return text;
}

/************************************************************************
 * Convert areas into vector of the linestrings defining its bounds
 *************************************************************************/
lanelet::ConstLineStrings3d cmessages::areas2linestrings(const lanelet::Areas & areas)
{
  lanelet::ConstLineStrings3d ls_ar;
  for (const auto & ar : areas) {
    lanelet::ConstLineStrings3d outer = ar.outerBound();
    ls_ar.insert(ls_ar.end(), outer.begin(), outer.end());
  }
  return ls_ar;
}

/********************************************************************
 * Convert vector of linestrings to vector of constlinestrings
 *********************************************************************/
lanelet::ConstLineStrings3d cmessages::to_const(const lanelet::LineStrings3d & lss)
{
  lanelet::ConstLineStrings3d out;
  for (const auto & ls : lss) {
    out.push_back(ls);
  }
  return out;
}

/**********************************************************************
 * Get color code of a lanelet from conflation based on its id
 ***********************************************************************/
std::string cmessages::id2col(
  const std::vector<std::pair<lanelet::Id, std::string>> & cols, const lanelet::Id & id)
{
  for (const auto & el : cols) {
    if (el.first == id) {
      return el.second;
    }
  }
  return "";
}
