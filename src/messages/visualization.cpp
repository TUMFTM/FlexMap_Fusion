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
// Date: 13.10.2023
// ========================================== //
//
//
#include "visualization.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

/**************/
/*Constructors*/
/**************/

cvisualization::cvisualization()
{
}

/****************/
/*public methods*/
/****************/

/*********************************************************************************
  * Convert lanelet boundaries to marker arrays
  **********************************************************************************/
visualization_msgs::msg::MarkerArray cvisualization::lanelets_boundary_as_marker_array(
  const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA & c,
  const bool viz_centerline, const std::string & additional_namespace)
{
  const float lss = 0.1;  // line string size
  const float lss_center = static_cast<float>(std::max(lss * 0.1, 0.02));

  lanelet::Ids added;
  visualization_msgs::msg::Marker left_line_strip;
  visualization_msgs::msg::Marker right_line_strip;
  visualization_msgs::msg::Marker start_bound_line_strip;
  visualization_msgs::msg::Marker center_line_strip;
  visualization_msgs::msg::Marker center_arrows;
  init_linestring_marker(
    &left_line_strip, "map", additional_namespace + "left_lane_bound", c);
  init_linestring_marker(
    &right_line_strip, "map", additional_namespace + "right_lane_bound", c);
  init_linestring_marker(
    &start_bound_line_strip, "map", additional_namespace + "lane_start_bound", c);
  init_linestring_marker(
    &center_line_strip, "map", additional_namespace + "center_lane_line", c);
  init_arrows_marker(
    &center_arrows, "map", additional_namespace + "center_line_arrows", c);

  for (const auto & lll : lanelets) {
    lanelet::ConstLineString3d left_ls = lll.leftBound();
    lanelet::ConstLineString3d right_ls = lll.rightBound();
    lanelet::ConstLineString3d center_ls = lll.centerline();
    lanelet::LineString3d start_bound_ls(lanelet::utils::getId());
    start_bound_ls.push_back(lanelet::Point3d(
      lanelet::utils::getId(), left_ls.front().x(), left_ls.front().y(), left_ls.front().z()));
    start_bound_ls.push_back(lanelet::Point3d(
      lanelet::utils::getId(), right_ls.front().x(), right_ls.front().y(), right_ls.front().z()));

    if (std::find(added.begin(), added.end(), left_ls.id()) == added.end()) {
      push_linestring_marker(&left_line_strip, left_ls, c, lss);
      added.push_back(left_ls.id());
    }
    if (std::find(added.begin(), added.end(), right_ls.id()) == added.end()) {
      push_linestring_marker(&right_line_strip, right_ls, c, lss);
      added.push_back(right_ls.id());
    }
    if (std::find(added.begin(), added.end(), start_bound_ls.id()) == added.end()) {
      push_linestring_marker(&start_bound_line_strip, start_bound_ls, c, lss);
      added.push_back(start_bound_ls.id());
    }
    if (viz_centerline && std::find(added.begin(), added.end(), center_ls.id()) == added.end()) {
      push_linestring_marker(&center_line_strip, center_ls, c, lss_center);
      push_arrows_marker(&center_arrows, center_ls, c);
      added.push_back(center_ls.id());
    }
  }

  visualization_msgs::msg::MarkerArray marker_array;
  if (!left_line_strip.points.empty()) {
    marker_array.markers.push_back(left_line_strip);
  }
  if (!right_line_strip.points.empty()) {
    marker_array.markers.push_back(right_line_strip);
  }
  if (!center_line_strip.points.empty()) {
    marker_array.markers.push_back(center_line_strip);
  }
  if (!start_bound_line_strip.points.empty()) {
    marker_array.markers.push_back(start_bound_line_strip);
  }
  if (!center_arrows.points.empty()) {
    marker_array.markers.push_back(center_arrows);
  }
  return marker_array;
}

/*********************************************************************************
  * Convert lanelet ids to marker arrays
  **********************************************************************************/
visualization_msgs::msg::MarkerArray cvisualization::generate_lanelet_id_marker(
  const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA & c,
  const double scale)
{
  visualization_msgs::msg::MarkerArray markers;
  for (const auto & ll : lanelets) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "lanelet_id";
    marker.id = static_cast<int32_t>(ll.id());
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const auto centerline = ll.centerline();
    const size_t target_position_index = centerline.size() / 2;
    const auto & target_position = centerline[target_position_index];
    marker.pose.position.x = target_position.x();
    marker.pose.position.y = target_position.y();
    marker.pose.position.z = target_position.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color = c;
    marker.scale.z = scale;
    marker.frame_locked = false;
    marker.text = std::to_string(ll.id());
    markers.markers.push_back(marker);
  }
  return markers;
}

/*********************************************************************************
  * Convert lanelets to triangle marker arrays
  **********************************************************************************/
visualization_msgs::msg::MarkerArray cvisualization::lanelets2triangle_marker_array(
  const std::string & ns, const lanelet::ConstLanelets & lanelets,
  const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  if (lanelets.empty()) {
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

  for (const auto & ll : lanelets) {
    std::vector<geometry_msgs::msg::Polygon> triangles;
    lanelet2triangle(ll, &triangles);

    for (const auto & tri : triangles) {
      geometry_msgs::msg::Point tri0[3];

      for (int i = 0; i < 3; i++) {
        m_message_conversion.toGeomMsgPt(tri.points[i], &tri0[i]);

        marker.points.push_back(tri0[i]);
        marker.colors.push_back(c);
      }
    }
  }
  if (!marker.points.empty()) {
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

/*********************************************************************************
 * Convert lanelets into colored triangle based on color code from conflation
 **********************************************************************************/
visualization_msgs::msg::MarkerArray cvisualization::colored_lanelets2marker_msg(
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
    lanelet2triangle(ll, &triangles);
    std_msgs::msg::ColorRGBA c;
    std::string col_code = id2col(cols, ll.id());
    set_color(&c, col_code, 0.5);

    for (const auto & tri : triangles) {
      geometry_msgs::msg::Point tri0[3];

      for (int i = 0; i < 3; i++) {
        m_message_conversion.toGeomMsgPt(tri.points[i], &tri0[i]);

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
visualization_msgs::msg::MarkerArray cvisualization::areas2marker_msg(
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
        m_message_conversion.toGeomMsgPt(tri.points[i], &tri0[i]);

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

/*****************************************************************************
 * Convert areas (rubber-sheet triangles) to markerarray (new namespace for
 * each triangle)
 ******************************************************************************/
visualization_msgs::msg::MarkerArray cvisualization::areas2marker_msg(
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
      &msg, linestrings2marker_msg(ls, ns_ind, col, thickness));
    i += 1;
  }
  return msg;
}

/**********************************************
 * Convert single linestring to markerarray
 ***********************************************/
visualization_msgs::msg::MarkerArray cvisualization::linestring2marker_msg(
  const lanelet::ConstLineString3d & ls, const std::string & ns, const std_msgs::msg::ColorRGBA & c,
  const float lss)
{
  visualization_msgs::msg::MarkerArray marker_array;
  if (ls.empty()) {
    return marker_array;
  }
  visualization_msgs::msg::Marker marker;
  init_linestring_marker(&marker, "map", ns, c);

  push_linestring_marker(&marker, ls, c, lss);
  marker_array.markers.push_back(marker);
  return marker_array;
}

/**********************************************
 * Convert multiple linestrings to markerarray
 ***********************************************/
visualization_msgs::msg::MarkerArray cvisualization::linestrings2marker_msg(
  const lanelet::ConstLineStrings3d & linestrings, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c, const float lss)
{
  visualization_msgs::msg::MarkerArray marker_array;
  if (linestrings.empty()) {
    return marker_array;
  }
  lanelet::Ids added;
  visualization_msgs::msg::Marker marker;
  init_linestring_marker(&marker, "map", ns, c);

  for (const auto & ls : linestrings) {
    if (std::find(added.begin(), added.end(), ls.id()) == added.end()) {
      push_linestring_marker(&marker, ls, c, lss);
      added.push_back(ls.id());
    }
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

/**************************************************************
 * Insert marker array at the end of another markerarray
 ***************************************************************/
void cvisualization::insert_marker_array(
  visualization_msgs::msg::MarkerArray * arr_src,
  const visualization_msgs::msg::MarkerArray & arr_in)
{
  arr_src->markers.insert(arr_src->markers.end(), arr_in.markers.begin(), arr_in.markers.end());
}

/**********************************************************************************
 * Set color of for a markerarray specified by string defining the color name
 * from "color.hpp"
 ***********************************************************************************/
void cvisualization::set_color(
  std_msgs::msg::ColorRGBA * color, const std::string & name, double a)
{
  s_TUMcolor cl(name);
  color->r = cl.r;
  color->g = cl.g;
  color->b = cl.b;
  color->a = a;
}

/********************************************************
 * Convert tags lanelets into markerarray
 * => keys and if they are indexed to be specified
 *********************************************************/
visualization_msgs::msg::MarkerArray cvisualization::tags2marker_msg(
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
visualization_msgs::msg::MarkerArray cvisualization::tags2marker_msg(
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

/************************************************************************
 * Convert areas into vector of the linestrings defining its bounds
 *************************************************************************/
lanelet::ConstLineStrings3d cvisualization::areas2linestrings(const lanelet::Areas & areas)
{
  lanelet::ConstLineStrings3d ls_ar;
  for (const auto & ar : areas) {
    lanelet::ConstLineStrings3d outer = ar.outerBound();
    ls_ar.insert(ls_ar.end(), outer.begin(), outer.end());
  }
  return ls_ar;
}

/******************************************************
 * Convert linestring into arrows at its points
 *******************************************************/
visualization_msgs::msg::MarkerArray cvisualization::linestring_arrows2marker_msg(
  const lanelet::ConstLineStrings3d & lss, const std::string & ns, const std::string & color)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);

  visualization_msgs::msg::MarkerArray marker_array;
  if (lss.empty()) {
    return marker_array;
  }
  lanelet::Ids added;
  visualization_msgs::msg::Marker marker;
  init_arrows_marker(&marker, "map", ns, col);
  for (const auto & ls : lss) {
    if (std::find(added.begin(), added.end(), ls.id()) == added.end()) {
      push_arrows_marker(&marker, ls, col);
      added.push_back(ls.id());
    }
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}

/*********************************************************************************
 * Convert controlpoints to markerarray (linestring from source to target point)
 **********************************************************************************/
visualization_msgs::msg::MarkerArray cvisualization::controlpoints2marker_msg(
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
    insert_marker_array(
      &msg, linestring2marker_msg(ls, ns_ind, col, thickness));
    i += 1;
  }
  return msg;
}

/***************************************************************************************
 * Convert reference or target polyline of match to markerarray with changing color
 ****************************************************************************************/
visualization_msgs::msg::MarkerArray cvisualization::match_pline2marker_msg(
  const std::vector<s_match> & matches, const std::vector<std::string> & colors,
  const std::string ns, const float thickness)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, colors[0], 0.99);

  visualization_msgs::msg::MarkerArray marker_array;
  if (matches.empty()) {
    return marker_array;
  }
  lanelet::Ids added;
  visualization_msgs::msg::Marker plineMarker;
  init_arrows_marker(&plineMarker, "map", ns, col);
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
        if (std::find(added.begin(), added.end(), ls.id()) == added.end()) {
          push_arrows_marker(&plineMarker, ls, col);
          push_linestring_marker(&plineMarker, ls, col, thickness);
          added.push_back(ls.id());
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
visualization_msgs::msg::MarkerArray cvisualization::match2marker_msg(
  const std::vector<s_match> & matches, const std::string color, const std::string ns,
  const float thickness)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.99);

  visualization_msgs::msg::MarkerArray marker_array;
  if (matches.empty()) {
    return marker_array;
  }
  lanelet::Ids added;
  visualization_msgs::msg::Marker plineMarker;
  init_arrows_marker(&plineMarker, "map", ns, col);

  for (const auto & matchEl : matches) {
    lanelet::ConstLineStrings3d matchConnections = matchEl.match_conn();
    if (!matchConnections.empty()) {
      for (const auto & ls : matchConnections) {
        if (std::find(added.begin(), added.end(), ls.id()) == added.end()) {
          push_arrows_marker(&plineMarker, ls, col);
          push_linestring_marker(&plineMarker, ls, col, thickness);
          added.push_back(ls.id());
        }
      }
    }
  }
  marker_array.markers.push_back(plineMarker);
  return marker_array;
}

/***********************************************************************************
  * Initialize linestring marker
  ************************************************************************************/
void cvisualization::init_linestring_marker(
  visualization_msgs::msg::Marker * marker, const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c)
{
  if (marker == nullptr) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lanelet2_extension.visualization"),
      __FUNCTION__ << ": marker is null pointer!");
    return;
  }

  marker->header.frame_id = frame_id;
  marker->header.stamp = rclcpp::Time();
  marker->frame_locked = false;
  marker->ns = ns;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

  marker->id = 0;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  marker->color = c;
}

/***********************************************************************************
  * Push linestring marker
  ************************************************************************************/
void cvisualization::push_linestring_marker(
  visualization_msgs::msg::Marker * marker, const lanelet::ConstLineString3d & ls,
  const std_msgs::msg::ColorRGBA & c, const float lss)
{
  if (marker == nullptr) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lanelet2_extension.visualization"),
      __FUNCTION__ << ": marker is null pointer!");
    return;
  }

  // fill out lane line
  if (ls.size() < 2) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lanelet2_extension.visualization"),
      __FUNCTION__ << ": marker line size is 1 or 0!");
    return;
  }
  for (auto i = ls.begin(); i + 1 != ls.end(); i++) {
    geometry_msgs::msg::Point p;
    const auto heading =
      static_cast<float>(std::atan2((*(i + 1)).y() - (*i).y(), (*(i + 1)).x() - (*i).x()));

    const auto x_offset = static_cast<float>(lss * 0.5 * std::sin(heading));
    const auto y_offset = static_cast<float>(lss * 0.5 * std::cos(heading));

    p.x = (*i).x() + x_offset;
    p.y = (*i).y() - y_offset;
    p.z = (*i).z();
    marker->points.push_back(p);
    p.x = (*i).x() - x_offset;
    p.y = (*i).y() + y_offset;
    p.z = (*i).z();
    marker->points.push_back(p);
    p.x = (*(i + 1)).x() + x_offset;
    p.y = (*(i + 1)).y() - y_offset;
    p.z = (*(i + 1)).z();
    marker->points.push_back(p);
    marker->colors.push_back(c);
    p.x = (*(i + 1)).x() - x_offset;
    p.y = (*(i + 1)).y() + y_offset;
    p.z = (*(i + 1)).z();
    marker->points.push_back(p);
    p.x = (*(i + 1)).x() + x_offset;
    p.y = (*(i + 1)).y() - y_offset;
    p.z = (*(i + 1)).z();
    marker->points.push_back(p);
    p.x = (*i).x() - x_offset;
    p.y = (*i).y() + y_offset;
    p.z = (*i).z();
    marker->points.push_back(p);
    marker->colors.push_back(c);
  }
}

/***********************************************************************************
  * Initialize arrow marker
  ************************************************************************************/
void cvisualization::init_arrows_marker(
  visualization_msgs::msg::Marker * marker, const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c)
{
  if (marker == nullptr) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lanelet2_extension.visualization"),
      __FUNCTION__ << ": marker is null pointer!");
    return;
  }

  marker->header.frame_id = frame_id;
  marker->header.stamp = rclcpp::Time();
  marker->frame_locked = false;
  marker->ns = ns;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

  marker->id = 0;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  marker->color = c;
}

/***********************************************************************************
  * Push arrow marker
  ************************************************************************************/
void cvisualization::push_arrows_marker(
  visualization_msgs::msg::Marker * marker, const lanelet::ConstLineString3d & ls,
  const std_msgs::msg::ColorRGBA & c)
{
  if (marker == nullptr) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lanelet2_extension.visualization"),
      __FUNCTION__ << ": marker is null pointer!");
    return;
  }

  // fill out lane line
  if (ls.size() < 2) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lanelet2_extension.visualization"),
      __FUNCTION__ << ": marker line size is 1 or 0!");
    return;
  }
  for (auto i = ls.begin(); i + 1 != ls.end(); i++) {
    const auto heading =
      static_cast<float>(std::atan2((*(i + 1)).y() - (*i).y(), (*(i + 1)).x() - (*i).x()));

    const float sin_offset = std::sin(heading);
    const float cos_offset = std::cos(heading);
    const double width = 0.3;
    const double height = 1.0;

    geometry_msgs::msg::Point p;
    p.x = (*i).x() + sin_offset * width;
    p.y = (*i).y() - cos_offset * width;
    p.z = (*i).z();
    marker->points.push_back(p);
    p.x = (*i).x() - sin_offset * width;
    p.y = (*i).y() + cos_offset * width;
    p.z = (*i).z();
    marker->points.push_back(p);
    p.x = (*i).x() + cos_offset * height;
    p.y = (*i).y() + sin_offset * height;
    p.z = (*i).z();
    marker->points.push_back(p);
    marker->colors.push_back(c);
  }
}

/**********************************************************************
 * Get color code of a lanelet from conflation based on its id
 ***********************************************************************/
std::string cvisualization::id2col(
  const std::vector<std::pair<lanelet::Id, std::string>> & cols, const lanelet::Id & id)
{
  for (const auto & el : cols) {
    if (el.first == id) {
      return el.second;
    }
  }
  return "";
}

/***************************************************************************
 * Convert lanelet to to a triangle for later conversion to markerarray
 ****************************************************************************/
void cvisualization::lanelet2triangle(
  const lanelet::ConstLanelet & ll, std::vector<geometry_msgs::msg::Polygon> * triangles)
{
  if (triangles == nullptr) {
    std::cerr << __FUNCTION__ << ": triangles is null pointer!" << std::endl;
    return;
  }

  triangles->clear();
  geometry_msgs::msg::Polygon ll_poly;
  lanelet2polygon(ll, &ll_poly);
  polygon2triangle(ll_poly, triangles);
}

/***************************************************************************
 * Convert area to to a triangle for later conversion to markerarray
 ****************************************************************************/
void cvisualization::area2triangle(
  const lanelet::Area & ar, std::vector<geometry_msgs::msg::Polygon> * triangles)
{
  if (triangles == nullptr) {
    std::cerr << __FUNCTION__ << ": triangles is null pointer!" << std::endl;
    return;
  }

  triangles->clear();
  geometry_msgs::msg::Polygon ar_poly;
  area2polygon(ar, &ar_poly);
  polygon2triangle(ar_poly, triangles);
}

/***************************************************************************
  * Convert lanelet to polygon
  ****************************************************************************/
void cvisualization::lanelet2polygon(
  const lanelet::ConstLanelet & ll, geometry_msgs::msg::Polygon * polygon)
{
  if (polygon == nullptr) {
    std::cerr << __FUNCTION__ << ": polygon is null pointer!" << std::endl;
    return;
  }

  lanelet::CompoundPolygon3d ll_poly = ll.polygon3d();

  polygon->points.clear();
  polygon->points.reserve(ll_poly.size());

  for (const auto & pt : ll_poly) {
    geometry_msgs::msg::Point32 pt32;
    m_message_conversion.toGeomMsgPt32(pt.basicPoint(), &pt32);
    polygon->points.push_back(pt32);
  }
}

/*****************************************************************************
 * Convert an area to a polygon for later conversion to a triangle
 ******************************************************************************/
void cvisualization::area2polygon(const lanelet::Area & ar, geometry_msgs::msg::Polygon * polygon)
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
    m_message_conversion.toGeomMsgPt32(pt.basicPoint(), &pt32);
    polygon->points.push_back(pt32);
  }
}

/*****************************************************************************
  * Convert an polygon to triangle for later creation of marker array
  ******************************************************************************/
void cvisualization::polygon2triangle(
  const geometry_msgs::msg::Polygon & polygon, std::vector<geometry_msgs::msg::Polygon> * triangles)
{
  geometry_msgs::msg::Polygon poly = polygon;
  if (!is_clock_wise(poly)) {
    std::reverse(poly.points.begin(), poly.points.end());
  }

  // ear clipping: find smallest internal angle in polygon
  int N = static_cast<int>(poly.points.size());

  // array of angles for each vertex
  std::vector<bool> b_is_acute_angle;
  b_is_acute_angle.assign(N, false);
  for (int i = 0; i < N; i++) {
    geometry_msgs::msg::Point32 p0;
    geometry_msgs::msg::Point32 p1;
    geometry_msgs::msg::Point32 p2;

    adjacent_points(i, N, poly, &p0, &p1, &p2);
    b_is_acute_angle.at(i) = is_acute_angle(p0, p1, p2);
  }

  // start ear clipping
  while (N >= 3) {
    int clipped_vertex = -1;

    for (int i = 0; i < N; i++) {
      const bool theta = b_is_acute_angle.at(i);
      if (theta) {
        geometry_msgs::msg::Point32 p0;
        geometry_msgs::msg::Point32 p1;
        geometry_msgs::msg::Point32 p2;
        adjacent_points(i, N, poly, &p0, &p1, &p2);

        int j_begin = (i + 2) % N;
        int j_end = (i - 1 + N) % N;
        bool is_ear = true;
        for (int j = j_begin; j != j_end; j = (j + 1) % N) {
          if (is_within_triangle(p0, p1, p2, poly.points.at(j))) {
            is_ear = false;
            break;
          }
        }

        if (is_ear) {
          clipped_vertex = i;
          break;
        }
      }
    }
    if (clipped_vertex < 0 || clipped_vertex >= N) {
      // print in yellow to indicate warning
      std::cerr << "\033[1;33mCould not find valid vertex for ear clipping triangulation. "
                   "Triangulation result might be invalid\033[0m"
                << std::endl;
      clipped_vertex = 0;
    }

    // create triangle
    geometry_msgs::msg::Point32 p0;
    geometry_msgs::msg::Point32 p1;
    geometry_msgs::msg::Point32 p2;
    adjacent_points(clipped_vertex, N, poly, &p0, &p1, &p2);
    geometry_msgs::msg::Polygon triangle;
    triangle.points.push_back(p0);
    triangle.points.push_back(p1);
    triangle.points.push_back(p2);
    triangles->push_back(triangle);

    // remove vertex of center of angle
    auto it = poly.points.begin();
    std::advance(it, clipped_vertex);
    poly.points.erase(it);

    // remove from angle list
    auto it_angle = b_is_acute_angle.begin();
    std::advance(it_angle, clipped_vertex);
    b_is_acute_angle.erase(it_angle);

    // update angle list
    N = static_cast<int>(poly.points.size());
    if (clipped_vertex == N) {
      clipped_vertex = 0;
    }
    adjacent_points(clipped_vertex, N, poly, &p0, &p1, &p2);
    b_is_acute_angle.at(clipped_vertex) = is_acute_angle(p0, p1, p2);

    int i_prev = (clipped_vertex == 0) ? N - 1 : clipped_vertex - 1;
    adjacent_points(i_prev, N, poly, &p0, &p1, &p2);
    b_is_acute_angle.at(i_prev) = is_acute_angle(p0, p1, p2);
  }
}

/***********************************************************************************
 * Extract (indexed) keys with corresponding value of a lanelet into a string
 ************************************************************************************/
std::string cvisualization::ls_attr_key(
  const lanelet::ConstLanelet & ll, const std::vector<std::string> & key, const bool & index)
{
  std::string text;
  if (index) {
    for (const std::string & it : key) {
      int i = 1;
      std::string key_ind = it + "_" + std::to_string(i);
      while (ll.hasAttribute(key_ind)) {
        text += key_ind + " -> " + ll.attribute(key_ind).value() + "\n";
        ++i;
        key_ind = it + "_" + std::to_string(i);
      }
    }
  } else {
    for (const std::string & it : key) {
      const std::string val = ll.attributeOr(it, "no_tag");
      text += it + " -> " + val + "\n";
    }
  }
  return text;
}

/***********************************************************************************
 * Extract (indexed) keys with corresponding value of a linestring into a string
 ************************************************************************************/
std::string cvisualization::ls_attr_key(
  const lanelet::ConstLineString3d & ls, const std::vector<std::string> & key, const bool & index)
{
  std::string text;
  if (index) {
    for (const std::string & it : key) {
      int i = 1;
      std::string key_ind = it + "_" + std::to_string(i);
      while (ls.hasAttribute(key_ind)) {
        text += key_ind + " -> " + ls.attribute(key_ind).value() + "\n";
        ++i;
        key_ind = it + "_" + std::to_string(i);
      }
    }
  } else {
    for (const std::string & it : key) {
      const std::string val = ls.attributeOr(it, "no_tag");
      text += it + " -> " + val + "\n";
    }
  }
  return text;
}

bool cvisualization::is_clock_wise(const geometry_msgs::msg::Polygon & polygon)
{
  const int N = static_cast<int>(polygon.points.size());
  const double x_offset = polygon.points[0].x;
  const double y_offset = polygon.points[0].y;
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.points.size(); ++i) {
    sum += (polygon.points[i].x - x_offset) * (polygon.points[(i + 1) % N].y - y_offset) -
           (polygon.points[i].y - y_offset) * (polygon.points[(i + 1) % N].x - x_offset);
  }

  return sum < 0.0;
}

bool cvisualization::is_acute_angle(
  const geometry_msgs::msg::Point32 & a, const geometry_msgs::msg::Point32 & o,
  const geometry_msgs::msg::Point32 & b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x) >= 0;
}

bool cvisualization::is_within_triangle(
  const geometry_msgs::msg::Point32 & a, const geometry_msgs::msg::Point32 & b,
  const geometry_msgs::msg::Point32 & c, const geometry_msgs::msg::Point32 & p)
{
  double c1 = (b.x - a.x) * (p.y - b.y) - (b.y - a.y) * (p.x - b.x);
  double c2 = (c.x - b.x) * (p.y - c.y) - (c.y - b.y) * (p.x - c.x);
  double c3 = (a.x - c.x) * (p.y - a.y) - (a.y - c.y) * (p.x - a.x);

  return (c1 > 0.0 && c2 > 0.0 && c3 > 0.0) || (c1 < 0.0 && c2 < 0.0 && c3 < 0.0);
}

void cvisualization::adjacent_points(
  const int i, const int N, const geometry_msgs::msg::Polygon & poly,
  geometry_msgs::msg::Point32 * p0, geometry_msgs::msg::Point32 * p1,
  geometry_msgs::msg::Point32 * p2)
{
  if (p0 == nullptr || p1 == nullptr || p2 == nullptr) {
    std::cerr << __FUNCTION__ << ": either p0, p1, or p2 is null pointer!" << std::endl;
    return;
  }

  *p1 = poly.points[i];
  if (i == 0) {
    *p0 = poly.points[N - 1];
  } else {
    *p0 = poly.points[i - 1];
  }

  if (i < N - 1) {
    *p2 = poly.points[i + 1];
  } else {
    *p2 = poly.points[0];
  }
}

/********************************************************************
 * Convert vector of linestrings to vector of constlinestrings
 *********************************************************************/
lanelet::ConstLineStrings3d cvisualization::to_const(const lanelet::LineStrings3d & lss)
{
  lanelet::ConstLineStrings3d out;
  for (const auto & ls : lss) {
    out.push_back(ls);
  }
  return out;
}
