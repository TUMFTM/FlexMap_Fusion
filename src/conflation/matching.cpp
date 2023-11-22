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
// Date: 24.04.2023
// ========================================== //
//
//
#include "matching.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

/**************/
/*Constructors*/
/**************/

cmatching::cmatching()
{
}

/****************/
/*public methods*/
/****************/

/************************************************************************
 * Collapse adjacent lanelets to a centerline to approximate the level
 * of detail of openstreetmap-data
 *************************************************************************/
bool cmatching::collapse_ll_map(
  const lanelet::LaneletMapPtr & map_ptr, lanelet::LineStrings3d & ls_col)
{
  // Get lanelets of map
  const lanelet::Lanelets lls = lanelet_layer(map_ptr);
  lanelet::Ids ids_coll;
  lanelet::LineStrings3d centerlines;

  // Extract centerlines of adjacent lanelets and put them into linestrings
  for (const auto & ll : lls) {
    if (!used_Id(ids_coll, ll)) {
      ids_coll.push_back(ll.id());
      lanelet::LineString3d center = get_centerline(lls, ids_coll, ll);
      centerlines.push_back(center);
    }
  }

  // Connect centerlines based on following/previous lanelets
  std::vector<std::pair<lanelet::Id, lanelet::Id>> ls_conn;
  for (auto & ls : centerlines) {
    connect_lss(lls, centerlines, ls_conn, ls);

    // Convert the resulting linestrings to 2D since openstreetmap-data is in 2D
    // => set z to zero, since visualization works on 3d
    set_z_zero(ls);
    ls_col.push_back(ls);
  }
  return true;
}

/*****************************************************************************************
 * Apply buffer-growing map-matching-algorithm
 * => modified version from "Zhang 2005: A generic matching algorithm for line networks of
 *    different resolutions"
 ******************************************************************************************/
bool cmatching::buffer_growing(
  rclcpp::Node & node, lanelet::LineStrings3d & src, lanelet::LineStrings3d & target,
  std::vector<s_match> & matches)
{
  // Initialize vector of ids to track linestrings that were already used
  lanelet::Ids ids;

  // Split linestrings of source and target dataset into segments (keeping attributes)
  lanelet::LineStrings3d src_seg = split_lss(node, src);
  lanelet::LineStrings3d target_seg = split_lss(node, target);

  // Apply algorithm (starting from an unused linestring segment)
  for (const auto & ls : src_seg) {
    if (!used_Id(ids, ls)) {
      // Instantiate new reference polyline
      lanelet::LineStrings3d pline = init_pline(node, ls, src_seg, ids);
      // Update tags with lanelets if linestring segment was inverted during pline generation
      for (auto & ls : pline) {
        if (ls.inverted()) {
          swap_tags(ls);
        }
      }
      // std::cout << pline.size() << std::endl;

      // Get initial buffer parameters as set in parameter file
      double buffer_V = node.get_parameter("buffer_V").as_double();
      double buffer_P = node.get_parameter("buffer_P").as_double();
      double buffer_rad = node.get_parameter("buffer_rad").as_double();
      int j = 0;
      // Initialize variables
      lanelet::Areas buf;
      std::vector<lanelet::LineStrings3d> candidates;
      lanelet::LineStrings3d matched_candidate;

      /***********************************************************************************
       * Find possible matching candidates by iteratively increasing the buffer parameters
       * if no candidates were found
       ************************************************************************************/
      while (candidates.empty() && j < 3) {
        // Initialize buffers around reference polyline segments
        buf = create_buffer(pline, buffer_V, buffer_P, buffer_rad);
        // Find alle matching candidates inside buffer
        candidates = matching_candidates(pline, buf, target_seg);
        // Exclude candidates that exceed geometric limits
        candidates = exclude_candidates(node, pline, candidates);

        // Increase buffer parameters (only to be used if candidates are empty)
        buffer_V *= 1.5;
        buffer_P *= 1.5;
        buffer_rad *= 1.5;
        ++j;
      }

      /******************************************************************************
       * Set candidate as matched candidate if only one candidate was found OR
       * select best candidate by a geometric similarity score if multiple candidates
       *******************************************************************************/
      // std::cout << "Size " << candidates.size() << std::endl;

      if (candidates.size() == 1) {
        matched_candidate = candidates.front();
      } else if (candidates.size() > 1) {
        matched_candidate = select_candidate(node, pline, candidates);
      }
      matches.push_back(s_match(pline, matched_candidate, buf));
      calc_geo_measures(node, matches.back());
    }
  }
  std::vector<double> stats = matching_stats(node, matches);
  std::cout.precision(10);
  std::cout << "\033[33m~~~~~> Matching statistics:\033[0m" << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Total amount of reference polylines: " << matches.size()
            << "\033[0m" << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Mean length of reference polylines: " << stats[0] << "\033[0m"
            << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Polylines filtered out due to length threshold: " << stats[1]
            << "\033[0m" << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Unmatched polylines: " << stats[2] << "\033[0m" << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Mean angle difference between matches: " << stats[3]
            << "\033[0m" << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Mean length difference between matches: " << stats[4]
            << "\033[0m" << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Mean chord difference between matches: " << stats[5]
            << "\033[0m" << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Mean S value of matches: " << stats[6] << "\033[0m"
            << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Mean score of matches: " << stats[7] << "\033[0m" << std::endl;
  std::cout << "\033[34m~~~~~~~~~~> Matching precision: " << stats[8] << "\033[0m" << std::endl;
  return true;
}

/*****************/
/*private methods*/
/*****************/

/********************************************
 * Get centerline of a part of the street
 *********************************************/
lanelet::LineString3d cmatching::get_centerline(
  const lanelet::Lanelets & lls, lanelet::Ids & ids, const lanelet::Lanelet & ll)
{
  // Create a vector with ids of lanelets contained in the resulting centerline of this ls
  lanelet::Ids ids_coll_for;
  lanelet::Ids ids_coll_back;
  ids_coll_for.push_back(ll.id());

  bool found_outer = true;

  lanelet::Lanelet right_outer = ll;
  lanelet::Lanelet left_outer = ll;

  // Find outer street bound on right side
  while (found_outer) {
    found_outer = false;
    for (const auto & llet : lls) {
      if (lanelet::geometry::rightOf(llet, right_outer) && !used_Id(ids, llet)) {
        right_outer = llet;
        ids.push_back(llet.id());
        ids_coll_for.push_back(llet.id());
        found_outer = true;
      }
    }
  }

  // Find outer street bound on left side (also consider inverted lanelets on
  // other side of road)
  found_outer = true;
  while (found_outer) {
    found_outer = false;
    for (const auto & llet : lls) {
      if (lanelet::geometry::leftOf(llet, left_outer) && !used_Id(ids, llet)) {
        left_outer = llet;
        ids.push_back(llet.id());
        ids_coll_for.push_back(llet.id());
        found_outer = true;
      } else if (lanelet::geometry::leftOf(llet.invert(), left_outer) && !used_Id(ids, llet)) {
        left_outer = llet.invert();
        ids.push_back(llet.id());
        ids_coll_back.push_back(llet.id());
        found_outer = true;
      }
    }
  }

  // Create new lanelet of outer Bounds and return its centerline
  lanelet::Lanelet ll_new(
    lanelet::utils::getId(), left_outer.leftBound(), right_outer.rightBound());

  // Create resulting linestring with the lanelet ids it is representing as attributes
  lanelet::LineString3d center(lanelet::utils::getId(), {});
  for (const auto & pt : ll_new.centerline()) {
    lanelet::Point3d pt_(lanelet::utils::getId(), pt.x(), pt.y(), pt.z());
    center.push_back(pt_);
  }
  // Forward lanelet ids as attributes
  int i = 1;
  for (const auto & id : ids_coll_for) {
    std::string key = "ll_id_forward_" + std::to_string(i);
    center.attributes()[key] = id;
    ++i;
  }
  // Backward lanelet ids as attributes
  i = 1;
  for (const auto & id : ids_coll_back) {
    std::string key = "ll_id_backward_" + std::to_string(i);
    center.attributes()[key] = id;
    ++i;
  }
  return center;
}

/******************************************************************
 * Connect a linestring with all previous/following linestrings
 *******************************************************************/
void cmatching::connect_lss(
  const lanelet::Lanelets & lls, lanelet::LineStrings3d & lss,
  std::vector<std::pair<lanelet::Id, lanelet::Id>> & conn, lanelet::LineString3d & ls)
{
  // Connect linestrings based on following/previous lanelets
  connect_dir(lls, lss, conn, ls, "ll_id_backward_");
  connect_dir(lls, lss, conn, ls, "ll_id_forward_");
}

/****************************************************************************************
 * Split vector of linestrings with plines into vector of segments (copy attributes)
 *****************************************************************************************/
lanelet::LineStrings3d cmatching::split_lss(rclcpp::Node & node, lanelet::LineStrings3d & lss)
{
  const double seg_len = node.get_parameter("seg_len").as_double();

  lanelet::LineStrings3d lss_split;
  for (auto & ls : lss) {
    for (auto it = ls.begin(); it != ls.end() - 1; ++it) {
      lanelet::LineString3d new_segment(lanelet::utils::getId(), {*it, *(it + 1)});
      // Only copy attributes, if segments does not contain a point that is only created
      // for connection reasons (see preprocessing step)
      if (!it->hasAttribute("connection") && !(it + 1)->hasAttribute("connection")) {
        new_segment.attributes() = ls.attributes();
      }
      // Further split segment if it is too long
      while (lanelet::geometry::length(new_segment) > seg_len) {
        double len = lanelet::geometry::length(new_segment);
        // Create new interpolated point
        lanelet::BasicPoint3d pt_inter =
          lanelet::geometry::interpolatedPointAtDistance(new_segment, len - seg_len);
        lanelet::Point3d pt__inter(
          lanelet::utils::getId(), pt_inter.x(), pt_inter.y(), pt_inter.z());

        // Create new linestring with len specified by "seg_len" and update original linestring
        lanelet::LineString3d new_segment_(
          lanelet::utils::getId(), {pt__inter, new_segment.back()}, ls.attributes());
        lss_split.push_back(new_segment_);
        new_segment.pop_back();
        new_segment.push_back(pt__inter);
      }
      lss_split.push_back(new_segment);
    }
  }
  return lss_split;
}

/*************************************************************************
 * Instantiate a polyline consisting of linestring segments with 2 points
 **************************************************************************/
lanelet::LineStrings3d cmatching::init_pline(
  rclcpp::Node & node, const lanelet::LineString3d & ls, lanelet::LineStrings3d & lss,
  lanelet::Ids & ids)
{
  // Get parameter and start with the given (unused) linestring
  double pline_angle = node.get_parameter("pline_angle").as_double() * std::atan(1.0) * 4 / 180.0;
  lanelet::LineStrings3d pline;
  ids.push_back(ls.id());
  pline.push_back(ls);

  // Extend polyline in backward/forward direction as long as the angle of the next line
  // segment is within the limit and the segment was not used previously by another polyline
  extend_ref_pline(pline, lss, pline_angle, ids, "backward");
  extend_ref_pline(pline, lss, pline_angle, ids, "forward");

  return pline;
}

/*******************************************************************************
 * Initialize buffers around each line segment based on the given parameters
 ********************************************************************************/
lanelet::Areas cmatching::create_buffer(
  const lanelet::LineStrings3d & lss, const double buffer_V, const double buffer_P,
  const double rad)
{
  /*********************************************************************
   * Buffer around linestring segment
   * => buffer_V: size vertical to segment direction
   * => buffer_V: size in segment direction (before and after segment)
   * => radius: radius on corners (here illustrate with slashes)
   *    ____________________________________
   *   /                                    \
   *  /                                      \
   * |                                        |
   * |                                        |
   * |          *                  *          |
   * |                                        |
   * |                                        |
   *  \                                      /
   *   \____________________________________/
   **********************************************************************/

  lanelet::Areas buffers;
  // Iterate through segments
  for (const auto & ls : lss) {
    // Get first and second point of segment and difference vector
    Eigen::Vector2d pt_f(ls.front().x(), ls.front().y());
    Eigen::Vector2d pt_s(ls.back().x(), ls.back().y());
    Eigen::Vector2d dir = (pt_s - pt_f);

    // Calculate normalized vector perpendicular to segment direction (left)
    Eigen::Vector2d offset_l = rot(90) * dir.normalized();

    // Define buffer points on front and back
    Eigen::Vector2d pt_f_l = pt_f - dir.normalized() * buffer_P + offset_l * (buffer_V - rad);
    Eigen::Vector2d pt_f_r = pt_f - dir.normalized() * buffer_P - offset_l * (buffer_V - rad);
    Eigen::Vector2d pt_s_l = pt_s + dir.normalized() * buffer_P + offset_l * (buffer_V - rad);
    Eigen::Vector2d pt_s_r = pt_s + dir.normalized() * buffer_P - offset_l * (buffer_V - rad);

    // PDefine points on sides
    Eigen::Vector2d pt_l_f = pt_f + offset_l * buffer_V - dir.normalized() * (buffer_P - rad);
    Eigen::Vector2d pt_r_f = pt_f - offset_l * buffer_V - dir.normalized() * (buffer_P - rad);
    Eigen::Vector2d pt_l_s = pt_s + offset_l * buffer_V + dir.normalized() * (buffer_P - rad);
    Eigen::Vector2d pt_r_s = pt_s - offset_l * buffer_V + dir.normalized() * (buffer_P - rad);

    // Define points on radius (at 45 degrees)
    Eigen::Vector2d pt_rad_f_l = pt_f - dir.normalized() * (buffer_P - rad) +
                                 offset_l * (buffer_V - rad) + rot(135) * dir.normalized() * rad;
    Eigen::Vector2d pt_rad_f_r = pt_f - dir.normalized() * (buffer_P - rad) -
                                 offset_l * (buffer_V - rad) + rot(225) * dir.normalized() * rad;
    Eigen::Vector2d pt_rad_s_l = pt_s + dir.normalized() * (buffer_P - rad) +
                                 offset_l * (buffer_V - rad) + rot(45) * dir.normalized() * rad;
    Eigen::Vector2d pt_rad_s_r = pt_s + dir.normalized() * (buffer_P - rad) -
                                 offset_l * (buffer_V - rad) + rot(315) * dir.normalized() * rad;

    // Convert Eigen-vectors to lanelet-points to create area
    std::vector<Eigen::Vector2d> points_{pt_f_l,     pt_f_r,     pt_s_l,     pt_s_r,
                                         pt_l_f,     pt_r_f,     pt_l_s,     pt_r_s,
                                         pt_rad_f_l, pt_rad_f_r, pt_rad_s_l, pt_rad_s_r};

    // Create linestrings and area
    lanelet::Points3d points;
    for (const auto & pt_ : points_) {
      lanelet::Point3d pt(lanelet::utils::getId(), pt_(0), pt_(1), 0.0);
      points.push_back(pt);
    }

    lanelet::LineString3d ls_l(lanelet::utils::getId(), {points[4], points[6]});
    lanelet::LineString3d lsc_sl1(lanelet::utils::getId(), {points[6], points[10]});
    lanelet::LineString3d lsc_sl2(lanelet::utils::getId(), {points[10], points[2]});
    lanelet::LineString3d ls_s(lanelet::utils::getId(), {points[2], points[3]});
    lanelet::LineString3d lsc_sr1(lanelet::utils::getId(), {points[3], points[11]});
    lanelet::LineString3d lsc_sr2(lanelet::utils::getId(), {points[11], points[7]});
    lanelet::LineString3d ls_r(lanelet::utils::getId(), {points[7], points[5]});
    lanelet::LineString3d lsc_fr1(lanelet::utils::getId(), {points[5], points[9]});
    lanelet::LineString3d lsc_fr2(lanelet::utils::getId(), {points[9], points[1]});
    lanelet::LineString3d ls_f(lanelet::utils::getId(), {points[1], points[0]});
    lanelet::LineString3d lsc_fl1(lanelet::utils::getId(), {points[0], points[8]});
    lanelet::LineString3d lsc_fl2(lanelet::utils::getId(), {points[8], points[4]});

    lanelet::Area ar(
      lanelet::utils::getId(), {ls_l, lsc_sl1, lsc_sl2, ls_s, lsc_sr1, lsc_sr2, ls_r, lsc_fr1,
                                lsc_fr2, ls_f, lsc_fl1, lsc_fl2});
    buffers.push_back(ar);
  }
  return buffers;
}

/***********************************************************************
 * Find all line segments inside given buffers and return as candidates
 ************************************************************************/
std::vector<lanelet::LineStrings3d> cmatching::matching_candidates(
  const lanelet::LineStrings3d & ref_pline, const lanelet::Areas & buffers,
  lanelet::LineStrings3d & lss)
{
  std::vector<lanelet::LineStrings3d> candidates;
  lanelet::LineStrings3d pline;
  lanelet::Ids ids;
  // Iterate through segments to find first one inside the buffer
  // => start form this segment to find following/previous segments inside
  for (const auto & ls : lss) {
    if (ls_inside_buffer(buffers, ls) && !used_Id(ids, ls)) {
      pline.clear();
      pline.push_back(ls);
      candidates.push_back(pline);
      ids.push_back(ls.id());

      // Extend given polyline with following/previous segments if they are
      // inside the buffers and not used already
      extend_candidates(candidates, pline, lss, ref_pline, buffers, ids, "backward");
      extend_candidates(candidates, pline, lss, ref_pline, buffers, ids, "forward");
    }
  }
  return candidates;
}

/************************************************************************
 * Exclude match candidates if one of their geometric measures to
 * the reference polyline exceeds the limits
 *************************************************************************/
std::vector<lanelet::LineStrings3d> cmatching::exclude_candidates(
  rclcpp::Node & node, const lanelet::LineStrings3d & ref,
  const std::vector<lanelet::LineStrings3d> & candidates)
{
  // Get parameters
  const double lim_angle = node.get_parameter("lim_angle").as_double() * std::atan(1.0) * 4 / 180.0;
  const double lim_length = node.get_parameter("lim_length").as_double();
  const double lim_chord = node.get_parameter("lim_chord").as_double();
  const double lim_poly = node.get_parameter("lim_poly").as_double();

  std::vector<lanelet::LineStrings3d> candidates_rem;
  // Exlude candidates that exceed limits
  if (!candidates.empty()) {
    for (auto & candidate : candidates) {
      // Calculate similarity measures
      double d_beta = angle_diff_pline(ref, candidate);
      double d_len = len_diff_pline(ref, candidate);
      double d_chord = chord_diff_pline(ref, candidate);
      double d_poly = poly_area_diff_pline(ref, candidate);

      // Only keep candidates that are in the limits
      if (d_beta < lim_angle && d_len < lim_length && d_chord < lim_chord && d_poly < lim_poly) {
        candidates_rem.push_back(candidate);
      }
    }
  }
  return candidates_rem;
}

/**********************************************************************
 * Select the best match candidate out of multiple ones by a
 * weighted score of geo-similarity measures
 ***********************************************************************/
lanelet::LineStrings3d cmatching::select_candidate(
  rclcpp::Node & node, const lanelet::LineStrings3d & ref,
  const std::vector<lanelet::LineStrings3d> & candidates)
{
  // Get parameters
  // limits/normalizing values
  const double lim_angle = node.get_parameter("lim_angle").as_double() * std::atan(1.0) * 4 / 180.0;
  const double lim_length = node.get_parameter("lim_length").as_double();
  const double lim_chord = node.get_parameter("lim_chord").as_double();
  const double lim_poly = node.get_parameter("lim_poly").as_double();

  // Weights
  const double w_angle = node.get_parameter("w_angle").as_double();
  const double w_len = node.get_parameter("w_length").as_double();
  const double w_chord = node.get_parameter("w_chord").as_double();
  const double w_poly = node.get_parameter("w_poly").as_double();

  std::vector<double> d_beta, d_len, d_chord, d_poly, d_ang_first, d_ang_last, scores;

  // Find the best candidate by geometric score
  for (auto & candidate : candidates) {
    // Calculate similarity measures
    d_beta.push_back(angle_diff_pline(ref, candidate));
    d_len.push_back(len_diff_pline(ref, candidate));
    d_chord.push_back(chord_diff_pline(ref, candidate));
    d_poly.push_back(poly_area_diff_pline(ref, candidate));
  }
  // Calculate weighted score
  for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
    double score = w_angle * (1.0 - d_beta[i] / lim_angle) + w_len * (1.0 - d_len[i] / lim_length) +
                   w_chord * (1.0 - d_chord[i] / lim_chord) + w_poly * (1.0 - d_poly[i] / lim_poly);
    scores.push_back(score);
  }
  // Find candidate with maximum score and return
  const int ind = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
  return candidates[ind];
}

/*****************************************************************************
 * Set geosimilarity measures for later evaluation of matching result
 ******************************************************************************/
void cmatching::calc_geo_measures(rclcpp::Node & node, s_match & match)
{
  // Get parameters
  // limits/normalizing values
  const double lim_angle = node.get_parameter("lim_angle").as_double() * std::atan(1.0) * 4 / 180.0;
  const double lim_length = node.get_parameter("lim_length").as_double();
  const double lim_chord = node.get_parameter("lim_chord").as_double();
  const double lim_poly = node.get_parameter("lim_poly").as_double();

  // Weights
  const double w_angle = node.get_parameter("w_angle").as_double();
  const double w_len = node.get_parameter("w_length").as_double();
  const double w_chord = node.get_parameter("w_chord").as_double();
  const double w_poly = node.get_parameter("w_poly").as_double();

  double d_ang, d_len, d_chord, d_poly, d_chamfer, score;
  if (!match.target_pline().empty()) {
    d_ang = angle_diff_pline(match.ref_pline(), match.target_pline());
    d_len = len_diff_pline(match.ref_pline(), match.target_pline());
    d_chord = chord_diff_pline(match.ref_pline(), match.target_pline());
    d_poly = poly_area_diff_pline(match.ref_pline(), match.target_pline());
    d_chamfer = chamfer_distance(match.ref_pline(), match.target_pline());
    score = w_angle * (1.0 - d_ang / lim_angle) + w_len * (1.0 - d_len / lim_length) +
            w_chord * (1.0 - d_chord / lim_chord) + w_poly * (1.0 - d_poly / lim_poly);
  } else {
    d_ang = 0;
    d_len = 0;
    d_chord = 0;
    d_poly = 0;
    d_chamfer = 0;
    score = 0;
  }
  const double len_ref_pline = lanelet::geometry::length(ls_seg2string(match.ref_pline()));
  match.set_geo_measures(d_ang, d_len, d_chord, d_poly, d_chamfer, len_ref_pline, score);
}

/*****************************************************************************
 * Calculate matching rate
 * => all reference lines for which a match was returned after the algorithm
 ******************************************************************************/
std::vector<double> cmatching::matching_stats(
  rclcpp::Node & node, const std::vector<s_match> & matches)
{
  std::vector<double> stats;
  std::vector<double> len_ref_pline, d_ang, d_len, d_chord, d_poly, d_chamfer, score;
  for (const auto & match : matches) {
    len_ref_pline.push_back(match.len_ref_pline());
    d_ang.push_back(match.d_ang());
    d_len.push_back(match.d_len());
    d_chord.push_back(match.d_chord());
    d_poly.push_back(match.d_poly());
    d_chamfer.push_back(match.d_chamfer());
    score.push_back(match.score());
  }

  // Mean length of reference polylines (unfiltered)
  stats.push_back(
    std::accumulate(len_ref_pline.begin(), len_ref_pline.end(), 0.0) / len_ref_pline.size());

  // Get indices where reference polyline is longer than threshold
  const double lim_ref_pline = node.get_parameter("lim_ref_pline").as_double();
  std::vector<size_t> ind_ref_pline;
  for (size_t i = 0; i < len_ref_pline.size(); ++i) {
    if (len_ref_pline[i] >= lim_ref_pline) {
      ind_ref_pline.push_back(i);
    }
  }

  // Filter out polylines based on length threshold
  std::vector<double> len_ref_pline_fil, d_ang_fil, d_len_fil, d_chord_fil, d_poly_fil,
    d_chamfer_fil, score_fil;
  for (const auto & index : ind_ref_pline) {
    len_ref_pline_fil.push_back(len_ref_pline[index]);
    d_ang_fil.push_back(d_ang[index]);
    d_len_fil.push_back(d_len[index]);
    d_chord_fil.push_back(d_chord[index]);
    d_poly_fil.push_back(d_poly[index]);
    d_chamfer_fil.push_back(d_chamfer[index]);
    score_fil.push_back(score[index]);
  }

  // Add polylines filtered out due to length threshold
  stats.push_back(static_cast<double>(score.size() - score_fil.size()));

  // Add polylines filtered out due to non-matches
  int count = std::count_if(score_fil.begin(), score_fil.end(), [](double s) { return s > 0; });
  stats.push_back(static_cast<double>(score_fil.size() - static_cast<size_t>(count)));

  // Calculate means
  stats.push_back(
    std::accumulate(d_ang_fil.begin(), d_ang_fil.end(), 0.0) / static_cast<double>(count));
  stats.push_back(
    std::accumulate(d_len_fil.begin(), d_len_fil.end(), 0.0) / static_cast<double>(count));
  stats.push_back(
    std::accumulate(d_chord_fil.begin(), d_chord_fil.end(), 0.0) / static_cast<double>(count));
  stats.push_back(
    std::accumulate(d_poly_fil.begin(), d_poly_fil.end(), 0.0) / static_cast<double>(count));
  stats.push_back(
    std::accumulate(score_fil.begin(), score_fil.end(), 0.0) / static_cast<double>(count));

  // Calculate precision
  const double lim_tp = node.get_parameter("lim_tp").as_double();
  int tp =
    std::count_if(score_fil.begin(), score_fil.end(), [lim_tp](double s) { return s >= lim_tp; });
  stats.push_back(static_cast<double>(tp) / static_cast<double>(count));
  return stats;
}

/******************************************************************************************
 * Connect centerlines of adjacent lanelets based on the following/previous lanelets in
 * the given direction
 *******************************************************************************************/
void cmatching::connect_dir(
  const lanelet::Lanelets & lls, lanelet::LineStrings3d & lss,
  std::vector<std::pair<lanelet::Id, lanelet::Id>> & conn, lanelet::LineString3d & ls,
  const std::string & key)
{
  // Get forward lanelets of current linestring
  int i = 1;
  std::string key_ind = key + std::to_string(i);

  while (ls.hasAttribute(key_ind)) {
    // Get corresponding lanelet to key
    lanelet::ConstLanelet ll = find_ll(lls, *ls.attribute(key_ind).asId());

    // Find following lanelets
    lanelet::ConstLanelets lls_foll = following_ll(lls, ll);

    // Find centerline corresponding to following lanelet and check whether it is
    // in the same direction
    for (const auto & ll_foll : lls_foll) {
      bool fol_forward = true;
      lanelet::LineString3d ls_foll = find_ls(lss, ll_foll, fol_forward);

      // Connect the two linestrings if they have not been connected yet
      if (!conn_ls(conn, ls, ls_foll)) {
        if (key_ind.find("forward") != std::string::npos) {
          connect_ls(ls, ls_foll, true, fol_forward);
        } else if (key_ind.find("backward") != std::string::npos) {
          connect_ls(ls, ls_foll, false, fol_forward);
        } else {
          std::cerr << __FUNCTION__
                    << "\033[1;31m: Cannot determine linestring direction from key !!\033[0m"
                    << std::endl;
        }
        std::pair<lanelet::Id, lanelet::Id> connected(ls.id(), ls_foll.id());
        conn.push_back(connected);
      }
    }
    ++i;
    key_ind = key + std::to_string(i);
  }
}

/******************************************************************************************
 * Extend reference polyline as long as the valence is < 3 and the angle of the next
 * segment is below the given limit
 *******************************************************************************************/
void cmatching::extend_ref_pline(
  lanelet::LineStrings3d & pline, lanelet::LineStrings3d & lss, const double angle_lim,
  lanelet::Ids & ids, const std::string & direction)
{
  const int dir = (direction == "forward") ? 1 : 0;

  // Initialize by finding the connected segments to the point in forward/backward direction
  lanelet::LineStrings3d connected;
  (dir == 1) ? find_ls_from_point(connected, lss, pline.back(), dir)
             : find_ls_from_point(connected, lss, pline.front(), dir);
  bool angle_within_lim = true;

  // Iterate as long as no intersection (valence >= 3) and angle within the limits
  while (!connected.empty() && connected.size() < 3 && angle_within_lim) {
    std::vector<double> angles;
    lanelet::LineStrings3d ls_angles;
    lanelet::LineString3d pline_seg = (dir == 1) ? pline.back() : pline.front();
    // Calculate angle between potential new segment and current pline segment
    for (const auto & ls_ : connected) {
      if (!used_Id(ids, ls_)) {
        angles.push_back(std::abs(angle_segment(pline_seg, ls_, false)));
        ls_angles.push_back(ls_);
      }
    }
    // Find index of smallest angle and add corresponding segment to pline
    if (!angles.empty()) {
      const int ind = std::distance(angles.begin(), std::min_element(angles.begin(), angles.end()));
      if (angles[ind] < angle_lim) {
        lanelet::LineString3d new_ = ls_angles[ind];
        if (dir == 1) {
          pline.push_back(new_);
        } else {
          pline.insert(pline.begin(), new_);
        }
        ids.push_back(new_.id());
        connected.clear();
        find_ls_from_point(connected, lss, new_, dir);
      } else {
        angle_within_lim = false;
      }
    } else {
      break;
    }
  }
}

/******************************************************************************
 * Extend the current linestring segment with its previous/following ones
 * as long as they are inside the buffers and add them as match candidates
 *******************************************************************************/
void cmatching::extend_candidates(
  std::vector<lanelet::LineStrings3d> & candidates, lanelet::LineStrings3d & pline,
  lanelet::LineStrings3d & lss, const lanelet::LineStrings3d & ref_pline,
  const lanelet::Areas & buffers, lanelet::Ids & ids, const std::string & direction)
{
  bool cont = true;
  const int dir = (direction == "forward") ? 1 : 0;

  // Initialize by finding the connected segments to the point in forward/backward direction
  lanelet::LineStrings3d connected;
  (dir == 1) ? find_ls_from_point(connected, lss, pline.back(), dir)
             : find_ls_from_point(connected, lss, pline.front(), dir);

  // Iterate as long as segment inside buffers
  while (!connected.empty() && cont) {
    std::vector<double> angles;
    lanelet::LineStrings3d ls_inside;
    cont = false;
    for (const auto & ls : connected) {
      if (ls_inside_buffer(buffers, ls) && !used_Id(ids, ls)) {
        ls_inside.push_back(ls);
        std::vector<double> min;
        for (const auto & lsref : ref_pline) {
          min.push_back(
            lanelet::geometry::distance2d(ls.front(), lsref.front()) +
            lanelet::geometry::distance2d(ls.back(), lsref.back()));
        }
        angles.push_back(std::abs(angle_segment(
          ref_pline[std::distance(min.begin(), std::min_element(min.begin(), min.end()))], ls,
          true)));
      }
    }
    // If multiple following/previous linestrings are inside the buffer
    // => the clost one to the reference pline is selected
    if (!angles.empty()) {
      const int ind = std::distance(angles.begin(), std::min_element(angles.begin(), angles.end()));
      lanelet::LineString3d new_ = ls_inside[ind];
      ids.push_back(new_.id());
      if (dir == 1) {
        pline.push_back(new_);
      } else {
        pline.insert(pline.begin(), new_);
      }
      candidates.push_back(pline);
      connected.clear();
      find_ls_from_point(connected, lss, new_, dir);
      cont = true;
    }
  }
}

/*********************************************
 * Check if a lanelet was already used
 **********************************************/
bool cmatching::used_Id(const lanelet::Ids & ids, const lanelet::ConstLanelet & ll)
{
  if (std::find(ids.begin(), ids.end(), ll.id()) != ids.end()) {
    return true;
  }
  return false;
}
/*********************************************
 * Check if a linestring was already used
 **********************************************/
bool cmatching::used_Id(const lanelet::Ids & ids, const lanelet::ConstLineString3d & ls)
{
  if (std::find(ids.begin(), ids.end(), ls.id()) != ids.end()) {
    return true;
  }
  return false;
}

/*********************************************
 * Get lanelets of a given map
 **********************************************/
lanelet::Lanelets cmatching::lanelet_layer(const lanelet::LaneletMapPtr & map_ptr)
{
  lanelet::Lanelets lls;
  if (!map_ptr) {
    std::cerr << "No map received!";
    return lls;
  }

  for (const auto & ll : map_ptr->laneletLayer) {
    lls.push_back(ll);
  }
  return lls;
}

/*********************************************************
 * Find a lanelet in a vector of lanelets given its id
 **********************************************************/
lanelet::ConstLanelet cmatching::find_ll(const lanelet::Lanelets & lls, const lanelet::Id & id)
{
  for (const auto & ll : lls) {
    if (ll.id() == id) {
      return ll;
    }
  }
  std::cerr << "\033[1;31m!! Couldn't find lanelet for id !!\033[0m" << std::endl;
  return lls.back();
}

/*********************************************************
 * Find following lanelets to given lanelet
 **********************************************************/
lanelet::ConstLanelets cmatching::following_ll(
  const lanelet::Lanelets & lls, const lanelet::ConstLanelet & ll)
{
  lanelet::ConstLanelets out;
  for (const auto & ll_ : lls) {
    if (lanelet::geometry::follows(ll, ll_)) {
      out.push_back(ll_);
    }
  }
  return out;
}

/*****************************************************************************
 * Find centerline-linestring that represents lanelet and get its orientation
 ******************************************************************************/
lanelet::LineString3d cmatching::find_ls(
  lanelet::LineStrings3d & lss, const lanelet::ConstLanelet & ll, bool & forward)
{
  for (const auto & ls : lss) {
    for (const auto & atr : ls.attributes()) {
      if (*atr.second.asId() == ll.id()) {
        forward = (atr.first.find("forward") != std::string::npos) ? true : false;
        return ls;
      }
    }
  }
  std::cerr << __FUNCTION__ << "\033[1;31m!! Couldn't find linestring for lanelet !!\033[0m"
            << std::endl;
  return lss.back();
}

/***************************************************************************
 * Find all linestrings that are connected to either the first or second
 * point of another linestring
 ****************************************************************************/
void cmatching::find_ls_from_point(
  lanelet::LineStrings3d & ls_pt, lanelet::LineStrings3d & lss, const lanelet::LineString3d & src,
  const int pos)
{
  // Consider first (pos = 0) or second (pos = 1) point of source linestring
  if (pos == 0) {
    for (auto & ls : lss) {
      if (ls.back().id() == src.front().id()) {
        ls_pt.push_back(ls);
      } else if (ls.front().id() == src.front().id()) {
        ls_pt.push_back(ls.invert());
      }
    }
  } else if (pos == 1) {
    for (auto & ls : lss) {
      if (ls.front().id() == src.back().id()) {
        ls_pt.push_back(ls);
      } else if (ls.back().id() == src.back().id()) {
        ls_pt.push_back(ls.invert());
      }
    }
  } else {
    std::cerr << __FUNCTION__ << ": Can only consider segments!" << std::endl;
  }
}

/**************************************************************
 * Connect two linestrings based on their orientation
 ***************************************************************/
void cmatching::connect_ls(
  lanelet::LineString3d & ls_pre, lanelet::LineString3d & ls_fol, const bool & pre_forward,
  const bool & fol_forward)
{
  // Generate point in the middle between end of pre-ls and beginning of foll-ls
  lanelet::Point3d pt_pre;
  lanelet::Point3d pt_fol;

  // Set previous and following point depending on forward or backward
  pt_pre = pre_forward ? ls_pre.back() : ls_pre.front();
  pt_fol = fol_forward ? ls_fol.front() : ls_fol.back();

  // Create new point if distance big enough between linestrings
  // (otherwise replace first point in following ls with last point in previous ls)
  if (lanelet::geometry::distance3d(pt_pre, pt_fol) > 1) {
    // Create intermediate point
    double x = (pt_pre.x() + pt_fol.x()) / 2.0;
    double y = (pt_pre.y() + pt_fol.y()) / 2.0;
    double z = (pt_pre.z() + pt_fol.z()) / 2.0;
    lanelet::Point3d pt(lanelet::utils::getId(), x, y, z);
    pt.attributes()["connection"] = "yes";

    // Insert point into both linestrings
    if (pre_forward) {
      ls_pre.push_back(pt);
    } else {
      ls_pre.insert(ls_pre.begin(), pt);
    }
    if (fol_forward) {
      ls_fol.insert(ls_fol.begin(), pt);
    } else {
      ls_fol.push_back(pt);
    }
  } else {
    if (fol_forward) {
      ls_fol.erase(ls_fol.begin());
      ls_fol.insert(ls_fol.begin(), pt_pre);
    } else {
      ls_fol.pop_back();
      ls_fol.push_back(pt_pre);
    }
  }
}

/**********************************************************
 * Check if two linestrings were already connected
 ***********************************************************/
bool cmatching::conn_ls(
  const std::vector<std::pair<lanelet::Id, lanelet::Id>> & conn,
  const lanelet::LineString3d & ls_pre, const lanelet::LineString3d & ls_fol)
{
  for (const auto & conn_el : conn) {
    if (
      (conn_el.first == ls_pre.id() && conn_el.second == ls_fol.id()) ||
      (conn_el.first == ls_fol.id() && conn_el.second == ls_pre.id())) {
      return true;
    }
  }
  return false;
}

/**********************************************************
 * Check if a linestring segment is inside the buffers
 * => check if first && second point is inside buffers
 ***********************************************************/
bool cmatching::ls_inside_buffer(const lanelet::Areas & buf, const lanelet::LineString3d & ls)
{
  bool firstPt, secondPt;
  firstPt = secondPt = false;
  if (ls.size() != 2) {
    std::cerr << __FUNCTION__ << ": Linestring is not a segment !!" << std::endl;
    return false;
  }

  // Check if first point of segment is inside buffers
  for (const auto & ar : buf) {
    if (lanelet::geometry::inside(ar, lanelet::utils::to2D(ls.front().basicPoint()))) {
      firstPt = true;
      break;
    }
  }
  // Check if second point of segment is inside buffers
  for (const auto & ar : buf) {
    if (lanelet::geometry::inside(ar, lanelet::utils::to2D(ls.back().basicPoint()))) {
      secondPt = true;
      break;
    }
  }
  if (firstPt && secondPt) {
    return true;
  }
  return false;
}

/**************************************************************
 * Calculate angle between first and last point of a polyline
 ***************************************************************/
double cmatching::angle_diff_pline(
  const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2)
{
  // Create linestrings consisting of end points of plines
  lanelet::Point3d ls1_start(
    lanelet::utils::getId(), ls1.front().front().x(), ls1.front().front().y(),
    ls1.front().front().z());
  lanelet::Point3d ls1_end(
    lanelet::utils::getId(), ls1.back().back().x(), ls1.back().back().y(), ls1.back().back().z());
  lanelet::Point3d ls2_start(
    lanelet::utils::getId(), ls2.front().front().x(), ls2.front().front().y(),
    ls2.front().front().z());
  lanelet::Point3d ls2_end(
    lanelet::utils::getId(), ls2.back().back().x(), ls2.back().back().y(), ls2.back().back().z());

  lanelet::LineString3d ls1_(lanelet::utils::getId(), {ls1_start, ls1_end});
  lanelet::LineString3d ls2_(lanelet::utils::getId(), {ls2_start, ls2_end});
  return std::abs(angle_segment(ls1_, ls2_, true));
}

/**************************************************************
 * Calculate length difference between two polylines
 ***************************************************************/
double cmatching::len_diff_pline(
  const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2)
{
  // Create connected linestring from segments
  const lanelet::LineString3d ls1_ = ls_seg2string(ls1);
  const lanelet::LineString3d ls2_ = ls_seg2string(ls2);

  // Return length difference of linestrings
  return std::abs(lanelet::geometry::length(ls1_) - lanelet::geometry::length(ls2_));
}

/**************************************************************
 * Calculate chord difference between two polylines
 ***************************************************************/
double cmatching::chord_diff_pline(
  const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2)
{
  double d1 = lanelet::geometry::distance2d(ls1.front().front(), ls1.back().back());
  double d2 = lanelet::geometry::distance2d(ls2.front().front(), ls2.back().back());
  return std::abs(d2 - d1);
}

/************************************************************************************
 * Calculate quotient of area enclosed by two polylines and the sum of their lengths
 *************************************************************************************/
double cmatching::poly_area_diff_pline(
  const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2)
{
  // Calculate polygon out of the two linestrings and divide it by the sum of their lengths
  // Create connected linestring from segments
  lanelet::LineString3d ls1_ = ls_seg2string(ls1);
  lanelet::LineString3d ls2_ = ls_seg2string(ls2);

  auto aligned = lanelet::geometry::align(ls1_, ls2_);

  lanelet::Polygon3d poly(lanelet::utils::getId(), {});
  for (auto & pt : aligned.first) {
    poly.push_back(pt);
  }
  for (auto & pt : aligned.second.invert()) {
    poly.push_back(pt);
  }
  return std::abs(
    lanelet::geometry::area((lanelet::utils::to2D(lanelet::utils::toHybrid(poly)))) /
    (lanelet::geometry::length(ls1_) + lanelet::geometry::length(ls2_)));
}

/******************************************************
 * Calculate angle between two linestring segments
 *******************************************************/
double cmatching::angle_segment(
  const lanelet::LineString3d & ls1, const lanelet::LineString3d & ls2, const bool align)
{
  Eigen::Vector2d v1, v2;
  if (align) {
    std::pair<lanelet::LineString3d, lanelet::LineString3d> aligned =
      lanelet::geometry::align(ls1, ls2);
    v1 << aligned.first.back().x() - aligned.first.front().x(),
      aligned.first.back().y() - aligned.first.front().y();
    v2 << aligned.second.back().x() - aligned.second.front().x(),
      aligned.second.back().y() - aligned.second.front().y();
  } else {
    v1 << ls1.back().x() - ls1.front().x(), ls1.back().y() - ls1.front().y();
    v2 << ls2.back().x() - ls2.front().x(), ls2.back().y() - ls2.front().y();
  }
  double angle = std::atan2(v1(0) * v2(1) - v2(0) * v1(1), v1.dot(v2));
  return angle;
}

/*******************************************************************
 * Calculate chamfer distance between two linestrings
 ********************************************************************/
double cmatching::chamfer_distance(
  const lanelet::LineStrings3d & ls1, const lanelet::LineStrings3d & ls2)
{
  // Create connected linestring from segments
  const lanelet::LineString3d ls1_ = ls_seg2string(ls1);
  const lanelet::LineString3d ls2_ = ls_seg2string(ls2);

  // Distance from ls1 to ls2
  double cd1 = 0;
  for (const auto & pt1 : ls1_) {
    Eigen::Vector2d v1(pt1.x(), pt1.y());
    std::vector<double> cd1_;
    for (const auto & pt2 : ls2_) {
      Eigen::Vector2d v2(pt2.x(), pt2.y());
      cd1_.push_back((v1 - v2).norm());
    }
    cd1 += cd1_[std::distance(cd1_.begin(), std::min_element(cd1_.begin(), cd1_.end()))];
  }
  cd1 /= ls1.size();

  // Distance from ls2 to ls1
  double cd2 = 0;
  for (const auto & pt2 : ls2_) {
    Eigen::Vector2d v2(pt2.x(), pt2.y());
    std::vector<double> cd2_;
    for (const auto & pt1 : ls1_) {
      Eigen::Vector2d v1(pt1.x(), pt1.y());
      cd2_.push_back((v1 - v2).norm());
    }
    cd2 += cd2_[std::distance(cd2_.begin(), std::min_element(cd2_.begin(), cd2_.end()))];
  }
  cd2 /= ls2.size();

  return cd1 + cd2;
}

/*****************************************
 * Create rotation matrix from angle
 ******************************************/
Eigen::Matrix2d cmatching::rot(double angle)
{
  double angle_rad = angle * std::atan(1.0) * 4 / 180.0;
  Eigen::Matrix2d mat;
  mat << std::cos(angle_rad), -std::sin(angle_rad), std::sin(angle_rad), std::cos(angle_rad);
  return mat;
}

/**********************************************
 * Set the z-coordinate of a linestring to 0
 ***********************************************/
void cmatching::set_z_zero(lanelet::LineString3d & ls)
{
  for (auto & pt : ls) {
    pt.z() = 0.0;
  }
}

/*******************************************************************
 * Convert vector of linestring segments to a single linestring
 ********************************************************************/
lanelet::LineString3d cmatching::ls_seg2string(const lanelet::LineStrings3d & ls)
{
  // Create connected linestring from segments
  lanelet::LineString3d ls_(lanelet::utils::getId(), {});

  // Take first point from segment and add it to linestring
  for (const auto & seg : ls) {
    ls_.push_back(
      lanelet::Point3d(lanelet::utils::getId(), seg.front().x(), seg.front().y(), seg.front().z()));
  }
  // Take last point and add it since it was not considered so far
  ls_.push_back(lanelet::Point3d(
    lanelet::utils::getId(), ls.back().back().x(), ls.back().back().y(), ls.back().back().z()));
  return ls_;
}

/**********************************************************
 * Swap forward/backward in attributes of a linestring
 ***********************************************************/
void cmatching::swap_tags(lanelet::LineString3d & ls)
{
  lanelet::Ids forward_Ids;
  lanelet::Ids backward_Ids;
  // Save current attributes
  for (auto & att : ls.attributes()) {
    if (att.first.find("forward") != std::string::npos) {
      forward_Ids.push_back(*att.second.asId());
    }
    if (att.first.find("backward") != std::string::npos) {
      backward_Ids.push_back(*att.second.asId());
    }
  }
  // Delete current attributes
  ls.attributes().clear();

  // backward as new forward attributes
  int i = 1;
  for (const auto & id : backward_Ids) {
    std::string key = "ll_id_forward_" + std::to_string(i);
    ls.attributes()[key] = id;
    ++i;
  }
  // forward as new backward attributes
  i = 1;
  for (const auto & id : forward_Ids) {
    std::string key = "ll_id_backward_" + std::to_string(i);
    ls.attributes()[key] = id;
    ++i;
  }
}
