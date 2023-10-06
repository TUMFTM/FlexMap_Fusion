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
// Date: 20.03.2023
// ========================================== //
//
//
#include "analysis.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

/**************/
/*Constructors*/
/**************/

canalysis::canalysis()
{
}

/****************/
/*public methods*/
/****************/

/************************************************************************
 * Calculate difference between aligned and rubber-sheeted trajectory
 * and export everything into txt-files for python-visualization
 *************************************************************************/
bool canalysis::traj_matching(
  rclcpp::Node & node, const lanelet::ConstLineString3d & src,
  const lanelet::ConstLineString3d & target, const lanelet::ConstLineString3d & target_al,
  const lanelet::ConstLineString3d & target_rs, const lanelet::Areas & tri,
  const std::vector<s_control_point> & cps, std::vector<double> & diff_al,
  std::vector<double> & diff_rs)
{
  diff_al.clear();
  diff_rs.clear();

  calc_diff(src, target_al, diff_al);
  calc_diff(src, target_rs, diff_rs);

  const std::string traj_matching_dir = "/traj_matching";
  create_output_dir(node, traj_matching_dir);

  write_ls(node, src, traj_matching_dir, "source.txt");
  write_ls(node, target, traj_matching_dir, "target.txt");
  write_ls(node, target_al, traj_matching_dir, "target_al.txt");
  write_ls(node, target_rs, traj_matching_dir, "target_rs.txt");
  write_areas(node, tri, traj_matching_dir, "triangles.txt");
  write_cp(node, cps, traj_matching_dir, "controlPoints.txt");
  write_double_vec(node, diff_al, traj_matching_dir, "diff_al.txt");
  write_double_vec(node, diff_rs, traj_matching_dir, "diff_rs.txt");
  return true;
}

/*************************************************************
 * Export matching data in txt-files for python-visualization
 **************************************************************/
bool canalysis::matching(
  rclcpp::Node & node, const std::vector<s_match> & matches, const lanelet::LineStrings3d & osm,
  const lanelet::ConstLanelets & lls, std::vector<std::pair<lanelet::Id, std::string>> & ll_cols,
  const lanelet::ConstLanelets & lls_updated)
{
  std::vector<double> dAng, dLen, dChord, dPoly, dChamfer, lenRefPline, score;
  for (const auto & match : matches) {
    dAng.push_back(match.d_ang());
    dLen.push_back(match.d_len());
    dChord.push_back(match.d_chord());
    dPoly.push_back(match.d_poly());
    dChamfer.push_back(match.d_chamfer());
    lenRefPline.push_back(match.len_ref_pline());
    score.push_back(match.score());
  }
  const std::string matching_dir = "/matching";
  create_output_dir(node, matching_dir);

  write_match_pline(node, matches, matching_dir, "reference_plines.txt");
  write_match_pline(node, matches, matching_dir, "target_plines.txt");
  write_match_pline(node, matches, matching_dir, "match_conn.txt");
  write_match_areas(node, matches, matching_dir, "buffers.txt");
  write_double_vec(node, dAng, matching_dir, "dAng.txt");
  write_double_vec(node, dLen, matching_dir, "dLen.txt");
  write_double_vec(node, dChord, matching_dir, "dChord.txt");
  write_double_vec(node, dPoly, matching_dir, "dPoly.txt");
  write_double_vec(node, dChamfer, matching_dir, "dChamfer.txt");
  write_double_vec(node, lenRefPline, matching_dir, "lenRefPline.txt");
  write_double_vec(node, score, matching_dir, "score.txt");
  write_lss(node, osm, matching_dir, "osm_linestrings.txt");
  write_lanelets(node, lls, matching_dir, "lanelets.txt");
  write_lanelets_cols(node, lls, ll_cols, matching_dir, "lanelets_colors.txt");
  write_lanelets(node, lls_updated, matching_dir, "lanelets_updated.txt");
  write_lanelets_WGS84(node, lls, matching_dir, "lanelets_WGS84.txt");

  return true;
}

/*****************/
/*private methods*/
/*****************/

/***************************************************************
 * Calculate difference between source & target trajectory
 * => vertical distance from target to source for each point
 ****************************************************************/
void canalysis::calc_diff(
  const lanelet::ConstLineString3d & src, const lanelet::ConstLineString3d & target,
  std::vector<double> & diff)
{
  // Iterate through points on target trajectory
  for (const auto & pt : target) {
    double d = 0;
    std::vector<double> dist;
    // Iterate through segments on source trajectory to find closest distance
    for (auto it = src.begin(); it != src.end() - 1; ++it) {
      // If segment length > 1cm => calculate minimum distance of target point to segment
      // Else => take minimum distance from two point on segment
      if (lanelet::geometry::distance2d(*it, *(it + 1)) > 0.01) {
        // Code to calculate distance from a point to a line SEGMENT, not a line
        Eigen::Vector2d ab((it + 1)->x() - it->x(), (it + 1)->y() - it->y());
        Eigen::Vector2d ap(pt.x() - it->x(), pt.y() - it->y());

        // u
        double u = ab.dot(ap) / ab.dot(ab);

        // 3 Cases
        // 1. Shortest distance point to segment = distance point to segment end point
        // 2. Shortest distance point to segment = distance point to segment start point
        // 3. Shortest distance point to segment = perpendicular distance
        if (u > 1) {
          double y = pt.y() - (it + 1)->y();
          double x = pt.x() - (it + 1)->x();
          d = sqrt(x * x + y * y);
        } else if (u < 0) {
          double y = pt.y() - it->y();
          double x = pt.x() - it->x();
          d = sqrt(x * x + y * y);
        } else {
          double x1 = ab(0);
          double y1 = ab(1);
          double x2 = ap(0);
          double y2 = ap(1);
          double mod = sqrt(x1 * x1 + y1 * y1);
          d = std::abs(x1 * y2 - y1 * x2) / mod;
        }
      } else {
        d = std::min(
          lanelet::geometry::distance2d(pt, *it), lanelet::geometry::distance2d(pt, *(it + 1)));
      }
      dist.push_back(d);
    }
    diff.push_back(*std::min_element(dist.begin(), dist.end()));
  }
}

/*******************************************************************************
 * Create subdirectory in the desired output-directory if not existing so far
 ********************************************************************************/
void canalysis::create_output_dir(rclcpp::Node & node, const std::string & dir_path)
{
  const std::string dir = node.get_parameter("analysis_output_dir").as_string() + dir_path;
  // Create if not a directory
  if (!std::filesystem::is_directory(dir)) {
    std::filesystem::create_directories(dir);
  }
}

/**************************************
 * Write linestring to txt-file
 ***************************************/
void canalysis::write_ls(
  rclcpp::Node & node, const lanelet::ConstLineString3d & ls, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & pt : ls) {
      file << pt.x() << " " << pt.y() << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/**************************************
 * Write linestrings to txt-file
 ***************************************/
void canalysis::write_lss(
  rclcpp::Node & node, const lanelet::LineStrings3d & lss, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & ls : lss) {
      for (const auto & pt : ls) {
        file << pt.x() << " " << pt.y() << " ";
      }
      file << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/***************************************
 * Write double-vector to txt-file
 ****************************************/
void canalysis::write_double_vec(
  rclcpp::Node & node, const std::vector<double> & vec, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & el : vec) {
      file << el << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/********************************
 * Write areas to txt-file
 *********************************/
void canalysis::write_areas(
  rclcpp::Node & node, const lanelet::Areas & areas, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & ar : areas) {
      lanelet::ConstLineStrings3d lss = ar.outerBound();
      for (const auto & ls : lss) {
        file << ls[0].x() << " " << ls[0].y() << " " << ls[1].x() << " " << ls[1].y() << " ";
      }
      file << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/****************************************
 * Write controlpoints to txt-file
 *****************************************/
void canalysis::write_cp(
  rclcpp::Node & node, const std::vector<s_control_point> & cps, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & cpt : cps) {
      lanelet::ConstPoint3d src = cpt.get_source_point();
      lanelet::ConstPoint3d target = cpt.get_target_point();
      file << src.x() << " " << src.y() << " " << target.x() << " " << target.y() << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/**********************************************************************************
 * Write reference or target polyline or match connection of a match to txt-file
 ***********************************************************************************/
void canalysis::write_match_pline(
  rclcpp::Node & node, const std::vector<s_match> & matches, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & match : matches) {
      lanelet::ConstLineStrings3d pline;
      if (file_name.find("ref") != std::string::npos) {
        pline = to_const(match.ref_pline());
      } else if (file_name.find("target") != std::string::npos) {
        pline = to_const(match.target_pline());
      } else if (file_name.find("conn") != std::string::npos) {
        pline = match.match_conn();
      } else {
        std::cerr << __FUNCTION__ << ": specify ref,target or conn in the filename!" << std::endl;
      }
      for (const auto & ls : pline) {
        file << ls.front().x() << " " << ls.front().y() << " " << ls.back().x() << " "
             << ls.back().y() << " ";
      }
      file << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/***********************************************
 * Write buffers of a match to txt-file
 ************************************************/
void canalysis::write_match_areas(
  rclcpp::Node & node, const std::vector<s_match> & matches, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & match : matches) {
      lanelet::Areas areas = match.buffers();

      for (const auto & area : areas) {
        lanelet::ConstLineStrings3d lss = area.outerBound();
        for (const auto & ls : lss) {
          file << ls.front().x() << " " << ls.front().y() << " " << ls.back().x() << " "
               << ls.back().y() << " ";
        }
      }
      file << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/***********************************************
 * Write lanelets of a map to txt-file
 ************************************************/
void canalysis::write_lanelets(
  rclcpp::Node & node, const lanelet::ConstLanelets & lls, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & ll : lls) {
      for (const auto & pt : ll.leftBound()) {
        file << pt.x() << " " << pt.y() << " ";
      }
      for (const auto & pt : ll.rightBound().invert()) {
        file << pt.x() << " " << pt.y() << " ";
      }
      file << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/***********************************************
 * Write lanelets of a map to txt-file
 ************************************************/
void canalysis::write_lanelets_WGS84(
  rclcpp::Node & node, const lanelet::ConstLanelets & lls, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  // Get map origin and projector type
  const std::string proj_type = node.get_parameter("proj_type").as_string();
  const double orig_lat = node.get_parameter("orig_lat").as_double();
  const double orig_lon = node.get_parameter("orig_lon").as_double();

  lanelet::GPSPoint position{orig_lat, orig_lon};
  lanelet::Origin orig{position};

  if (file.is_open()) {
    for (const auto & ll : lls) {
      for (const auto & pt : ll.leftBound()) {
        if (proj_type == "MGRS") {
          lanelet::projection::MGRSProjector projector;
          projector.setMGRSCode({position});
          lanelet::GPSPoint pt_WGS84 = projector.reverse(pt.basicPoint());
          file << std::setprecision(15) << pt_WGS84.lat << " " << pt_WGS84.lon << " ";
        } else if (proj_type == "UTM") {
          lanelet::projection::UtmProjector projector{orig};
          lanelet::GPSPoint pt_WGS84 = projector.reverse(pt.basicPoint());
          file << std::setprecision(15) << pt_WGS84.lat << " " << pt_WGS84.lon << " ";
        }
      }
      for (const auto & pt : ll.rightBound().invert()) {
        if (proj_type == "MGRS") {
          lanelet::projection::MGRSProjector projector;
          projector.setMGRSCode({position});
          lanelet::GPSPoint pt_WGS84 = projector.reverse(pt.basicPoint());
          file << std::setprecision(15) << pt_WGS84.lat << " " << pt_WGS84.lon << " ";
        } else if (proj_type == "UTM") {
          lanelet::projection::UtmProjector projector{orig};
          lanelet::GPSPoint pt_WGS84 = projector.reverse(pt.basicPoint());
          file << std::setprecision(15) << pt_WGS84.lat << " " << pt_WGS84.lon << " ";
        }
      }
      file << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/***********************************************
 * Write colors of lanelets of a map to txt-file
 ************************************************/
void canalysis::write_lanelets_cols(
  rclcpp::Node & node, const lanelet::ConstLanelets & lls,
  std::vector<std::pair<lanelet::Id, std::string>> & ll_cols, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path =
    node.get_parameter("analysis_output_dir").as_string() + dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & ll : lls) {
      // Find color string
      std::string col_string = "";
      for (const auto & el : ll_cols) {
        if (el.first == ll.id()) {
          col_string = el.second;
          break;
        }
      }
      file << col_string << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}

/********************************************************************
 * Convert vector of linestrings to vector of constlinestrings
 *********************************************************************/
lanelet::ConstLineStrings3d canalysis::to_const(const lanelet::LineStrings3d & lss)
{
  lanelet::ConstLineStrings3d out;
  for (const auto & ls : lss) {
    out.push_back(ls);
  }
  return out;
}
