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
#include "align.hpp"
#include "analysis.hpp"
#include "conflation.hpp"
#include "extract_network.hpp"
#include "file_in.hpp"
#include "file_out.hpp"
#include "matching.hpp"
#include "messages.hpp"
#include "param.hpp"
#include "rubber_sheeting.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

class clanelet2_osm : public rclcpp::Node
{
public:
  /***************************************************************
   * Main constructor to be called after the node was triggered
   * => calls all member functions
   ****************************************************************/
  explicit clanelet2_osm(const std::string & name);
  ~clanelet2_osm();

  /**************************************************************
   * initialize publishers for visualization in RVIZ
   ***************************************************************/
  void initialize_publisher();

  /********************************************************************
   * Load GNSS-trajectory and SLAM-poses from txt-files
   * Load lanelet map from .osm-file
   * Download and load openstreetmap-excerpt corresponding to GPS-
   * trajectory
   * Set reference and target map (defined by user as parameter)
   * => paths to files specified in command-line or launch-file
   *********************************************************************/
  void load_data();

  /**********************************************************************
   * Extract road networks from openstreetmap-data
   ***********************************************************************/
  void get_network();

  /***********************************************************
   * Align GPS and SLAM trajectory (target to reference)
   * => use Umeyama-algorithm or ICP
   ************************************************************/
  void align_traj();

  /*****************************************************************************
   * Publish reference and aligned target trajectory to select
   * control Points for Rubber-Sheeting
   ******************************************************************************/
  void publish_traj();

  /*************************************************************************
   * Perform Rubber-sheeting to further transform target-trajectory
   * to reference
   * => also transform corresponding map and (point cloud if desired)
   **************************************************************************/
  void rubber_sheeting();

  /***********************************************
   * Publish Rubber-sheeting results
   ************************************************/
  void publish_rs();

  /*********************************************************************
   * Perform conflation from openstreetmap to lanelet-map
   **********************************************************************/
  void conflation();

  /************************************************************************************
   * Publish conflation geometry, lanelet-map and openstreetmap-road network
   *************************************************************************************/
  void publish_map();

  /***********************************************************
   * Write conflated lanelet-map to file
   ************************************************************/
  void write_map();

  /******************************************************************************************
   * Perform analysis calculations and save all desired data to txt-files for later
   * visualization with python
   *******************************************************************************************/
  void analysis();

private:
  std::string node_name;
  // User parameters
  std::string traj_path;
  std::string poses_path;
  std::string map_path;
  std::string osm_path;
  std::string proj_type;
  std::string align_type;
  std::string out_path;

  // Module classes
  cfile_in m_file_loader;
  cfile_out m_file_writer;
  cextract_network m_extract;
  calign m_align;
  crubber_sheeting m_rubber_sheeting;
  cmatching m_matching;
  cconflation m_conflation;
  cmessages m_msgs;
  canalysis m_analysis;

  // Objects
  // Trajectories
  lanelet::GPSPoints traj_GPS;
  lanelet::ConstLineString3d traj_master;
  lanelet::ConstLineString3d traj_target;
  lanelet::ConstLineString3d traj_align;
  lanelet::ConstLineString3d traj_rs;

  // Transformation
  Eigen::Matrix3d trans_al;
  std::vector<s_control_point> control_points;
  lanelet::Areas triangles;
  std::vector<Eigen::Matrix3d> trans_rs;

  // Conflation
  std::vector<s_match> matches;

  // Master and target map
  lanelet::LaneletMapPtr master_map_lanelet_ptr;
  lanelet::LaneletMapPtr target_map_lanelet_ptr;

  // Reference map
  lanelet::LaneletMapPtr ll_map_lanelet_ptr;
  lanelet::ConstLanelets ll_lanelets;
  lanelet::ConstLanelets ll_regular_lanelets;
  std::vector<std::pair<lanelet::Id, std::string>> ll_regular_cols;
  lanelet::ConstLanelets ll_shoulder_lanelets;
  std::vector<lanelet::ConstLineString3d> ll_stop_lines;
  lanelet::ConstLanelets to_be_deleted;
  lanelet::ConstLineStrings3d ll_collapsed;

  // Update reference map
  lanelet::LaneletMapPtr ll_map_new;
  lanelet::ConstLanelets ll_lanelets_new;
  lanelet::ConstLanelets ll_regular_lanelets_new;
  lanelet::ConstLanelets ll_shoulder_lanelets_new;
  std::vector<lanelet::ConstLineString3d> ll_stop_lines_new;

  // OSM map
  lanelet::LaneletMapPtr osm_map_lanelet_ptr;
  lanelet::LineStrings3d osm_all_linestrings;
  lanelet::ConstLineStrings3d osm_motorway_linestrings;
  lanelet::ConstLineStrings3d osm_highway_linestrings;
  lanelet::ConstLineStrings3d osm_road_linestrings;

  // Messages
  visualization_msgs::msg::MarkerArray msg_traj_master_markers;
  visualization_msgs::msg::MarkerArray msg_traj_target_markers;
  visualization_msgs::msg::MarkerArray msg_traj_align_markers;
  visualization_msgs::msg::MarkerArray msg_traj_rs_markers;
  visualization_msgs::msg::MarkerArray msg_rs_geom_markers;
  visualization_msgs::msg::MarkerArray msg_confl_geom_markers;
  visualization_msgs::msg::MarkerArray msg_ll_map_markers;
  visualization_msgs::msg::MarkerArray msg_ll_map_new_markers;
  visualization_msgs::msg::MarkerArray msg_osm_map_markers;

  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_master_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_target_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_align_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_rs_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rs_geom_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_confl_geom_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_ll_map_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_ll_map_new_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_osm_map_markers;
};
