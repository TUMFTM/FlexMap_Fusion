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
#include "file_in.hpp"
#include "file_out.hpp"
#include "messages.hpp"
#include "param_kiss_icp_georef.hpp"
#include "rubber_sheeting.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

class ckiss_icp_georef : public rclcpp::Node
{
public:
  /***************************************************************
   * Main constructor to be called after the node was triggered
   * => calls all member functions
   ****************************************************************/
  explicit ckiss_icp_georef(const std::string & name);
  ~ckiss_icp_georef();

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

  /***********************************************************
   * Write pcd-map to file
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
  std::string pcd_path;
  std::string pcd_out_path;

  // Module classes
  cfile_in m_file_loader;
  calign m_align;
  crubber_sheeting m_rubber_sheeting;
  cmessages m_msgs;
  cfile_out m_file_writer;
  canalysis m_analysis;

  // Objects
  // Trajectories
  lanelet::GPSPoints traj_GPS;
  lanelet::ConstLineString3d traj_master;
  lanelet::ConstLineString3d traj_target;
  lanelet::ConstLineString3d traj_align;
  lanelet::ConstLineString3d traj_rs;
  // PCD map
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_map;

  // Transformation
  Eigen::Matrix3d trans_al;
  std::vector<s_control_point> control_points;
  lanelet::Areas triangles;
  std::vector<Eigen::Matrix3d> trans_rs;

  // Messages
  visualization_msgs::msg::MarkerArray msg_traj_master_markers;
  visualization_msgs::msg::MarkerArray msg_traj_target_markers;
  visualization_msgs::msg::MarkerArray msg_traj_align_markers;
  visualization_msgs::msg::MarkerArray msg_traj_rs_markers;
  visualization_msgs::msg::MarkerArray msg_rs_geom_markers;
  sensor_msgs::msg::PointCloud2 msg_pcd_map;

  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_master_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_target_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_align_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_rs_markers;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rs_geom_markers;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_map;
};
