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
#include "kiss_icp_georef.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

/**************/
/*Constructors*/
/**************/

/***************************************************************
 * Main constructor to be called after the node was triggered
 * => calls all member functions
 ****************************************************************/
ckiss_icp_georef::ckiss_icp_georef(const std::string & name) : Node(name)
{
  this->node_name = name;
  this->declare_parameter("node_name", node_name);

  // Load parameters from command line and config file
  get_params(
    *this, this->traj_path, this->poses_path, this->pcd_path, this->pcd_out_path);

  // Initialize publishers
  initialize_publisher();

  // Load trajectory, SLAM poses, ref map and download osm-data
  load_data();

  // Align GPS and SLAM trajectory
  align_traj();

  // Publish trajectory to select control Points for Rubber-Sheeting
  publish_traj();

  // Perform rubber-sheeting to further transform traj and map
  rubber_sheeting();

  // Publish rubber-sheeting data
  publish_rs();

  // Write pcd to file
  write_map();

  // Analysis and save
  analysis();
  std::cout << "\033[1;36m===> Done!\033[0m" << std::endl;
}

ckiss_icp_georef::~ckiss_icp_georef()
{
}

/****************/
/*public methods*/
/****************/

/**************************************************************
 * initialize publishers for visualization in RVIZ
 ***************************************************************/
void ckiss_icp_georef::initialize_publisher()
{
  rclcpp::QoS durable_qos_pub{1};
  durable_qos_pub.transient_local();

  this->pub_traj_master_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/traj/traj_master_markers", durable_qos_pub);
  this->pub_traj_target_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/traj/traj_target_markers", durable_qos_pub);
  this->pub_traj_align_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/traj/traj_align_markers", durable_qos_pub);
  this->pub_traj_rs_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/traj/traj_rs_markers", durable_qos_pub);
  this->pub_rs_geom_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/rs/geom_markers", durable_qos_pub);
  this->pub_pcd_map = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "lof/rs/pcd_map", durable_qos_pub);
}

/********************************************************************
 * Load GPS-trajectory and SLAM-poses from txt-files
 * => paths to files specified in command-line or launch-file
 *********************************************************************/
void ckiss_icp_georef::load_data()
{
  lanelet::ConstLineString3d traj_GPS_proj;
  lanelet::ConstLineString3d traj_SLAM;
  // GPS trajectory
  if (m_file_loader.read_traj_GPS_from_file(
        *this, this->traj_path, this->traj_GPS, traj_GPS_proj)) {
    std::cout << "\033[1;36m===> GPS trajectory with " << this->traj_GPS.size()
              << " points: Loaded!\033[0m" << std::endl;
    std::cout.precision(17);
    // std::cout << this->get_parameter("orig_lat").as_double() << std::endl <<
    //  this->get_parameter("orig_lon").as_double() << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Trajectory loading !!");
  }

  // SLAM poses
  if (m_file_loader.read_poses_SLAM_from_file(*this, this->poses_path, traj_SLAM)) {
    std::cout << "\033[1;36m===> Poses with " << traj_SLAM.size() << " points: Loaded!\033[0m"
              << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Pose loading !!");
  }

  // PCD map
  if (m_file_loader.read_pcd_from_file(*this, this->pcd_path, this->pcd_map)) {
    std::cout << "\033[1;36mPoint Cloud with " << pcd_map->width * pcd_map->height
            << " points: Loaded!\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during PCD loading !!");
  }

  // Set source and target traj depending on parameter
  if (this->get_parameter("master").as_string() == "GPS") {
    this->traj_master = traj_GPS_proj;
    this->traj_target = traj_SLAM;
  } else if (this->get_parameter("master").as_string() == "SLAM") {
    this->traj_master = traj_SLAM;
    this->traj_target = traj_GPS_proj;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Specify a correct master parameter !!");
  }
}

/***********************************************************
 * Align GPS and SLAM trajectory (target to reference)
 * => use Umeyama-algorithm or ICP
 ************************************************************/
void ckiss_icp_georef::align_traj()
{
  // Calculate transformation
  bool btrans_al = m_align.get_transformation(
    *this, this->traj_master, this->traj_target, this->trans_al);

  // Transform poses and lanelet2 map (2D)
  m_align.transform_ls(this->traj_target, this->traj_align, this->trans_al);

  if (btrans_al) {
    std::cout << "\033[1;36m===> GPS points and SLAM poses aligned with Umeyama-algorithm!\033[0m"
    << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during trajectory alignment !!");
  }
}

/*****************************************************************************
 * Publish reference and aligned target trajectory to select
 * control Points for Rubber-Sheeting
 ******************************************************************************/
void ckiss_icp_georef::publish_traj()
{
  // Trajectory
  m_msgs.linestring2marker_msg(
    this->traj_master, this->msg_traj_master_markers, "WEBGreen", "Trajectory", 1);
  // Poses
  m_msgs.linestring2marker_msg(
    this->traj_target, this->msg_traj_target_markers, "Black", "Poses", 1);
  m_msgs.linestring2marker_msg(
    this->traj_align, this->msg_traj_align_markers, "WEBBlueDark", "Poses_align", 1);

  // Publish messages on topics
  this->pub_traj_master_markers->publish(this->msg_traj_master_markers);
  this->pub_traj_target_markers->publish(this->msg_traj_target_markers);
  this->pub_traj_align_markers->publish(this->msg_traj_align_markers);
}

/*************************************************************************
 * Perform Rubber-sheeting to further transform target-trajectory
 * to reference
 * => also transform corresponding map and (point cloud if desired)
 **************************************************************************/
void ckiss_icp_georef::rubber_sheeting()
{
  // Get controlpoints from RVIZ
  m_rubber_sheeting.select_control_points(
    *this, this->traj_master, this->traj_align, this->control_points);
  // Calculate triangulation and transformation matrices
  bool btrans_rs = m_rubber_sheeting.get_transformation(
    *this, this->traj_align, this->control_points, this->triangles, this->trans_rs);

  // Transform trajectory, map and point cloud map if desired
  m_rubber_sheeting.transform_ls(this->traj_align, this->traj_rs, this->triangles, this->trans_rs);

  // Transform point cloud map if desired by user and GPS is the master
  if (
    this->get_parameter("transform_pcd").as_bool() &&
    this->get_parameter("master").as_string() == "GPS") {
    m_rubber_sheeting.transform_pcd(
      *this, this->triangles, this->trans_rs, this->trans_al, this->pcd_map);
  }

  if (btrans_rs) {
    std::cout << "\033[1;36m===> Finished Rubber-Sheeting!\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Rubber-Sheeting !!");
  }
}

/***********************************************
 * Publish Rubber-sheeting results
 ************************************************/
void ckiss_icp_georef::publish_rs()
{
  // Create messages
  // RS Geometry
  m_msgs.rs2marker_msg(this->triangles, this->control_points, this->msg_rs_geom_markers);

  // Transformed trajectory
  m_msgs.linestring2marker_msg(
    this->traj_rs, this->msg_traj_rs_markers, "WEBBlueBright", "traj_rubber_sheeted", 1);

  // PCD map
  m_msgs.pcd_map2msg(this->pcd_map, this->msg_pcd_map);

  // Publish on topics
  this->pub_rs_geom_markers->publish(this->msg_rs_geom_markers);
  this->pub_traj_rs_markers->publish(this->msg_traj_rs_markers);
  this->pub_pcd_map->publish(this->msg_pcd_map);
}

/***********************************************************
 * Write pcd-map to file
 ************************************************************/
void ckiss_icp_georef::write_map()
{
  if (m_file_writer.write_pcd_to_path(*this, this->pcd_out_path, this->pcd_map)) {
    std::cout << "\033[1;36mPoint Cloud Map written to " << pcd_out_path << "!\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Map writing !!");
  }
}

/******************************************************************************************
 * Perform analysis calculations and save all desired data to txt-files for later
 * visualization with python
 *******************************************************************************************/
void ckiss_icp_georef::analysis()
{
  const std::string path = this->get_parameter("analysis_output_dir").as_string();
  bool btraj_matching;
  btraj_matching = false;

  if (this->get_parameter("analysis_traj_matching").as_bool()) {
    std::vector<double> diff_al;
    std::vector<double> diff_rs;
    btraj_matching = m_analysis.traj_matching(
      *this, this->traj_master, this->traj_target, this->traj_align, this->traj_rs, this->triangles,
      this->control_points, diff_al, diff_rs);
  }

  if (btraj_matching) {
    std::cout << "\033[1;36m===> Analysis calculations saved in " << path << "/ !\033[0m"
              << std::endl;
  }
}

/************
 * main
 *************/
int main(int argc, char ** argv)
{
  // Init
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ckiss_icp_georef>("kiss_icp_georef"));
  rclcpp::shutdown();
  return 0;
}
