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
#include "lanelet2_osm.hpp"

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
clanelet2_osm::clanelet2_osm(const std::string & name) : Node(name)
{
  this->node_name = name;
  this->declare_parameter("node_name", node_name);

  // Load parameters from command line and config file
  get_params(
    *this, this->traj_path, this->poses_path, this->map_path, this->osm_path, this->out_path,
    this->proj_type, this->align_type);

  // Initialize publishers
  initialize_publisher();

  // Load trajectory, SLAM poses, ref map and download osm-data
  load_data();

  // Extract road network
  get_network();

  // Align GPS and SLAM trajectory
  align_traj();

  // Publish trajectory to select control Points for Rubber-Sheeting
  publish_traj();

  // Perform rubber-sheeting to further transform traj and map
  rubber_sheeting();

  // Publish rubber-sheeting data
  publish_rs();

  // Conflation
  conflation();

  // Publish data
  publish_map();

  // Write map to file
  write_map();

  // Analysis and save
  analysis();
  std::cout << "\033[1;36m===> Done!\033[0m" << std::endl;
}

clanelet2_osm::~clanelet2_osm()
{
}

/****************/
/*public methods*/
/****************/

/**************************************************************
 * initialize publishers for visualization in RVIZ
 ***************************************************************/
void clanelet2_osm::initialize_publisher()
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
  this->pub_confl_geom_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/confl/geom_markers", durable_qos_pub);
  this->pub_ll_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/map/ll_map_markers", durable_qos_pub);
  this->pub_ll_map_new_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/map/ll_map_new_markers", durable_qos_pub);
  this->pub_osm_map_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "lof/map/osm_map_markers", durable_qos_pub);
}

/********************************************************************
 * Load GPS-trajectory and SLAM-poses from txt-files
 * Load lanelet map from .osm-file
 * Download and load openstreetmap-excerpt corresponding to GPS-
 * trajectory
 * Set reference and target map (defined by user as parameter)
 * => paths to files specified in command-line or launch-file
 *********************************************************************/
void clanelet2_osm::load_data()
{
  lanelet::ConstLineString3d traj_GPS_proj;
  lanelet::ConstLineString3d traj_SLAM;
  // GPS trajectory
  if (m_file_loader.read_traj_GPS_from_file(
        *this, this->traj_path, this->proj_type, this->traj_GPS, traj_GPS_proj)) {
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

  // Reference map
  if (m_file_loader.read_map_from_file(
        *this, this->map_path, this->proj_type, this->ll_map_lanelet_ptr)) {
    std::cout << "\033[1;36m===> Lanelet2 Map: Loaded!\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Map loading !!");
  }

  // OSM excerpt
  if (m_file_loader.download_osm_file(*this, this->osm_path)) {
    std::cout << "\033[33m~~~~~> OpenStreetMap-Download successful to " << this->osm_path
              << "\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Map download !!");
  }
  if (m_file_loader.read_map_from_file(
        *this, this->osm_path, this->proj_type, this->osm_map_lanelet_ptr)) {
    std::cout << "\033[1;36m===> OSM Map: Loaded!\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during OSM-map loading !!");
  }

  // Set source and target traj depending on parameter
  if (this->get_parameter("master").as_string() == "GPS") {
    this->traj_master = traj_GPS_proj;
    this->traj_target = traj_SLAM;

    this->master_map_lanelet_ptr = osm_map_lanelet_ptr;
    this->target_map_lanelet_ptr = ll_map_lanelet_ptr;
  } else if (this->get_parameter("master").as_string() == "SLAM") {
    this->traj_master = traj_SLAM;
    this->traj_target = traj_GPS_proj;

    this->master_map_lanelet_ptr = ll_map_lanelet_ptr;
    this->target_map_lanelet_ptr = osm_map_lanelet_ptr;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Specify a correct master parameter !!");
  }
}

/**********************************************************************
 * Extract road networks from openstreetmap-data
 ***********************************************************************/
void clanelet2_osm::get_network()
{
  if (!m_extract.osm_map_extract(
        this->osm_map_lanelet_ptr, this->osm_all_linestrings, this->osm_motorway_linestrings,
        this->osm_highway_linestrings, this->osm_road_linestrings)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(this->node_name), "!! Error during road network extraction of osm map !!");
  }
}

/***********************************************************
 * Align GPS and SLAM trajectory (target to reference)
 * => use Umeyama-algorithm or ICP
 ************************************************************/
void clanelet2_osm::align_traj()
{
  // Calculate transformation
  bool btrans_al = m_align.get_transformation(
    *this, this->traj_master, this->traj_target, this->trans_al, this->align_type);

  // Transform poses and lanelet2 map (2D)
  m_align.transform_ls(this->traj_target, this->traj_align, this->trans_al);
  m_align.transform_map(this->target_map_lanelet_ptr, this->trans_al);

  if (btrans_al) {
    std::cout << "\033[1;36m===> GPS points and SLAM poses aligned with " << align_type
              << "-algorithm!\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during trajectory alignment !!");
  }
}

/*****************************************************************************
 * Publish reference and aligned target trajectory to select
 * control Points for Rubber-Sheeting
 ******************************************************************************/
void clanelet2_osm::publish_traj()
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
void clanelet2_osm::rubber_sheeting()
{
  // Get controlpoints from RVIZ
  m_rubber_sheeting.select_control_points(
    *this, this->traj_master, this->traj_align, this->control_points);
  // Calculate triangulation and transformation matrices
  bool btrans_rs = m_rubber_sheeting.get_transformation(
    *this, this->traj_align, this->control_points, this->triangles, this->trans_rs);

  // Transform trajectory, map and point cloud map if desired
  m_rubber_sheeting.transform_ls(this->traj_align, this->traj_rs, this->triangles, this->trans_rs);
  m_rubber_sheeting.transform_map(this->target_map_lanelet_ptr, this->triangles, this->trans_rs);

  // Transform point cloud map if desired by user and lanelet map is not the master map
  if (
    this->get_parameter("transform_pcd").as_bool() &&
    this->get_parameter("master").as_string() == "GPS") {
    m_rubber_sheeting.transform_pcd(*this, this->triangles, this->trans_rs, this->trans_al);
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
void clanelet2_osm::publish_rs()
{
  // Create messages
  // RS Geometry
  m_msgs.rs2marker_msg(this->triangles, this->control_points, this->msg_rs_geom_markers);

  // Transformed trajectory
  m_msgs.linestring2marker_msg(
    this->traj_rs, this->msg_traj_rs_markers, "WEBBlueBright", "traj_rubber_sheeted", 1);

  // Publish on topics
  this->pub_rs_geom_markers->publish(this->msg_rs_geom_markers);
  this->pub_traj_rs_markers->publish(this->msg_traj_rs_markers);
}

/*********************************************************************
 * Perform conflation from openstreetmap to lanelet-map
 **********************************************************************/
void clanelet2_osm::conflation()
{
  // Matching
  lanelet::LineStrings3d ll_coll;
  bool coll = m_matching.collapse_ll_map(this->ll_map_lanelet_ptr, ll_coll);

  bool bG = m_matching.buffer_growing(*this, ll_coll, this->osm_all_linestrings, this->matches);

  // Conflation
  bool rmT = m_conflation.remove_tags(this->ll_map_lanelet_ptr);
  bool confl = m_conflation.conflate_lanelet_OSM(
    this->ll_map_lanelet_ptr, this->matches, this->ll_regular_cols, this->to_be_deleted);
  this->ll_map_new = std::make_shared<lanelet::LaneletMap>();
  bool nM = m_conflation.create_updated_map(
    this->ll_map_lanelet_ptr, this->ll_map_new, this->to_be_deleted);

  if (coll && bG && rmT && confl && nM) {
    std::cout << "\033[1;36m===> Finished conflation!\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Conflation !!");
  }

  for (const auto & ls : ll_coll) {
    this->ll_collapsed.push_back(ls);
  }
}

/************************************************************************************
 * Publish conflation geometry, lanelet-map and openstreetmap-road network
 *************************************************************************************/
void clanelet2_osm::publish_map()
{
  m_msgs.confl2marker_msg(this->ll_collapsed, this->matches, this->msg_confl_geom_markers);

  if (!m_extract.ll_map_extract(
        this->ll_map_lanelet_ptr, this->ll_lanelets, this->ll_regular_lanelets,
        this->ll_shoulder_lanelets, this->ll_stop_lines)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(this->node_name),
      "!! Error during road network extraction of original lanelet map !!");
  }
  if (!m_extract.ll_map_extract(
        this->ll_map_new, this->ll_lanelets_new, this->ll_regular_lanelets_new,
        this->ll_shoulder_lanelets_new, this->ll_stop_lines_new)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(this->node_name),
      "!! Error during road network extraction of updated lanelet map !!");
  }
  // Lanelet map
  m_msgs.col_map_net2marker_msg(
    *this, this->ll_lanelets, this->ll_regular_lanelets, this->ll_shoulder_lanelets,
    this->ll_stop_lines, this->ll_regular_cols, this->msg_ll_map_markers);

  m_msgs.map_net2marker_msg(
    *this, this->ll_lanelets_new, this->ll_regular_lanelets_new, this->ll_shoulder_lanelets_new,
    this->ll_stop_lines_new, this->msg_ll_map_new_markers);

  // OSM map
  m_msgs.osm_net2marker_msg(
    this->osm_motorway_linestrings, this->osm_highway_linestrings, this->osm_road_linestrings,
    this->msg_osm_map_markers);

  // Publish messages
  this->pub_confl_geom_markers->publish(this->msg_confl_geom_markers);
  this->pub_ll_map_markers->publish(this->msg_ll_map_markers);
  this->pub_ll_map_new_markers->publish(this->msg_ll_map_new_markers);
  this->pub_osm_map_markers->publish(this->msg_osm_map_markers);
  std::cout << "\033[1;36m===> Data published as MarkerArray!\033[0m" << std::endl;
}

/***********************************************************
 * Write conflated lanelet-map to file
 ************************************************************/
void clanelet2_osm::write_map()
{
  if (m_file_writer.write_map_to_path(*this, this->out_path, this->proj_type, this->ll_map_new)) {
    std::cout << "\033[1;36m===> Lanelet2 Map written to " << out_path << "\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Map writing !!");
  }
}

/******************************************************************************************
 * Perform analysis calculations and save all desired data to txt-files for later
 * visualization with python
 *******************************************************************************************/
void clanelet2_osm::analysis()
{
  const std::string path = this->get_parameter("analysis_output_dir").as_string();
  bool btraj_matching, bmatching;
  btraj_matching = bmatching = false;

  if (this->get_parameter("analysis_traj_matching").as_bool()) {
    std::vector<double> diff_al;
    std::vector<double> diff_rs;
    btraj_matching = m_analysis.traj_matching(
      *this, this->traj_master, this->traj_target, this->traj_align, this->traj_rs, this->triangles,
      this->control_points, diff_al, diff_rs);
  }
  if (this->get_parameter("analysis_matching").as_bool()) {
    bmatching = m_analysis.matching(
      *this, this->matches, this->osm_all_linestrings, this->ll_lanelets, this->ll_regular_cols,
      this->ll_lanelets_new);
  }

  if (btraj_matching || bmatching) {
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
  rclcpp::spin(std::make_shared<clanelet2_osm>("lanelet2_osm"));
  rclcpp::shutdown();
  return 0;
}
