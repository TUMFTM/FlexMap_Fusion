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
// Date: 21.03.2023
// ========================================== //
//
//
#include "file_in.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

/**************/
/*Constructors*/
/**************/

cfile_in::cfile_in()
{
}

/****************/
/*public methods*/
/****************/

/**********************************************************
 * Read GPS-trajectory from txt-file in the format:
 * "lat lon\n"
 * Project trajectory to local coordinate system with first
 * GPS-point as origin
 ***********************************************************/
bool cfile_in::read_traj_GPS_from_file(
  rclcpp::Node & node, const std::string & traj_path, lanelet::GPSPoints & traj_GPS,
  lanelet::ConstLineString3d & traj_local)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  lanelet::GPSPoint gps_pt;
  lanelet::BasicPoint3d pt_basic;
  lanelet::LineString3d ls(lanelet::utils::getId(), {});

  // Read trajectory in GPS format
  double lat, lon;

  std::ifstream infile(traj_path);
  std::string line;
  std::vector<double> lat_vec, lon_vec;

  while (std::getline(infile, line)) {
    std::istringstream iss(line);

    if (!(iss >> lat >> lon)) {
      RCLCPP_ERROR(rclcpp::get_logger(node_name), "Trajectory in wrong format!");
    }
    gps_pt.lat = lat;
    lat_vec.push_back(lat);
    gps_pt.lon = lon;
    lon_vec.push_back(lon);
    traj_GPS.emplace_back(gps_pt);
  }
  // Calculate origin => first trajectory point
  int num_points = traj_GPS.size();

  double orig_lat, orig_lon;
  if (node.get_parameter("customZeroPoint").as_bool()) {
    orig_lat = node.get_parameter("zeroLat").as_double();
    orig_lon = node.get_parameter("zeroLong").as_double();
  } else {
    orig_lat = traj_GPS[0].lat;
    orig_lon = traj_GPS[0].lon;
  }

  std::cout << "OUTPUT: " << orig_lat << " " << orig_lon << std::endl;

  // Calculate boundaries of map piece
  double max_lat, max_lon, min_lat, min_lon;
  min_lat = *std::min_element(lat_vec.begin(), lat_vec.end());
  max_lat = *std::max_element(lat_vec.begin(), lat_vec.end());
  min_lon = *std::min_element(lon_vec.begin(), lon_vec.end());
  max_lon = *std::max_element(lon_vec.begin(), lon_vec.end());

  node.declare_parameter("orig_lat", orig_lat);
  node.declare_parameter("orig_lon", orig_lon);

  node.declare_parameter("min_lat", min_lat);
  node.declare_parameter("max_lat", max_lat);
  node.declare_parameter("min_lon", min_lon);
  node.declare_parameter("max_lon", max_lon);

  // Project trajectory
  lanelet::GPSPoint position{orig_lat, orig_lon};
  lanelet::Origin orig{position};

  lanelet::projection::UtmProjector projector{orig};
  for (int i = 0; i < num_points; i++) {
    pt_basic = projector.forward(traj_GPS[i]);
    lanelet::Point3d pt(lanelet::utils::getId(), pt_basic.x(), pt_basic.y(), pt_basic.z());
    ls.push_back(pt);
  }
  traj_local = ls;
  return true;
}

/*********************************************************
 * Read SLAM poses from txt-file in the format:
 * "r1 r2 r3 x r4 r5 r6 y r7 r8 r9 z\n"
 **********************************************************/
bool cfile_in::read_poses_SLAM_from_file(
  rclcpp::Node & node, const std::string & poses_path, lanelet::ConstLineString3d & poses)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  lanelet::LineString3d ls(lanelet::utils::getId(), {});
  // Read poses
  double x, y, z;
  float r1, r2, r3, r4, r5, r6, r7, r8, r9;

  std::ifstream infile(poses_path);
  std::string line;

  while (std::getline(infile, line)) {
    std::istringstream iss(line);

    if (!(iss >> r1 >> r2 >> r3 >> x >> r4 >> r5 >> r6 >> y >> r7 >> r8 >> r9 >> z)) {
      RCLCPP_ERROR(rclcpp::get_logger(node_name), "Poses in wrong format!");
      return false;
    }
    lanelet::Point3d pt(lanelet::utils::getId(), x, y, z);
    ls.push_back(pt);
  }
  poses = ls;
  return true;
}

/*********************************************
 * Read .osm file based on projector
 **********************************************/
bool cfile_in::read_map_from_file(
  rclcpp::Node & node, const std::string & map_path, lanelet::LaneletMapPtr & map_ptr)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  // Declare error messages during loading
  lanelet::ErrorMessages errors{};

  // Get map origin
  const double orig_lat = node.get_parameter("orig_lat").as_double();
  const double orig_lon = node.get_parameter("orig_lon").as_double();

  lanelet::GPSPoint position{orig_lat, orig_lon};
  lanelet::Origin orig{position};

  // Check for valid filename
  if (map_path.substr(map_path.length() - 4) != ".osm") {
    RCLCPP_ERROR(
      rclcpp::get_logger(node_name), "The provided map_path does not lead to a .osm file!");
    return false;
  }

  // Set projector and load file
  lanelet::projection::UtmProjector projector{orig};
  map_ptr = lanelet::load(map_path, projector, &errors);

  // Overwrite projected point coordinates with local x-/y-values due to VMB settings
  for (auto & pt : map_ptr->pointLayer) {
    if (pt.hasAttribute("local_x") && pt.hasAttribute("local_y")) {
      pt.x() = *pt.attributes()["local_x"].asDouble();
      pt.y() = *pt.attributes()["local_y"].asDouble();
    }
  }

  // Return map and output potential errors
  if (errors.empty()) {
    return true;
  } else {
    std::ofstream error_file;
    error_file.open("errors_map_loading.txt");
    for (const auto & error_ele : errors) {
      error_file << error_ele << std::endl;
    }
    error_file.close();
    std::cout << "\033[33m~~~~~> Map loaded with errors "
              << "(written to errors_map_loading.txt)\033[0m" << std::endl;
    return true;
  }
}

/*********************************************
 * Read .pcd map corresponding to poses
 **********************************************/
bool cfile_in::read_pcd_from_file(
  rclcpp::Node & node, const std::string & pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr & pcm)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  // Check for valid filename
  if (pcd_path.substr(pcd_path.length() - 4) != ".pcd") {
    RCLCPP_ERROR(
      rclcpp::get_logger(node_name), "The provided pcd_path does not lead to a .pcd file!");
    return false;
  }

  // Read input cloud based on provided file path
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file Point cloud!\n");
    return -1;
  }
  pcm = cloud;
  return true;
}

/*************************************************************************************
 * Download openstreetmap-excerpt for specified rectangle of GPS-coordinates via API
 **************************************************************************************/
bool cfile_in::download_osm_file(rclcpp::Node & node, const std::string & file_path)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  // Define URL for download
  // Get map boundaries
  const std::string min_lat = std::to_string(node.get_parameter("min_lat").as_double());
  const std::string max_lat = std::to_string(node.get_parameter("max_lat").as_double());
  const std::string min_lon = std::to_string(node.get_parameter("min_lon").as_double());
  const std::string max_lon = std::to_string(node.get_parameter("max_lon").as_double());

  // Set url
  const std::string url = "https://api.openstreetmap.org/api/0.6/map?bbox=" + min_lon + "," +
                          min_lat + "," + max_lon + "," + max_lat;

  CURL * curl;
  FILE * fp;
  CURLcode res;
  curl = curl_easy_init();
  if (curl) {
    // Set url
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    // Set logging (change to  <1L> for full logging)
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 0L);
    // Set progress meter
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
    // Open file
    fp = fopen(file_path.c_str(), "wb");
    if (fp) {
      // Define file to write to
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
      // Perform download
      res = curl_easy_perform(curl);
      // Cleanup curl stuff and close file
      curl_easy_cleanup(curl);
      fclose(fp);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(node_name), "Error during file download!");
      return false;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Error during file download!");
    return false;
  }
  if (res == CURLE_OK) {
    return true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Error during file download!");
    return false;
  }
}
