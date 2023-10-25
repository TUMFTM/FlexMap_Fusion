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
//
//
#include "file_out.hpp"

#include <string>

/**************/
/*Constructors*/
/**************/

cfile_out::cfile_out()
{
}

/****************/
/*public methods*/
/****************/

/******************************************************************
 * Write a map to .osm file with specified projector and origin
 * from GPS-trajectory
 *******************************************************************/
bool cfile_out::write_map_to_path(
  rclcpp::Node & node, const std::string & out_path, const lanelet::LaneletMapPtr & map_ptr)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  lanelet::ErrorMessages errors{};

  // Get map origin
  const double orig_lat = node.get_parameter("orig_lat").as_double();
  const double orig_lon = node.get_parameter("orig_lon").as_double();

  lanelet::GPSPoint position{orig_lat, orig_lon};
  lanelet::Origin orig{position};

  // Check for valid output filename
  if (
    out_path.substr(out_path.length() - 4) != ".osm" &&
    out_path.substr(out_path.length() - 4) != ".bin") {
    RCLCPP_ERROR(
      rclcpp::get_logger(node_name),
      "The provided out_path does not specify a file ending in .osm or .bin!");
    return false;
  }

  // Define projector and write map
  lanelet::projection::UtmProjector projector{orig};
  lanelet::write(out_path, *map_ptr, projector, &errors);

  // Output errors if existing
  if (!errors.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Errors during file writing");
    for (const auto & error_ele : errors) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(node_name), error_ele);
    }
    return false;
  }
  return true;
}

/******************************************************************
 * Write transformed pcd map to new file
 *******************************************************************/
bool cfile_out::write_pcd_to_path(
  rclcpp::Node & node, const std::string & pcd_out_path,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map)
{
  // write to file
  if (node.get_parameter("save_ascii").as_bool()) {
    pcl::io::savePCDFileASCII(pcd_out_path, *pcd_map);
  } else {
    pcl::io::savePCDFileBinary(pcd_out_path, *pcd_map);
  }
  return true;
}
