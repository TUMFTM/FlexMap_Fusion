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
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <vector>

class cfile_out
{
public:
  cfile_out();

  /******************************************************************
   * Write a map to .osm file with specified projector and origin
   * from GPS-trajectory
   *******************************************************************/
  bool write_map_to_path(
    rclcpp::Node & node, const std::string & out_path, const lanelet::LaneletMapPtr & map_ptr);

  /******************************************************************
   * Write transformed pcd map to new file
   *******************************************************************/
  bool write_pcd_to_path(
    rclcpp::Node & node, const std::string & pcd_out_path,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_map);
private:
};
