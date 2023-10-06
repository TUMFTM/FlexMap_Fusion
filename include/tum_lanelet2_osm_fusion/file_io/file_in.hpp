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

#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <rclcpp/rclcpp.hpp>

#include <curl/curl.h>
#include <curl/easy.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <string>
#include <vector>

class cfile_in
{
public:
  cfile_in();

  /**********************************************************
   * Read GPS-trajectory from txt-file in the format:
   * "lat lon\n"
   * Project trajectory to local coordinate system with first
   * GPS-point as origin
   ***********************************************************/
  bool read_traj_GPS_from_file(
    rclcpp::Node & node, const std::string & traj_path, const std::string & proj_type,
    lanelet::GPSPoints & traj_GPS, lanelet::ConstLineString3d & traj_local);

  /*********************************************************
   * Read SLAM poses from txt-file in the format:
   * "r1 r2 r3 x r4 r5 r6 y r7 r8 r9 z\n"
   **********************************************************/
  bool read_poses_SLAM_from_file(
    rclcpp::Node & node, const std::string & poses_path, lanelet::ConstLineString3d & poses);

  /*********************************************
   * Read .osm file based on projector
   **********************************************/
  bool read_map_from_file(
    rclcpp::Node & node, const std::string & map_path, const std::string & proj_type,
    lanelet::LaneletMapPtr & map_ptr);

  /*************************************************************************************
   * Download openstreetmap-excerpt for specified rectangle of GPS-coordinates via API
   **************************************************************************************/
  bool download_osm_file(rclcpp::Node & node, const std::string & file_path);

private:
};
