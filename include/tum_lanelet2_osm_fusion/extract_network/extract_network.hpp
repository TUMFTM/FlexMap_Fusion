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
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

class cextract_network
{
public:
  cextract_network();

  /**********************************************************************
   * Extract roads, shoulder lanelets and stop lines from lanelet map
   ***********************************************************************/
  bool ll_map_extract(
    const lanelet::LaneletMapPtr & map_ptr, lanelet::ConstLanelets & all,
    lanelet::ConstLanelets & regular, lanelet::ConstLanelets & shoulder);

  /**************************************************************
   * Extract street network from openstreetmap-map
   * => extract linestrings that have certain attributes
   ***************************************************************/
  bool osm_map_extract(
    const lanelet::LaneletMapPtr & map_ptr, lanelet::LineStrings3d & all,
    lanelet::ConstLineStrings3d & motorways, lanelet::ConstLineStrings3d & highways,
    lanelet::ConstLineStrings3d & roads);

private:
  /*****************************************************************************
   * Return linestrings out of a map that have specific values for the key
   * "highway"
   ******************************************************************************/
  lanelet::LineStrings3d query_osm_network(
    const lanelet::LaneletMapPtr & map_ptr, const std::vector<std::string> & values);

  /*****************************************************************************
   * Return lanelets that are of a specific subtype specified in the vector
   ******************************************************************************/
  lanelet::ConstLanelets subtype_lanelets(
    const lanelet::ConstLanelets & lls, const std::vector<std::string> & subtypes);
};
