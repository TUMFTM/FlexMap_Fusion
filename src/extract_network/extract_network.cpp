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
#include "extract_network.hpp"

#include <iostream>
#include <string>
#include <vector>

/**************/
/*Constructors*/
/**************/

cextract_network::cextract_network()
{
}

/****************/
/*public methods*/
/****************/

/**********************************************************************
 * Extract roads, shoulder lanelets and stop lines from lanelet map
 ***********************************************************************/
bool cextract_network::ll_map_extract(
  const lanelet::LaneletMapPtr & map_ptr, lanelet::ConstLanelets & all,
  lanelet::ConstLanelets & regular, lanelet::ConstLanelets & shoulder,
  std::vector<lanelet::ConstLineString3d> & stop_lines)
{
  // Get elements from lanelet map
  // Lanelets
  all = lanelet::utils::query::laneletLayer(map_ptr);
  shoulder = lanelet::utils::query::shoulderLanelets(all);
  const std::vector<std::string> regular_subtypes{
    "highway", "road", "play_street", "bus_lane", "bicycle_lane"};
  regular = subtype_lanelets(all, regular_subtypes);

  // Stop lines and traffic lights
  stop_lines = lanelet::utils::query::stopLinesLanelets(regular);
  return true;
}

/**************************************************************
 * Extract street network from openstreetmap-map
 * => extract linestrings that have certain attributes
 ***************************************************************/
bool cextract_network::osm_map_extract(
  const lanelet::LaneletMapPtr & map_ptr, lanelet::LineStrings3d & all,
  lanelet::ConstLineStrings3d & motorways, lanelet::ConstLineStrings3d & highways,
  lanelet::ConstLineStrings3d & roads)
{
  std::vector<std::string> val_motorway{"motorway", "motorway_link"};
  std::vector<std::string> val_highway{"primary",  "primary_link",  "secondary", "secondary_link",
                                       "tertiary", "tertiary_link", "trunk",     "trunk_link"};
  std::vector<std::string> val_road{"unclassified", "residential", "service"};

  lanelet::LineStrings3d ls_motorways = query_osm_network(map_ptr, val_motorway);
  lanelet::LineStrings3d ls_highways = query_osm_network(map_ptr, val_highway);
  lanelet::LineStrings3d ls_roads = query_osm_network(map_ptr, val_road);

  all.insert(all.end(), ls_motorways.begin(), ls_motorways.end());
  all.insert(all.end(), ls_highways.begin(), ls_highways.end());
  all.insert(all.end(), ls_roads.begin(), ls_roads.end());

  // Set constant street types motorways, highways, roads
  for (const auto & ls : ls_motorways) {
    motorways.push_back(ls);
  }
  for (const auto & ls : ls_highways) {
    highways.push_back(ls);
  }
  for (const auto & ls : ls_roads) {
    roads.push_back(ls);
  }
  // std::cout << motorways.size() << std::endl
  //          << highways.size() << std::endl
  //          << roads.size() << std::endl
  //          << all.size()
  //          << std::endl;
  return true;
}

/****************/
/*private methods*/
/****************/

/*****************************************************************************
 * Return linestrings out of a map that have specific values for the key
 * "highway"
 ******************************************************************************/
lanelet::LineStrings3d cextract_network::query_osm_network(
  const lanelet::LaneletMapPtr & map_ptr, const std::vector<std::string> & values)
{
  lanelet::LineStrings3d res;
  std::string key = "highway";

  // Iterate through linestrings in map
  for (const auto & linestring : map_ptr->lineStringLayer) {
    // Get value of linestring tag at specified key
    // (=> key in OSM, = highway for any kind of road in OSM)
    const std::string val = linestring.attributeOr(key, "none");

    // Add linestring to output linestrings if it matches one of the provided values
    for (const auto it : values) {
      if (val == it) {
        res.push_back(linestring);
      }
    }
  }
  return res;
}

/*****************************************************************************
 * Return lanelets that are of a specific subtype specified in the vector
 ******************************************************************************/
lanelet::ConstLanelets cextract_network::subtype_lanelets(
  const lanelet::ConstLanelets & lls, const std::vector<std::string> & subtypes)
{
  lanelet::ConstLanelets subtype_ll;

  for (const auto & ll : lls) {
    if (ll.hasAttribute(lanelet::AttributeName::Subtype)) {
      const std::string attr = ll.attribute(lanelet::AttributeName::Subtype).value();
      if (std::find(subtypes.begin(), subtypes.end(), attr) != subtypes.end()) {
        subtype_ll.push_back(ll);
      }
    }
  }
  return subtype_ll;
}
