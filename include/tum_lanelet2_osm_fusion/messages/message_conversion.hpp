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
// Date: 13.10.2023
// ==========================================
//
//
#pragma once

#include "color.hpp"
#include "utility.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_io/Io.h>

#include <string>
#include <utility>
#include <vector>

class cmessage_conversion
{
public:
  cmessage_conversion();

  void toGeomMsgPt(const geometry_msgs::msg::Point32 & src, geometry_msgs::msg::Point * dst);
  void toGeomMsgPt32(const Eigen::Vector3d & src, geometry_msgs::msg::Point32 * dst);
};