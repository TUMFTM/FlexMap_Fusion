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
// Date: 13.10.2023
// ========================================== //
//
//
#include "message_conversion.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

/**************/
/*Constructors*/
/**************/

cmessage_conversion::cmessage_conversion()
{
}

/****************/
/*public methods*/
/****************/

void cmessage_conversion::toGeomMsgPt(
  const geometry_msgs::msg::Point32 & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::cerr << __FUNCTION__ << "pointer is null!";
    return;
  }
  dst->x = src.x;
  dst->y = src.y;
  dst->z = src.z;
}

void cmessage_conversion::toGeomMsgPt32(
  const Eigen::Vector3d & src, geometry_msgs::msg::Point32 * dst)
{
  if (dst == nullptr) {
    std::cerr << __FUNCTION__ << "pointer is null!" << std::endl;
    return;
  }
  dst->x = static_cast<float>(src.x());
  dst->y = static_cast<float>(src.y());
  dst->z = static_cast<float>(src.z());
}
