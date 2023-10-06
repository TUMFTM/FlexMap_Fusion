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
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>

#include <algorithm>
#include <string>
#include <vector>

/**************************************
 * Struct to represent controlpoint
 ***************************************/
struct s_control_point
{
public:
  s_control_point(const double vx, const double vy, const double ux, const double uy);
  lanelet::Point3d get_source_point() const;
  lanelet::Point3d get_target_point() const;

private:
  lanelet::Point3d v;  // Source point
  lanelet::Point3d u;  // Target point
};

/**************************************
 * Struct to represent match
 ***************************************/
struct s_match
{
public:
  s_match(
    const lanelet::LineStrings3d & ref_pl, const lanelet::LineStrings3d & target_pl,
    const lanelet::Areas & buf);
  void set_geo_measures(
    const double & d_bet, const double & d_l, const double & d_cho, const double & d_pol,
    const double & d_cham, const double & len_ref_pl, const double & s);
  lanelet::LineStrings3d ref_pline() const;
  lanelet::LineStrings3d target_pline() const;
  lanelet::Areas buffers() const;
  double d_ang() const;
  double d_len() const;
  double d_chord() const;
  double d_poly() const;
  double d_chamfer() const;
  double len_ref_pline() const;
  double score() const;
  lanelet::ConstLineStrings3d match_conn() const;
  void update_ref_tags(
    const std::string & key, const lanelet::Id & first_Id, const lanelet::Id & second_Id,
    const int ind);
  void remove_tag_ref_pline(const lanelet::Id & id);

private:
  lanelet::LineStrings3d ref_pl;
  lanelet::LineStrings3d target_pl;
  lanelet::Areas buf;
  double d_bet;
  double d_l;
  double d_cho;
  double d_pol;
  double d_cham;
  double len_ref_pl;
  double s;

  lanelet::Point3d closest_on_lss(lanelet::Point3d & pt, const lanelet::LineStrings3d & lss) const;
};

/********************
 * Control points
 *********************/

s_control_point::s_control_point(const double vx, const double vy, const double ux, const double uy)
{
  // Source point
  this->v.setId(lanelet::utils::getId());
  this->v.x() = vx;
  this->v.y() = vy;
  this->v.z() = 0.0;

  // Target point
  this->u.setId(lanelet::utils::getId());
  this->u.x() = ux;
  this->u.y() = uy;
  this->u.z() = 0.0;
}

lanelet::Point3d s_control_point::get_source_point() const
{
  return this->v;
}
lanelet::Point3d s_control_point::get_target_point() const
{
  return this->u;
}

/****************
 * Match
 ****************/

s_match::s_match(
  const lanelet::LineStrings3d & ref_pl, const lanelet::LineStrings3d & target_pl,
  const lanelet::Areas & buf)
{
  this->ref_pl = ref_pl;
  this->target_pl = target_pl;
  this->buf = buf;
}
lanelet::LineStrings3d s_match::ref_pline() const
{
  return this->ref_pl;
}
lanelet::LineStrings3d s_match::target_pline() const
{
  return this->target_pl;
}
lanelet::Areas s_match::buffers() const
{
  return this->buf;
}
double s_match::d_ang() const
{
  return this->d_bet;
}
double s_match::d_len() const
{
  return this->d_l;
}
double s_match::d_chord() const
{
  return this->d_cho;
}
double s_match::d_poly() const
{
  return this->d_pol;
}
double s_match::d_chamfer() const
{
  return this->d_cham;
}
double s_match::len_ref_pline() const
{
  return this->len_ref_pl;
}
double s_match::score() const
{
  return this->s;
}

/*******************************************************
 * Set geosimilarity values for each match
 ********************************************************/
void s_match::set_geo_measures(
  const double & d_bet, const double & d_l, const double & d_cho, const double & d_pol,
  const double & d_cham, const double & len_ref_pl, const double & s)
{
  this->d_bet = d_bet;
  this->d_l = d_l;
  this->d_cho = d_cho;
  this->d_pol = d_pol;
  this->d_cham = d_cham;
  this->len_ref_pl = len_ref_pl;
  this->s = s;
}

/*****************************************************************************
 * Return connection lines of a match between reference and target polyline
 ******************************************************************************/
lanelet::ConstLineStrings3d s_match::match_conn() const
{
  lanelet::ConstLineStrings3d match_connections;
  if (!this->target_pl.empty()) {
    for (const auto & ls : this->target_pl) {
      lanelet::Point3d pt(
        lanelet::utils::getId(), (ls.front().x() + ls.back().x()) / 2.0,
        (ls.front().y() + ls.back().y()) / 2.0, (ls.front().z() + ls.back().z()) / 2.0);
      lanelet::Point3d pt_ = closest_on_lss(pt, this->ref_pl);
      lanelet::LineString3d ls_(lanelet::utils::getId(), {pt, pt_});
      match_connections.push_back(ls_);
    }
  }
  return match_connections;
}

/***********************************************************************************************
 * update lanelet forward/backward tags of reference polyline after lanelets have been splitted
 ************************************************************************************************/
void s_match::update_ref_tags(
  const std::string & key, const lanelet::Id & first_Id, const lanelet::Id & second_Id,
  const int ind)
{
  for (auto it = this->ref_pl.begin() + ind; it != this->ref_pl.end(); ++it) {
    auto obj = *it;
    if (obj.hasAttribute(key)) {
      if (*obj.attribute(key).asId() == first_Id) {
        obj.attributes()[key] = second_Id;
      }
    }
  }
}

/***********************************************************************************************
 * remove an id that is a attribute value of the reference polyline after the corresponding
 * lanelet has been deleted
 ************************************************************************************************/
void s_match::remove_tag_ref_pline(const lanelet::Id & id)
{
  for (auto & seg : this->ref_pl) {
    auto & attr = seg.attributes();
    auto it_to_delete = attr.begin();
    bool found = false;
    for (auto it = attr.begin(); it != attr.end(); ++it) {
      if (it->second == std::to_string(id)) {
        it_to_delete = it;
        found = true;
        break;
      }
    }
    if (found) {
      attr.erase(it_to_delete);
    }
  }
}

/*****************************************************************************
 * Return closest point to another point on a set on linestring segments
 ******************************************************************************/
lanelet::Point3d s_match::closest_on_lss(
  lanelet::Point3d & pt, const lanelet::LineStrings3d & lss) const
{
  std::vector<double> dist;
  lanelet::Points3d pts;
  if (lss.empty()) {
    std::cerr << "\033[31m" << __FUNCTION__ << ": LineStrings are empty!\033[0m" << std::endl;
  }

  for (const auto & ls : lss) {
    lanelet::Point3d pt_(
      lanelet::utils::getId(), (ls.front().x() + ls.back().x()) / 2.0,
      (ls.front().y() + ls.back().y()) / 2.0, (ls.front().z() + ls.back().z()) / 2.0);
    pts.push_back(pt_);
    dist.push_back(lanelet::geometry::distance2d(pt, pt_));
  }
  const int ind = std::distance(dist.begin(), std::min_element(dist.begin(), dist.end()));
  return pts[ind];
}
