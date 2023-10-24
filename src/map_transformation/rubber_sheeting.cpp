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

#include "rubber_sheeting.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

/**************/
/*Constructors*/
/**************/

crubber_sheeting::crubber_sheeting()
{
}

/****************/
/*public methods*/
/****************/

/*****************************************************************
 * Get controlpoints from RVIZ as published point by the user
 ******************************************************************/
bool crubber_sheeting::select_control_points(
  rclcpp::Node & node, const lanelet::ConstLineString3d & src,
  const lanelet::ConstLineString3d & target, std::vector<s_control_point> & cps)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  cps.clear();

  // Get amount of control points from user input
  const int num_control_points = node.get_parameter("rs_num_controlPoints").as_int();
  std::cout << "\033[33m~~~~~> Select " << num_control_points << " control points!\033[0m"
            << std::endl;

  // Loop through amount of control points * 2 (source and target coordinates) to get messages
  // from RVIZ
  geometry_msgs::msg::PointStamped msg;
  lanelet::Points3d cp_inter;
  int cp_count = 0;

  for (int i = 0; i < num_control_points * 2; ++i) {
    // Output instruction for user
    if (cp_inter.size() == 0) {
      std::cout << "\033[32m~~~~~~~~~~> Publish a point on the master trajectory (green) "
                << "in RVIZ!\033[0m" << std::endl;
    } else {
      std::cout << "\033[32m~~~~~~~~~~> Publish a point on the aligned target trajectory (blue) "
                << "in RVIZ!\033[0m" << std::endl;
    }

    // Create subscription and wait for point message
    auto sub = node.create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, [](const std::shared_ptr<geometry_msgs::msg::PointStamped>) {});
    bool received_msg = rclcpp::wait_for_message(msg, sub, node.get_node_options().context());

    // Convert message to lanelet point and write in array
    lanelet::Point3d pt(lanelet::utils::getId(), msg.point.x, msg.point.y, 0.0);

    // Find closest point on corresponding trajectory
    if (cp_inter.size() == 0) {
      closest_on_ls(pt, src);
    } else {
      closest_on_ls(pt, target);
    }

    // Add selected point to intermediate vector and output coordinates
    cp_inter.push_back(pt);
    std::cout << "\033[34m~~~~~~~~~~> Selected point: x = " << pt.x() << "m, y = " << pt.y()
              << "m!\033[0m" << std::endl;

    // Add to controlpoint if complete otherwise repeat for second point
    if (received_msg) {
      if (cp_inter.size() == 2) {
        s_control_point P(cp_inter[0].x(), cp_inter[0].y(), cp_inter[1].x(), cp_inter[1].y());
        cps.push_back(P);
        ++cp_count;

        std::cout << "\033[1;32m~~~~~~~~~~> Controlpoint " << cp_count << " added!\033[0m"
                  << std::endl;
        cp_inter.clear();
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(node_name), "You didn't publish a point!");
    }
  }
  return true;
}

/************************************************************************************
 * Calculate triangles and transformation matrices for rubber-sheet transformation
 * according to:
 * White 1985: Piecewise Linear Rubber-Sheet Map Transformation
 *************************************************************************************/
bool crubber_sheeting::get_transformation(
  rclcpp::Node & node, const lanelet::ConstLineString3d & target,
  std::vector<s_control_point> & cps, lanelet::Areas & tri, std::vector<Eigen::Matrix3d> & trans)
{
  (void)node;
  // Compute rectangle that is 5% larger than necessary for rubber sheet
  lanelet::Area target_rec = enclosing_rectangle(target);
  // Compute base rectangle
  lanelet::Area src_rec = src_rectangle(target_rec, cps);
  // Add corner points to control points
  corner2cp(target_rec, src_rec, cps);
  // Triangulation
  triangulation(cps, tri);
  // Calculate transoformation matrix for each triangle
  tranformation_matrices(cps, tri, trans);

  return true;
}

/*****************************************************************
 * Transform whole map according to rubber-sheet trafo
 ******************************************************************/
bool crubber_sheeting::transform_map(
  const lanelet::LaneletMapPtr & map_ptr, const lanelet::Areas & tri,
  const std::vector<Eigen::Matrix3d> & trans)
{
  for (auto & pt : map_ptr->pointLayer) {
    // Find area the point is in
    int i = 0;
    for (auto & triangle : tri) {
      if (lanelet::geometry::inside(triangle, lanelet::utils::to2D(pt).basicPoint())) {
        // Transform point
        transform_pt(pt, trans[i]);
        break;
      }
      ++i;
    }
  }
  return true;
}

/***********************************************************************
 * Transfrom linestring according to rubber-sheet transformation
 ************************************************************************/
bool crubber_sheeting::transform_ls(
  const lanelet::ConstLineString3d & ls, lanelet::ConstLineString3d & ls_trans,
  const lanelet::Areas & tri, const std::vector<Eigen::Matrix3d> & trans)
{
  // Initialize output linestring to be filled -> non-const
  lanelet::LineString3d ls_t(lanelet::utils::getId(), {});

  // Transform geometry
  for (auto & pt : ls) {
    // Find area the point is in
    int i = 0;
    for (auto & triangle : tri) {
      if (lanelet::geometry::inside(triangle, lanelet::utils::to2D(pt).basicPoint())) {
        const Eigen::Vector3d point(pt.x(), pt.y(), 1.0);
        const Eigen::Vector3d pt_trans = trans[i] * point;
        lanelet::Point3d pt_t(lanelet::utils::getId(), {pt_trans(0), pt_trans(1), 0.0});
        ls_t.push_back(pt_t);
        break;
      }
      ++i;
    }
  }
  ls_trans = ls_t;
  return true;
}

/*********************************************************************************
 * Transform corresponding pointcloud map to lanelet map (only x-/y-coordinates)
 **********************************************************************************/
bool crubber_sheeting::transform_pcd(
  rclcpp::Node & node, const lanelet::Areas & tri, const std::vector<Eigen::Matrix3d> & trans,
  const Eigen::Matrix3d & trans_al)
{
  const std::string pcd_path = node.get_parameter("pcd_path").as_string();

  // Read input cloud based on provided file path
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file Point cloud!\n");
    return -1;
  }
  std::cout << "\033[1;36mPoint Cloud with " << cloud->width * cloud->height
            << " points: Loaded!\033[0m" << std::endl;

  // create new cloud for output
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  cloud_out.width = cloud->width;
  cloud_out.height = cloud->height;
  cloud_out.is_dense = false;
  cloud_out.points.resize(cloud_out.width * cloud_out.height);

  // Transform points and write into output cloud
  int ind_pt = 0;
  for (const auto & point : *cloud) {
    // Align point with alignment transformation matrix
    int i = 0;
    const Eigen::Vector3d pt_(point.x, point.y, 1.0);
    const Eigen::Vector3d pt_al = trans_al.inverse() * pt_;

    // Create new, aligned point
    lanelet::Point3d pt(lanelet::utils::getId(), pt_al(0), pt_al(1), 1.0);

    // Find area the point is in
    for (auto & triangle : tri) {
      if (lanelet::geometry::inside(triangle, lanelet::utils::to2D(pt).basicPoint())) {
        // Rubber-sheet point
        const Eigen::Vector3d pt_rs = trans[i] * pt_al;
        cloud_out[ind_pt].x = pt_rs(0);
        cloud_out[ind_pt].y = pt_rs(1);
        cloud_out[ind_pt].z = point.z;
        break;
      }
      ++i;
    }
    ++ind_pt;
  }

  // write to file
  if (node.get_parameter("save_ascii").as_bool()) {
    pcl::io::savePCDFileASCII("out_pcd.pcd", cloud_out);
  } else {
    pcl::io::savePCDFileBinary("out_pcd.pcd", cloud_out);
  }
  std::cout << "\033[1;36mPoint Cloud transformed and written to out_pcd.pcd!\033[0m" << std::endl;
  return true;
}

/*********************************************************************************
 * Transform corresponding pointcloud map according to SLAM poses - GPS traj
 **********************************************************************************/
bool crubber_sheeting::transform_pcd(
  rclcpp::Node & node, const lanelet::Areas & tri, const std::vector<Eigen::Matrix3d> & trans,
  const Eigen::Matrix3d & trans_al, pcl::PointCloud<pcl::PointXYZ>::Ptr & pcm)
{
  // Get point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud = *pcm;
  // Transform points and write into output cloud
  int ind_pt = 0;
  for (auto & point : cloud) {
    // Align point with alignment transformation matrix
    int i = 0;
    const Eigen::Vector3d pt_(point.x, point.y, 1.0);
    const Eigen::Vector3d pt_al = trans_al.inverse() * pt_;

    // Create new, aligned point
    lanelet::BasicPoint2d pt(pt_al(0), pt_al(1));

    // Find area the point is in
    for (auto & triangle : tri) {
      if (lanelet::geometry::inside(triangle, pt)) {
        // Rubber-sheet point
        const Eigen::Vector3d pt_rs = trans[i] * pt_al;
        cloud[ind_pt].x = pt_rs(0);
        cloud[ind_pt].y = pt_rs(1);
        cloud[ind_pt].z = point.z;
        break;
      }
      ++i;
    }
    ++ind_pt;
  }

}

/*****************/
/*private methods*/
/*****************/

/**************************************************************
 * Find closest point on given linestring for given point
 ***************************************************************/
void crubber_sheeting::closest_on_ls(lanelet::Point3d & pt, const lanelet::ConstLineString3d & ls)
{
  // Initialize
  std::vector<double> dist;
  for (const auto & pt_ : ls) {
    double d = lanelet::geometry::distance2d(pt, pt_);
    dist.push_back(d);
  }
  const int ind = std::distance(dist.begin(), std::min_element(dist.begin(), dist.end()));
  pt.x() = ls[ind].x();
  pt.y() = ls[ind].y();
  pt.z() = ls[ind].z();
}

/************************************************************
 * Define enclosing target rectangle for rubber-sheeting
 *************************************************************/
lanelet::Area crubber_sheeting::enclosing_rectangle(const lanelet::ConstLineString3d & target)
{
  // Get maximum x and y values from target trajectory
  std::vector<double> x, y;

  for (const auto & pt : target) {
    x.push_back(pt.x());
    y.push_back(pt.y());
  }

  double min_x = *std::min_element(x.begin(), x.end());
  double max_x = *std::max_element(x.begin(), x.end());
  double min_y = *std::min_element(y.begin(), y.end());
  double max_y = *std::max_element(y.begin(), y.end());

  // Get 5% of size of rectangle
  double dx, dy;
  dx = 0.05 * std::abs(max_x - min_x);
  dy = 0.05 * std::abs(max_y - min_y);

  // Define area
  lanelet::Point3d bottom_left{lanelet::utils::getId(), min_x - dx, min_y - dy, 0.0};
  lanelet::Point3d top_left{lanelet::utils::getId(), min_x - dx, max_y + dy, 0.0};
  lanelet::Point3d top_right{lanelet::utils::getId(), max_x + dx, max_y + dy, 0.0};
  lanelet::Point3d bottom_right{lanelet::utils::getId(), max_x + dx, min_y - dy, 0.0};

  lanelet::LineString3d ls1(lanelet::utils::getId(), {bottom_left, top_left});
  lanelet::LineString3d ls2(lanelet::utils::getId(), {top_left, top_right});
  lanelet::LineString3d ls3(lanelet::utils::getId(), {top_right, bottom_right});
  lanelet::LineString3d ls4(lanelet::utils::getId(), {bottom_right, bottom_left});

  lanelet::Area ar(lanelet::utils::getId(), {ls1, ls2, ls3, ls4});
  return ar;
}

/******************************************************************************
 * Compute source rectangle from control points and target rectangle
 *******************************************************************************/
lanelet::Area crubber_sheeting::src_rectangle(
  const lanelet::Area & target_rec, const std::vector<s_control_point> & cps)
{
  lanelet::Points3d pts;

  for (auto & ls : target_rec.outerBound()) {
    // Iterate through controlpoints to find closest one
    std::vector<double> vx, vy, d;
    for (auto & cpt : cps) {
      vx.push_back(ls[0].x() + cpt.get_source_point().x() - cpt.get_target_point().x());
      vy.push_back(ls[0].y() + cpt.get_source_point().y() - cpt.get_target_point().y());
      lanelet::Point3d pt(lanelet::utils::getId(), vx.back(), vy.back(), 0.0);

      d.push_back(std::abs(lanelet::geometry::distance2d(pt, cpt.get_source_point())));
    }
    const int ind = std::distance(d.begin(), std::min_element(d.begin(), d.end()));
    lanelet::Point3d pt_rec(lanelet::utils::getId(), vx[ind], vy[ind], 0.0);
    pts.push_back(pt_rec);
  }

  // Construct area
  lanelet::LineString3d ls1(lanelet::utils::getId(), {pts[0], pts[1]});
  lanelet::LineString3d ls2(lanelet::utils::getId(), {pts[1], pts[2]});
  lanelet::LineString3d ls3(lanelet::utils::getId(), {pts[2], pts[3]});
  lanelet::LineString3d ls4(lanelet::utils::getId(), {pts[3], pts[0]});
  lanelet::Area ar(lanelet::utils::getId(), {ls1, ls2, ls3, ls4});
  return ar;
}

/**********************************************************************
 * Add points from source and target rectangle to control points
 ***********************************************************************/
void crubber_sheeting::corner2cp(
  const lanelet::Area & target_rec, const lanelet::Area & src_rec,
  std::vector<s_control_point> & cps)
{
  lanelet::ConstLineStrings3d ls_target = target_rec.outerBound();
  lanelet::ConstLineStrings3d ls_src = src_rec.outerBound();

  for (size_t i = 0; i < ls_target.size(); ++i) {
    s_control_point P(ls_src[i][0].x(), ls_src[i][0].y(), ls_target[i][0].x(), ls_target[i][0].y());
    cps.push_back(P);
  }
}

/*********************************************************************
 * Calculate triangluation according to
 * White 1985: Piecewise Linear Rubber-Sheet Map Transformation
 **********************************************************************/
void crubber_sheeting::triangulation(const std::vector<s_control_point> & cps, lanelet::Areas & tri)
{
  // Construct base triangles (diagonal through rectangle)
  lanelet::Point3d bottom_left = (cps.end() - 4)->get_target_point();
  lanelet::Point3d top_left = (cps.end() - 3)->get_target_point();
  lanelet::Point3d top_right = (cps.end() - 2)->get_target_point();
  lanelet::Point3d bottom_right = (cps.end() - 1)->get_target_point();

  lanelet::LineString3d ls11(lanelet::utils::getId(), {bottom_left, top_left});
  lanelet::LineString3d ls12(lanelet::utils::getId(), {top_left, top_right});
  lanelet::LineString3d ls13(lanelet::utils::getId(), {top_right, bottom_left});
  lanelet::Area delta_1(lanelet::utils::getId(), {ls11, ls12, ls13});

  lanelet::LineString3d ls21(lanelet::utils::getId(), {bottom_left, top_right});
  lanelet::LineString3d ls22(lanelet::utils::getId(), {top_right, bottom_right});
  lanelet::LineString3d ls23(lanelet::utils::getId(), {bottom_right, bottom_left});
  lanelet::Area delta_2(lanelet::utils::getId(), {ls21, ls22, ls23});

  tri.push_back(delta_1);
  tri.push_back(delta_2);

  for (auto it = cps.begin(); it != cps.end() - 4; ++it) {
    lanelet::Point3d pt = it->get_target_point();

    // Get current amount of triangles
    int sz_tr = tri.size();
    int ind_tri = 0;
    // Find triangle the control point is in
    for (int j = 0; j < sz_tr; ++j) {
      // Check if point is inside triangle
      if (lanelet::geometry::inside(tri[j], lanelet::utils::to2D(pt).basicPoint())) {
        // Get outer Bound
        lanelet::LineStrings3d outer = tri[j].outerBound();

        // Create 3 new areas and add them to triangles
        for (auto & ls : outer) {
          lanelet::LineString3d ls2(lanelet::utils::getId(), {ls[1], pt});
          lanelet::LineString3d ls3(lanelet::utils::getId(), {pt, ls[0]});

          lanelet::Area ar(lanelet::utils::getId(), {ls, ls2, ls3});
          tri.push_back(ar);
        }
        ind_tri = j;
      }
    }
    // Delete old triangle
    tri.erase(tri.begin() + ind_tri);

    // Perform quadrilateral test on three latest triangles
    quadrilateral_test(tri);
  }
}

/**********************************************************
 * Calculate transformation matrices for triangles
 * => solve linear equations for three points
 ***********************************************************/
void crubber_sheeting::tranformation_matrices(
  const std::vector<s_control_point> & cps, const lanelet::Areas & tri,
  std::vector<Eigen::Matrix3d> & trans)
{
  lanelet::ConstPoints3d target;
  lanelet::ConstPoints3d source;

  for (auto & triangle : tri) {
    // Clean up vector
    target.clear();
    source.clear();

    // Get vertices of triangle
    for (auto & ls : triangle.outerBound()) {
      lanelet::ConstPoint3d vertex = ls[0];

      // Get corresponding control Point
      for (auto & cpt : cps) {
        if (cpt.get_target_point().id() == vertex.id()) {
          target.push_back(cpt.get_target_point());
          source.push_back(cpt.get_source_point());
        }
      }
    }
    // Check if 3 control Points found for triangle
    if (target.size() != 3 || source.size() != 3) {
      std::cerr << "\033[31m" << __FUNCTION__
                << ": Couldn't find three control points for triangle !!\033[0m" << std::endl;
    }

    // Construct linear equations and solve to find transformation matrix for triangle
    Eigen::Matrix3d trans_ = solve_linear(source, target);
    trans.push_back(trans_);
  }
}

/****************************************************************
 * Perfrom quadrilateral test to avoid narrow triangles
 *****************************************************************/
void crubber_sheeting::quadrilateral_test(lanelet::Areas & tri)
{
  // Iterate over last three triangles (from last iteration)
  for (auto it_tri = tri.end() - 3; it_tri != tri.end(); ++it_tri) {
    // Find adjacent triangle in other triangles if existing
    for (auto it_adj = tri.begin(); it_adj != tri.end() - 3; ++it_adj) {
      // Check if areas are adjacent
      if (lanelet::geometry::adjacent(*it_tri, *it_adj)) {
        lanelet::LineStrings3d ls_tri = it_tri->outerBound();
        lanelet::LineStrings3d ls_adj = it_adj->outerBound();

        // First diagonal always common linestring => first ls in new triangle
        lanelet::LineString3d diag_1 = *ls_tri.begin();
        // Get index of common linestring in adjacent triangle
        int i = 0;
        int ind_common = 0;
        for (auto & ls : ls_adj) {
          if (std::abs(lanelet::geometry::length(diag_1) - lanelet::geometry::length(ls)) < 1e-3) {
            ind_common = i;
          }
          ++i;
        }
        // Construct new diagonal and potential new triangles
        ls_adj.erase(ls_adj.begin() + ind_common);
        lanelet::LineString3d diag_2(lanelet::utils::getId(), {});
        lanelet::LineStrings3d ls_new1;
        lanelet::LineStrings3d ls_new2;

        if (ind_common == 0 || ind_common == 2) {
          diag_2.push_back(ls_adj[0][1]);
          diag_2.push_back(ls_tri[1][1]);
          ls_new1.push_back(ls_adj[1]);
          ls_new1.push_back(ls_tri[1]);
          ls_new1.push_back(diag_2.invert());
          ls_new2.push_back(ls_tri[2]);
          ls_new2.push_back(ls_adj[0]);
          ls_new2.push_back(diag_2);
        } else {
          diag_2.push_back(ls_adj[1][1]);
          diag_2.push_back(ls_tri[1][1]);
          ls_new1.push_back(ls_adj[0]);
          ls_new1.push_back(ls_tri[1]);
          ls_new1.push_back(diag_2.invert());
          ls_new2.push_back(ls_tri[2]);
          ls_new2.push_back(ls_adj[1]);
          ls_new2.push_back(diag_2);
        }

        lanelet::Area tri_new1(lanelet::utils::getId(), ls_new1);
        lanelet::Area tri_new2(lanelet::utils::getId(), ls_new2);
        // Check if quadrilateral is convex (= diagonals intersect)
        if (lanelet::geometry::intersects3d(diag_1, diag_2, 0.1)) {
          double h1 = std::min(triangle_height(tri_new1), triangle_height(tri_new2));
          double h2 = std::min(triangle_height(*it_tri), triangle_height(*it_adj));
          // Swap if minimum height of new triangles is greater than existing triangles
          if (h1 >= h2) {
            *it_tri = tri_new1;
            *it_adj = tri_new2;
          }
        }
      }
    }
  }
}

/****************************************************
 * Calculate the height of a triangle
 *****************************************************/
double crubber_sheeting::triangle_height(const lanelet::Area & ar)
{
  // Compute area
  const double area = triangle_area(ar);
  // Get longest side of triangle
  lanelet::ConstLineStrings3d ls = ar.outerBound();

  std::vector<double> len;
  len.push_back(lanelet::geometry::length(ls[0]));
  len.push_back(lanelet::geometry::length(ls[1]));
  len.push_back(lanelet::geometry::length(ls[2]));
  const double longest = *std::max_element(len.begin(), len.end());
  // Compute triangle height
  const double height = 2.0 * area / longest;
  return height;
}

/***********************************************
 * Calculate the area of a triangle
 ************************************************/
double crubber_sheeting::triangle_area(const lanelet::Area & ar)
{
  const lanelet::ConstLineStrings3d ls = ar.outerBound();
  // Convert first two linestrings into vectors so that they span a triangle
  const lanelet::ConstLineString3d ls1 = (*ls.begin()).invert();
  const lanelet::ConstLineString3d ls2 = *(ls.begin() + 1);

  const Eigen::Vector3d side1(
    ls1[1].x() - ls1[0].x(), ls1[1].y() - ls1[0].y(), ls1[1].z() - ls1[0].z());
  const Eigen::Vector3d side2(
    ls2[1].x() - ls2[0].x(), ls2[1].y() - ls2[0].y(), ls2[1].z() - ls2[0].z());

  // Compute norm of cross product = area of parallelogram
  const double area = 0.5 * side1.cross(side2).norm();
  return area;
}

/************************************************************
 * Transform point according to transformation matrix
 *************************************************************/
void crubber_sheeting::transform_pt(lanelet::Point3d & pt, const Eigen::Matrix3d & trans)
{
  const Eigen::Vector3d point(pt.x(), pt.y(), 1.0);
  const Eigen::Vector3d pt_trans = trans * point;
  pt.x() = pt_trans(0);
  pt.y() = pt_trans(1);
  pt.z() = pt.z();
}

/**************************************************************************
 * Solve linear equations defined by three points forming a triangle
 * => calculate rotation matrix
 ***************************************************************************/
Eigen::Matrix3d crubber_sheeting::solve_linear(
  const lanelet::ConstPoints3d & src, const lanelet::ConstPoints3d & target)
{
  // Construct linear equations
  // b
  Eigen::VectorXd b(9);
  b << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  int i = 0;

  for (auto & pt : src) {
    b(3 * i) = pt.x();
    b(3 * i + 1) = pt.y();
    ++i;
  }
  // A
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
  int j = 0;
  for (auto & pt : target) {
    A.block(3 * j, 0, 1, 3) << pt.x(), pt.y(), 1.0;
    A.block(3 * j + 1, 3, 1, 3) << pt.x(), pt.y(), 1.0;
    A.block(3 * j + 2, 6, 1, 3) << pt.x(), pt.y(), 1.0;
    ++j;
  }
  // solve
  const Eigen::VectorXd ti = A.colPivHouseholderQr().solve(b);

  // Reshape to get matrix
  Eigen::Matrix3d trans;
  trans << ti(0), ti(1), ti(2), ti(3), ti(4), ti(5), ti(6), ti(7), ti(8);
  return trans;
}
