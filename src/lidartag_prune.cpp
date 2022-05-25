/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE. The views and conclusions contained in the
 * software and documentation are those of the authors and should not be
 * interpreted as representing official policies, either expressed or implied,
 * of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.brucerobot.com/
 */

#include <lidartag/lidartag.hpp>
#include <lidartag/ultra_puck.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>

#include <fstream>

#define SQRT2 1.41421356237

using namespace std;

namespace BipedLab
{
/*
 * A valid cluster, valid tag, the points from the original point cloud that
 * belong to the cluster could be estimated from the LiDAR system Therefore, if
 * the points on the tag is too less, which means it is not a valid tag where it
 * might just a shadow of a valid tag
 */
bool LidarTag::clusterPointsCheck(ClusterFamily_t & cluster)
{
  auto distance = sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2));

  int max_points = LidarTag::areaPoints(distance, payload_size_ * SQRT2, payload_size_ * SQRT2);
  int min_points = LidarTag::areaPoints(distance, payload_size_ / SQRT2, payload_size_ / SQRT2);

  if (cluster.data.size() < min_points) {  // cluster.data.size() > maxPoints) { // ||
                                          // cluster.data.size() < minPoints) {
    return false;
  } else {
    return true;
  }
}

/*
 * A function to get a number of points on a given-distance tag or object
 */
int LidarTag::areaPoints(
  const double & distance, const double & obj_width, const double & obj_height)
{
  // double WAngle = obj_width * (1 + SQRT2) / abs(distance);

  // if (WAngle>=1) return (int) 1e6; // return big number to reject the cluster

  // double HAngle = obj_height * (1 + SQRT2) / abs(distance);
  // if (HAngle>=1) return (int) 1e6; // return big number to reject the cluster

  // double HorizontalAngle = asin(WAngle); // in radian
  // double VerticalAngle = asin(HAngle); // in radian
  // int num_of_vertical_ring = floor(VerticalAngle *
  // lidar_system_.beam_per_vertical_radian); int num_of_horizontal_points =
  // floor(HorizontalAngle * lidar_system_.point_per_horizontal_radian);

  // // use 3 instead of 2 becasue of we assume the tag would be put in the
  // dense
  // // region of LiDAR (LiDAR is denser in the middle)
  // int Area = floor(3 * (num_of_vertical_ring * num_of_horizontal_points) / (1 +
  // SQRT2));

  // cout << "distance: " << distance << endl;
  // //cout << "HorizontalAngle: " << HorizontalAngle << endl;
  // //cout << "VerticalAngle: " << VerticalAngle << endl;

  int num_of_horizontal_points = ceil(obj_width / (distance * tan(0.1 * M_PI / 180)));

  // int num_of_horizontal_points = 2 * atan((obj_width / 2) / abs(distance)) *
  //    lidar_system_.point_per_horizontal_radian;
  double half_vertical_angle = atan((obj_height / 2) / abs(distance)) * 180 / M_PI;

  int num_of_vertical_ring = 0;
  for (int i = 0; i < UltraPuckV2::beams; ++i) {
    if (half_vertical_angle > abs(UltraPuckV2::el[i])) {
      num_of_vertical_ring++;
    }
  }
  int area = num_of_vertical_ring * num_of_horizontal_points;

  // cout << "num_of_vertical_ring: " << num_of_vertical_ring << endl;
  // cout << "num_of_horizontal_points: " << num_of_horizontal_points << endl;
  // cout << "Area: " << Area << endl;
  // cout << "Points / Radian: " << lidar_system_.point_per_horizontal_radian <<
  // endl;

  return area;
}

/*
 * A function to calculate the upper bound of points that can exist in a cluster
 * based on the payload size
 */
int LidarTag::maxPointsCheck(ClusterFamily_t & cluster)
{
  int ring = std::round(beam_num_ / 2);
  int longer_side_ring = std::round((cluster.top_ring + cluster.bottom_ring) / 2);
  double point_resolution = 2 * M_PI / lidar_system_.ring_average_table[longer_side_ring].average;
  auto distance =
    std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
  float payload_w = SQRT2 * payload_size_;
  int num_horizontal_points = std::ceil(payload_w / (distance * std::sin(point_resolution)));
  int num_vertical_ring = std::abs(cluster.top_ring - cluster.bottom_ring) + 1;
  int expected_points = num_vertical_ring * num_horizontal_points;

  if ((cluster.data.size() + cluster.edge_points.size()) > expected_points + 700) {
    result_statistics_.cluster_removal.maximum_return++;
    result_statistics_.remaining_cluster_size--;

    if (mark_cluster_validity_) {
      cluster.valid = false;
      cluster.detail_valid = LidartagErrorCode::ClusterMinPointsCriteria2;
    }
  }

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== maxPointsCheck ====");
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance << ", num_horizontal_points: "
      << num_horizontal_points);
    RCLCPP_DEBUG_STREAM(get_logger(), "Expected Points: " << expected_points);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: "
      << cluster.data.size() + cluster.edge_points.size());

    if ((cluster.data.size() + cluster.edge_points.size()) > expected_points)
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    else
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << true);
  }
  return expected_points;
}

/*
 * Fit a plane to a cluster. Returns false if unable to estimate a plane.
 * Otherwise, returns the number of inliers and the coefficients of the plane.
 */
bool LidarTag::rejectWithPlanarCheck(
  ClusterFamily_t & cluster, pcl::PointIndices::Ptr inliers,
  pcl::ModelCoefficients::Ptr coefficients, std::ostream & fplanefit)
{
  // Convert cluster family into pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.resize(cluster.data.size() + cluster.edge_points.size());

  for (std::size_t i = 0; i < cluster.edge_points.size(); ++i) {
    cloud->points[i].x = cluster.edge_points[i].point.x;
    cloud->points[i].y = cluster.edge_points[i].point.y;
    cloud->points[i].z = cluster.edge_points[i].point.z;
  }

  for (std::size_t i = cluster.edge_points.size(); i < cloud->points.size(); ++i) {
    cloud->points[i].x = cluster.data[i - cluster.edge_points.size()].point.x;
    cloud->points[i].y = cluster.data[i - cluster.edge_points.size()].point.y;
    cloud->points[i].z = cluster.data[i - cluster.edge_points.size()].point.z;
  }

  // Create segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(params_.distance_to_plane_threshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  cluster.inliers = inliers->indices.size();
  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== rejectWithPlanarCheck ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: "
      << cluster.data.size() + cluster.edge_points.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "Inliers     : " << inliers->indices.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "Outliers    : "
      << cluster.data.size() - inliers->indices.size());
  }
  if (inliers->indices.size() == 0) {
    if (debug_info_) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
      RCLCPP_WARN_STREAM(get_logger(), "Failed to fit a plane model to the cluster id="
        << cluster.cluster_id);
    }

    result_statistics_.cluster_removal.plane_fitting++;
    result_statistics_.remaining_cluster_size--;

    return false;
  }

  if (debug_info_)
    RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << true);
  if (log_data_) {
    fplanefit << "-----------------begining--------------------" << endl;
    fplanefit << "Successfully fit plane!" << endl;
    fplanefit << "Cluster Size: " << cluster.data.size() << endl;
    fplanefit << "Inliers     : " << inliers->indices.size() << endl;
    fplanefit << "Outliers    : " << cluster.data.size() - inliers->indices.size() << endl;
  }

  return true;
}

}  // namespace BipedLab
