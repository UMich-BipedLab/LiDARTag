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

#include <pcl/common/intersections.h>

#include <iostream>
#include <string>

#define SQRT2 1.41421356237

using namespace std;
using namespace std::chrono_literals;

namespace BipedLab
{
/*
 * A function to draw a point in rviz
 */
void LidarTag::assignMarker(
  visualization_msgs::msg::Marker & marker, const uint32_t shape, const string name_space, 
  const double r, const double g, const double b, const PointXYZRI & point, const int count, 
  const double size, const string text)
{
  marker.header.frame_id = lidar_frame_;
  marker.header.stamp = current_scan_time_;
  marker.ns = name_space;
  marker.id = count;
  marker.type = shape;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.text = text;
  // should disappear along with updateing rate
  marker.lifetime = rclcpp::Duration::from_seconds(sleep_time_for_vis_ * 10); 

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
}

void LidarTag::assignVectorMarker(
  visualization_msgs::msg::Marker & marker, const uint32_t shape, const string name_space, const double r,
  const double g, const double b, const int count, const double size, Eigen::Vector3f edge_vector,
  const PointXYZRI & centroid, const string text)
{
  const double tag_depth = 0.02;
  marker.header.frame_id = lidar_frame_;
  marker.header.stamp = current_scan_time_;
  marker.ns = name_space;
  marker.id = count;
  marker.type = shape;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.text = text;
  // should disappear along with updateing rate
  marker.lifetime = rclcpp::Duration::from_seconds(sleep_time_for_vis_ * 10); 
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  geometry_msgs::msg::Point p1;
  p1.x = 0;  // centroid.x;
  p1.y = 0;  // centroid.y;
  p1.z = 0;  // centroid.z;
  marker.points.push_back(p1);

  geometry_msgs::msg::Point p2;
  p2.x = edge_vector[0];
  p2.y = edge_vector[1];
  p2.z = edge_vector[2];
  marker.points.push_back(p2);
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.scale.x = tag_depth;
  marker.scale.y = size;
  marker.scale.z = size;
  // Set the color -- be sure to set alpha to something non-zero!
}
void LidarTag::plotIdealFrame()
{
  const double tag_depth = 0.02;
  visualization_msgs::msg::MarkerArray frame_mark_array;
  for (int k = 0; k < tag_size_list_.size(); ++k) {
    visualization_msgs::msg::Marker line_list;
    line_list.id = 0;
    line_list.header = point_cloud_header_;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.ns = "tag_size_" + to_string(tag_size_list_[k]);
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    line_list.scale.x = 0.01;

    vector<vector<int>> vertex = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
    for (int i = 0; i < 4; ++i) {
      vector<int> v = vertex[i];
      geometry_msgs::msg::Point p;
      p.x = -tag_depth;
      p.y = v[0] * tag_size_list_[k] / 2;  //payload_size_
      p.z = v[1] * tag_size_list_[k] / 2;
      line_list.points.push_back(p);
      p.x = tag_depth;
      line_list.points.push_back(p);
    }

    for (int j = -1; j <= 1; j += 2) {
      for (int i = 0; i < 4; ++i) {
        vector<int> v = vertex[i];
        geometry_msgs::msg::Point p;
        p.x = j * tag_depth;
        p.y = v[0] * tag_size_list_[k] / 2;
        p.z = v[1] * tag_size_list_[k] / 2;
        line_list.points.push_back(p);
        v = vertex[(i + 1) % 4];
        p.y = v[0] * tag_size_list_[k] / 2;
        p.z = v[1] * tag_size_list_[k] / 2;
        line_list.points.push_back(p);
      }
    }
    frame_mark_array.markers.push_back(line_list);
  }

  ideal_frame_pub_->publish(frame_mark_array);
}

void LidarTag::plotTagFrame(const ClusterFamily_t & cluster)
{
  visualization_msgs::msg::Marker line_list;
  line_list.id = cluster.cluster_id;
  line_list.header = point_cloud_header_;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
  // line_list.ns = "tag_size_" + to_string(tag_size_list_[k]);
  line_list.color.r = 0.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;
  line_list.scale.x = 0.01;
  line_list.lifetime = rclcpp::Duration::from_seconds(sleep_time_for_vis_ * 10);

  vector<vector<int>> vertex = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
  const double tag_depth = 0.02;
  
  for (int i = 0; i < 4; ++i) {
    vector<int> v = vertex[i];
    Eigen::Vector4f corner_lidar1(
      -tag_depth, v[0] * cluster.tag_size / 2, v[1] * cluster.tag_size / 2, 1);
    Eigen::Vector4f corner_lidar2(
      tag_depth, v[0] * cluster.tag_size / 2, v[1] * cluster.tag_size / 2, 1);
    Eigen::Vector4f corner_tag1 = cluster.pose.homogeneous * corner_lidar1;
    Eigen::Vector4f corner_tag2 = cluster.pose.homogeneous * corner_lidar2;
    geometry_msgs::msg::Point p;
    p.x = corner_tag1(0);
    p.y = corner_tag1(1);  //payload_size_
    p.z = corner_tag1(2);
    line_list.points.push_back(p);
    p.x = corner_tag2(0);
    p.y = corner_tag2(1);  //payload_size_
    p.z = corner_tag2(2);
    line_list.points.push_back(p);
  }

  for (int j = -1; j <= 1; j += 2) {
    for (int i = 0; i < 4; ++i) {
      vector<int> v = vertex[i];
      Eigen::Vector4f corner_lidar1(
        j * tag_depth, v[0] * cluster.tag_size / 2, v[1] * cluster.tag_size / 2, 1);
      Eigen::Vector4f corner_tag1 = cluster.pose.homogeneous * corner_lidar1;
      geometry_msgs::msg::Point p;
      p.x = corner_tag1(0);
      p.y = corner_tag1(1);
      p.z = corner_tag1(2);
      line_list.points.push_back(p);
      v = vertex[(i + 1) % 4];
      Eigen::Vector4f corner_lidar2(
        j * tag_depth, v[0] * cluster.tag_size / 2, v[1] * cluster.tag_size / 2, 1);
      Eigen::Vector4f corner_tag2 = cluster.pose.homogeneous * corner_lidar2;
      p.x = corner_tag2(0);
      p.y = corner_tag2(1);
      p.z = corner_tag2(2);
      line_list.points.push_back(p);
    }
  }

  tag_frame_pub_->publish(line_list);
}

visualization_msgs::msg::Marker LidarTag::visualizeVector(
  Eigen::Vector3f edge_vector, PointXYZRI centroid, int id)
{
  const double tag_depth = 0.02;
  visualization_msgs::msg::Marker edge;
  edge.id = id;
  edge.header = point_cloud_header_;
  edge.type = visualization_msgs::msg::Marker::LINE_STRIP;
  edge.color.g = 1.0;
  edge.color.a = 1.0;
  edge.scale.x = tag_depth;

  geometry_msgs::msg::Point p;
  p.x = centroid.x;
  p.y = centroid.y;
  p.z = centroid.z;
  edge.points.push_back(p);

  // edge_vector = (edge_vector*payload_size_).eval();
  p.x += edge_vector[0];
  p.y += edge_vector[1];
  p.z += edge_vector[2];
  edge.points.push_back(p);

  //edge_vector_pub_->publish(edge);
  return edge;
}
/*
 * A function to prepare for displaying results in rviz
 */
void LidarTag::clusterToPclVectorAndMarkerPublisher(
  std::vector<ClusterFamily_t> & clusters, pcl::PointCloud<PointXYZRI>::Ptr out_cluster,
  pcl::PointCloud<PointXYZRI>::Ptr out_edge_cluster, pcl::PointCloud<PointXYZRI>::Ptr out_payload,
  pcl::PointCloud<PointXYZRI>::Ptr out_payload_3d, pcl::PointCloud<PointXYZRI>::Ptr out_target,
  pcl::PointCloud<PointXYZRI>::Ptr out_initial_target, pcl::PointCloud<PointXYZRI>::Ptr edge_group_1,
  pcl::PointCloud<PointXYZRI>::Ptr edge_group_2, pcl::PointCloud<PointXYZRI>::Ptr edge_group_3,
  pcl::PointCloud<PointXYZRI>::Ptr edge_group_4, pcl::PointCloud<PointXYZRI>::Ptr boundary_pts,
  pcl::PointCloud<PointXYZRI>::Ptr initial_corners, visualization_msgs::msg::MarkerArray & cluster_array)
{
  const double tag_depth = 0.02;
  /* initialize random seed for coloring the marker*/
  srand(time(NULL));
  visualization_msgs::msg::MarkerArray bound_mark_array;
  visualization_msgs::msg::MarkerArray payload_mark_array;
  visualization_msgs::msg::MarkerArray id_mark_array;

  lidartag_msgs::msg::CornersArray corners_array;
  lidartag_msgs::msg::CornersArray boundary_corners_array;

  visualization_msgs::msg::MarkerArray corners_markers_array;
  visualization_msgs::msg::MarkerArray boundary_corners_markers_array;

  lidartag_msgs::msg::LidarTagDetectionArray detections_array;


  // Used to identify between multiple clusters in a single point
  // cloud in the analysis file. The id being reset to 1 each time
  // the function is called is supposed to indicate in the output
  // file that the proceeding clusters belong to a new payload
  int cluster_pc_id = 1;

  int points_in_clusters = 0;

  int clustercount = 0;
  for (int key = 0; key < clusters.size(); ++key) {
    ClusterFamily_t & cluster = clusters[key];
    if (cluster.valid != 1 && static_cast<int>(cluster.detail_valid) < static_cast<int>(LidartagErrorCode::Line1EstimationCriteria)){ 
      continue;
    }

    if (mark_cluster_validity_) {
      for (int i = 0; i < cluster.edge_points.size(); ++i) {
        if (cluster.edge_points[i].valid != 1) {
          continue;
        }
        
        out_edge_cluster->push_back(cluster.edge_points[i].point);
      }

      for (int i = 0; i < cluster.edge_group1.size(); ++i) {
        edge_group_1->push_back(cluster.edge_group1[i].point);
      }

      for (int i = 0; i < cluster.edge_group2.size(); ++i) {
        edge_group_2->push_back(cluster.edge_group2[i].point);
      }

      for (int i = 0; i < cluster.edge_group3.size(); ++i) {
        edge_group_3->push_back(cluster.edge_group3[i].point);
      }

      for (int i = 0; i < cluster.edge_group4.size(); ++i) {
        edge_group_4->push_back(cluster.edge_group4[i].point);
      }
    }

    // Add the initial corners obtained through either line or rectangle estimation
    *initial_corners += cluster.initial_corners;

    if (cluster.valid != 1 && cluster.detail_valid != LidartagErrorCode::DecodingErrorCriteria){ 
      continue;
    }

    LidarTag::plotTagFrame(cluster);

    if (cluster.valid != 1){ 
      continue;
    }
    
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::Marker boundary_marker;

    // pick a random color for each cluster
    double r = (double)rand() / RAND_MAX;
    double g = (double)rand() / RAND_MAX;
    double b = (double)rand() / RAND_MAX;

    // Draw boundary marker of each cluster
    LidarTag::assignMarker(
      boundary_marker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(cluster.cluster_id), r, g, b, cluster.top_most_point, 0,
      tag_depth);
    bound_mark_array.markers.push_back(boundary_marker);
    LidarTag::assignMarker(
      boundary_marker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(cluster.cluster_id), r, g, b, cluster.bottom_most_point, 1,
      tag_depth);
    bound_mark_array.markers.push_back(boundary_marker);
    LidarTag::assignMarker(
      boundary_marker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(cluster.cluster_id), r, g, b, cluster.front_most_point, 2,
      tag_depth);
    bound_mark_array.markers.push_back(boundary_marker);
    LidarTag::assignMarker(
      boundary_marker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(cluster.cluster_id), r, g, b, cluster.back_most_point, 3,
      tag_depth);
    bound_mark_array.markers.push_back(boundary_marker);
    LidarTag::assignMarker(
      boundary_marker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(cluster.cluster_id), r, g, b, cluster.right_most_point, 4,
      tag_depth);
    bound_mark_array.markers.push_back(boundary_marker);
    LidarTag::assignMarker(
      boundary_marker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(cluster.cluster_id), r, g, b, cluster.left_most_point, 5,
      tag_depth);
    bound_mark_array.markers.push_back(boundary_marker);

    // Display cluster information
    // how many points are supposed to be on the this tag
    // int AvePoints = LidarTag::areaPoints(cluster.average.x,
    // payload_size_, payload_size_);
    LidarTag::assignMarker(
      boundary_marker, visualization_msgs::msg::Marker::SPHERE,
      "AveragePoint" + to_string(cluster.cluster_id), 1, 0, 0, cluster.average, 1, 0.05);
    bound_mark_array.markers.push_back(boundary_marker);

    // int SupposedPoints = LidarTag::areaPoints(cluster.average.x,
    // payload_size_, payload_size_);
    LidarTag::assignMarker(
      boundary_marker, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      "Text" + to_string(cluster.cluster_id), 1, 1, 1, cluster.average, 1, 0.05,
      string(
        to_string(cluster.cluster_id) + ", " + "\nAt: " + to_string(cluster.average.x) +
        ", " + to_string(cluster.average.y) + ", " + to_string(cluster.average.z) +
        "\nNormal vector: " + to_string(cluster.normal_vector(0)) + ", " +
        to_string(cluster.normal_vector(1)) + ", " + to_string(cluster.normal_vector(2)) +
        "\nActual points: " + to_string(cluster.data.size()) + ", " +
        "\nNumber of inliers: " + to_string(cluster.inliers) + ", " +
        "\nPercentages of inliers: " + to_string(cluster.percentages_inliers) + ", " +
        "\nBoundary points: " + to_string(cluster.boundary_pts) + ", " +
        "\nBoundary rings: " + to_string(cluster.boundary_rings) + ", " +
        "\nPayload points: " + to_string(cluster.payload_without_boundary) + ", " +
        "\nPose_xyz: " + to_string(cluster.pose_tag_to_lidar.translation[0]) + ", " +
        to_string(cluster.pose_tag_to_lidar.translation[1]) + ", " +
        to_string(cluster.pose_tag_to_lidar.translation[2]) +
        "\nPose_rpy: " + to_string(cluster.pose_tag_to_lidar.roll) + ", " +
        to_string(cluster.pose_tag_to_lidar.pitch) + ", " +
        to_string(cluster.pose_tag_to_lidar.yaw) +
        "\nIntensity: " + to_string(cluster.max_intensity.intensity) + ", " +
        to_string(cluster.min_intensity.intensity)));
    bound_mark_array.markers.push_back(boundary_marker);
    
    LidarTag::assignVectorMarker(
      boundary_marker, visualization_msgs::msg::Marker::ARROW,
      "NormalVector_z" + to_string(cluster.cluster_id), 0, 0, 1, 2, 0.01,
      cluster.principal_axes.col(2), cluster.average);
    bound_mark_array.markers.push_back(boundary_marker);
   
    if (id_decoding_) {
      visualization_msgs::msg::Marker id_marker;
      LidarTag::assignMarker(
        id_marker, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        "Text" + to_string(cluster.cluster_id), 1, 1, 1, cluster.average, 1,
        cluster.tag_size * 0.7, string(to_string(cluster.rkhs_decoding.id)));
      id_mark_array.markers.push_back(id_marker);
    }
    // Draw payload boundary marker
    visualization_msgs::msg::Marker payload_marker;

    if (adaptive_thresholding_) {
      // Upper boundary
      for (int i = 0; i < cluster.tag_edges.upper_line.size(); ++i) {
        LidarTag::assignMarker(
          payload_marker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadUpperBoundary_" + to_string(cluster.cluster_id), 0, 0, 1,
          cluster.tag_edges.upper_line[i]->point, i, 0.015);
        payload_mark_array.markers.push_back(payload_marker);
      }

      // Lower boundary
      for (int i = 0; i < cluster.tag_edges.lower_line.size(); ++i) {
        LidarTag::assignMarker(
          payload_marker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadLowerBoundary_" + to_string(cluster.cluster_id), 0, 0, 1,
          cluster.tag_edges.lower_line[i]->point, i, 0.015);
        payload_mark_array.markers.push_back(payload_marker);
      }

      // Left boundary (green)
      for (int i = 0; i < cluster.tag_edges.left_line.size(); ++i) {
        LidarTag::assignMarker(
          payload_marker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadLeftBoundary_" + to_string(cluster.cluster_id), 0, 1, 0,
          cluster.tag_edges.left_line[i]->point, i, 0.015);
        payload_mark_array.markers.push_back(payload_marker);
      }

      // Right boundary (red)
      for (int i = 0; i < cluster.tag_edges.right_line.size(); ++i) {
        LidarTag::assignMarker(
          payload_marker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadRightBoundary_" + to_string(cluster.cluster_id), 1, 0, 0,
          cluster.tag_edges.right_line[i]->point, i, 0.015);
        payload_mark_array.markers.push_back(payload_marker);
      }
    } else {
      int count = 0;
      for (int i = 0; i < cluster.payload_boundary_ptr.size(); ++i) {
        
        LidarTag::assignMarker(
          payload_marker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadBoundary_" + to_string(cluster.cluster_id), 1, 0, 0,
          cluster.payload_boundary_ptr[i]->point, i, 0.015);
        payload_mark_array.markers.push_back(payload_marker);
        count++;
        // }
      }
    }

    // Add the initial corners obtained through either line or rectangle estimation
    *initial_corners += cluster.initial_corners;

    // corner points and RANSAC line
    if (adaptive_thresholding_) {
      Eigen::Vector4f eigen_point;
      PointXYZRI point;  // just for conversion

      for (int i = 0; i < 4; ++i) {  // 4 corners
        // Corners
        if (i != 3) {
          pcl::lineWithLineIntersection(
            cluster.line_coeff[i], cluster.line_coeff[i + 1], eigen_point, 1e-2);
        } else {
          pcl::lineWithLineIntersection(
            cluster.line_coeff[i], cluster.line_coeff[0], eigen_point, 1e-2);
        }

        LidarTag::eigenVectorToPointXYZRI(eigen_point, point);
        LidarTag::assignMarker(
          payload_marker, visualization_msgs::msg::Marker::SPHERE,
          "Corner_" + to_string(cluster.cluster_id), 0, 1, 1, point, i, tag_depth);
        payload_mark_array.markers.push_back(payload_marker);

        // RANSAC
        LidarTag::assignMarker(
          payload_marker, visualization_msgs::msg::Marker::ARROW,
          "RANSAC" + to_string(cluster.cluster_id), 1, 1, 0, point, i, 0.01);
        double length = sqrt(
          pow(cluster.line_coeff[i][3], 2) + pow(cluster.line_coeff[i][4], 2) +
          pow(cluster.line_coeff[i][5], 2));
        payload_marker.scale.x = 0.15;
        payload_marker.pose.orientation.x = cluster.line_coeff[i][3] / length;
        payload_marker.pose.orientation.y = cluster.line_coeff[i][4] / length;
        payload_marker.pose.orientation.z = cluster.line_coeff[i][5] / length;
        payload_mark_array.markers.push_back(payload_marker);
      }
    }

    // Draw all detected-filled cluster points

    for (int i = 0; i < cluster.data.size(); ++i) {
      if (cluster.data[i].valid != 1) {
        continue;
      }
      points_in_clusters++;

      out_cluster->push_back(cluster.data[i].point);

      double intensity = cluster.data[i].point.intensity;
      LidarTag::assignMarker(
        marker, visualization_msgs::msg::Marker::SPHERE, to_string(cluster.cluster_id), intensity,
        intensity, intensity, cluster.data[i].point, i, 0.01);
      cluster_array.markers.push_back(marker);
    }
    if (mark_cluster_validity_ && params_.use_intensity_channel) {
      
      for (int ring = 0; ring < beam_num_; ++ring) {
        if (cluster.payload_right_boundary_ptr[ring] != 0) {
          boundary_pts->push_back(cluster.payload_right_boundary_ptr[ring]->point);
        }
        if (cluster.payload_left_boundary_ptr[ring] != 0) {
          boundary_pts->push_back(cluster.payload_left_boundary_ptr[ring]->point);
        }
      }
    }

    if (id_decoding_ && params_.use_intensity_channel) {
      for (int i = 0; i < cluster.rkhs_decoding.associated_pattern_3d->cols(); ++i) {
        PointXYZRI point;
        point.x = cluster.rkhs_decoding.associated_pattern_3d->col(i)(0);
        point.y = cluster.rkhs_decoding.associated_pattern_3d->col(i)(1);
        point.z = cluster.rkhs_decoding.associated_pattern_3d->col(i)(2);

        if (point.x >= 0) {
          point.intensity = 200;
        } else {
          point.intensity = 50;
        }

        out_payload->push_back(point);
      }
      for (int i = 0; i < cluster.rkhs_decoding.template_points_3d.cols(); ++i) {
        PointXYZRI point;
        point.x = cluster.rkhs_decoding.template_points_3d.col(i)(0);
        point.y = cluster.rkhs_decoding.template_points_3d.col(i)(1);
        point.z = cluster.rkhs_decoding.template_points_3d.col(i)(2);
        point.intensity = cluster.rkhs_decoding.template_points_3d.col(i)(3);
        out_payload_3d->push_back(point);
      }
    }
    if (mark_cluster_validity_ && params_.use_intensity_channel) {
      for (int i = 0; i < cluster.rkhs_decoding.initial_template_points.cols(); ++i) {
        PointXYZRI point;
        point.x = cluster.rkhs_decoding.initial_template_points.col(i)(0);
        point.y = cluster.rkhs_decoding.initial_template_points.col(i)(1);
        point.z = cluster.rkhs_decoding.initial_template_points.col(i)(2);
        point.intensity = cluster.rkhs_decoding.initial_template_points.col(i)(3);
        out_initial_target->push_back(point);
      }
      for (int i = 0; i < cluster.rkhs_decoding.template_points.cols(); ++i) {
        PointXYZRI point;
        point.x = cluster.rkhs_decoding.template_points.col(i)(0);
        point.y = cluster.rkhs_decoding.template_points.col(i)(1);
        point.z = cluster.rkhs_decoding.template_points.col(i)(2);
        point.intensity = cluster.rkhs_decoding.template_points.col(i)(3);
        out_target->push_back(point);
      }
    }

    if (id_decoding_) {
      addCorners(cluster, corners_array, corners_markers_array);
      
      //if (getBoundaryCorners(cluster, boundary_pts)) {
      addBoundaryCorners(cluster, boundary_corners_array, boundary_corners_markers_array);
      //}
    }
    // Publish to a lidartag channel
    detectionArrayPublisher(cluster, detections_array);
  }

  detections_array.header = point_cloud_header_;
  corners_array.header = point_cloud_header_;
  boundary_corners_array.header = point_cloud_header_;

  corners_array_pub_->publish(corners_array);
  boundary_corners_array_pub_->publish(boundary_corners_array);
  corners_markers_pub_->publish(corners_markers_array);
  boundary_corners_markers_pub_->publish(boundary_corners_markers_array);
  boundary_marker_pub_->publish(bound_mark_array);
  cluster_marker_pub_->publish(cluster_array);
  payload_marker_pub_->publish(payload_mark_array);
  id_marker_pub_->publish(id_mark_array);
  detection_array_pub_->publish(detections_array);
  
  colorClusters(clusters);
  displayClusterPointSize(clusters);
  displayClusterIndexNumber(clusters);
  publishLidartagCluster(clusters);
}

void LidarTag::addCorners(
  const ClusterFamily_t & cluster, 
  lidartag_msgs::msg::CornersArray & corners_array_msg,
  visualization_msgs::msg::MarkerArray & corners_markers_msg)
{
  // Tag in tag coordinates (counter clock-wise)
  std::vector<Eigen::Vector2f> vertex = {
    Eigen::Vector2f{-1.f, -1.f}, Eigen::Vector2f{1.f, -1.f},
    Eigen::Vector2f{1.f, 1.f}, Eigen::Vector2f{-1.f, 1.f}};

  std::vector<geometry_msgs::msg::Point> vertex_msg;
  vertex_msg.resize(4);
  
  // Calculate the tag corners based on the detection's pose and geometry
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector2f & v = vertex[i];
    Eigen::Vector4f corner_lidar(
      0.f, v[0] * cluster.tag_size / 2.f, v[1] * cluster.tag_size / 2.f, 1.f);
    
    Eigen::Vector4f tag_corner = cluster.pose.homogeneous * corner_lidar;
    geometry_msgs::msg::Point & p = vertex_msg[i];
    p.x = tag_corner.x();
    p.y = tag_corner.y();  //_payload_size
    p.z = tag_corner.z();
  }

  addCornersAux(cluster, vertex_msg, corners_array_msg, corners_markers_msg);
}

void LidarTag::addBoundaryCorners(
  const ClusterFamily_t & cluster, 
  lidartag_msgs::msg::CornersArray & corners_array_msg,
  visualization_msgs::msg::MarkerArray & corners_markers_msg)
{
  // Tag in tag coordinates (counter clock-wise)
  std::vector<Eigen::Vector2f> vertex = {
    Eigen::Vector2f{-0.75f, -0.75f}, Eigen::Vector2f{0.75f, -0.75f},
    Eigen::Vector2f{0.75f, 0.75f}, Eigen::Vector2f{-0.75f, 0.75f}};

  std::vector<geometry_msgs::msg::Point> vertex_msg;
  vertex_msg.resize(4);
  
  // Calculate the tag's boundary corners based on the detection's pose and geometry
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector2f & v = vertex[i];
    Eigen::Vector4f corner_lidar(
      0.f, v[0] * cluster.tag_size / 2.f, v[1] * cluster.tag_size / 2.f, 1.f);
    
    Eigen::Vector4f tag_boundary_corner = cluster.pose.homogeneous * corner_lidar;
    geometry_msgs::msg::Point & p = vertex_msg[i];
    p.x = tag_boundary_corner.x();
    p.y = tag_boundary_corner.y();  //_payload_size
    p.z = tag_boundary_corner.z();
  }

  addCornersAux(cluster, vertex_msg, corners_array_msg, corners_markers_msg);
}

void LidarTag::addCornersAux(
  const ClusterFamily_t & cluster,
  const std::vector<geometry_msgs::msg::Point> & vertex_msg, 
  lidartag_msgs::msg::CornersArray & corners_array_msg,
  visualization_msgs::msg::MarkerArray & corners_markers_msg)
{
  // Fill the message fieds
  visualization_msgs::msg::Marker marker, bottom_left_marker, bottom_right_marker, top_right_marker,
    top_left_marker, center_marker;
  
  marker.header.frame_id = lidar_frame_;
  marker.header.stamp = clock_->now();  
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(sleep_time_for_vis_ * 10);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;  // Don't forget to set the alpha!

  lidartag_msgs::msg::Corners corners;
  corners.bottom_left = vertex_msg[0];
  corners.bottom_right = vertex_msg[1];
  corners.top_right = vertex_msg[2];
  corners.top_left = vertex_msg[3];
  corners.corners = vertex_msg;
  corners.rotation = cluster.rkhs_decoding.rotation_angle;
  corners.header = marker.header;
  corners.id = cluster.cluster_id;

  bottom_left_marker = marker;
  bottom_left_marker.ns = "tag_id_" + std::to_string(cluster.cluster_id);
  bottom_left_marker.pose.position = corners.bottom_left;
  bottom_left_marker.color.r = 1.0;
  bottom_left_marker.color.g = 0.0;
  bottom_left_marker.color.b = 0.0;
  bottom_left_marker.id = 0;
  
  bottom_right_marker = marker;
  bottom_right_marker.ns = "tag_id_" + std::to_string(cluster.cluster_id);
  bottom_right_marker.pose.position = corners.bottom_right;
  bottom_right_marker.color.r = 0.0;
  bottom_right_marker.color.g = 1.0;
  bottom_right_marker.color.b = 0.0;
  bottom_right_marker.id = 1;
  
  top_right_marker = marker;
  top_right_marker.ns = "tag_id_" + std::to_string(cluster.cluster_id);
  top_right_marker.pose.position = corners.top_right;
  top_right_marker.color.r = 0.0;
  top_right_marker.color.g = 0.0;
  top_right_marker.color.b = 1.0;
  top_right_marker.id = 2;
  
  top_left_marker = marker;
  top_left_marker.ns = "tag_id_" + std::to_string(cluster.cluster_id);
  top_left_marker.pose.position = corners.top_left;
  top_left_marker.color.r = 1.0;
  top_left_marker.color.g = 0.0;
  top_left_marker.color.b = 1.0;
  top_left_marker.id = 3;

  center_marker = marker;
  center_marker.ns = "tag_id_" + std::to_string(cluster.cluster_id);
  center_marker.pose.position.x = cluster.average.x;
  center_marker.pose.position.y = cluster.average.y;
  center_marker.pose.position.z = cluster.average.z;
  center_marker.color.r = 0.0;
  center_marker.color.g = 1.0;
  center_marker.color.b = 1.0;
  center_marker.id = 4;

  corners_markers_msg.markers.push_back(bottom_left_marker);
  corners_markers_msg.markers.push_back(bottom_right_marker);
  corners_markers_msg.markers.push_back(top_right_marker);
  corners_markers_msg.markers.push_back(top_left_marker);
  
  corners_array_msg.corners.push_back(corners);
}

void LidarTag::colorClusters(const std::vector<ClusterFamily_t> & clusters)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster_buff(
    new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB colored_point;
  colored_cluster_buff->reserve(point_cloud_size_);
  int r, g, b;
  srand(100);

  for (int key = 0; key < clusters.size(); ++key) {

    if (params_.debug_cluster_id != -1 && params_.debug_cluster_id != key) {
      continue;
    }

    const ClusterFamily_t & cluster = clusters[key];
    if (cluster.valid != 1) {
      r = rand() % 255;
      g = rand() % 255;
      b = rand() % 255;
      for (int i = 0; i < cluster.data.size(); ++i) {
        colored_point.x = cluster.data[i].point.x;
        colored_point.y = cluster.data[i].point.y;
        colored_point.z = cluster.data[i].point.z;
        colored_point.r = r;
        colored_point.g = g;
        colored_point.b = b;
        colored_cluster_buff->points.push_back(colored_point);
      }

      for (int i = 0; i < cluster.edge_points.size(); ++i) {
        colored_point.x = cluster.edge_points[i].point.x;
        colored_point.y = cluster.edge_points[i].point.y;
        colored_point.z = cluster.edge_points[i].point.z;
        colored_point.r = r;
        colored_point.g = g;
        colored_point.b = b;
        colored_cluster_buff->points.push_back(colored_point);
      }
    }
  }

  sensor_msgs::msg::PointCloud2 colored_cluster_buff_msg;
  pcl::toROSMsg(*colored_cluster_buff, colored_cluster_buff_msg);
  colored_cluster_buff_msg.header.frame_id = lidar_frame_;
  colored_cluster_buff_msg.header.stamp = clock_->now();
  colored_cluster_buff_pub_->publish(colored_cluster_buff_msg);
}

void LidarTag::displayClusterPointSize(const std::vector<ClusterFamily_t> & cluster_buff)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(cluster_buff.size());
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = lidar_frame_;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(sleep_time_for_vis_ * 10);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;

  int points_size = 0;
  
  for (std::size_t i = 0; i < cluster_buff.size(); ++i) {
    points_size = cluster_buff[i].data.size() + cluster_buff[i].edge_points.size();
    
    if (points_size > 50 || cluster_buff[i].cluster_id == params_.debug_cluster_id) {
      marker.header.stamp = clock_->now();
      marker.id = cluster_buff[i].cluster_id;
      marker.text = to_string(marker.id) + "/" + to_string(points_size);
      marker.ns = "ps_marker_" + to_string(cluster_buff[i].cluster_id);
      marker.pose.position.x = cluster_buff[i].average.x;
      marker.pose.position.y = cluster_buff[i].average.y;
      marker.pose.position.z = cluster_buff[i].average.z;
      marker_array.markers.push_back(marker);
    }
  }

  ps_cluster_buff_pub_->publish(marker_array);
}

void LidarTag::displayClusterIndexNumber(const std::vector<ClusterFamily_t> & cluster_buff)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(cluster_buff.size());
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = lidar_frame_;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(sleep_time_for_vis_ * 10);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;

  int points_size = 0;
  for (std::size_t i = 0; i < cluster_buff.size(); ++i) {
    points_size = cluster_buff[i].data.size() + cluster_buff[i].edge_points.size();
    
    if (points_size > 50 || cluster_buff[i].cluster_id == params_.debug_cluster_id) {
      marker.header.stamp = clock_->now();
      marker.id = cluster_buff[i].cluster_id;
      marker.text = to_string(i) + "/" + to_string(static_cast<int>(cluster_buff[i].detail_valid));
      marker.ns = "in_marker_" + to_string(cluster_buff[i].cluster_id);
      marker.pose.position.x = cluster_buff[i].average.x;
      marker.pose.position.y = cluster_buff[i].average.y;
      marker.pose.position.z = cluster_buff[i].average.z;
      marker_array.markers.push_back(marker);
    }
  }
  
  in_cluster_buff_pub_->publish(marker_array);
}

}  // namespace BipedLab
