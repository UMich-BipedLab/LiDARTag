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

#include <pcl/common/intersections.h>

#include "rclcpp/rclcpp.hpp" // package
#include "lidartag.h"

#include <iostream>
#include <string>

#define SQRT2 1.41421356237

using namespace std;

namespace BipedLab
{
/*
 * A function to draw a point in rviz
 */
void LidarTag::_assignMarker(
  visualization_msgs::msg::Marker & Marker, const uint32_t Shape, const string NameSpace, 
  const double r, const double g, const double b, const PointXYZRI & point, const int Count, 
  const double Size, const string Text)
{
  Marker.header.frame_id = _pub_frame;
  Marker.header.stamp = _current_scan_time;
  Marker.ns = NameSpace;
  Marker.id = Count;
  Marker.type = Shape;
  Marker.action = visualization_msgs::msg::Marker::ADD;
  Marker.pose.position.x = point.x;
  Marker.pose.position.y = point.y;
  Marker.pose.position.z = point.z;
  Marker.pose.orientation.x = 0.0;
  Marker.pose.orientation.y = 0.0;
  Marker.pose.orientation.z = 0.0;
  Marker.pose.orientation.w = 1.0;
  Marker.text = Text;
  Marker.lifetime =
        Marker.lifetime = rclcpp::Duration(_sleep_time_for_vis * 10); // should disappear along with updateing rate

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  Marker.scale.x = Size;
  Marker.scale.y = Size;
  Marker.scale.z = Size;

  // Set the color -- be sure to set alpha to something non-zero!
  Marker.color.r = r;
  Marker.color.g = g;
  Marker.color.b = b;
  Marker.color.a = 1.0;
}

void LidarTag::_assignVectorMarker(
  visualization_msgs::msg::Marker & Marker, const uint32_t Shape, const string NameSpace, const double r,
  const double g, const double b, const int Count, const double Size, Eigen::Vector3f edge_vector,
  const PointXYZRI & centriod, const string Text)
{
        // LidarTag::_assignMarker(Marker, visualization_msgs::msg::Marker::SPHERE, NameSpace,
  // NameSpace,
  //              r, g, b,
  //              centriod,
  //              Count, Size, Text);

  // PointXYZRI p2;
  // p2.x = centriod.x + edge_vector[0];
  // p2.y = centriod.y + edge_vector[1];
  // p2.z = centriod.z + edge_vector[2];
        // LidarTag::_assignMarker(Marker, visualization_msgs::msg::Marker::SPHERE, NameSpace,
  // NameSpace,
  //              r, g, b,
  //              p2,
  //              Count+1, Size, Text);

  Marker.header.frame_id = _pub_frame;
  Marker.header.stamp = _current_scan_time;
  Marker.ns = NameSpace;
  Marker.id = Count;
  Marker.type = Shape;
  Marker.action = visualization_msgs::msg::Marker::ADD;

  Marker.text = Text;
  Marker.lifetime =
        Marker.lifetime = rclcpp::Duration(_sleep_time_for_vis); // should disappear along with updateing rate
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  geometry_msgs::msg::Point p1;
  p1.x = 0;  // centriod.x;
  p1.y = 0;  // centriod.y;
  p1.z = 0;  // centriod.z;
  Marker.points.push_back(p1);

  geometry_msgs::msg::Point p2;
  p2.x = edge_vector[0];
  p2.y = edge_vector[1];
  p2.z = edge_vector[2];
  Marker.points.push_back(p2);
  Marker.color.r = r;
  Marker.color.g = g;
  Marker.color.b = b;
  Marker.scale.x = 0.02;
  Marker.scale.y = Size;
  Marker.scale.z = Size;
  // Set the color -- be sure to set alpha to something non-zero!
}
void LidarTag::_plotIdealFrame()
{
  visualization_msgs::msg::MarkerArray FrameMarkArray;
  for (int k = 0; k < _tag_size_list.size(); ++k) {
    visualization_msgs::msg::Marker line_list;
    line_list.id = 0;
    line_list.header = _point_cloud_header;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.ns = "tag_size_" + to_string(_tag_size_list[k]);
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    line_list.scale.x = 0.01;

    vector<vector<int>> vertex = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
    for (int i = 0; i < 4; ++i) {
      vector<int> v = vertex[i];
      geometry_msgs::msg::Point p;
      p.x = -0.02;
      p.y = v[0] * _tag_size_list[k] / 2;  //_payload_size
      p.z = v[1] * _tag_size_list[k] / 2;
      line_list.points.push_back(p);
      p.x = 0.02;
      line_list.points.push_back(p);
    }

    for (int j = -1; j <= 1; j += 2) {
      for (int i = 0; i < 4; ++i) {
        vector<int> v = vertex[i];
        geometry_msgs::msg::Point p;
        p.x = j * 0.02;
        p.y = v[0] * _tag_size_list[k] / 2;
        p.z = v[1] * _tag_size_list[k] / 2;
        line_list.points.push_back(p);
        v = vertex[(i + 1) % 4];
        p.y = v[0] * _tag_size_list[k] / 2;
        p.z = v[1] * _tag_size_list[k] / 2;
        line_list.points.push_back(p);
      }
    }
    FrameMarkArray.markers.push_back(line_list);
  }

  _ideal_frame_pub->publish(FrameMarkArray);
}

        // visualization_msgs::msg::Marker line_list;
// line_list.id = 0;
// line_list.header = _point_cloud_header;
        // line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
// line_list.color.g = 1.0;
// line_list.color.a = 1.0;
// line_list.scale.x = 0.01;

// vector<vector<int>> vertex = {{1,1},{1,-1},{-1,-1}, {-1,1}};
// for(int i = 0; i < 4; ++i){
//     vector<int> v = vertex[i];
        //     geometry_msgs::Point p;
//     p.x = -0.02;
//     p.y = v[0]*1.22/2;    //_payload_size
//     p.z = v[1]*1.22/2;
//     line_list.points.push_back(p);
//     p.x = 0.02;
//     line_list.points.push_back(p);
// }

// for(int j = -1; j<=1; j+=2){
//   for(int i = 0; i < 4; ++i){
//       vector<int> v = vertex[i];
        //       geometry_msgs::Point p;
//       p.x = j* 0.02;
//       p.y = v[0]*1.22/2;
//       p.z = v[1]*1.22/2;
//       line_list.points.push_back(p);
//       v = vertex[(i+1)%4];
//       p.y = v[0]*1.22/2;
//       p.z = v[1]*1.22/2;
//       line_list.points.push_back(p);
//   }
// }

void LidarTag::_plotTagFrame(const ClusterFamily_t & cluster)
{
  visualization_msgs::msg::Marker line_list;
  line_list.id = cluster.cluster_id;
  line_list.header = _point_cloud_header;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
  // line_list.ns = "tag_size_" + to_string(_tag_size_list[k]);
  line_list.color.r = 0.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;
  line_list.scale.x = 0.01;

  vector<vector<int>> vertex = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
  for (int i = 0; i < 4; ++i) {
    vector<int> v = vertex[i];
    Eigen::Vector4f corner_lidar1(
      -0.02, v[0] * cluster.tag_size / 2, v[1] * cluster.tag_size / 2, 1);
    Eigen::Vector4f corner_lidar2(
      0.02, v[0] * cluster.tag_size / 2, v[1] * cluster.tag_size / 2, 1);
    Eigen::Vector4f corner_tag1 = cluster.pose.homogeneous * corner_lidar1;
    Eigen::Vector4f corner_tag2 = cluster.pose.homogeneous * corner_lidar2;
    geometry_msgs::msg::Point p;
    p.x = corner_tag1(0);
    p.y = corner_tag1(1);  //_payload_size
    p.z = corner_tag1(2);
    line_list.points.push_back(p);
    p.x = corner_tag2(0);
    p.y = corner_tag2(1);  //_payload_size
    p.z = corner_tag2(2);
    line_list.points.push_back(p);
  }

  for (int j = -1; j <= 1; j += 2) {
    for (int i = 0; i < 4; ++i) {
      vector<int> v = vertex[i];
      Eigen::Vector4f corner_lidar1(
        j * 0.02, v[0] * cluster.tag_size / 2, v[1] * cluster.tag_size / 2, 1);
      Eigen::Vector4f corner_tag1 = cluster.pose.homogeneous * corner_lidar1;
      geometry_msgs::msg::Point p;
      p.x = corner_tag1(0);
      p.y = corner_tag1(1);
      p.z = corner_tag1(2);
      line_list.points.push_back(p);
      v = vertex[(i + 1) % 4];
      Eigen::Vector4f corner_lidar2(
        j * 0.02, v[0] * cluster.tag_size / 2, v[1] * cluster.tag_size / 2, 1);
      Eigen::Vector4f corner_tag2 = cluster.pose.homogeneous * corner_lidar2;
      p.x = corner_tag2(0);
      p.y = corner_tag2(1);
      p.z = corner_tag2(2);
      line_list.points.push_back(p);
    }
  }

  _tag_frame_pub->publish(line_list);
}

visualization_msgs::msg::Marker LidarTag::_visualizeVector(
  Eigen::Vector3f edge_vector, PointXYZRI centriod, int ID)
{
  visualization_msgs::msg::Marker edge;
  edge.id = ID;
  edge.header = _point_cloud_header;
  edge.type = visualization_msgs::msg::Marker::LINE_STRIP;
  edge.color.g = 1.0;
  edge.color.a = 1.0;
  edge.scale.x = 0.02;

  geometry_msgs::msg::Point p;
  p.x = centriod.x;
  p.y = centriod.y;
  p.z = centriod.z;
  edge.points.push_back(p);

  // edge_vector = (edge_vector*_payload_size).eval();
  p.x += edge_vector[0];
  p.y += edge_vector[1];
  p.z += edge_vector[2];
  edge.points.push_back(p);

  //_edge_vector_pub->publish(edge);
  return edge;
}
/*
 * A function to prepare for displaying results in rviz
 */
void LidarTag::_clusterToPclVectorAndMarkerPublisher(
  const std::vector<ClusterFamily_t> & Cluster, pcl::PointCloud<PointXYZRI>::Ptr OutCluster,
  pcl::PointCloud<PointXYZRI>::Ptr OutEdgeCluster, pcl::PointCloud<PointXYZRI>::Ptr OutPayload,
  pcl::PointCloud<PointXYZRI>::Ptr OutPayload3D, pcl::PointCloud<PointXYZRI>::Ptr OutTarget,
  pcl::PointCloud<PointXYZRI>::Ptr OutInitialTarget, pcl::PointCloud<PointXYZRI>::Ptr EdgeGroup1,
  pcl::PointCloud<PointXYZRI>::Ptr EdgeGroup2, pcl::PointCloud<PointXYZRI>::Ptr EdgeGroup3,
  pcl::PointCloud<PointXYZRI>::Ptr EdgeGroup4, pcl::PointCloud<PointXYZRI>::Ptr BoundaryPts,
  visualization_msgs::msg::MarkerArray & ClusterArray)
{
  /* initialize random seed for coloring the marker*/
  srand(time(NULL));
  visualization_msgs::msg::MarkerArray BoundMarkArray;
  visualization_msgs::msg::MarkerArray PayloadMarkArray;
  visualization_msgs::msg::MarkerArray IDMarkArray;
  // Used to identify between multiple clusters in a single point
  // cloud in the analysis file. The id being reset to 1 each time
  // the function is called is supposed to indicate in the output
  // file that the proceeding clusters belong to a new payload
  int cluster_pc_id = 1;

  int PointsInClusters = 0;

  int Clustercount = 0;
  for (int Key = 0; Key < Cluster.size(); ++Key) {
    if (Cluster[Key].valid != 1) {
      continue;
    }
    LidarTag::_plotTagFrame(Cluster[Key]);
    // std::cout << "Valid Cluster number "<< ++Clustercount << ": " << Key << "
    // Size: " << Cluster[Key].data.size() << std::endl; if (Clustercount != 5)
    // continue; for (int j=0; j<_beam_num; ++j){
    //     cout << "\033[1;31m============== \033[0m\n";
    //     cout << "(i, j): " << Key << ", " << j << endl;
    //     int MaxIndex = Cluster[Key].max_min_index_of_each_ring[j].max;
    //     int MinIndex = Cluster[Key].max_min_index_of_each_ring[j].min;
    //     cout << "MaxIndex: " << MaxIndex << endl;
    //     cout << "MinIndex: " << MinIndex << endl;
    // }

    visualization_msgs::msg::Marker Marker;
    visualization_msgs::msg::Marker BoundaryMarker;

    // pick a random color for each cluster
    double r = (double)rand() / RAND_MAX;
    double g = (double)rand() / RAND_MAX;
    double b = (double)rand() / RAND_MAX;

    // Draw boundary marker of each cluster
    LidarTag::_assignMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(Cluster[Key].cluster_id), r, g, b, Cluster[Key].top_most_point, 0,
      0.02);
    BoundMarkArray.markers.push_back(BoundaryMarker);
    LidarTag::_assignMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(Cluster[Key].cluster_id), r, g, b, Cluster[Key].bottom_most_point, 1,
      0.02);
    BoundMarkArray.markers.push_back(BoundaryMarker);
    LidarTag::_assignMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(Cluster[Key].cluster_id), r, g, b, Cluster[Key].front_most_point, 2,
      0.02);
    BoundMarkArray.markers.push_back(BoundaryMarker);
    LidarTag::_assignMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(Cluster[Key].cluster_id), r, g, b, Cluster[Key].back_most_point, 3,
      0.02);
    BoundMarkArray.markers.push_back(BoundaryMarker);
    LidarTag::_assignMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(Cluster[Key].cluster_id), r, g, b, Cluster[Key].right_most_point, 4,
      0.02);
    BoundMarkArray.markers.push_back(BoundaryMarker);
    LidarTag::_assignMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::CUBE,
      "Boundary" + to_string(Cluster[Key].cluster_id), r, g, b, Cluster[Key].left_most_point, 5,
      0.02);
    BoundMarkArray.markers.push_back(BoundaryMarker);

    // Display cluster information
    // how many points are supposed to be on the this tag
    // int AvePoints = LidarTag::_areaPoints(Cluster[Key].average.x,
    // _payload_size, _payload_size);
    LidarTag::_assignMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::SPHERE,
      "AveragePoint" + to_string(Cluster[Key].cluster_id), 1, 0, 0, Cluster[Key].average, 1, 0.05);
    BoundMarkArray.markers.push_back(BoundaryMarker);

    // int SupposedPoints = LidarTag::_areaPoints(Cluster[Key].average.x,
    // _payload_size, _payload_size);
    LidarTag::_assignMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      "Text" + to_string(Cluster[Key].cluster_id), 1, 1, 1, Cluster[Key].average, 1, 0.05,
      string(
        to_string(Cluster[Key].cluster_id) + ", " + "\nAt: " + to_string(Cluster[Key].average.x) +
        ", " + to_string(Cluster[Key].average.y) + ", " + to_string(Cluster[Key].average.z) +
        "\nNormal vector: " + to_string(Cluster[Key].normal_vector(0)) + ", " +
        to_string(Cluster[Key].normal_vector(1)) + ", " + to_string(Cluster[Key].normal_vector(2)) +
        // "\nSupposed points: " +
        // to_string((int)SupposedPoints/_points_threshold_factor) +
        "\nActual points: " + to_string(Cluster[Key].data.size()) + ", " +
        "\nNumber of inliers: " + to_string(Cluster[Key].inliers) + ", " +
        "\nPercentages of inliers: " + to_string(Cluster[Key].percentages_inliers) + ", " +
        "\nBoundary points: " + to_string(Cluster[Key].boundary_pts) + ", " +
        "\nBoundary rings: " + to_string(Cluster[Key].boundary_rings) + ", " +
        "\nPayload points: " + to_string(Cluster[Key].payload_without_boundary) + ", " +
        "\nPose_xyz: " + to_string(Cluster[Key].pose_tag_to_lidar.translation[0]) + ", " +
        to_string(Cluster[Key].pose_tag_to_lidar.translation[1]) + ", " +
        to_string(Cluster[Key].pose_tag_to_lidar.translation[2]) +
        "\nPose_rpy: " + to_string(Cluster[Key].pose_tag_to_lidar.roll) + ", " +
        to_string(Cluster[Key].pose_tag_to_lidar.pitch) + ", " +
        to_string(Cluster[Key].pose_tag_to_lidar.yaw) +
        "\nIntensity: " + to_string(Cluster[Key].max_intensity.intensity) + ", " +
        to_string(Cluster[Key].min_intensity.intensity)));
    BoundMarkArray.markers.push_back(BoundaryMarker);
    // if (_write_CSV){
    // std::fstream file;
    // std::string path("/home/alex/catkin_ws/src/LidarTag/output");

    // write poses
    // file.open(path + "/LidarTag.txt",
    //            ios::in|ios::out|ios::app);
    // if (file.is_open()){
    //     file << Cluster[Key].cluster_id << ", ";
    //     file << Cluster[Key].pose.translation[0] << ", ";
    //     file << Cluster[Key].pose.translation[1] << ", ";
    //     file << Cluster[Key].pose.translation[2] << ", ";
    //     file << Cluster[Key].pose.roll << ", ";
    //     file << Cluster[Key].pose.pitch << ", ";
    //     file << Cluster[Key].pose.yaw << "\n ";
    //     file.close();
    // }
    // else{
    //     cout << "FAILED opening LidarTag.txt" << endl;
    //     exit(0);
    // }

    // write Timing
    // file.open(path + "/Analysis.txt", ios::in|ios::out|ios::app);
    // if (file.is_open()){
    //     file << cluster_pc_id++ << ",";
    //     file << _timing.computation_time << ",";
    //     file << _timing.edgingand_clustering_time << ",";
    //     file << _timing.point_check_time << ",";
    //     file << _timing.line_fitting_time << ",";
    //     file << _timing.payload_extraction_time << ",";
    //     file << _timing.normal_vector_time << ",";
    //     file << _timing.payload_decoding_time << ",";
    //     file << _timing.tag_to_robot_time << ",";
    //     file << Cluster[Key].data.size() << ",";
    //     file << Cluster[Key].payload_without_boundary << ",";
    //     file << _result_statistics.point_cloud_size << ",";
    //     file << _result_statistics.edge_cloud_size << ",";
    //     file << _result_statistics.original_cluster_size << ",";
    //     file << _result_statistics.cluster_removal.removed_by_point_check <<
    //     ","; file << _result_statistics.remaining_cluster_size << ","; file
    //     << _result_statistics.cluster_removal.boundary_point_check << ",";
    //     file << _result_statistics.cluster_removal.no_edge_check << "\n";
    //     file.close();
    // }
    // else{
    //     cout << "FAILED opening Analysis.txt" << endl;
    //     exit(0);
    // }
    // }
    // Principal Axes
    // Normal vector

    // LidarTag::_assignVectorMarker(BoundaryMarker,
    // visualization_msgs::msg::Marker::ARROW,
    //                       "NormalVector_x" +
    //                       to_string(Cluster[Key].cluster_id), 1, 0, 0, 0,
    //                       0.01, Cluster[Key].principal_axes.col(0),
    //                       Cluster[Key].average);
    // BoundMarkArray.markers.push_back(BoundaryMarker);
    // LidarTag::_assignVectorMarker(BoundaryMarker,
    // visualization_msgs::msg::Marker::ARROW,
    //                       "NormalVector_y" +
    //                       to_string(Cluster[Key].cluster_id), 0, 1, 0, 1,
    //                       0.01, Cluster[Key].principal_axes.col(1),
    //                       Cluster[Key].average);
    // BoundMarkArray.markers.push_back(BoundaryMarker);
    LidarTag::_assignVectorMarker(
      BoundaryMarker, visualization_msgs::msg::Marker::ARROW,
      "NormalVector_z" + to_string(Cluster[Key].cluster_id), 0, 0, 1, 2, 0.01,
      Cluster[Key].principal_axes.col(2), Cluster[Key].average);
    BoundMarkArray.markers.push_back(BoundaryMarker);
    // BoundaryMarker.scale.x = 0;
    // BoundaryMarker.scale.y = 0;
    // BoundaryMarker.scale.z = 0;
    // // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0)
    // ?
    // //                                     -Cluster[Key].normal_vector(0) :
    // Cluster[Key].normal_vector(0); geometry_msgs::Point start_point;
    // geometry_msgs::Point end_point;
    // start_point.x = Cluster[Key].average.x;
    // start_point.y = Cluster[Key].average.y;
    // start_point.z = Cluster[Key].average.z;
    // BoundaryMarker.points.push_back(start_point);
    // end_point.x = Cluster[Key].average.x +
    // Cluster[Key].principal_axes.col(0)(0); end_point.y =
    // Cluster[Key].average.y + Cluster[Key].principal_axes.col(0)(1);
    // end_point.z = Cluster[Key].average.z +
    // Cluster[Key].principal_axes.col(0)(2);
    // BoundaryMarker.points.push_back(end_point);
    // BoundaryMarker.pose.orientation.x = 0;
    // BoundaryMarker.pose.orientation.y = 0;
    // BoundaryMarker.pose.orientation.z = 0;
    // BoundaryMarker.pose.orientation.w = 0;
    // BoundMarkArray.markers.push_back(BoundaryMarker);

    // LidarTag::_assignMarker(BoundaryMarker,
    // visualization_msgs::msg::Marker::ARROW,
    //                       "NormalVector_y" +
    //                       to_string(Cluster[Key].cluster_id), 0, 1, 0,
    //                       Cluster[Key].average, 1, 0.01);
    // BoundaryMarker.scale.x = 0;
    // BoundaryMarker.scale.y = 0;
    // BoundaryMarker.scale.z = 0;
    // // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0)
    // ?
    // //                                     -Cluster[Key].normal_vector(0) :
    // Cluster[Key].normal_vector(0);
    // BoundaryMarker.points.push_back(start_point);
    // end_point.x = Cluster[Key].average.x +
    // Cluster[Key].principal_axes.col(1)(0); end_point.y =
    // Cluster[Key].average.y + Cluster[Key].principal_axes.col(1)(1);
    // end_point.z = Cluster[Key].average.z +
    // Cluster[Key].principal_axes.col(1)(2);
    // BoundaryMarker.points.push_back(end_point);
    // BoundaryMarker.pose.orientation.x = 0;
    // BoundaryMarker.pose.orientation.y = 0;
    // BoundaryMarker.pose.orientation.z = 0;
    // BoundaryMarker.pose.orientation.w = 0;
    // BoundMarkArray.markers.push_back(BoundaryMarker);

    // LidarTag::_assignMarker(BoundaryMarker,
    // visualization_msgs::msg::Marker::ARROW,
    //                       "NormalVector_z" +
    //                       to_string(Cluster[Key].cluster_id), 0, 0, 1,
    //                       Cluster[Key].average, 1, 0.01);
    // BoundaryMarker.scale.x = 0;
    // BoundaryMarker.scale.y = 0;
    // BoundaryMarker.scale.z = 0;
    // // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0)
    // ?
    // //                                     -Cluster[Key].normal_vector(0) :
    // Cluster[Key].normal_vector(0);
    // BoundaryMarker.points.push_back(start_point);
    // end_point.x = Cluster[Key].average.x +
    // Cluster[Key].principal_axes.col(2)(0); end_point.y =
    // Cluster[Key].average.y + Cluster[Key].principal_axes.col(2)(1);
    // end_point.z = Cluster[Key].average.z +
    // Cluster[Key].principal_axes.col(2)(2);
    // BoundaryMarker.points.push_back(end_point);
    // BoundaryMarker.pose.orientation.x = 0;
    // BoundaryMarker.pose.orientation.y = 0;
    // BoundaryMarker.pose.orientation.z = 0;
    // BoundaryMarker.pose.orientation.w = 0;
    // BoundMarkArray.markers.push_back(BoundaryMarker);

    /*
    LidarTag::_assignMarker(BoundaryMarker, visualization_msgs::msg::Marker::ARROW,
                          "NormalVector1" + to_string(Cluster[Key].cluster_id),
                          1, 1, 0,
                          Cluster[Key].average, 1, 0.01);
    BoundaryMarker.scale.x = 0.15;
    // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?
    //                                     -Cluster[Key].normal_vector(0) :
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.x =
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.y =
    Cluster[Key].normal_vector(2); BoundaryMarker.pose.orientation.z =
    Cluster[Key].normal_vector(1); BoundaryMarker.pose.orientation.w = 0;
    BoundMarkArray.markers.push_back(BoundaryMarker);

    LidarTag::_assignMarker(BoundaryMarker, visualization_msgs::msg::Marker::ARROW,
                          "NormalVector2" + to_string(Cluster[Key].cluster_id),
                          1, 1, 0,
                          Cluster[Key].average, 1, 0.01);
    BoundaryMarker.scale.x = 0.15;
    // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?
    //                                     -Cluster[Key].normal_vector(0) :
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.x =
    Cluster[Key].normal_vector(1); BoundaryMarker.pose.orientation.y =
    Cluster[Key].normal_vector(2); BoundaryMarker.pose.orientation.z =
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.w = 0;
    BoundMarkArray.markers.push_back(BoundaryMarker);

    LidarTag::_assignMarker(BoundaryMarker, visualization_msgs::msg::Marker::ARROW,
                          "NormalVector3" + to_string(Cluster[Key].cluster_id),
                          1, 1, 0,
                          Cluster[Key].average, 1, 0.01);
    BoundaryMarker.scale.x = 0.15;
    // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?
    //                                     -Cluster[Key].normal_vector(0) :
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.x =
    Cluster[Key].normal_vector(1); BoundaryMarker.pose.orientation.y =
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.z =
    Cluster[Key].normal_vector(2); BoundaryMarker.pose.orientation.w = 0;
    BoundMarkArray.markers.push_back(BoundaryMarker);

    LidarTag::_assignMarker(BoundaryMarker, visualization_msgs::msg::Marker::ARROW,
                          "NormalVector4" + to_string(Cluster[Key].cluster_id),
                          1, 1, 0,
                          Cluster[Key].average, 1, 0.01);
    BoundaryMarker.scale.x = 0.15;
    // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?
    //                                     -Cluster[Key].normal_vector(0) :
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.x =
    Cluster[Key].normal_vector(2); BoundaryMarker.pose.orientation.y =
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.z =
    Cluster[Key].normal_vector(1); BoundaryMarker.pose.orientation.w = 0;
    BoundMarkArray.markers.push_back(BoundaryMarker);

    LidarTag::_assignMarker(BoundaryMarker, visualization_msgs::msg::Marker::ARROW,
                          "NormalVector5" + to_string(Cluster[Key].cluster_id),
                          1, 1, 0,
                          Cluster[Key].average, 1, 0.01);
    BoundaryMarker.scale.x = 0.15;
    // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?
    //                                     -Cluster[Key].normal_vector(0) :
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.x =
    Cluster[Key].normal_vector(2); BoundaryMarker.pose.orientation.y =
    Cluster[Key].normal_vector(1); BoundaryMarker.pose.orientation.z =
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.w = 0;
    BoundMarkArray.markers.push_back(BoundaryMarker);

    LidarTag::_assignMarker(BoundaryMarker, visualization_msgs::msg::Marker::ARROW,
                          "NormalVector6" + to_string(Cluster[Key].cluster_id),
                          1, 1, 0,
                          Cluster[Key].average, 1, 0.01);
    BoundaryMarker.scale.x = 0.15;
    // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?
    //                                     -Cluster[Key].normal_vector(0) :
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.x =
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.y =
    Cluster[Key].normal_vector(1); BoundaryMarker.pose.orientation.z =
    Cluster[Key].normal_vector(2); BoundaryMarker.pose.orientation.w = 0;
    BoundMarkArray.markers.push_back(BoundaryMarker);

    LidarTag::_assignMarker(BoundaryMarker, visualization_msgs::msg::Marker::ARROW,
                          "NormalVector7" + to_string(Cluster[Key].cluster_id),
                          1, 1, 0,
                          Cluster[Key].average, 1, 0.01);
    BoundaryMarker.scale.x = 0.15;
    // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?
    //                                     -Cluster[Key].normal_vector(0) :
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.x =
    Cluster[Key].normal_vector(0); BoundaryMarker.pose.orientation.y =
    Cluster[Key].normal_vector(1); BoundaryMarker.pose.orientation.z =
    Cluster[Key].normal_vector(2); BoundaryMarker.pose.orientation.w = 0;
    BoundMarkArray.markers.push_back(BoundaryMarker);
    */

    // LidarTag::_assignMarker(BoundaryMarker,
    // visualization_msgs::msg::Marker::SPHERE,
    //                       "Average" + to_string(Cluster[Key].cluster_id),
    //                       r, g, b,
    //                       Cluster[Key].average, 5, 0.04);
    // BoundMarkArray.markers.push_back(BoundaryMarker);

    //_cluster_pub->publish(BoundaryMarkerList);
    // LidarTag::_assignLine(BoundaryMarkerList,
    // visualization_msgs::msg::Marker::LINE_STRIP,
    //                    r, g, b,
    //                    Cluster[Key], 1);
    //_cluster_pub->publish(BoundaryMarkerList);

    // _boundary_marker_pub->publish(BoundMarkArray);
    if (_id_decoding) {
      visualization_msgs::msg::Marker IDMarker;
      LidarTag::_assignMarker(
        IDMarker, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        "Text" + to_string(Cluster[Key].cluster_id), 1, 1, 1, Cluster[Key].average, 1,
        Cluster[Key].tag_size * 0.7, string(to_string(Cluster[Key].rkhs_decoding.id)));
      IDMarkArray.markers.push_back(IDMarker);
    }
    // Draw payload boundary marker
    visualization_msgs::msg::Marker PayloadMarker;

    if (_adaptive_thresholding) {
      // Upper boundary
      for (int i = 0; i < Cluster[Key].tag_edges.upper_line.size(); ++i) {
        LidarTag::_assignMarker(
          PayloadMarker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadUpperBoundary_" + to_string(Cluster[Key].cluster_id), 0, 0, 1,
          Cluster[Key].tag_edges.upper_line[i]->point, i, 0.015);
        PayloadMarkArray.markers.push_back(PayloadMarker);
      }

      // Lower boundary
      for (int i = 0; i < Cluster[Key].tag_edges.lower_line.size(); ++i) {
        LidarTag::_assignMarker(
          PayloadMarker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadLowerBoundary_" + to_string(Cluster[Key].cluster_id), 0, 0, 1,
          Cluster[Key].tag_edges.lower_line[i]->point, i, 0.015);
        PayloadMarkArray.markers.push_back(PayloadMarker);
      }

      // Left boundary (green)
      for (int i = 0; i < Cluster[Key].tag_edges.left_line.size(); ++i) {
        LidarTag::_assignMarker(
          PayloadMarker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadLeftBoundary_" + to_string(Cluster[Key].cluster_id), 0, 1, 0,
          Cluster[Key].tag_edges.left_line[i]->point, i, 0.015);
        PayloadMarkArray.markers.push_back(PayloadMarker);
      }

      // Right boundary (red)
      for (int i = 0; i < Cluster[Key].tag_edges.right_line.size(); ++i) {
        LidarTag::_assignMarker(
          PayloadMarker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadRightBoundary_" + to_string(Cluster[Key].cluster_id), 1, 0, 0,
          Cluster[Key].tag_edges.right_line[i]->point, i, 0.015);
        PayloadMarkArray.markers.push_back(PayloadMarker);
      }
    } else {
      // cout << "size1: " << Cluster[Key].payload_boundary_ptr.size() << endl;
      int count = 0;
      for (int i = 0; i < Cluster[Key].payload_boundary_ptr.size(); ++i) {
        // cout << "i: " << i << endl;
        // if (Cluster[Key].payload_boundary_ptr[i].size()==0) continue;
        // else{
        //     // cout << "size2: " <<
        //     Cluster[Key].payload_boundary_ptr[i].size() << endl;
        //     LidarTag::_assignMarker(PayloadMarker,
        //     visualization_msgs::msg::Marker::SPHERE,
        //                           "PayloadBoundary_" +
        //                           to_string(Cluster[Key].cluster_id), 1, 0,
        //                           0,
        //                           Cluster[Key].payload_boundary_ptr[i][0]->point,
        //                           count, 0.015);
        //     PayloadMarkArray.markers.push_back(PayloadMarker);
        //     count ++;

        //     LidarTag::_assignMarker(PayloadMarker,
        //     visualization_msgs::msg::Marker::SPHERE,
        //                           "PayloadBoundary_" +
        //                           to_string(Cluster[Key].cluster_id), 1, 0,
        //                           0,
        //                           Cluster[Key].payload_boundary_ptr[i][1]->point,
        //                           count, 0.015);
        //     count ++;
        //     PayloadMarkArray.markers.push_back(PayloadMarker);
        // }
        // for (int k=0; k<Cluster[Key].payload_boundary_ptr[i].size(); ++k){
        // cout << "(i, k): (" << i << "," << k << ")" << endl;
        // cout << "size2: " << Cluster[Key].payload_boundary_ptr[i].size() <<
        // endl;
        LidarTag::_assignMarker(
          PayloadMarker, visualization_msgs::msg::Marker::SPHERE,
          "PayloadBoundary_" + to_string(Cluster[Key].cluster_id), 1, 0, 0,
          Cluster[Key].payload_boundary_ptr[i]->point, i, 0.015);
        PayloadMarkArray.markers.push_back(PayloadMarker);
        count++;
        // }
      }
    }

    // Text
    /*
    LidarTag::_assignMarker(PayloadMarker,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING, "PayloadUpperBoundary_text_" +
    to_string(Cluster[Key].cluster_id), 1, 1, 1,
                          Cluster[Key].tag_edges.upper_line[0]->point, 5, 0.05,
                          string("Ring: " +
    to_string(Cluster[Key].tag_edges.upper_line[0]->point.ring))
                          );
    PayloadMarkArray.markers.push_back(PayloadMarker);

    LidarTag::_assignMarker(PayloadMarker,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING, "PayloadLowerBoundary_text_" +
    to_string(Cluster[Key].cluster_id), 1, 1, 1,
                          Cluster[Key].tag_edges.lower_line[0]->point, 5, 0.05,
                          string("Ring: " +
    to_string(Cluster[Key].tag_edges.lower_line[0]->point.ring))
                          );
    PayloadMarkArray.markers.push_back(PayloadMarker);
    */

    // corner points and RANSAC line
    if (_adaptive_thresholding) {
      Eigen::Vector4f EigenPoint;
      PointXYZRI point;  // just for conversion

      for (int i = 0; i < 4; ++i) {  // 4 corners
        // Corners
        if (i != 3) {
          pcl::lineWithLineIntersection(
            Cluster[Key].line_coeff[i], Cluster[Key].line_coeff[i + 1], EigenPoint, 1e-2);
        } else {
          pcl::lineWithLineIntersection(
            Cluster[Key].line_coeff[i], Cluster[Key].line_coeff[0], EigenPoint, 1e-2);
        }

        LidarTag::_eigenVectorToPointXYZRI(EigenPoint, point);
        LidarTag::_assignMarker(
          PayloadMarker, visualization_msgs::msg::Marker::SPHERE,
          "Corner_" + to_string(Cluster[Key].cluster_id), 0, 1, 1, point, i, 0.02);
        PayloadMarkArray.markers.push_back(PayloadMarker);

        // RANSAC
        LidarTag::_assignMarker(
          PayloadMarker, visualization_msgs::msg::Marker::ARROW,
          "RANSAC" + to_string(Cluster[Key].cluster_id), 1, 1, 0, point, i, 0.01);
        double Length = sqrt(
          pow(Cluster[Key].line_coeff[i][3], 2) + pow(Cluster[Key].line_coeff[i][4], 2) +
          pow(Cluster[Key].line_coeff[i][5], 2));
        PayloadMarker.scale.x = 0.15;
        PayloadMarker.pose.orientation.x = Cluster[Key].line_coeff[i][3] / Length;
        PayloadMarker.pose.orientation.y = Cluster[Key].line_coeff[i][4] / Length;
        PayloadMarker.pose.orientation.z = Cluster[Key].line_coeff[i][5] / Length;
        PayloadMarkArray.markers.push_back(PayloadMarker);
      }
    }

    // Add lines to the tag
    // LidarTag::_assignLine(PayloadMarker, PayloadMarkArray,
    //                    visualization_msgs::msg::Marker::LINE_STRIP,
    //                    "left_line_" + to_string(Cluster[Key].cluster_id),
    //                    r, g, b,
    //                    PointTL, PointTR, 1);
    // PayloadMarkArray.markers.push_back(PayloadMarker);

    // TODO: Too many for loops, could be done in a better way.
    // Some of the loops are inside of the _detectionArrayPublisher
    // Changes NEEDED!

    // Draw all detected-filled cluster points

    for (int i = 0; i < Cluster[Key].data.size(); ++i) {
      if (Cluster[Key].data[i].valid != 1) {
        continue;
      }
      // ROS_INFO("Cluster[Key].data[i].valid == 1");
      PointsInClusters++;
      // std::cout << Cluster[Key].data[i].point.ring << " index:" <<
      // Cluster[Key].data[i].index << " x: " << Cluster[Key].data[i].point.x <<
      // " y: " << Cluster[Key].data[i].point.y <<" z:
      // "<<Cluster[Key].data[i].point.z << endl;
      OutCluster->push_back(Cluster[Key].data[i].point);

      double Intensity = Cluster[Key].data[i].point.intensity;
      LidarTag::_assignMarker(
        Marker, visualization_msgs::msg::Marker::SPHERE, to_string(Cluster[Key].cluster_id), Intensity,
        Intensity, Intensity, Cluster[Key].data[i].point, i, 0.01);
      ClusterArray.markers.push_back(Marker);
    }
    if (_mark_cluster_validity) {
      for (int i = 0; i < Cluster[Key].edge_points.size(); ++i) {
        if (Cluster[Key].edge_points[i].valid != 1) continue;
        OutEdgeCluster->push_back(Cluster[Key].edge_points[i].point);
      }
      for (int i = 0; i < Cluster[Key].edge_group1.size(); ++i) {
        EdgeGroup1->push_back(Cluster[Key].edge_group1[i].point);
      }
      for (int i = 0; i < Cluster[Key].edge_group2.size(); ++i) {
        EdgeGroup2->push_back(Cluster[Key].edge_group2[i].point);
      }
      for (int i = 0; i < Cluster[Key].edge_group3.size(); ++i) {
        EdgeGroup3->push_back(Cluster[Key].edge_group3[i].point);
      }
      for (int i = 0; i < Cluster[Key].edge_group4.size(); ++i) {
        EdgeGroup4->push_back(Cluster[Key].edge_group4[i].point);
      }
      for (int ring = 0; ring < _beam_num; ++ring) {
        if (Cluster[Key].payload_right_boundary_ptr[ring] != 0) {
          BoundaryPts->push_back(Cluster[Key].payload_right_boundary_ptr[ring]->point);
        }
        if (Cluster[Key].payload_left_boundary_ptr[ring] != 0) {
          BoundaryPts->push_back(Cluster[Key].payload_left_boundary_ptr[ring]->point);
        }
      }
    }
    // Add all payload points to a vector and will be publish to another
    // channel for both visualizatoin and creating training datasets
    // cout << "payload size2: " << Cluster[Key].payload.size() << endl;
    // for (int i=0; i<Cluster[Key].data.size(); ++i){
    //     if (Cluster[Key].data[i].valid != 1) continue;
    //     Eigen::Vector4f p(Cluster[Key].data[i].point.x,
    //     Cluster[Key].data[i].point.y, Cluster[Key].data[i].point.z, 1);
    //     Eigen::Vector4f tp = Cluster[Key].ini_pose.homogeneous * p;
    //     PointXYZRI point;
    //     point.x = tp[0];
    //     point.y = tp[1];
    //     point.z = tp[2];
    //     point.intensity = Cluster[Key].data[i].point.intensity;
    //     OutPayload->push_back(point);
    //     OutPayload->push_back(Cluster[Key].data[i].point);
    //     // cout << "Intensity: " << Cluster[Key].payload[i]->point.intensity
    //     << endl;
    // }

    // for (int i=0; i<Cluster[Key].edge_points.size(); ++i){
    //     if (Cluster[Key].edge_points[i].valid != 1) continue;
    //     Eigen::Vector4f p(Cluster[Key].edge_points[i].point.x,
    //     Cluster[Key].edge_points[i].point.y,
    //     Cluster[Key].edge_points[i].point.z, 1); Eigen::Vector4f tp =
    //     Cluster[Key].ini_pose.homogeneous * p; PointXYZRI point; point.x =
    //     tp[0]; point.y = tp[1]; point.z = tp[2]; point.intensity =
    //     Cluster[Key].edge_points[i].point.intensity;
    //     OutPayload->push_back(point);
    //     OutPayload->push_back(Cluster[Key].edge_points[i].point);
    //     // cout << "Intensity: " << Cluster[Key].payload[i]->point.intensity
    //     << endl;
    // }
    // ROS_INFO_STREAM("rows: " <<
    // Cluster[Key].rkhs_decoding.template_points_3d.rows());
    // ROS_INFO_STREAM("cols: " <<
    // Cluster[Key].rkhs_decoding.template_points_3d.cols());
    if (_id_decoding) {
      for (int i = 0; i < Cluster[Key].rkhs_decoding.associated_pattern_3d->cols(); ++i) {
        PointXYZRI point;
        point.x = Cluster[Key].rkhs_decoding.associated_pattern_3d->col(i)(0);
        point.y = Cluster[Key].rkhs_decoding.associated_pattern_3d->col(i)(1);
        point.z = Cluster[Key].rkhs_decoding.associated_pattern_3d->col(i)(2);
        if (point.x >= 0) {
          point.intensity = 200;
        } else {
          point.intensity = 50;
        }
        // point.intensity =
        // Cluster[Key].rkhs_decoding.associated_pattern_3d->col(i)(3);
        OutPayload->push_back(point);
      }
      for (int i = 0; i < Cluster[Key].rkhs_decoding.template_points_3d.cols(); ++i) {
        PointXYZRI point;
        point.x = Cluster[Key].rkhs_decoding.template_points_3d.col(i)(0);
        point.y = Cluster[Key].rkhs_decoding.template_points_3d.col(i)(1);
        point.z = Cluster[Key].rkhs_decoding.template_points_3d.col(i)(2);
        point.intensity = Cluster[Key].rkhs_decoding.template_points_3d.col(i)(3);
        OutPayload3D->push_back(point);
      }
    }
    if (_mark_cluster_validity) {
      for (int i = 0; i < Cluster[Key].rkhs_decoding.initial_template_points.cols(); ++i) {
        PointXYZRI point;
        point.x = Cluster[Key].rkhs_decoding.initial_template_points.col(i)(0);
        point.y = Cluster[Key].rkhs_decoding.initial_template_points.col(i)(1);
        point.z = Cluster[Key].rkhs_decoding.initial_template_points.col(i)(2);
        point.intensity = Cluster[Key].rkhs_decoding.initial_template_points.col(i)(3);
        OutInitialTarget->push_back(point);
      }
      for (int i = 0; i < Cluster[Key].rkhs_decoding.template_points.cols(); ++i) {
        PointXYZRI point;
        point.x = Cluster[Key].rkhs_decoding.template_points.col(i)(0);
        point.y = Cluster[Key].rkhs_decoding.template_points.col(i)(1);
        point.z = Cluster[Key].rkhs_decoding.template_points.col(i)(2);
        point.intensity = Cluster[Key].rkhs_decoding.template_points.col(i)(3);
        OutTarget->push_back(point);
      }
    }
    if (_id_decoding) {
      addCorners(_tag_corners, Cluster[Key]);
      getBoundaryCorners(Cluster[Key], BoundaryPts);
      addBoundaryCorners(_tag_boundary_corners, Cluster[Key]);
    }
    // Publish to a lidartag channel
    _detectionArrayPublisher(Cluster[Key]);
  }
  // if (PointsInClusters>_filling_max_points_threshold) {
  //     cout << "Too many points on a tag" << endl;
  //     // exit(-1);
  // }
  detectionsToPub.header = _point_cloud_header;
  detectionsToPub.frame_index = 0; // _point_cloud_header.seq; TODO: deprecated
  pub_corners_array_.header = _point_cloud_header;
  pub_corners_array_.frame_index = 0; // _point_cloud_header.seq; TODO: deprecated
  _boundary_corners_array_.header = _point_cloud_header;
  _boundary_corners_array_.frame_index = 0; //_point_cloud_header.seq; TODO: deprecated
  // cout << "BoundaryMarkerList size/10: " <<
  // BoundaryMarkerList.points.size()/10 << endl;
  _corners_array_pub->publish(pub_corners_array_);
  _boundary_corners_array_pub->publish(_boundary_corners_array_);
  _boundary_marker_pub->publish(BoundMarkArray);
  _cluster_marker_pub->publish(ClusterArray);
  _payload_marker_pub->publish(PayloadMarkArray);
  _id_marker_pub->publish(IDMarkArray);
  // srand(time(0));
  // if (rand()%10 < 7)
  _detectionArray_pub->publish(detectionsToPub);
  colorClusters(Cluster);
  displayClusterPointSize(Cluster);
  displayClusterIndexNumber(Cluster);
  LidarTag::publishLidartagCluster(Cluster);
}

void LidarTag::publishClusterInfo(const ClusterFamily_t cluster)
{
  jsk_msgs::msg::OverlayText detail_valid_text;
  std::string output_str;
  Eigen::IOFormat mat_format(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
  Eigen::Matrix3f M;
  Eigen::Matrix3f input_mat;
  M = _U * _r * _V.transpose();
  input_mat = _Vertices.rightCols(4) * _payload_vertices.transpose();

  int ring = std::round(_beam_num / 2);
  int longer_side_ring = std::round((cluster.top_ring + cluster.bottom_ring) / 2);
  double point_resolution = 2 * M_PI / _LiDAR_system.ring_average_table[ring].average;
  auto distance =
    std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
  int num_vertical_ring = std::abs(cluster.top_ring - cluster.bottom_ring) + 1;
  int num_horizontal_points =
    std::ceil((SQRT2 * _payload_size) / (distance * std::sin(point_resolution)));

  std::stringstream R_ss;
  std::stringstream U_ss;
  std::stringstream V_ss;
  std::stringstream r_ss;
  std::stringstream M_ss;
  std::stringstream vertices_ss;
  std::stringstream input_ss;
  std::stringstream payload_vertices_ss;
  std::stringstream ordered_payload_vertices_ss;
  std::stringstream pa_ss;
  R_ss << _R.format(mat_format);
  U_ss << _U.format(mat_format);
  V_ss << _V.format(mat_format);
  r_ss << _r.format(mat_format);
  M_ss << M.format(mat_format);
  vertices_ss << _Vertices.format(mat_format);
  payload_vertices_ss << _payload_vertices.format(mat_format);
  ordered_payload_vertices_ss << _ordered_payload_vertices.format(mat_format);
  input_ss << input_mat.format(mat_format);
  pa_ss << cluster.principal_axes.format(mat_format);

  detail_valid_text.action = jsk_msgs::msg::OverlayText::ADD;
  detail_valid_text.left = 200;
  detail_valid_text.top = 200;
  detail_valid_text.width = 800;
  detail_valid_text.height = 800;
  detail_valid_text.text_size = 12;
  output_str =
    "cluster_id = " + to_string(cluster.cluster_id) + "\n" + "valid = " + to_string(cluster.valid) +
    "\n" + "detail_valid = " + to_string(cluster.detail_valid) + "\n" +
    "pose_estimation_status = " + to_string(cluster.pose_estimation_status) + "\n" +
    "number of horizontal_points = " + to_string(num_horizontal_points) + "\n" +
    "total duration = " + to_string(_timing.total_duration) + "\n" + "exceeded points size = " +
    to_string(((cluster.data.size() + cluster.edge_points.size()) - cluster.expected_points)) +
    "\n" + "average.x = " + to_string(cluster.average.x) + "\n" +
    "average.y = " + to_string(cluster.average.y) + "\n" +
    "average.z = " + to_string(cluster.average.z) + "\n" +
    "pose.roll = " + to_string(cluster.pose.roll) + "\n" +
    "pose.pitch = " + to_string(cluster.pose.pitch) + "\n" +
    "pose.yaw = " + to_string(cluster.pose.yaw) + "\n" +
    "initial_pose.roll = " + to_string(cluster.initial_pose.roll) + "\n" +
    "initial_pose.pitch = " + to_string(cluster.initial_pose.pitch) + "\n" +
    "initial_pose.yaw = " + to_string(cluster.initial_pose.yaw) + "\n" +
    //   "intersection1_x,y = " + to_string(_intersection1[0]) + ", " +
    //   to_string(_intersection1[1]) + "\n" +
    //   "intersection2_x,y = " + to_string(_intersection2[0]) + ", " +
    //   to_string(_intersection2[1]) + "\n" +
    //   "intersection3_x,y = " + to_string(_intersection3[0]) + ", " +
    //   to_string(_intersection3[1]) + "\n" +
    //   "intersection4_x,y = " + to_string(_intersection4[0]) + ", " +
    //   to_string(_intersection4[1]) + "\n" +
    //     "payload_vertices = " + payload_vertices_ss.str() + "\n" +
    //   "ordered_payload_vertices = " + ordered_payload_vertices_ss.str() +
    //   "\n" + "input = " + input_ss.str() + "\n" +
    // "vertices         = " + vertices_ss.str() + "\n" +
    //   "princlple axis   = " + pa_ss.str() + "\n" +
    //   "U * r * Vt       = " + M_ss.str() + "\n" +
    //   "U                = " + U_ss.str() + "\n" +
    //   "r                = " + r_ss.str() + "\n" +
    //   "V                = " + V_ss.str() + "\n" +
    "R(U*Vt)          = " + R_ss.str() + "\n" +
    "max_intensity = " + to_string(cluster.max_intensity.intensity) + "\n" +
    "min_intensity = " + to_string(cluster.min_intensity.intensity) + "\n" +
    "top ring = " + to_string(cluster.top_ring) + "\n" +
    "bottom ring = " + to_string(cluster.bottom_ring) + "\n" +
    //   "edge_inliers = " + to_string(cluster.edge_inliers) + "\n" +
    //   "data_inliers = " + to_string(cluster.data_inliers) + "\n" +
    //   "inliers = " + to_string(cluster.inliers) + "\n" +
    "percentages_inliers = " + to_string(cluster.percentages_inliers) + "\n" +
    //   "boundary_pts = " + to_string(cluster.boundary_pts) + "\n" +
    //   "payload_without_boundary = " +
    //   to_string(cluster.payload_without_boundary) + "\n" +
    "tag_size = " + to_string(cluster.tag_size) +
    "\n"
    "payload_size = " +
    to_string(_payload_size) + "\n";
  // "special case = " +
  //   to_string(cluster.special_case) + "\n";

  detail_valid_text.text = output_str;
  _detail_valid_text_pub->publish(detail_valid_text);
} 

void LidarTag::getBoundaryCorners(
  ClusterFamily_t cluster, pcl::PointCloud<PointXYZRI>::Ptr boundaryPts)
{
  Eigen::Vector4f line1;
  Eigen::Vector4f line2;
  Eigen::Vector4f line3;
  Eigen::Vector4f line4;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud4(new pcl::PointCloud<pcl::PointXYZ>);

  cloud1->points.resize(boundaryPts->points.size());
  cloud1->clear();
  cloud2->points.resize(boundaryPts->points.size());
  cloud2->clear();
  cloud3->points.resize(boundaryPts->points.size());
  cloud3->clear();
  cloud4->points.resize(boundaryPts->points.size());
  cloud4->clear();
  line_cloud1->points.resize(boundaryPts->points.size());
  line_cloud1->clear();
  line_cloud2->points.resize(boundaryPts->points.size());
  line_cloud2->clear();
  line_cloud3->points.resize(boundaryPts->points.size());
  line_cloud3->clear();
  line_cloud4->points.resize(boundaryPts->points.size());
  line_cloud4->clear();

  for (size_t i = 0; i < boundaryPts->points.size(); i++) {
    Eigen::Vector3f edge_point(
      boundaryPts->points[i].x - cluster.average.x, boundaryPts->points[i].y - cluster.average.y,
      boundaryPts->points[i].z - cluster.average.z);
    Eigen::Matrix<float, 3, 3, Eigen::DontAlign> transform_matrix = cluster.principal_axes;
    transform_matrix = (transform_matrix.transpose()).eval();
    Eigen::Vector3f transformed_edge_point = transform_matrix * edge_point;
    pcl::PointXYZ p;
    p.x = transformed_edge_point(0);
    p.y = transformed_edge_point(1);
    p.z = 0;
    LidarPoints_t group_point;
    group_point.point = boundaryPts->points[i];
    if (transformed_edge_point(0) > 0) {
      if (transformed_edge_point(1) > 0) {
        cloud1->points.push_back(p);
        group_point.point.intensity = 0;
        cluster.edge_group1.push_back(group_point);
      } else {
        cloud2->points.push_back(p);
        group_point.point.intensity = 85;
        cluster.edge_group2.push_back(group_point);
      }
    } else {
      if (transformed_edge_point(1) > 0) {
        cloud4->points.push_back(p);
        group_point.point.intensity = 170;
        cluster.edge_group4.push_back(group_point);
      } else {
        cloud3->points.push_back(p);
        group_point.point.intensity = 255;
        cluster.edge_group3.push_back(group_point);
      }
    }
  }
  if (!LidarTag::_getLines(cloud1, line1, line_cloud1))
    RCLCPP_WARN_STREAM(get_logger(), "Can not get boundary line1");

  if (!LidarTag::_getLines(cloud2, line2, line_cloud2))
    RCLCPP_WARN_STREAM(get_logger(), "Can not get boundary line2");

  if (!LidarTag::_getLines(cloud3, line3, line_cloud3))
    RCLCPP_WARN_STREAM(get_logger(), "Can not get boundary line3");

  if (!LidarTag::_getLines(cloud4, line4, line_cloud4))
    RCLCPP_WARN_STREAM(get_logger(), "Can not get boundary line4");

  Eigen::Vector3f intersection1 = LidarTag::_getintersec(line1, line2);
  Eigen::Vector3f intersection2 = LidarTag::_getintersec(line2, line3);
  Eigen::Vector3f intersection3 = LidarTag::_getintersec(line3, line4);
  Eigen::Vector3f intersection4 = LidarTag::_getintersec(line1, line4);
  Eigen::MatrixXf payload_vertices(3, 4);
  payload_vertices.col(0) = cluster.principal_axes * intersection1;
  payload_vertices.col(1) = cluster.principal_axes * intersection2;
  payload_vertices.col(2) = cluster.principal_axes * intersection3;
  payload_vertices.col(3) = cluster.principal_axes * intersection4;

  Eigen::MatrixXf ordered_payload_vertices = _getOrderedCorners(payload_vertices, cluster);
  Eigen::MatrixXf Vertices = Eigen::MatrixXf::Zero(3, 5);
  utils::formGrid(Vertices, 0, 0, 0, cluster.tag_size);
  Eigen::Matrix3f R;
  std::vector<Eigen::MatrixXf> mats;
  mats = utils::fitGrid_new(Vertices, R, ordered_payload_vertices);

  Eigen::MatrixXf::Index col;
  payload_vertices.row(1).minCoeff(&col);
  utils::eigen2Corners(payload_vertices.col(col), _tag_boundary_corners.right);

  payload_vertices.row(1).maxCoeff(&col);
  utils::eigen2Corners(payload_vertices.col(col), _tag_boundary_corners.left);

  payload_vertices.row(2).minCoeff(&col);
  utils::eigen2Corners(payload_vertices.col(col), _tag_boundary_corners.down);

  payload_vertices.row(2).maxCoeff(&col);
  utils::eigen2Corners(payload_vertices.col(col), _tag_boundary_corners.top);
}

void LidarTag::addCorners(corners tag_corners, ClusterFamily_t cluster)
{
  visualization_msgs::msg::Marker marker, left_marker, right_marker, top_marker, down_marker;
  lidartag_msgs::msg::Corners pub_corners;
  marker.header.frame_id = _pub_frame;
  marker.header.stamp = _clock->now();
  pub_corners.header = marker.header;
  pub_corners.id = cluster.cluster_id;
  pub_corners.frame_index = 0; //_point_cloud_header.seq; TODO: deprecated
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(0.1);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  left_marker = right_marker = top_marker = down_marker = marker;

  left_marker.ns = "tag_left_corners";
  left_marker.pose.position.x = tag_corners.left.x + cluster.average.x;
  left_marker.pose.position.y = tag_corners.left.y + cluster.average.y;
  left_marker.pose.position.z = tag_corners.left.z + cluster.average.z;
  pub_corners.left = left_marker.pose.position;
  left_marker.color.r = 1.0;
  left_marker.color.g = 0.0;
  left_marker.color.b = 0.0;
  _left_corners_pub->publish(left_marker);

  right_marker.ns = "tag_right_corners";
  right_marker.pose.position.x = tag_corners.right.x + cluster.average.x;
  right_marker.pose.position.y = tag_corners.right.y + cluster.average.y;
  right_marker.pose.position.z = tag_corners.right.z + cluster.average.z;
  pub_corners.right = right_marker.pose.position;
  right_marker.color.r = 0.0;
  right_marker.color.g = 0.0;
  right_marker.color.b = 1.0;
  _right_corners_pub->publish(right_marker);

  down_marker.ns = "tag_down_corners";
  down_marker.pose.position.x = tag_corners.down.x + cluster.average.x;
  down_marker.pose.position.y = tag_corners.down.y + cluster.average.y;
  down_marker.pose.position.z = tag_corners.down.z + cluster.average.z;
  pub_corners.down = down_marker.pose.position;
  down_marker.color.r = 0.0;
  down_marker.color.g = 1.0;
  down_marker.color.b = 0.0;
  _down_corners_pub->publish(down_marker);

  top_marker.ns = "tag_top_corners";
  top_marker.pose.position.x = tag_corners.top.x + cluster.average.x;
  top_marker.pose.position.y = tag_corners.top.y + cluster.average.y;
  top_marker.pose.position.z = tag_corners.top.z + cluster.average.z;
  pub_corners.top = top_marker.pose.position;
  top_marker.color.r = 1.0;
  top_marker.color.g = 0.0;
  top_marker.color.b = 1.0;
  _top_corners_pub->publish(top_marker);

  pub_corners_array_.corners.push_back(pub_corners);
}

void LidarTag::addBoundaryCorners(corners tag_corners, ClusterFamily_t cluster)
{
  visualization_msgs::msg::Marker marker, left_marker, right_marker, top_marker, down_marker;
  lidartag_msgs::msg::Corners pub_corners;
  marker.header.frame_id = _pub_frame;
  marker.header.stamp = _clock->now();
  pub_corners.header = marker.header;
  pub_corners.id = cluster.cluster_id;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(0.1);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  left_marker = right_marker = top_marker = down_marker = marker;

  left_marker.ns = "tag_left_boundary_corners";
  left_marker.pose.position.x = tag_corners.left.x + cluster.average.x;
  left_marker.pose.position.y = tag_corners.left.y + cluster.average.y;
  left_marker.pose.position.z = tag_corners.left.z + cluster.average.z;
  pub_corners.left = left_marker.pose.position;
  left_marker.color.r = 1.0;
  left_marker.color.g = 0.0;
  left_marker.color.b = 0.0;
  _left_boundary_corners_pub->publish(left_marker);

  right_marker.ns = "tag_right_boundary_corners";
  right_marker.pose.position.x = tag_corners.right.x + cluster.average.x;
  right_marker.pose.position.y = tag_corners.right.y + cluster.average.y;
  right_marker.pose.position.z = tag_corners.right.z + cluster.average.z;
  pub_corners.right = right_marker.pose.position;
  right_marker.color.r = 0.0;
  right_marker.color.g = 0.0;
  right_marker.color.b = 1.0;
  _right_boundary_corners_pub->publish(right_marker);

  down_marker.ns = "tag_down_boundary_corners";
  down_marker.pose.position.x = tag_corners.down.x + cluster.average.x;
  down_marker.pose.position.y = tag_corners.down.y + cluster.average.y;
  down_marker.pose.position.z = tag_corners.down.z + cluster.average.z;
  pub_corners.down = down_marker.pose.position;
  down_marker.color.r = 0.0;
  down_marker.color.g = 1.0;
  down_marker.color.b = 0.0;
  _down_boundary_corners_pub->publish(down_marker);

  top_marker.ns = "tag_top_boundary_corners";
  top_marker.pose.position.x = tag_corners.top.x + cluster.average.x;
  top_marker.pose.position.y = tag_corners.top.y + cluster.average.y;
  top_marker.pose.position.z = tag_corners.top.z + cluster.average.z;
  pub_corners.top = top_marker.pose.position;
  top_marker.color.r = 1.0;
  top_marker.color.g = 0.0;
  top_marker.color.b = 1.0;
  _top_boundary_corners_pub->publish(top_marker);

  _boundary_corners_array_.corners.push_back(pub_corners);
}

void LidarTag::colorClusters(const std::vector<ClusterFamily_t> & cluster)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster_buff(
    new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB colored_point;
  colored_cluster_buff->reserve(_point_cloud_size);
  int r, g, b;
  srand(100);

  for (int key = 0; key < cluster.size(); ++key) {
    if (cluster[key].valid != 1) {
      r = rand() % 255;
      g = rand() % 255;
      b = rand() % 255;
      for (int i = 0; i < cluster[key].data.size(); ++i) {
        colored_point.x = cluster[key].data[i].point.x;
        colored_point.y = cluster[key].data[i].point.y;
        colored_point.z = cluster[key].data[i].point.z;
        colored_point.r = r;
        colored_point.g = g;
        colored_point.b = b;
        colored_cluster_buff->points.push_back(colored_point);
      }
    }
  }
  sensor_msgs::msg::PointCloud2 colored_cluster_buff_msg;
  pcl::toROSMsg(*colored_cluster_buff, colored_cluster_buff_msg);
  colored_cluster_buff_msg.header.frame_id = _pub_frame;
  colored_cluster_buff_msg.header.stamp = _clock->now();
  _colored_cluster_buff_pub->publish(colored_cluster_buff_msg);
}
void LidarTag::displayClusterPointSize(const std::vector<ClusterFamily_t> & cluster_buff)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(cluster_buff.size());
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = _pub_frame;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(0.1);
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
    if (points_size > 50) {
      marker.header.stamp = _clock->now();
      marker.id = cluster_buff[i].cluster_id;
      marker.text = to_string(points_size);
      marker.ns = "ps_marker_" + to_string(cluster_buff[i].cluster_id);
      marker.pose.position.x = cluster_buff[i].average.x;
      marker.pose.position.y = cluster_buff[i].average.y;
      marker.pose.position.z = cluster_buff[i].average.z;
      marker_array.markers.push_back(marker);
    }
  }
  _ps_cluster_buff__pub->publish(marker_array);
}

void LidarTag::displayClusterIndexNumber(const std::vector<ClusterFamily_t> & cluster_buff)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(cluster_buff.size());
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = _pub_frame;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(0.1);
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
    if (points_size > 50) {
      marker.header.stamp = _clock->now();
      marker.id = cluster_buff[i].cluster_id;
      marker.text = to_string(i) + "/" + to_string(cluster_buff[i].detail_valid);
      marker.ns = "in_marker_" + to_string(cluster_buff[i].cluster_id);
      marker.pose.position.x = cluster_buff[i].average.x;
      marker.pose.position.y = cluster_buff[i].average.y;
      marker.pose.position.z = cluster_buff[i].average.z;
      marker_array.markers.push_back(marker);
    }
  }
  _in_cluster_buff__pub->publish(marker_array);
}

}  // namespace BipedLab
