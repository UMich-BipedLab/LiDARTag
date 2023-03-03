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

#pragma once

#include "nanoflann.hpp"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <chrono> // high_resolution_clock
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <filesystem>

namespace BipedLab {
typedef velodyne_pointcloud::PointXYZIR PointXYZRI;
typedef struct QuickDecodeEntry {
  uint64_t rcode;    // the queried code
  uint16_t id;       // the tag id (a small integer)
  uint16_t hamming;  // how many errors corrected?
  uint16_t rotation; // number of rotations [0, 3]
} QuickDecodeEntry_t;

typedef struct QuickDecode {
  int nentries;
  QuickDecodeEntry_t *entries;
} QuickDecode_t;

typedef struct PayloadVoting {
  PointXYZRI *p;
  float weight;
  int cell;
  PointXYZRI centroid;
} PayloadVoting_t;

// velodyne_pointcloud::PointXYZIR operator+ (const PointXYZRI& p1, const
// PointXYZRI p2) {
//         PointXYZRI tmp;
//         tmp.x = p1.x + p2.x;
//         tmp.y = p1.y + p2.y;
//         tmp.z = p1.z + p2.z;
//         tmp.intensity = p1.intensity + p2.intensity;
//         return tmp;
// };
typedef struct MaxMin {
  int min;
  int average;
  int max;
} MaxMin_t;

struct angleComparision {
  bool operator()(const float &i, const float &j) const {
    // if (std::abs(i - j) > 0.0017)
    //     return true;
    // else
    //     return false;
    // return (std::abs(i - j) > 0.004);
    // return (std::abs(i - j) > 0.005);
    // const int i_int = static_cast<int>(i * 1000);
    // const int j_int = static_cast<int>(j * 1000);
    // return i_int < j_int;

    float threshold = 0.3;
    if (std::abs(i - j) < threshold) {
      return false;
    } else {
      return i < j;
    }
  }
  // bool operator() (const pair<float, float> &lhs, const pair<float,float>
  // &rhs) const{
  //     return (lhs.second - lhs.first > rhs.second - rhs.first);
  // }
};

// Structure for LiDAR system
typedef struct LiDARSystem {
  std::vector<std::vector<int>>
      point_count_table; // point per ring  PointCountTable[Scan][ring]
  std::vector<MaxMin_t> max_min_table; // max min points in a scan
  std::vector<MaxMin_t>
      ring_average_table; // max, min, average points in a ring, examed through
                          // out a few seconds
  // std::vector<float> angle_list; // store the angle of each point
  std::set<float, angleComparision> angle_list;

  double points_per_square_meter_at_one_meter; // TODO: only assume place the
                                               // tag at dense-point area
  double beam_per_vertical_radian;
  double point_per_horizontal_radian;
} LiDARSystem_t;

// Struture for LiDAR PointCloud with index
typedef struct LiDARPoints {
  PointXYZRI point;
  int index;
  int valid;
  double tag_size; // only take abs value due to uncertain direction
  double
      box_width; // Also account for direction by knowing tag is white to black
  double threshold_intensity;
} LidarPoints_t;

typedef struct TagLines {
  int upper_ring;
  int lower_ring;
  std::vector<LidarPoints_t *> upper_line; // basically just a specific ring,
                                           // just point to it should be fine
  std::vector<LidarPoints_t *> lower_line; // same above
  std::vector<LidarPoints_t *> left_line;  // same
  std::vector<LidarPoints_t *> right_line; // same above

  std::vector<LidarPoints_t *> bottom_left;  // basically just a specific ring,
                                             // just point to it should be fine
  std::vector<LidarPoints_t *> bottom_right; // same above
  std::vector<LidarPoints_t *> top_left;     // same
  std::vector<LidarPoints_t *> top_right;    // same above
} TagLines_t;

typedef struct TagBoundaries {
  int status;                            // 0 is up right, 1 is tilted
  std::vector<LidarPoints_t *> line_one; // basically just a specific ring, just
                                         // point to it should be fine
  std::vector<LidarPoints_t *> line_two; // same above
  std::vector<LidarPoints_t *> line_three; // same
  std::vector<LidarPoints_t *> line_four;  // same above
} TagBoundaries_t;

typedef struct Homogeneous {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  float roll;
  float pitch;
  float yaw;
  Eigen::Matrix<float, 3, 1, Eigen::DontAlign> translation;
  Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation;
  Eigen::Matrix<float, 4, 4, Eigen::DontAlign> homogeneous;
} Homogeneous_t;

typedef struct Grid {
  float cx;
  float cz;
  float cy;
} Grid_t;

typedef struct RKHSDecoding {
  Eigen::MatrixXf initial_template_points;
  Eigen::MatrixXf template_points;
  Eigen::MatrixXf template_points_xyz;
  Eigen::VectorXf template_points_feat;
  Eigen::MatrixXf template_points_3d;
  Eigen::MatrixXf *associated_pattern_3d;
  std::vector<float> score;
  int num_points;
  int size_num;
  int rotation_angle;
  double ell;
  double ave_intensity;
  int id;
  float id_score;
} RKHSDecoding_t;

typedef struct {
  float x;
  float y;
  float z;
} point;
typedef struct {
  point top;
  point down;
  point left;
  point right;
} corners;

enum class LidartagErrorCode //C++11 scoped enum
{
  NoError = 0,
  ClusterMinPointsCriteria = 1,
  ClusterMaxPointsCriteria = 2, // Related parameters: cluster_check_max_points (do not use on mems)
  PlanarCheckCriteria = 3,
  PlanarOutliersCriteria = 4,
  DecodingPointsCriteria = 5, // Related parameters: payload_intensity_threshold
  DecodingRingsCriteria = 6,
  CornerEstimationMinPointsCriteria = 7,
  Line1EstimationCriteria = 8,
  Line2EstimationCriteria = 9,
  Line3EstimationCriteria = 10,
  Line4EstimationCriteria = 11,
  RectangleEstimationCirteria = 12,
  TagSizeEstimationCriteria = 13,
  OptimizationErrorCriteria = 14,
  DecodingErrorCriteria = 15
};

typedef struct ClusterFamily {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int cluster_id;
  int valid;
  int top_ring;
  int bottom_ring;
  PointXYZRI top_most_point;
  PointXYZRI bottom_most_point;

  PointXYZRI front_most_point;
  PointXYZRI back_most_point;

  PointXYZRI right_most_point;
  PointXYZRI left_most_point;

  PointXYZRI average;                  // Average point
  PointXYZRI max_intensity;            // Maximux intensity point
  PointXYZRI min_intensity;            // Minimum intensity point
  pcl::PointCloud<LidarPoints_t> data; // data doesn't have edge points
  pcl::PointCloud<LidarPoints_t> edge_points;
  pcl::PointCloud<LidarPoints_t> transformed_edge_points;
  pcl::PointCloud<PointXYZRI> initial_corners;

  // If the first point of the ring is the cluster.
  // If so, the the indices fo the two sides will be far away
  int special_case;
  Eigen::MatrixXf merged_data;   // this includes edge and filled-in points
  Eigen::MatrixXf merged_data_h; // this includes edge and filled-in points

  std::vector<MaxMin_t>
      max_min_index_of_each_ring; // to fill in points between end points in
                                  // this cluster
  std::vector<std::vector<LidarPoints_t *>>
      ordered_points_ptr; // of the cluster (to find black margin of the tag)
  std::vector<double>
      accumulate_intensity_of_each_ring; // to find the upper/lower lines of the
                                         // tag
  TagLines_t tag_edges;                  // store line segment from points
  TagBoundaries_t tag_boundaries;

  std::vector<LidarPoints_t *>
      payload_right_boundary_ptr; // of the cluster (to find black margin of the
                                  // tag)
  std::vector<LidarPoints_t *>
      payload_left_boundary_ptr; // of the cluster (to find black margin of the
                                 // tag)
  std::vector<LidarPoints_t *>
      payload_boundary_ptr; // of the cluster (to find black margin of the tag)
  int data_inliers;
  int edge_inliers;
  int inliers;
  double percentages_inliers;
  int boundary_pts;
  int boundary_rings;
  pcl::PointCloud<LidarPoints_t *> payload; // payload points with boundary
  pcl::PointCloud<LidarPoints_t *> RLHS_decoding; // payload points transformed
  int payload_without_boundary; // size of payload points withpout boundary
  double tag_size;
  double box_width;

  pcl::PointCloud<LidarPoints_t> edge_group1;
  pcl::PointCloud<LidarPoints_t> edge_group2;
  pcl::PointCloud<LidarPoints_t> edge_group3;
  pcl::PointCloud<LidarPoints_t> edge_group4;

  // Eigen::Vector3f NormalVector; // Normal vectors of the payload
  Eigen::Matrix<float, 3, 1, Eigen::DontAlign> normal_vector;
  Eigen::Matrix<float, 3, 3, Eigen::DontAlign> principal_axes;
  QuickDecodeEntry_t entry;
  Homogeneous_t pose_tag_to_lidar;
  Homogeneous_t pose;
  Homogeneous_t initial_pose;
  tf2::Transform transform;

  RKHSDecoding_t rkhs_decoding; //

  /* VectorXf:
   *          point_on_line.x : the X coordinate of a point on the line
   *          point_on_line.y : the Y coordinate of a point on the line
   *          point_on_line.z : the Z coordinate of a point on the line
   *          line_direction.x : the X coordinate of a line's direction
   *          line_direction.y : the Y coordinate of a line's direction
   *          line_direction.z : the Z coordinate of a line's direction
   */
  std::vector<Eigen::VectorXf>
      line_coeff; // Upper, left, bottom, right line (count-clockwise)
  LidartagErrorCode detail_valid;
  int pose_estimation_status;
  int expected_points;

} ClusterFamily_t;

typedef struct GrizTagFamily {
  // How many codes are there in this tag family?
  uint32_t ncodes;

  // The codes in the family.
  uint64_t *codes;

  // how wide (in bit-sizes) is the black border? (usually 1)
  uint32_t black_border;

  // how many bits tall and wide is it? (e.g. 36bit tag ==> 6)
  uint32_t d;

  // minimum hamming distance between any two codes. (e.g. 36h11 => 11)
  uint32_t h;

  // a human-readable name, e.g., "tag36h11"
  char *name;

  // some detector implementations may preprocess codes in order to
  // accelerate decoding.  They put their data here. (Do not use the
  // same apriltag_family instance in more than one implementation)
  void *impl;
} GrizTagFamily_t;

typedef struct ClusterRemoval {
  int minimum_return;
  int maximum_return;
  int plane_fitting;        // v
  int plane_outliers;       // v
  int boundary_point_check; // v
  int minimum_ring_points;
  int no_edge_check; // v
  int line_fitting;
  int pose_optimization;
  int decoding_failure;

  // for weighted gaussian
  int decoder_not_return;
  int decoder_fail_corner;
} ClusterRemoval_t;

typedef struct Statistics {
  ClusterRemoval_t cluster_removal;
  int original_cluster_size;
  int remaining_cluster_size;
  int point_cloud_size;
  int edge_cloud_size;
} Statistics_t;

typedef struct Timing {
  // in ms
  std::chrono::steady_clock::time_point start_total_time;
  std::chrono::steady_clock::time_point start_computation_time;
  std::chrono::steady_clock::time_point timing;

  double duration;
  double total_time;
  double total_duration;
  double edging_and_clustering_time;
  double to_pcl_vector_time;
  double fill_in_time;
  double point_check_time;
  double plane_fitting_removal_time;
  double line_fitting_time;
  double organize_points_time;
  double pca_time;
  double split_edge_time;
  double pose_optimization_time;
  double store_template_time;
  double payload_decoding_time;

  double normal_vector_time;
  double tag_to_robot_time;
} Timing_t;

typedef struct TimeDecoding {
  // in ms
  std::chrono::steady_clock::time_point timing;

  double original;
  double matrix;
  double vectorization;
  double tbb_original;
  double tbb_vectorization;
  double manual_scheduling_tbb_vectorization;
  double tbb_scheduling_tbb_vectorization;
  double tbb_kd_tree;
} TimeDecoding_t;

typedef struct TestCluster {
  int flag;
  ClusterFamily_t new_cluster;
} TestCluster_t;

typedef struct Debug {
  std::vector<ClusterFamily_t *> point_check;
  std::vector<ClusterFamily_t *> boundary_point;
  std::vector<ClusterFamily_t *> no_edge;
  std::vector<ClusterFamily_t *> extract_payload;
} Debug_t;

typedef struct PathLeafString {
  std::string
  operator()(const std::filesystem::directory_entry &entry) const {
    return entry.path().filename().string();
  }
} PathLeafString_t;

typedef nanoflann::KDTreeEigenMatrixAdaptor<
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, -1,
    nanoflann::metric_L2, false>
    kd_tree_t;

typedef Eigen::Triplet<float> Trip_t;

} // namespace BipedLab
