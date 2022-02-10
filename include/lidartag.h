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

#ifndef LIDARTAG_H
#define LIDARTAG_H

#include <fstream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2/transform_datatypes.h" // KL: why does this not work with <>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
//#include <jsk_msgs/msg/overlay_text.hpp>

// threading
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <tbb/tbb.h>

// To transform to pcl format
#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/PCLPointCloud2.h"

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>

#include <lidartag_msgs/msg/corners.hpp>
#include <lidartag_msgs/msg/corners_array.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>
#include "thread_pool.h"
#include "types.h"
#include "utils.h"

#include "apriltag_utils.h"

namespace BipedLab {
class LidarTag: public rclcpp::Node {
public:
  LidarTag(const rclcpp::NodeOptions & options);
  ~LidarTag();

private:

  struct LidarTagParams
  {
    double linkage_threshold;
    double ransac_threshold;
    int fine_cluster_threshold;       // TODO: REPLACE WITH TAG PARAMETERS
    int filling_gap_max_index;        // TODO: CHECK
    int filling_max_points_threshold; // TODO: REMOVE
    double points_threshold_factor;   // TODO: CHECK
    double distance_to_plane_threshold;
    double max_outlier_ratio;
    int num_points_for_plane_feature;
    double nearby_factor;
    int minimum_ring_boundary_points;
    int np_ring;
    double linkage_tunable;
    int cluster_max_index;
    int cluster_min_index;
    int cluster_max_points_size;
    int cluster_min_points_size;
    bool debug_single_pointcloud;
    double debug_point_x;
    double debug_point_y;
    double debug_point_z;
    int debug_cluster_id;
  } params_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  bool loop_{true}; // pcl visualization
  double inlier_size_;
  std::ofstream fbuff_;
  double cluster_buff_time_stamp_sec_;
  double cluster_buff_time_stamp_nsec_;

  // Flags for functioning
  int adaptive_thresholding_; // Use adaptive thresholding or not
  int collect_dataset_;       // To collect dataset (only publish one tag)
  int sleep_to_display_;      // Sleep for visualization
  double sleep_time_for_vis_; // Sleep for how long?
  int valgrind_check_;        // Debugging with Valgrind
  int fake_tag_;
  int decode_method_; // Which decode methods to use?

  int optimization_solver_;
  int decode_mode_;
  int grid_viz_; // visualize remapping grid
  bool mark_cluster_validity_;
  bool plane_fitting_;     // whether perform plane fitting
  bool pose_optimization_; // optimize pose or not
  bool id_decoding_;       // decode id or not
  bool calibration_;
  bool has_ring_;        // data has ring_num or not
  bool ring_estimation_; // need to estimate ring_num or not
  bool ring_estimated_;
  bool pcl_visualize_cluster_ = false;
  int num_accumulation_; // Accumuate # of scans as a full scan of lidar
  int iter_;             // iterations of frame
  double clearance_;

  std::unique_ptr<boost::thread> extraction_thread_;

  // ROS
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar1_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_pointstag_pub__;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge3_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge4_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr payload_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr payload3d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr initag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_marker_pub_;  // cluster markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_marker_pub_; // cluster boundary
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr id_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr payload_marker_pub_; // payload boundary
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr payload_grid_pub_; // grid visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr payload_grid_line_pub_; // grid visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ideal_frame_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tag_frame_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edge_vector_pub_;
  rclcpp::Publisher<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr lidartag_pose_pub_; // Publish LiDAR pose
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_points_pub_; // Points after minor clusters removed
  rclcpp::Publisher<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr detection_array_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidartag_cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidartag_cluster_edge_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidartag_cluster_transformed_edge_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intersection_marker_array_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_edge_pc_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr average_point_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr beforetransformed_edge_pc_pub_;
  rclcpp::Publisher<lidartag_msgs::msg::CornersArray>::SharedPtr corners_array_pub_;
  rclcpp::Publisher<lidartag_msgs::msg::CornersArray>::SharedPtr boundary_corners_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corners_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_corners_markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr boundary_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cluster_buff_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ps_cluster_buff_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr in_cluster_buff_pub_;

  // Flag
  int point_cloud_received_; // check if a scan of point cloud has received or
                             // not
  int stop_; // Just a switch for exiting this program while using valgrind

  // Queue for pc data
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> point_cloud1_queue_;

  // LiDAR parameters
  bool sensor_qos_;
  rclcpp::Time current_scan_time_; // store current time of the lidar scan
  std::string pointcloud_topic_; // subscribe channel
  std::string pub_frame_;        // publish under what frame?
  std::string lidartag_detections_topic_;
  std::string corners_array_topic_;
  // Overall LiDAR system parameters
  LiDARSystem_t lidar_system_;
  int max_queue_size_;
  int beam_num_;
  double vertical_fov_;

  // PointCould data (for a single scan)
  int point_cloud_size_;
  std_msgs::msg::Header point_cloud_header_;

  // Edge detection parameters
  double depth_threshold_;

  // If on-board processing is limited, limit range of points
  double distance_bound_;
  double distance_threshold_;

  // fiducial marker parameters
  double payload_size_;      // physical payload size
  int tag_family_;           // what tag family ie tag16h5, this should be 16
  int tag_hamming_distance_; // what tag family ie tag16h5, this should be 5
  int max_decode_hamming_;   // max hamming while decoding
  int black_border_;         // black boarder of the fiducial marker
  int num_codes_;
  int num_tag_sizes_;
  std::vector<std::vector<Eigen::MatrixXf>> function_dic_;
  std::vector<std::vector<Eigen::MatrixXf>> function_dic_xyz_;  // for kd tree
  std::vector<std::vector<Eigen::VectorXf>> function_dic_feat_; // for kdtree
  std::vector<std::vector<std::string>> rkhs_function_name_list_;

  // Cluster parameters
  std::vector<double> tag_size_list_;
  double optimization_percent_;

  bool print_ros_info_;
  bool debug_info_;
  bool debug_time_;
  bool debug_decoding_time_;
  bool log_data_;
  bool derivative_method_;
  // Payload
  // releasing
  double payload_intensity_threshold_;
  double opt_lb_;
  double opt_ub_;
  double coa_tunable_;
  double tagsize_tunable_;
  int min_returns_per_grid_;

  GrizTagFamily_t * tf;

  // threading
  int num_threads_;
  std::shared_ptr<ThreadPool> thread_vec_;

  // lock
  boost::mutex point_cloud1_queue_lock_;

  // Debug
  Timing_t timing_;
  TimeDecoding_t time_decoding_;
  sensor_msgs::msg::PointCloud2::SharedPtr debug_pc_;

  // Statistics
  Statistics_t result_statistics_;
  std::string outputs_path_;
  std::string library_path_;

  /* [Main loop]
   * main loop of process
   */
  void mainLoop();

  /* [basic ros]
   * A function to get all parameters from a roslaunch
   * if not get all parameters then it will use hard-coded parameters
   */
  void getParameters();

  /* [basic ros]
             * A function to make sure the program has received at least one pointcloud
             * from subscribed channel
             * at the very start of this program
             */
            /* [basic ros]
   * A function to push the received pointcloud into a queue in the class
   */
  inline void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  /* [Transform/Publish]
   * A function to transform pcl msgs to ros msgs and then publish
   * WhichPublisher should be a string of "organized" or "original"
   * regardless lowercase and uppercase
   */
  void publishPointcloud(
    const pcl::PointCloud<PointXYZRI>::Ptr & t_source_PC,
    const std::string & t_frame_name,
    std::string t_which_publisher);

  /* [Type transformation]
   * A function to transform from a customized type (LiDARpoints_t) of vector of
   * vector (EdgeBuff) into a standard type (PointXYZRI) of pcl vector (out)
   */
  void buffToPclVector(const std::vector<std::vector<LidarPoints_t>> & t_edge_buff,
    pcl::PointCloud<PointXYZRI>::Ptr t_out);

  /* [Pre-Processing]
   * A function to slice the Veloydyne full points to sliced pointed
   * based on ring number
   */
  inline void fillInOrderedPointcloud(const pcl::PointCloud<PointXYZRI>::Ptr & t_pcl_pointcloud,
    std::vector<std::vector<LidarPoints_t>> & t_ordered_buff);
  /*
   * A function to compute angle between the line from origin to this point
   * and z=0 plane in lidar
   * */
  float getAnglefromPt(PointXYZRI & t_point);

  void getAngleVector(const pcl::PointCloud<PointXYZRI>::Ptr & pcl_pointcloud,
    std::vector<float> & angles);
  /* [Type transformation]
   * A function to get pcl OrderedBuff from a ros sensor-msgs form of
   * pointcould queue
   */
  std::vector<std::vector<LidarPoints_t>> getOrderBuff();
  // void GetOrderBuff(std::vector<std::vector<PointXYZRI>>& OrderedBuff);

  /* [LiDAR analysis]
   * A function to get a LiDAR system parameters such as max, min points per
   * scan and how many points per ring
   */
  void analyzeLidarDevice();

  /* [LiDAR analysis]
   * A function to find maximum points and minimum points in a single scan, i.e.
   * to find extrema within 32 rings
   */
  void maxMinPtsInAScan(
    std::vector<int> & t_point_count_table,
    std::vector<MaxMin_t> & t_max_min_table,
    std::vector<MaxMin_t> & t_ring_average_table,
    const std::vector<std::vector<LidarPoints_t>> & t_ordered_buff);

  /* [LiDAR analysis]
   * A function to calculate how many points are supposed to be on a cluster at
   * 1 meter away
   */
  void pointsPerSquareMeterAtOneMeter();

  /*
   * A function to get a number of points on a given-distance tag or object
   * from LiDAR analysis
   */
  int areaPoints(
    const double & t_distance, const double & t_obj_width, const double & t_obj_height);

  /* [LidarTag detection]
   * Given lidar pointcloud, this function performs
   * lidartag detection and decode the corresponding id
   */
  pcl::PointCloud<PointXYZRI>::Ptr lidarTagDetection(
    const std::vector<std::vector<LidarPoints_t>> & t_ordered_buff,
    std::vector<ClusterFamily_t> & t_cluster_buff);

  /* [Edge detection and clustering]
   * A function to
   * (1) calculate the depth gradient and the intensity gradient at a point of a
   * pointcloud (2) group detected 'edge' into different group
   */
  void gradientAndGroupEdges(
    const std::vector<std::vector<LidarPoints_t>> & t_ordered_buff,
    std::vector<std::vector<LidarPoints_t>> & t_edge_buff,
    std::vector<ClusterFamily_t> & t_cluster_buff);

  /* [Edge detection from n consecutive points]
   *<consecutive n points from ring i index j>
   * A function to
   * (1) kick out points are not near to each other
   * (2) find edge points from two side points of these points
   * Return value : 0 mean no edge point, 1 mean the left side point is the edge
   *point Return value : 2 mean the right side point is the edge point, 3 mean
   *two side points are edge points
   */
  int getEdgePoints(const std::vector<std::vector<LidarPoints_t>> & OrderedBuff,
    int i, int j, int n);

  /* [Clustering-Linkage]
   * A function to cluster a single edge point into a new cluster or an existing
   * cluster
   */
  void clusterClassifier(const LidarPoints_t & point,
    std::vector<ClusterFamily_t> & t_cluster_buff);

  /* [Clustering-Update]
   * A function update some information about a cluster if this point belongs to
   * this cluster; if not belonging to this cluster then return and create a new
   * one
   */
  void updateCluster(const LidarPoints_t &t_point,
    ClusterFamily_t & t_old_cluster,
    TestCluster_t * t_new_cluster);

  /* [Adaptive-Clustering]
   * A function that determines if a point is within a given cluster adaptively
   * based on the ring number and range of the point.
   */
  bool isWithinCluster(const LidarPoints_t & point, ClusterFamily_t & cluster);

  /* [Adaptive-Clustering]
   * A function that determines if a point is within a given cluster
   * horizontally and adaptively based on the range of the point.
   */
  bool isWithinClusterHorizon(const LidarPoints_t & point,
    ClusterFamily_t & cluster, double threshold);

  /* [Clustering-Validation] <For all Clusters>
   * A function to
   * (1) remove invalid cluster based on the index is too far or not
   * (2) fill in the points between index of edges
   * (3) after filling, if the points are too less (based on the analyzed system
   *     and given distant of the cluster), then remove this cluster
   * (4) Adaptive thresholding (Maximize and minimize intensity) by comparing
   *     with the average value
   */
  void fillInCluster(const std::vector<std::vector<LidarPoints_t>> & t_ordered_buff,
    std::vector<ClusterFamily_t> & t_cluster_buff);

  /* [Clustering-Validation] <For "a" cluster>
   * A valid cluster, valid tag, the points from the original point cloud that
   * belong to the cluster could be estimated from the LiDAR system Therefore,
   * if the points on the tag is too less, which means it is not a valid tag
   * where it might just a shadow of a valid tag
   */
  bool clusterPointsCheck(ClusterFamily_t & t_cluster);

  /* [Clustering-Validation] <A cluster> TODO:RENAME
   * A function to
   * (1) do adaptive thresholding (Maximize and minimize intensity) by comparing
   *     with the average value and
   * (2) sort points with ring number and re-index with current cluster into
   *     tag_edges vector so as to do regression boundary lines
   * (3) It will *remove* if linefitting fails
   */
  bool adaptiveThresholding(ClusterFamily_t & t_cluster);

  /* [Clustering-Validation] <A cluster>
   * A function to fit 4 lines of a payload in a cluster by
   * (1) finding the edges of the payload (how to find is stated below)
   * (2) rejecting and removing the cluster if one of the line is too short
   */
  bool detectPayloadBoundries(ClusterFamily_t & t_cluster);

  /* [Payload extraction] <A cluster>
   * A function to extract the payload points from a valid cluster.
   * Let's say we have 10 points on the left boundary (line) of the tag and 9
   * points on the right boundary (line) of the tag. It is separated into two
   * parts.
   * TODO: should use medium instead of max points
   *  (i) Find the max points that we have on a ring in this cluster by
   *      exmaming the average points on the first 1/2 rings int((10+9)/4)
   * (ii) For another half of the rings, we just find the start index and add
   * the average number of points to the payload points
   */
  void extractPayloadWOThreshold(ClusterFamily_t & t_cluster);

  /* <A cluster>
   * A function to calculate the average point of valid edge points
   */
  void organizeDataPoints(ClusterFamily_t & t_cluster);

  /* [Edge points and principal axes]
   * A function to transform the edge points to the tag frame
   */
  bool transformSplitEdges(ClusterFamily_t & t_cluster);

  /* <A cluster>
   * A function to store transformed points
   */
  void storeTemplatePts(ClusterFamily_t & t_cluster);

  /* [Unordered corner points]
   * A function to reorder the undered corner points from PCA
   */
  Eigen::MatrixXf getOrderedCorners(
      Eigen::MatrixXf & t_payload_vertices, ClusterFamily_t & t_cluster);

  /* [two lines]
   * A function to compute the intersection of two lines
   */
  Eigen::Vector3f getintersec(const Eigen::Vector4f & t_line1,
    const Eigen::Vector4f & t_line2);

  /* [four corner points]
   * A function to compute tag size according to the corner points of the tag
   */
  bool estimateTargetSize(ClusterFamily_t & t_cluster,
    const Eigen::Vector3f & point1,
    const Eigen::Vector3f & point2,
    const Eigen::Vector3f & point3,
    const Eigen::Vector3f & point4);

  /* [A set of 2D points]
   * A function to transform the edge points to the tag frame
   */
  bool getLines(pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud,
    Eigen::Vector4f & t_line,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & line_cloud);

  /* [Type transformation]
   * A function to transform an eigen type of vector to pcl point type
   */
  void eigenVectorToPointXYZRI(const Eigen::Vector4f & t_vector, PointXYZRI & t_point);

  /* [Type transformation]
   * A function to transform a pcl point type to an eigen vector
   */
  void pointXYZRIToEigenVector(const PointXYZRI & point, Eigen::Vector4f & vector);

  /* [Normal vector]
   * A function to estimate the normal vector of a potential payload
   */
  // Eigen::MatrixXf
  void estimatePrincipleAxis(ClusterFamily_t & cluster);

  /* [pose]
   * A function to estimate the pose of a potential payload
   */
  [[maybe_unused]] Homogeneous_t estimatePose(ClusterFamily_t & cluster);

  /*[oritented vector]
   */
  Eigen::Vector3f estimateEdgeVector(ClusterFamily_t & cluster);
  /* [pose]
   * A function to optimize the pose of a potential payload with  L1
   * optimization
   */
  int optimizePose(ClusterFamily_t & cluster);

  bool optimizePoseGrad(ClusterFamily_t & cluster);

  /* [Pose: tag to robot]
   * A function to publish pose of tag to the robot
   */
  [[maybe_unused]] void tagToRobot(const int & t_cluster_id, const Eigen::Vector3f & t_normal_vec,
    Homogeneous_t & t_pose, tf2::Transform & t_transform, const PointXYZRI & t_ave,
    lidartag_msgs::msg::LidarTagDetectionArray & lidartag_pose_array);

  /* [Payload decoding]
   * A function to decode payload with different means
   * 0: Naive decoding
   * 1: Weighted Gaussian
   * 2: Deep learning
   * 3: Gaussian Process
   * 4: ?!
   */
  bool decodePayload(ClusterFamily_t & t_cluster);

  /* [Decoder]
   * A function to determine a codeword on a payload using equal weight
   * methods
   */
  void getCodeNaive(std::string & t_code, pcl::PointCloud<LidarPoints_t *> t_payload);

  /* [Decoder]
   * Decode using Weighted Gaussian weight
   * return  0: normal
   * return -1: not enough return
   * return -2: fail corner detection
   */
  int getCodeWeightedGaussian(
    std::string & code, Homogeneous_t & t_pose, int & t_payload_points,
    const PointXYZRI & ave, const pcl::PointCloud<LidarPoints_t *> & t_payload,
    const std::vector<LidarPoints_t *> & t_payload_boundary_ptr);

  /* [Decoder]
   * 1) Transfrom the payload points to 3D-shape pc
   * 2) Compute inner product
   * 3)
   */
  int getCodeRKHS(RKHSDecoding_t & rkhs_decoding, const double & tag_size);

  Eigen::MatrixXf construct3DShapeMarker(RKHSDecoding_t & rkhs_decoding, const double & ell);

  float computeFunctionInnerProduct(const Eigen::MatrixXf & pc1,
    const Eigen::MatrixXf & pc2,
    const float & ell);

  void computeFunctionOriginalInnerProduct(
    const Eigen::MatrixXf & pc1, const float & num_pc1,
    const Eigen::MatrixXf & pc2, const float & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void computeFunctionMatrixInnerProduct(
    const Eigen::MatrixXf & pc1, const float & num_pc1,
    const Eigen::MatrixXf & pc2, const float & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void computeFunctionVectorInnerProduct(
    const Eigen::MatrixXf & pc1, const float & num_pc1,
    const Eigen::MatrixXf & pc2, const float & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void assignClusterPose(
    const Homogeneous_t & h_tl, Homogeneous_t & h_lt, const int & rotation_angle);

  void singleTask(
    const Eigen::ArrayXf & x_ary, const Eigen::ArrayXf & y_ary,
    const Eigen::ArrayXf & z_ary, const Eigen::ArrayXf & i_ary,
    const Eigen::MatrixXf & pc1_j, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void singleTaskFixedSize(
    const Eigen::ArrayXf & x_ary,
    const Eigen::ArrayXf & y_ary,
    const Eigen::ArrayXf & z_ary,
    const Eigen::ArrayXf & i_ary,
    const Eigen::MatrixXf & pc1_j, const float & geo_sig,
    const float & feature_ell, const float & geo_ell,
    float & score);

  void multipleTasks(
    const Eigen::ArrayXf & x_ary, const Eigen::ArrayXf & y_ary,
    const Eigen::ArrayXf & z_ary, const Eigen::ArrayXf & i_ary,
    const Eigen::MatrixXf & pc1_j, const float & geo_sig,
    const float & feature_ell, const float & geo_ell,
    float & score);

  void computeFunctionVectorInnerProductThreading(
    const Eigen::MatrixXf & pc1, const int & num_pc1,
    const Eigen::MatrixXf & pc2, const int & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void test(
    const Eigen::ArrayXf & x_ary, const Eigen::ArrayXf & y_ary,
    const Eigen::ArrayXf & z_ary, const Eigen::ArrayXf & i_ary,
    const Eigen::MatrixXf & pc1_j, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void computeFunctionVectorInnerProductTBBThreadingNoScheduling(
    const Eigen::MatrixXf & pc1, const int & num_pc1,
    const Eigen::MatrixXf & pc2, const int & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void computeFunctionVectorInnerProductTBBThreadingManualScheduling(
    const Eigen::MatrixXf & pc1, const int & num_pc1,
    const Eigen::MatrixXf & pc2, const int & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void computeFunctionVectorInnerProductTBBThreadingTBBScheduling(
    const Eigen::MatrixXf & pc1, const int & num_pc1,
    const Eigen::MatrixXf & pc2, const int & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void computeFunctionOriginalInnerProductTBB(
    const Eigen::MatrixXf & pc1, const float & num_pc1,
    const Eigen::MatrixXf & pc2, const float & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void computeFunctionOriginalInnerProductKDTree(
    const Eigen::MatrixXf & pc1, const int & num_pc1,
    const Eigen::MatrixXf & pc2, const int & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  void computeFunctionInnerProductModes(
    const int mode, const Eigen::MatrixXf & pc1, const float & num_pc1,
    const Eigen::MatrixXf & pc2, const float & num_pc2, const float & geo_sig,
    const float & feature_ell, const float & geo_ell, float & score);

  /* [Decoder]
   * Create hash table of chosen tag family
   */
  void initDecoder();

  /* [Decoder]
   * Feed a code to test if initialized correctly
   */
  [[maybe_unused]] void testInitDecoder();

  /* [ros-visualization]
   * A function to prepare for visualization results in rviz
   */
  void clusterToPclVectorAndMarkerPublisher(
    std::vector<ClusterFamily_t> & t_cluster,
    pcl::PointCloud<PointXYZRI>::Ptr t_out_cluster,
    pcl::PointCloud<PointXYZRI>::Ptr t_out_edge_cluster,
    pcl::PointCloud<PointXYZRI>::Ptr t_out_payload,
    pcl::PointCloud<PointXYZRI>::Ptr t_out_payload3d,
    pcl::PointCloud<PointXYZRI>::Ptr t_out_target,
    pcl::PointCloud<PointXYZRI>::Ptr t_ini_out_target,
    pcl::PointCloud<PointXYZRI>::Ptr t_edge1,
    pcl::PointCloud<PointXYZRI>::Ptr t_edge2,
    pcl::PointCloud<PointXYZRI>::Ptr t_edge3,
    pcl::PointCloud<PointXYZRI>::Ptr t_edge4,
    pcl::PointCloud<PointXYZRI>::Ptr t_boundary_pts,
  visualization_msgs::msg::MarkerArray & t_marker_array);

  void plotIdealFrame();

  void plotTagFrame(const ClusterFamily_t & t_cluster);

  visualization_msgs::msg::Marker visualizeVector(
    Eigen::Vector3f edge_vector, PointXYZRI centriod, int t_id);
  /* [accumulating temporal cluster]
   * A function to save temporal clusters data
   */
  // void _saveTemporalCluster(const std::vector<ClusterFamily_t> &t_cluster,
  // std::vector<std::vector<pcl::PointCloud<LidarPoints_t>>> &matData);

  /* [save points to mat files]
   * A function to save points as .mat data
   */
  // void
  // _saveMatFiles(std::vector<std::vector<pcl::PointCloud<LidarPoints_t>>>&
  // matData);

            // [A function to put clusterFamily to LidarTagDetectionArray]
  void detectionArrayPublisher(
    const ClusterFamily_t & cluster, lidartag_msgs::msg::LidarTagDetectionArray & detections_array);

  /* [Drawing]
   * A function to draw lines in rviz
   */
  [[maybe_unused]] void assignLine(
    visualization_msgs::msg::Marker & marker,
    visualization_msgs::msg::MarkerArray t_mark_array,
    const uint32_t shape, const std::string ns, const double r,
    const double g, const double b, const PointXYZRI t_point1,
    const PointXYZRI t_point2, const int t_count);

  /* [Drawing]
   * A function to assign markers in rviz
   */
  void assignMarker(
    visualization_msgs::msg::Marker & t_marker,
    const uint32_t t_shape, const std::string t_namespace,
    const double r, const double g, const double b,
    const PointXYZRI & t_point, const int t_count,
    const double t_size, const std::string text = "");


  void assignVectorMarker(
    visualization_msgs::msg::Marker & t_marker, const uint32_t t_shape,
    const std::string t_namespace, const double r, const double g, const double b,
    const int t_count, const double t_size, Eigen::Vector3f t_edge_vector,
    const PointXYZRI &t_centriod, const std::string text = "");

  void printStatistics(const std::vector<ClusterFamily_t> & clusterBuff);

  std::vector<int> getValidClusters(const std::vector<ClusterFamily_t> & clusterBuff);

  int maxPointsCheck(ClusterFamily_t & cluster);

  bool rejectWithPlanarCheck(
    ClusterFamily_t & cluster, pcl::PointIndices::Ptr inliers,
    pcl::ModelCoefficients::Ptr coefficients, std::ostream &fplanefit);

  void initFunctionDecoder();


  /*****************************************************
   * not used
   *****************************************************/

  // Clean up
  void visualiseClusterBuff(std::vector<ClusterFamily_t> & cluster_buff);
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * nothing);
  void publishLidartagCluster(const std::vector<ClusterFamily_t> & cluster_buff);
  //void publishClusterInfo(const ClusterFamily_t cluster); jsk packages are missing
  void publishIntersections(const std::vector<Eigen::VectorXf> intersection_list);
  void printClusterResult(const std::vector<ClusterFamily_t> & cluster_buff);
  void addCorners(
    const ClusterFamily_t & cluster,
    lidartag_msgs::msg::CornersArray & corners_array_msg,
    visualization_msgs::msg::MarkerArray & corners_markers_msg);
  void addBoundaryCorners(
    const ClusterFamily_t & cluster,
    lidartag_msgs::msg::CornersArray & corners_array_msg,
    visualization_msgs::msg::MarkerArray & corners_markers_msg);
  void addCornersAux(
    const ClusterFamily_t & cluster,
    const std::vector<geometry_msgs::msg::Point> & vertex_msg,
    lidartag_msgs::msg::CornersArray & corners_array_msg,
    visualization_msgs::msg::MarkerArray & corners_markers_msg);
  //bool getBoundaryCorners(ClusterFamily_t & cluster, pcl::PointCloud<PointXYZRI>::Ptr boundaryPts); deprecated
  void colorClusters(const std::vector<ClusterFamily_t> & cluster);
  void displayClusterPointSize(const std::vector<ClusterFamily_t> & cluster_buff);
  void displayClusterIndexNumber(const std::vector<ClusterFamily_t> & cluster_buff);

}; // GrizTag
} // namespace BipedLab
#endif
