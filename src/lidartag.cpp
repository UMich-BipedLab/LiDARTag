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

#include <pcl/ModelCoefficients.h>
#include <pcl/common/intersections.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/filters/passthrough.h>

#include <lidartag/lidartag.hpp>
#include <lidartag/apriltag_utils.hpp>
#include <lidartag/utils.hpp>
#include <lidartag/ultra_puck.hpp>

#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <math.h>   /* sqrt, pow(a,b) */
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* clock_t, clock, CLOCKS_PER_SEC */
#include <unistd.h>
#include <algorithm>  // std::sort
#include <fstream>    // log files
#include <nlopt.hpp>
#include <thread>
#include <iomanip>
#include <sstream>
#include <tbb/global_control.h>

/* CONSTANT */
#define SQRT2 1.41421356237
#define MAX_INTENSITY 255
#define MIN_INTENSITY 0

using namespace std;
using namespace std::chrono;

#define UPDATE_LIDARTAG_PARAM(PARAM_STRUCT, NAME) \
  update_param(parameters, #NAME, PARAM_STRUCT.NAME)

namespace
{
template <typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lidartag"), "Setting parameter [" << name << "] to " << value);
  }
}
}

namespace BipedLab
{
LidarTag::LidarTag(const rclcpp::NodeOptions & options) :
  Node("lidar_tag_node", options), broadcaster_(*this), point_cloud_received_(0),
  clock_(this->get_clock()), lidar_frame_("velodyne"), // what frame of the published pointcloud should be
  stop_(0)
{
  LidarTag::getParameters();

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  if (id_decoding_) {
    cout << "\033[1;32m\n\n===== loading tag family ===== \033[0m\n";
    LidarTag::initDecoder();
  }

  if (decode_method_ != 0 && decode_method_ != 1 && decode_method_ != 2 && decode_method_ != 3) {
    RCLCPP_ERROR(get_logger(), "Please use 0, 1, 2, or 3 for decode_method in the launch file");
    RCLCPP_INFO_STREAM(get_logger(), "currently using: "<< decode_method_);
  }

  cout << "\033[1;32m=========================== \033[0m\n";
  cout << "\033[1;32m=========================== \033[0m\n";

  RCLCPP_INFO(get_logger(), "ALL INITIALIZED!");

  lidar1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_input", rclcpp::SensorDataQoS(), std::bind(&LidarTag::pointCloudCallback,
    this, std::placeholders::_1));

  edge_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("whole_edged_pc", 10);
  transformed_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_points", 10);
  transformed_pointstag_pub__ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_points_tag", 10);
  edge1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("edge_group_1", 10);
  edge2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("edge_group_2", 10);
  edge3_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("edge_group_3", 10);
  edge4_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("edge_group_4", 10);
  boundary_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("boundary_pts", 10);
  initial_corners_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("initial_corners", 10);
  cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("detected_pc", 10);
  payload_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("associated_pattern_3d", 10);
  payload3d_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("template_points_3d", 10);
  tag_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("template_points", 10);
  initag_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("initial_template_points", 10);
  boundary_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("boundary_marker", 10);
  id_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("id_markers", 10);
  cluster_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_marker", 10);
  payload_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("payload_edges", 10);
  payload_grid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("grid", 10);
  payload_grid_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("grid_line", 10);
  ideal_frame_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("ideal_frame", 10);
  tag_frame_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("tag_frame", 10);
  edge_vector_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("edge_vector", 10);
  lidartag_pose_pub_ =
    this->create_publisher<lidartag_msgs::msg::LidarTagDetectionArray>("lidar_tag_pose", 1);
  clustered_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_edge_pc", 5);
  detection_array_pub_ = this->create_publisher<lidartag_msgs::msg::LidarTagDetectionArray>(
    "detections_array", 10);
  lidartag_cluster_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidartag_cluster_points", 10);
  lidartag_cluster_edge_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidartag_cluster_edge_points", 10);
  lidartag_cluster_transformed_edge_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "lidartag_cluster_trasformed_edge_points", 10);
  intersection_marker_array_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("intesection_markers", 10);
  transformed_edge_pc_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_edge_pc", 10);
  average_point_pub_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("average_point", 10);
  beforetransformed_edge_pc_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("before_transformed_edge_pc", 10);
  corners_array_pub_ =
    this->create_publisher<lidartag_msgs::msg::CornersArray>("corners_array", 10);

  corners_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("corners_markers", 10);

  boundary_corners_array_pub_ =
    this->create_publisher<lidartag_msgs::msg::CornersArray>("boundary_corners_array", 10);
  boundary_corners_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("boundary_corners_markers", 10);

  colored_cluster_buff_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cluster_buff", 10);
  ps_cluster_buff_pub_ =this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "cluster_buff_points_size_markers", 10);
  in_cluster_buff_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "cluster_buff_index_number_markers", 10);
  boundary_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("boundary_points", 10);
  ordered_pointcloud_markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("ordered_pointcloud_markers", 10);

  RCLCPP_INFO(get_logger(), "Waiting for pointcloud data");

  // set parameter callback
  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&LidarTag::paramCallback, this, std::placeholders::_1));
}

LidarTag::~LidarTag()
{
  extraction_thread_->join();
}

rcl_interfaces::msg::SetParametersResult LidarTag::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  LidarTagParams params = params_;

  try {
    UPDATE_LIDARTAG_PARAM(params, linkage_threshold);
    UPDATE_LIDARTAG_PARAM(params, ransac_threshold);
    UPDATE_LIDARTAG_PARAM(params, fine_cluster_threshold);
    UPDATE_LIDARTAG_PARAM(params, filling_gap_max_index);
    UPDATE_LIDARTAG_PARAM(params, points_threshold_factor);
    UPDATE_LIDARTAG_PARAM(params, distance_to_plane_threshold);
    UPDATE_LIDARTAG_PARAM(params, max_outlier_ratio);
    UPDATE_LIDARTAG_PARAM(params, num_points_for_plane_feature);
    UPDATE_LIDARTAG_PARAM(params, nearby_factor);
    UPDATE_LIDARTAG_PARAM(params, minimum_ring_boundary_points);
    UPDATE_LIDARTAG_PARAM(params, linkage_tunable);
    UPDATE_LIDARTAG_PARAM(params, cluster_max_index);
    UPDATE_LIDARTAG_PARAM(params, cluster_min_index);
    UPDATE_LIDARTAG_PARAM(params, cluster_max_points_size);
    UPDATE_LIDARTAG_PARAM(params, cluster_min_points_size);
    UPDATE_LIDARTAG_PARAM(params, cluster_check_max_points);
    UPDATE_LIDARTAG_PARAM(params, depth_bound);
    UPDATE_LIDARTAG_PARAM(params, min_rkhs_score);
    UPDATE_LIDARTAG_PARAM(params, optional_fix_cluster);
    UPDATE_LIDARTAG_PARAM(params, use_rectangle_model);
    UPDATE_LIDARTAG_PARAM(params, rectangle_model_use_ransac);
    UPDATE_LIDARTAG_PARAM(params, rectangle_model_max_iterations);
    UPDATE_LIDARTAG_PARAM(params, rectangle_model_max_error);
    UPDATE_LIDARTAG_PARAM(params, rectangle_fix_point_groups);
    UPDATE_LIDARTAG_PARAM(params, refine_cluster_with_intersections);
    UPDATE_LIDARTAG_PARAM(params, use_intensity_channel);
    UPDATE_LIDARTAG_PARAM(params, use_borders_as_corners);
    UPDATE_LIDARTAG_PARAM(params, debug_single_pointcloud);
    UPDATE_LIDARTAG_PARAM(params, debug_point_x);
    UPDATE_LIDARTAG_PARAM(params, debug_point_y);
    UPDATE_LIDARTAG_PARAM(params, debug_point_z);
    UPDATE_LIDARTAG_PARAM(params, debug_cluster_id);
    UPDATE_LIDARTAG_PARAM(params, debug_ring_id);
    UPDATE_LIDARTAG_PARAM(params, debug_scan_id);

    // transaction succeeds, now assign values
    params_ = params;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  // These parameters are linked , so it is better to update them like this
  params_.linkage_threshold = params_.linkage_tunable * payload_size_ * clearance_;

  rectangle_estimator_->setFilterByCoefficients(false);
  rectangle_estimator_->setInlierError(params_.rectangle_model_max_error);
  rectangle_estimator_->setFixPointGroups(params_.rectangle_fix_point_groups);
  rectangle_estimator_->setMaxIterations(params_.rectangle_model_max_iterations);
  rectangle_estimator_->setRANSAC(params_.rectangle_model_use_ransac);

  return result;
}

/*
 * Main loop
 */
void LidarTag::mainLoop()
{
  // Exam the minimum distance of each point in a ring
  RCLCPP_INFO(get_logger(), "Analyzing LiDAR Device");
  LidarTag::analyzeLidarDevice();

  RCLCPP_INFO(get_logger(), "Start points of interest extraction");

  pcl::PointCloud<PointXYZRI>::Ptr clusterpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr clusteredgepc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr payloadpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr boundarypc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr tagpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr ini_tagpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr payload3dpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr edge_group1(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr edge_group2(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr edge_group3(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr edge_group4(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr initial_corners_pc(new pcl::PointCloud<PointXYZRI>);

  clusterpc->reserve(point_cloud_size_);
  clusteredgepc->reserve(point_cloud_size_);
  tagpc->reserve(point_cloud_size_);
  ini_tagpc->reserve(point_cloud_size_);
  edge_group1->reserve(point_cloud_size_);
  edge_group2->reserve(point_cloud_size_);
  edge_group3->reserve(point_cloud_size_);
  edge_group4->reserve(point_cloud_size_);
  payloadpc->reserve(point_cloud_size_);
  payload3dpc->reserve(point_cloud_size_);
  boundarypc->reserve(point_cloud_size_);
  initial_corners_pc->reserve(point_cloud_size_);

  int valgrind_check = 0;

  //tbb::task_scheduler_init tbb_init() deprecated;
  tbb::global_control c(tbb::global_control::max_allowed_parallelism, num_threads_);

  int curr_frame = 0;
  int frame_of_interest = 9;
  int accumulated_scan = 1;
  std::vector<std::vector<LidarPoints_t>> ordered_buff(beam_num_);

  while (rclcpp::ok()) {
    if (debug_time_) {
      timing_ = {
        std::chrono::steady_clock::now(), std::chrono::steady_clock::now(),
        std::chrono::steady_clock::now(),
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    }

    if (debug_decoding_time_) {
      time_decoding_ = {std::chrono::steady_clock::now(), 0, 0, 0, 0, 0, 0, 0, 0};
    }

    // Try to take a pointcloud from the buffer
    if (num_accumulation_ == 1) {
      ordered_buff = LidarTag::getOrderBuff();

      if (ordered_buff.empty()) {
        continue;
      }
    } else {
      std::vector<std::vector<LidarPoints_t>> ordered_buff_cur = LidarTag::getOrderBuff();

      if (ordered_buff_cur.empty()) {
        continue;
      }

      if (accumulated_scan < num_accumulation_) {
        for (int ring = 0; ring < beam_num_; ++ring) {
          ordered_buff[ring].insert(
            ordered_buff[ring].end(), ordered_buff_cur[ring].begin(), ordered_buff_cur[ring].end());
        }

        accumulated_scan++;
        point_cloud_size_ += point_cloud_size_;
        continue;
      }
      accumulated_scan = 1;
    }

    // A vector of clusters
    std::vector<ClusterFamily_t> clusterbuff;

    pcl::PointCloud<PointXYZRI>::Ptr extracted_poi_pc =
      LidarTag::lidarTagDetection(ordered_buff, clusterbuff);

    if (log_data_) {
      printClusterResult(clusterbuff);
      printStatistics(clusterbuff);
    }

    if (pcl_visualize_cluster_) {
      visualiseClusterBuff(clusterbuff);
    }

    clusterpc->clear();
    clusteredgepc->clear();
    payloadpc->clear();
    payload3dpc->clear();
    tagpc->clear();
    ini_tagpc->clear();
    boundarypc->clear();
    edge_group1->clear();
    edge_group2->clear();
    edge_group3->clear();
    edge_group4->clear();
    initial_corners_pc->clear();

    ordered_pointcloud_markers_pub_->publish(ordered_pointcloud_markers_);
    ordered_pointcloud_markers_.markers.clear();

    for (int ring = 0; ring < beam_num_; ++ring) {
      std::vector<LidarPoints_t>().swap(ordered_buff[ring]);
    }

    point_cloud_size_ = 0;

    visualization_msgs::msg::MarkerArray cluster_markers;

    LidarTag::clusterToPclVectorAndMarkerPublisher(
      clusterbuff, clusterpc, clusteredgepc, payloadpc, payload3dpc, tagpc, ini_tagpc, edge_group1,
      edge_group2, edge_group3, edge_group4, boundarypc, initial_corners_pc, cluster_markers);

    // publish lidartag corners
    // publish results for rviz
    LidarTag::plotIdealFrame();
    LidarTag::publishPointcloud(extracted_poi_pc, lidar_frame_, string("wholeedge"));
    LidarTag::publishPointcloud(clusteredgepc, lidar_frame_, string("clusteredgepc"));
    LidarTag::publishPointcloud(clusterpc, lidar_frame_, string("cluster"));
    LidarTag::publishPointcloud(edge_group1, lidar_frame_, string("edgegroup1"));
    LidarTag::publishPointcloud(edge_group2, lidar_frame_, string("edgegroup2"));
    LidarTag::publishPointcloud(edge_group3, lidar_frame_, string("edgegroup3"));
    LidarTag::publishPointcloud(edge_group4, lidar_frame_, string("edgegroup4"));
    LidarTag::publishPointcloud(boundarypc, lidar_frame_, string("boundarypc"));
    LidarTag::publishPointcloud(initial_corners_pc, lidar_frame_, string("initialcornerspc"));

    if (collect_dataset_) {
      if (result_statistics_.remaining_cluster_size == 1) {
        LidarTag::publishPointcloud(payloadpc, lidar_frame_, string("payload"));
        LidarTag::publishPointcloud(payload3dpc, lidar_frame_, string("payload3d"));
        LidarTag::publishPointcloud(tagpc, lidar_frame_, string("target"));
        LidarTag::publishPointcloud(ini_tagpc, lidar_frame_, string("initialtarget"));
      } else if (result_statistics_.remaining_cluster_size > 1)
        cout << "More than one!! " << endl;
      else
        cout << "Zero!! " << endl;
    } else {
      LidarTag::publishPointcloud(payloadpc, lidar_frame_, string("payload"));
      LidarTag::publishPointcloud(payload3dpc, lidar_frame_, string("payload3d"));
      LidarTag::publishPointcloud(tagpc, lidar_frame_, string("target"));
      LidarTag::publishPointcloud(ini_tagpc, lidar_frame_, string("initialtarget"));
    }

    if (sleep_to_display_) {
      rclcpp::sleep_for(std::chrono::milliseconds(int(1000*sleep_time_for_vis_)));
    }

    if (debug_time_) {
      timing_.total_time =
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.start_total_time);
    }

    if (valgrind_check_) {
      valgrind_check++;

      if (valgrind_check > 0) {
        RCLCPP_ERROR(get_logger(), "valgrind out");
        stop_ = 1;
        break;
      }
    }
  } // rclcpp::ok()

}

/*
 * A function to get all parameters from a roslaunch
 * if not get all parameters then it will use hard-coded parameters
 */
void LidarTag::getParameters() {

  this->declare_parameter<double>("distance_threshold");
  this->declare_parameter<int>("sleep_to_display");
  this->declare_parameter<double>("sleep_time_for_visulization");
  this->declare_parameter<int>("valgrind_check");
  this->declare_parameter<int>("fake_data");
  this->declare_parameter<bool>("write_csv");
  this->declare_parameter<bool>("mark_cluster_validity");
  this->declare_parameter<bool>("plane_fitting");
  this->declare_parameter<bool>("optimize_pose");
  this->declare_parameter<bool>("decode_id");
  this->declare_parameter<std::string>("assign_id");
  this->declare_parameter<bool>("has_ring");
  this->declare_parameter<bool>("estimate_ring");
  this->declare_parameter<int>("adaptive_thresholding");
  this->declare_parameter<int>("collect_data");
  this->declare_parameter<std::string>("pointcloud_topic");
  this->declare_parameter<int>("max_queue_size");
  this->declare_parameter<int>("beam_number");
  this->declare_parameter<int>("tag_family");
  this->declare_parameter<int>("tag_hamming_distance");
  this->declare_parameter<int>("max_decode_hamming");
  this->declare_parameter<int>("black_border");
  this->declare_parameter<double>("distance_bound");
  this->declare_parameter<double>("intensity_bound");
  this->declare_parameter<double>("depth_bound");
  this->declare_parameter<int>("fine_cluster_threshold");
  this->declare_parameter<double>("horizontal_fov");
  this->declare_parameter<double>("vertical_fov");
  this->declare_parameter<bool>("use_organized_pointcloud");
  this->declare_parameter<int>("fill_in_gap_threshold");
  this->declare_parameter<double>("points_threshold_factor");
  this->declare_parameter<double>("line_intensity_bound");
  this->declare_parameter<double>("payload_intensity_threshold");
  this->declare_parameter<std::string>("latest_model");
  this->declare_parameter<std::string>("weight_path");
  this->declare_parameter<int>("max_points_on_payload");
  this->declare_parameter<int>("xyzri");
  this->declare_parameter<int>("min_retrun_per_grid");
  this->declare_parameter<int>("optimization_solver");
  this->declare_parameter<int>("decode_method");
  this->declare_parameter<int>("decode_mode");
  this->declare_parameter<int>("grid_viz");
  this->declare_parameter<std::string>("outputs_path");
  this->declare_parameter<std::string>("library_path");
  this->declare_parameter<int>("num_codes");
  this->declare_parameter<double>("distance_to_plane_threshold");
  this->declare_parameter<double>("max_outlier_ratio");
  this->declare_parameter<int>("num_points_for_plane_feature");
  this->declare_parameter<double>("nearby_factor");
  this->declare_parameter<int>("number_points_ring");
  this->declare_parameter<double>("linkage_tunable");
  this->declare_parameter<int>("linkage_ring_max_dist");
  this->declare_parameter<std::vector<double>>("tag_size_list");
  this->declare_parameter<bool>("euler_derivative");
  this->declare_parameter<int>("num_threads");
  this->declare_parameter<bool>("print_info");
  this->declare_parameter<bool>("debug_info");
  this->declare_parameter<bool>("debug_time");
  this->declare_parameter<bool>("debug_decoding_time");
  this->declare_parameter<bool>("log_data");
  this->declare_parameter<double>("optimize_percentage");
  this->declare_parameter<bool>("calibration");
  this->declare_parameter<int>("minimum_ring_boundary_points");
  this->declare_parameter<double>("optimize_up_bound");
  this->declare_parameter<double>("optimize_low_bound");
  this->declare_parameter<int>("num_accumulation");
  this->declare_parameter<double>("coa_tunable");
  this->declare_parameter<double>("tagsize_tunable");
  this->declare_parameter<int>("cluster_max_index");
  this->declare_parameter<int>("cluster_min_index");
  this->declare_parameter<int>("cluster_max_points_size");
  this->declare_parameter<int>("cluster_min_points_size");
  this->declare_parameter<bool>("cluster_check_max_points");
  this->declare_parameter<bool>("debug_single_pointcloud");
  this->declare_parameter<double>("debug_point_x");
  this->declare_parameter<double>("debug_point_y");
  this->declare_parameter<double>("debug_point_z");
  this->declare_parameter<int>("debug_cluster_id");
  this->declare_parameter<int>("debug_ring_id");
  this->declare_parameter<int>("debug_scan_id");
  this->declare_parameter<bool>("pcl_visualize_cluster");
  this->declare_parameter<double>("clearance");
  this->declare_parameter<bool>("optional_fix_cluster");
  this->declare_parameter<bool>("use_rectangle_model");
  this->declare_parameter<bool>("rectangle_model_use_ransac");
  this->declare_parameter<int>("rectangle_model_max_iterations");
  this->declare_parameter<double>("rectangle_model_max_error");
  this->declare_parameter<bool>("rectangle_fix_point_groups");
  this->declare_parameter<bool>("refine_cluster_with_intersections");
  this->declare_parameter<bool>("use_intensity_channel");
  this->declare_parameter<bool>("use_borders_as_corners");
  this->declare_parameter<double>("min_rkhs_score");

  // Naive hamming decoding
  this->declare_parameter<double>("hamming_decoding_min_white_border_bits");
  this->declare_parameter<double>("hamming_decoding_min_black_boder_bits");
  this->declare_parameter<double>("hamming_decoding_min_payload_bits");
  this->declare_parameter<double>("hamming_decoding_min_payload_margin");
  this->declare_parameter<double>("hamming_decoding_intensity_threshold");
  this->declare_parameter<double>("hamming_decoding_rbf_sigma");
  this->declare_parameter<double>("hamming_decoding_decoding_bit_threshold");

  bool GotThreshold = this->get_parameter("distance_threshold", distance_threshold_);
  bool GotSleepToDisplay = this->get_parameter("sleep_to_display", sleep_to_display_);
  bool GotSleepTimeForVis = this->get_parameter("sleep_time_for_visulization", sleep_time_for_vis_);
  bool GotValgrindCheck = this->get_parameter("valgrind_check", valgrind_check_);
  bool GotFakeTag = this->get_parameter("fake_data", fake_tag_);
  bool GotMarkValidity = this->get_parameter("mark_cluster_validity", mark_cluster_validity_);
  bool GotPlaneFitting = this->get_parameter("plane_fitting", plane_fitting_);
  bool GotOptPose = this->get_parameter("optimize_pose", pose_optimization_);
  bool GotDecodeId = this->get_parameter("decode_id", id_decoding_);
  bool GotRingState = this->get_parameter("has_ring", has_ring_);
  bool GotRingEstimation = this->get_parameter("estimate_ring", ring_estimation_);
  bool GotAdaptiveThresholding =
    this->get_parameter("adaptive_thresholding", adaptive_thresholding_);
  bool GotCollectData = this->get_parameter("collect_data", collect_dataset_);
  bool GotMaxQueueSize = this->get_parameter("max_queue_size", max_queue_size_);
  bool GotBeamNum = this->get_parameter("beam_number", beam_num_);
  bool GotSize = this->get_parameter("tag_size", payload_size_);

  bool GotTagFamily = this->get_parameter("tag_family", tag_family_);
  bool GotTagHamming = this->get_parameter("tag_hamming_distance", tag_hamming_distance_);
  bool GotMaxDecodeHamming = this->get_parameter("max_decode_hamming", max_decode_hamming_);
  bool GotBlackBorder = this->get_parameter("black_border", black_border_);

  bool GotDistanceBound = this->get_parameter("distance_bound", distance_bound_);
  bool GotDepthBound = this->get_parameter("depth_bound", params_.depth_bound);
  bool GotFineClusterThreshold =
    this->get_parameter("fine_cluster_threshold", params_.fine_cluster_threshold);
  bool GotHorizontalFOV = this->get_parameter("horizontal_fov", horizontal_fov_);
  bool GotVerticalFOV = this->get_parameter("vertical_fov", vertical_fov_);
  bool GotUseOrganizedPointcloud = this->get_parameter("use_organized_pointcloud", use_organized_pointcloud_);
  bool GotFillInGapThreshold =
    this->get_parameter("fill_in_gap_threshold", params_.filling_gap_max_index);
  bool GotPointsThresholdFactor =
    this->get_parameter("points_threshold_factor", params_.points_threshold_factor);
  bool GotPayloadIntensityThreshold =
    this->get_parameter("payload_intensity_threshold", payload_intensity_threshold_);

  bool GotMinPerGrid = this->get_parameter("min_retrun_per_grid", min_returns_per_grid_);
  bool GotOptimizationMethod = this->get_parameter("optimization_solver", optimization_solver_);
  bool GotDecodeMethod = this->get_parameter("decode_method", decode_method_);
  bool GotDecodeMode = this->get_parameter("decode_mode", decode_mode_);
  bool GotGridViz = this->get_parameter("grid_viz", grid_viz_);

  bool GotOutPutPath = this->get_parameter("outputs_path", outputs_path_);
  bool GotLibraryPath = this->get_parameter("library_path", library_path_);
  bool GotNumCodes = this->get_parameter("num_codes", num_codes_);

  bool GotDistanceToPlaneThreshold =
    this->get_parameter("distance_to_plane_threshold", params_.distance_to_plane_threshold);
  bool GotMaxOutlierRatio = this->get_parameter("max_outlier_ratio", params_.max_outlier_ratio);
  bool GotNumPoints =
    this->get_parameter("num_points_for_plane_feature", params_.num_points_for_plane_feature);
  bool GotNearBound = this->get_parameter("nearby_factor", params_.nearby_factor);
  bool GotCoefficient = this->get_parameter("linkage_tunable", params_.linkage_tunable);
  bool GotLinkageRing = this->get_parameter("linkage_ring_max_dist", params_.linkage_ring_max_dist);

  bool GotTagSizeList = this->get_parameter<std::vector<double>>("tag_size_list", tag_size_list_);
  bool GotDerivativeMethod = this->get_parameter("euler_derivative", derivative_method_);
  bool GotNumThreads = this->get_parameter("num_threads", num_threads_);
  bool GotPrintInfo = this->get_parameter("print_info", print_ros_info_);
  bool GotDebuginfo = this->get_parameter("debug_info", debug_info_);
  bool GotDebugtime = this->get_parameter("debug_time", debug_time_);
  bool GotDebugDecodingtime = this->get_parameter("debug_decoding_time", debug_decoding_time_);
  bool GotLogData = this->get_parameter("log_data", log_data_);
  bool GotOptimizePercent = this->get_parameter("optimize_percentage", optimization_percent_);
  bool GotCalibration = this->get_parameter("calibration", calibration_);
  bool GotMinimumRingPoints =
    this->get_parameter("minimum_ring_boundary_points", params_.minimum_ring_boundary_points);
  bool GotUpbound = this->get_parameter("optimize_up_bound", opt_ub_);
  bool GotLowbound = this->get_parameter("optimize_low_bound", opt_lb_);
  bool GotNumAccumulation = this->get_parameter("num_accumulation", num_accumulation_);
  bool GotCoaTunable = this->get_parameter("coa_tunable", coa_tunable_);
  bool GotTagsizeTunable = this->get_parameter("tagsize_tunable", tagsize_tunable_);
  bool GotMaxClusterIndex = this->get_parameter("cluster_max_index", params_.cluster_max_index);
  bool GotMinClusterIndex = this->get_parameter("cluster_min_index", params_.cluster_min_index);
  bool GotMaxClusterPointsSize =
    this->get_parameter("cluster_max_points_size", params_.cluster_max_points_size);
  bool GotMinClusterPointsSize =
    this->get_parameter("cluster_min_points_size", params_.cluster_min_points_size);
  bool GotClusterCheckMaxPoints =
    this->get_parameter("cluster_check_max_points", params_.cluster_check_max_points);
  bool GotDebugSinglePointcloud =
    this->get_parameter("debug_single_pointcloud", params_.debug_single_pointcloud);
  bool GotDebugPointX =
    this->get_parameter("debug_point_x", params_.debug_point_x);
  bool GotDebugPointY =
    this->get_parameter("debug_point_y", params_.debug_point_y);
  bool GotDebugPointZ =
    this->get_parameter("debug_point_z", params_.debug_point_z);
  bool GotDebugClusterId =
    this->get_parameter("debug_cluster_id", params_.debug_cluster_id);
  bool GotDebugRingId =
    this->get_parameter("debug_ring_id", params_.debug_ring_id);
  bool GotDebugScanId =
    this->get_parameter("debug_scan_id", params_.debug_scan_id);
  bool GotVisualizeCluster = this->get_parameter("pcl_visualize_cluster", pcl_visualize_cluster_);
  bool GotClearance = this->get_parameter("clearance", clearance_);
  bool GotOptionalFixCluster = this->get_parameter("optional_fix_cluster",
    params_.optional_fix_cluster);
  bool GotUSeRectangleModel = this->get_parameter("use_rectangle_model",
    params_.use_rectangle_model);
  bool GotRectangleModelUseRansac = this->get_parameter("rectangle_model_use_ransac",
    params_.rectangle_model_use_ransac);
  bool GotRectangleModelMaxIterations = this->get_parameter("rectangle_model_max_iterations",
    params_.rectangle_model_max_iterations);
  bool GotRectangleMaxError = this->get_parameter("rectangle_model_max_error",
    params_.rectangle_model_max_error);
  bool GotRectangleFixPointGroups = this->get_parameter("rectangle_fix_point_groups",
    params_.rectangle_fix_point_groups);
  bool GotRefineClusterWithInteractions = this->get_parameter("refine_cluster_with_intersections",
    params_.refine_cluster_with_intersections);
  bool GotUseIntensityChannel = this->get_parameter("use_intensity_channel",
    params_.use_intensity_channel);
  bool GotUseBordersAsCorners = this->get_parameter("use_borders_as_corners",
    params_.use_borders_as_corners);
  bool GotMinRKHSScore = this->get_parameter("min_rkhs_score", params_.min_rkhs_score);

  double hamming_decoding_min_white_border_bits, hamming_decoding_min_black_boder_bits,
    hamming_decoding_min_payload_bits, hamming_decoding_min_payload_margin,
    hamming_decoding_intensity_threshold, hamming_decoding_rbf_sigma,
    hamming_decoding_decoding_bit_threshold;

  bool GotHammingDecodingMinWhiteBorderBits = this->get_parameter(
    "hamming_decoding_min_white_border_bits", hamming_decoding_min_white_border_bits);
  bool GotHammingDecodingMinBlackBoderBits = this->get_parameter(
    "hamming_decoding_min_black_boder_bits", hamming_decoding_min_black_boder_bits);
  bool GotHammingDecodingMinPayloadBits =
    this->get_parameter("hamming_decoding_min_payload_bits", hamming_decoding_min_payload_bits);
  bool GotHammingDecodingMinPayloadMargin =
    this->get_parameter("hamming_decoding_min_payload_margin", hamming_decoding_min_payload_margin);
  bool GotHammingDecodingIntensityThreshold = this->get_parameter(
    "hamming_decoding_intensity_threshold", hamming_decoding_intensity_threshold);
  bool GotHammingDecodingRbfSigma =
    this->get_parameter("hamming_decoding_rbf_sigma", hamming_decoding_rbf_sigma);
  bool GotHammingDecodingDecodingBitThreshold = this->get_parameter(
    "hamming_decoding_decoding_bit_threshold", hamming_decoding_decoding_bit_threshold);

  rectangle_estimator_ = std::make_shared<RectangleEstimator>();
  rectangle_estimator_->setFilterByCoefficients(false);
  rectangle_estimator_->setInlierError(params_.rectangle_model_max_error);
  rectangle_estimator_->setFixPointGroups(params_.rectangle_fix_point_groups);
  rectangle_estimator_->setMaxIterations(params_.rectangle_model_max_iterations);
  rectangle_estimator_->setRANSAC(params_.rectangle_model_use_ransac);

  hamming_decoding_ = std::make_shared<NaiveHammingDecoding>(std::to_string(tag_family_),
    library_path_ + "/templates", hamming_decoding_min_white_border_bits, hamming_decoding_min_black_boder_bits,
    hamming_decoding_min_payload_bits, hamming_decoding_min_payload_margin,
    hamming_decoding_intensity_threshold, hamming_decoding_rbf_sigma,
    hamming_decoding_decoding_bit_threshold);

  bool pass = utils::checkParameters(
    {GotFakeTag,
     GotMaxQueueSize,
     GotBeamNum,
     GotOptPose,
     GotDecodeId,
     GotPlaneFitting,
     GotOutPutPath,
     GotDistanceBound,
     GotDepthBound,
     GotTagFamily,
     GotTagHamming,
     GotMaxDecodeHamming,
     GotFineClusterThreshold,
     GotVerticalFOV,
     GotUseOrganizedPointcloud,
     GotFillInGapThreshold,
     GotMaxOutlierRatio,
     GotPointsThresholdFactor,
     GotDistanceToPlaneThreshold,
     GotAdaptiveThresholding,
     GotCollectData,
     GotSleepToDisplay,
     GotSleepTimeForVis,
     GotValgrindCheck,
     GotPayloadIntensityThreshold,
     GotBlackBorder,
     GotMinPerGrid,
     GotDecodeMethod,
     GotDecodeMode,
     GotOptimizationMethod,
     GotGridViz,
     GotThreshold,
     GotNumPoints,
     GotNearBound,
     GotCoefficient,
     GotLinkageRing,
     GotTagSizeList,
     GotNumThreads,
     GotPrintInfo,
     GotOptimizePercent,
     GotDebuginfo,
     GotDebugtime,
     GotLogData,
     GotDebugDecodingtime,
     GotLibraryPath,
     GotNumCodes,
     GotCalibration,
     GotMinimumRingPoints,
     GotRingState,
     GotRingEstimation,
     GotNumAccumulation,
     GotDerivativeMethod,
     GotUpbound,
     GotLowbound,
     GotCoaTunable,
     GotTagsizeTunable,
     GotMaxClusterIndex,
     GotMinClusterIndex,
     GotMaxClusterPointsSize,
     GotMinClusterPointsSize,
     GotClusterCheckMaxPoints,
     GotDebugSinglePointcloud,
     GotDebugPointX,
     GotDebugPointY,
     GotDebugPointZ,
     GotDebugClusterId,
     GotDebugRingId,
     GotDebugScanId,
     GotVisualizeCluster,
     GotClearance,
     GotOptionalFixCluster,
     GotUSeRectangleModel,
     GotRectangleModelUseRansac,
     GotRectangleModelMaxIterations,
     GotRectangleMaxError,
     GotRefineClusterWithInteractions,
     GotUseIntensityChannel,
     GotUseBordersAsCorners,
     GotMinRKHSScore,
     GotHammingDecodingMinWhiteBorderBits,
     GotHammingDecodingMinBlackBoderBits,
     GotHammingDecodingMinPayloadBits,
     GotHammingDecodingMinPayloadMargin,
     GotHammingDecodingIntensityThreshold,
     GotHammingDecodingRbfSigma,
     GotHammingDecodingDecodingBitThreshold});

  if (!pass) {
    rclcpp::shutdown();
  } else {
    cout << "\033[1;32m=========================== \033[0m\n";
    cout << "use parameters from the launch file\n";
    cout << "\033[1;32m=========================== \033[0m\n";
  }

  std::sort(tag_size_list_.begin(), tag_size_list_.end());
  num_tag_sizes_ = tag_size_list_.size();
  payload_size_ = tag_size_list_.back();

  const auto num_processor = std::thread::hardware_concurrency();
  num_threads_ = std::min((int)num_processor, num_threads_);

  iter_ = 0;

  params_.linkage_threshold = params_.linkage_tunable * payload_size_ * clearance_;

  params_.ransac_threshold = payload_size_ / 10;

  RCLCPP_INFO(get_logger(), "Use %i-beam LiDAR\n", beam_num_);
  RCLCPP_INFO(get_logger(), "Use %i threads\n", num_threads_);
  RCLCPP_INFO(get_logger(), "depth_bound: %f \n", params_.depth_bound);
  RCLCPP_INFO(get_logger(), "payload_size_: %f \n", payload_size_);
  RCLCPP_INFO(get_logger(), "horizontal_fov_: %f \n", horizontal_fov_);
  RCLCPP_INFO(get_logger(), "vertical_fov_: %f \n", vertical_fov_);
  RCLCPP_INFO(get_logger(), "fine_cluster_threshold: %i \n", params_.fine_cluster_threshold);
  RCLCPP_INFO(get_logger(), "filling_gap_max_index: %i \n", params_.filling_gap_max_index);
  RCLCPP_INFO(get_logger(), "points_threshold_factor: %f \n", params_.points_threshold_factor);
  RCLCPP_INFO(get_logger(), "adaptive_thresholding_: %i \n", adaptive_thresholding_);
  RCLCPP_INFO(get_logger(), "collect_dataset_: %i \n", collect_dataset_);
  RCLCPP_INFO(get_logger(), "decode_method: %i \n", decode_method_);
  RCLCPP_INFO(get_logger(), "linkage_hreshold_: %f \n", params_.linkage_threshold);
  RCLCPP_INFO(get_logger(), "ransac_threshold: %f \n", params_.ransac_threshold);
  RCLCPP_INFO(get_logger(), "num_accumulation_: %i \n", num_accumulation_);
}

/*
 * A function to get pcl ordered_buff
 * from a ros sensor-msgs form of pointcould queue
 * */
std::vector<std::vector<LidarPoints_t>> LidarTag::getOrderBuff()
{
  point_cloud1_queue_lock_.lock();

  if (point_cloud1_queue_.size() == 0) {
    point_cloud1_queue_lock_.unlock();

    vector<vector<LidarPoints_t>> empty;
    return empty;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr msg = point_cloud1_queue_.front();
  point_cloud1_queue_.pop();
  point_cloud1_queue_lock_.unlock();
  current_scan_time_ = msg->header.stamp;

  // Convert to sensor_msg to pcl type
  pcl::PointCloud<PointXYZRI>::Ptr pcl_pointcloud(new pcl::PointCloud<PointXYZRI>);
  pcl::fromROSMsg(*msg, *pcl_pointcloud);

  // Debug force not use intensity
  if (!params_.use_intensity_channel) {
    for(auto & point : pcl_pointcloud->points) {
      point.intensity = 0;
    }
  }

  if (!has_ring_ && !ring_estimated_) {
    std::vector<float> angles;
    getAngleVector(pcl_pointcloud, angles);

    std::ofstream fangles;
    fangles.open(outputs_path_ + "/angles.txt", std::ofstream::out | std::ofstream::app);

    if (!fangles.is_open()) {
      cout << "Could not open angles.txt: " << outputs_path_ << "\n Currently at: " << __LINE__
        << endl;
      exit(0);
    }

    fangles << std::endl;

    for (int i = 0; i < pcl_pointcloud->size(); ++i) {
      fangles << angles[i] << ",";
    }

    fangles << std::endl;
    fangles.close();
    ring_estimated_ = true;
    vector<vector<LidarPoints_t>> empty;

    return empty;
  }

  // Ordered pointcloud with respect to its own ring number
  std::vector<std::vector<LidarPoints_t>> ordered_buff(beam_num_);
  
  if (use_organized_pointcloud_) {
    fillInOrderedPointcloudFromOrganizedPointcloud(pcl_pointcloud, ordered_buff);
  }
  else {
    fillInOrderedPointcloudFromUnorganizedPointcloud(pcl_pointcloud, ordered_buff);
  }
  
  point_cloud_size_ = pcl_pointcloud->size();

  return ordered_buff;
}

/*
 * A function to get a LiDAR system parameters such as max,
 * min points per scan and how many points per ring
 * The data format is:
 *
 * (A) point_count_table:
 * point_count_table[Scan][Ring]
 * -----------------------------------------
 *      1    2   3   4   5   6   7 ... scan
 * -----------------------------------------
 * 0  17xx  ...
 * -----------------------------------------
 * 1  16xx  ...
 * -----------------------------------------
 * 2
 * .
 * .
 * .
 * 31
 * ring
 *
 *
 * (B) max_min_table:
 * ----------------------------------------
 *   1      2      3 .... scan
 *  Max    Max    Max
 *  Min    Min    Min
 * ---------------------------------------
 */
void LidarTag::analyzeLidarDevice()
{
  ring_estimated_ = false;
  lidar_system_.point_count_table.resize(100);
  lidar_system_.ring_average_table.reserve(beam_num_);

  // Initialize the table
  MaxMin_t max_min{(int)1e5, -1, -1};  // min, ave, max

  for (int j = 0; j < beam_num_; ++j) {
    lidar_system_.ring_average_table.push_back(max_min);
  }

  // Calulate for each scan with a few seconds
  int i = 0;
  int num_scan = 0;
  clock_t begin = clock();
  int accumulated_scan = 1;
  std::vector<std::vector<LidarPoints_t>> ordered_buff(beam_num_);

  while (rclcpp::ok()) {
    if (num_accumulation_ == 1) {
      ordered_buff = LidarTag::getOrderBuff();

      if (ordered_buff.empty()) {
        continue;
      }
    } else {
      std::vector<std::vector<LidarPoints_t>> ordered_buff_cur = LidarTag::getOrderBuff();

      if (ordered_buff_cur.empty()) {
        continue;
      }

      if (accumulated_scan < num_accumulation_) {
        for (int ring = 0; ring < beam_num_; ++ring) {
          ordered_buff[ring].insert(
            ordered_buff[ring].end(), ordered_buff_cur[ring].begin(), ordered_buff_cur[ring].end());
        }

        accumulated_scan++;
        continue;
      }

      accumulated_scan = 1;
    }

    LidarTag::maxMinPtsInAScan(
      lidar_system_.point_count_table[num_scan], lidar_system_.max_min_table,
      lidar_system_.ring_average_table, ordered_buff);

    num_scan++;
    clock_t end = clock();

    for (int ring = 0; ring < beam_num_; ++ring) {
      std::vector<LidarPoints_t>().swap(ordered_buff[ring]);
    }

    if ((((double)(end - begin) / CLOCKS_PER_SEC) > 3) || num_scan >= 100) {
      break;
    }
  }

  for (auto i = lidar_system_.ring_average_table.begin();
    i != lidar_system_.ring_average_table.end(); ++i)
  {
    i->average /= num_scan;
  }

  LidarTag::pointsPerSquareMeterAtOneMeter();
}

/*
 * A function to calculate how many points are supposed to be
 * on a cluster at 1 meter away
 */
void LidarTag::pointsPerSquareMeterAtOneMeter()
{
  double system_average;

  for (auto i = lidar_system_.ring_average_table.begin();
       i != lidar_system_.ring_average_table.end(); ++i) {
    system_average += i->average;
  }

  system_average /= lidar_system_.ring_average_table.size();
  lidar_system_.beam_per_vertical_radian = beam_num_ / utils::deg2Rad(vertical_fov_);
  lidar_system_.point_per_horizontal_radian = system_average / utils::deg2Rad(horizontal_fov_);
}

/*
 * A function to find maximum points and minimum points in a single scan,
 * i.e. to find extrema within 32 rings
 */
void LidarTag::maxMinPtsInAScan(
  std::vector<int> & point_count_table, std::vector<MaxMin_t> & max_min_table,
  std::vector<MaxMin_t> & ring_average_table,
  const std::vector<std::vector<LidarPoints_t>> & ordered_buff)
{
  // every scan should have different largest/ smallest numbers
  int largest = -1;
  int smallest = 100000;
  MaxMin_t max_min;

  int i = 0;

  for (auto ring = ordered_buff.begin(); ring != ordered_buff.end(); ++ring) {
    int ring_size = (*ring).size();
    point_count_table.push_back(ring_size);

    // first time (we had initialized with -1)
    if (ring_average_table[i].average < 0) {
      ring_average_table[i].average = ring_size;
    } else {
      ring_average_table[i].average = (ring_average_table[i].average + ring_size);
    }

    if (ring_average_table[i].max < ring_size) {
      ring_average_table[i].max = ring_size;
    }

    if (ring_average_table[i].min > ring_size) {
      ring_average_table[i].min = ring_size;
    }

    if (ring_size > largest) {
      largest = ring_size;
      max_min.max = ring_size;
    }

    if (ring_size < smallest) {
      smallest = ring_size;
      max_min.min = ring_size;
    }
    i++;
  }
  max_min_table.push_back(max_min);
}

/*
 * A function to transfer pcl msgs to ros msgs and then publish
 * which_publisher should be a string of "organized" or "original"
 * regardless lowercase and uppercase
 */
void LidarTag::publishPointcloud(
  const pcl::PointCloud<PointXYZRI>::Ptr & source_pc, const std::string & frame,
  std::string which_publisher)
{
  utils::tranferToLowercase(which_publisher);  // check letter cases
  sensor_msgs::msg::PointCloud2 pcs_waited_to_pub;
  pcl::toROSMsg(*source_pc, pcs_waited_to_pub);
  pcs_waited_to_pub.header.frame_id = frame;

  try {
    if (which_publisher == "wholeedge")
      edge_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "original")
      original_pc_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "cluster") {
      pcs_waited_to_pub.header = point_cloud_header_;
      pcs_waited_to_pub.header.stamp = clock_->now();
      cluster_pub_->publish(pcs_waited_to_pub);
    }
    else if (which_publisher == "payload")
      payload_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "payload3d")
      payload3d_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "target")
      tag_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "edgegroup1")
      edge1_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "edgegroup2")
      edge2_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "edgegroup3")
      edge3_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "edgegroup4")
      edge4_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "boundarypc")
      boundary_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "initialcornerspc")
      initial_corners_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "initialtarget")
      initag_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "clusteredgepc") {
      pcs_waited_to_pub.header.stamp = current_scan_time_;
      pcs_waited_to_pub.header.frame_id = lidar_frame_;
      clustered_points_pub_->publish(pcs_waited_to_pub);
    }
    else if (which_publisher == "transpts")
      transformed_points_pub_->publish(pcs_waited_to_pub);
    else if (which_publisher == "transptstag")
      transformed_pointstag_pub__->publish(pcs_waited_to_pub);
    else {
      throw "No such Publisher exists";
    }
  } catch (const char * msg) {
    cout << "\033[1;31m========================= \033[0m\n";
    cerr << msg << endl;
    cout << "\033[1;31m========================= \033[0m\n";
    exit(-1);
  }
}

/* [basic ros]
 * A function to push the received pointcloud into a queue in the class
 */
void LidarTag::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc)
{
  lidar_frame_ = pc->header.frame_id;

  // flag to make sure it receives a pointcloud
  // at the very begining of the program

  if (!point_cloud_received_) {
    RCLCPP_INFO(get_logger(), "Got the first pointcloud. Starting LidarTag");
    extraction_thread_ = std::make_unique<boost::thread>(&LidarTag::mainLoop, this);
  }

  // to debug the lidartag we can lach the same cloud over and over
  if (params_.debug_single_pointcloud && debug_pc_ == nullptr) {
    point_cloud1_queue_lock_.lock();
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> empty;
    std::swap(point_cloud1_queue_, empty );
    debug_pc_ = pc;
    point_cloud1_queue_lock_.unlock();
  }
  else if (!params_.debug_single_pointcloud && debug_pc_ != nullptr)
  {
    debug_pc_.reset();
  }

  point_cloud_received_ = 1;
  point_cloud_header_ = pc->header;

  point_cloud1_queue_lock_.lock();

  while (point_cloud1_queue_.size() >= max_queue_size_)
    point_cloud1_queue_.pop();

  if (params_.debug_single_pointcloud) {
    point_cloud1_queue_.push(debug_pc_);
  }
  else {
    point_cloud1_queue_.push(pc);
  }

  point_cloud1_queue_lock_.unlock();
}

/*
 * A function to slice the Veloydyne full points to sliced pointed
 * based on ring number
 * */
inline void LidarTag::fillInOrderedPointcloudFromUnorganizedPointcloud(
  const pcl::PointCloud<PointXYZRI>::Ptr & pcl_pointcloud,
  std::vector<std::vector<LidarPoints_t>> & ordered_buff)
{
  if (!has_ring_ && ring_estimated_) {
    std::string ring_list;

    for (auto && it : lidar_system_.angle_list) {
      ring_list += (std::to_string(it) + " ");
    }

    RCLCPP_INFO_STREAM_ONCE(get_logger(),"Estimate Ring List Size: " << lidar_system_.angle_list.size());
    RCLCPP_INFO_STREAM_ONCE(get_logger(),"Estimate Ring List: " << ring_list);

    assert(("Ring List Error", lidar_system_.angle_list.size() <= beam_num_));
  }

  LidarPoints_t lidar_point;
  int index[beam_num_] = {0};
  std::set<float>::iterator it;

  for (auto && p : *pcl_pointcloud) {

    if (!has_ring_ && ring_estimated_) {
      float angle = getAnglefromPt(p);
      it = lidar_system_.angle_list.find(angle);
      p.ring = std::distance(lidar_system_.angle_list.begin(), it);
    }

    assert(("Ring Estimation Error", p.ring < beam_num_));

    lidar_point.point = p;
    lidar_point.index = index[p.ring];
    lidar_point.valid = 1;
    ordered_buff[p.ring].push_back(lidar_point);
    index[p.ring] += 1;
  }

  addOrderedPointcloudMarkers(ordered_buff);
}

/*
 * A function to slice the Veloydyne full points to sliced pointed
 * based on ring number
 * */
inline void LidarTag::fillInOrderedPointcloudFromOrganizedPointcloud(
  const pcl::PointCloud<PointXYZRI>::Ptr & pcl_pointcloud,
  std::vector<std::vector<LidarPoints_t>> & ordered_buff)
{
  if (pcl_pointcloud->points.size() != pcl_pointcloud->width * pcl_pointcloud->height) {
    RCLCPP_ERROR(get_logger(), "The number of points does not coincide with the pointcloud structure");
  }

  if (pcl_pointcloud->height == 1) {
    RCLCPP_ERROR(get_logger(), "The pointcloud has a singleton height. It is probably not an organized pointcloud");
  }

  LidarPoints_t lidar_point;
  std::set<float>::iterator it;

  const int width = pcl_pointcloud->width;
  const int height = pcl_pointcloud->height;
  int pointcloud_index = 0;

  for (auto && p : *pcl_pointcloud) {

    lidar_point.point = p;
    lidar_point.point.ring = pointcloud_index / width;
    lidar_point.index = pointcloud_index % width;

    assert(("Ring Estimation Error", lidar_point.point.ring < beam_num_));
    
    lidar_point.valid = 1;
    ordered_buff[lidar_point.point.ring].push_back(lidar_point);
    pointcloud_index++;
  }

  addOrderedPointcloudMarkers(ordered_buff);
}

/*
 * A function to compute angle between the line from origin to this point
 * and z=0 plane in lidar
 * */
float LidarTag::getAnglefromPt(PointXYZRI & point)
{
  float distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
  float theta = utils::rad2Deg(std::atan2(point.z, distance));

  return theta;
}

void LidarTag::getAngleVector(
  const pcl::PointCloud<PointXYZRI>::Ptr & pcl_pointcloud, std::vector<float> & angles)
{
  for (auto && p : *pcl_pointcloud) {
    if (p.x == 0 && p.y == 0 && p.z == 0) {
      continue;
    }

    float angle = LidarTag::getAnglefromPt(p);
    angles.push_back(angle);
    lidar_system_.angle_list.insert(angle);
  }
}

/*
 * Main function
 */
pcl::PointCloud<PointXYZRI>::Ptr LidarTag::lidarTagDetection(
  const std::vector<std::vector<LidarPoints_t>> & ordered_buff,
  std::vector<ClusterFamily_t> & cluster_buff)
{
  if (debug_info_ || debug_time_) {
    RCLCPP_INFO_STREAM(get_logger(), "--------------- Begining ---------------");
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "--------------- Begining ---------------");
  }

  pcl::PointCloud<PointXYZRI>::Ptr out(new pcl::PointCloud<PointXYZRI>);
  out->reserve(point_cloud_size_);

  // Buff to store all detected edges
  std::vector<std::vector<LidarPoints_t>> edge_buff(beam_num_);

  // calculate gradient for depth and intensity as well as
  // group them into diff groups
  result_statistics_ = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0};

  timing_.start_computation_time = std::chrono::steady_clock::now();
  LidarTag::gradientAndGroupEdges(ordered_buff, edge_buff, cluster_buff);

  if (debug_time_) {
    timing_.edging_and_clustering_time = utils::spendElapsedTimeMilli(
      std::chrono::steady_clock::now(), timing_.start_computation_time);
    timing_.timing = std::chrono::steady_clock::now();
  }

  // transform from a vector of vector (edge_buff) into a pcl vector (out)
  boost::thread BuffToPclVectorThread(
    &LidarTag::buffToPclVector, this, boost::ref(edge_buff), out);

  if (debug_time_) {
    timing_.to_pcl_vector_time =
      utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
  }

  LidarTag::fillInCluster(ordered_buff, cluster_buff);
  BuffToPclVectorThread.join();
  result_statistics_.point_cloud_size = point_cloud_size_;
  result_statistics_.edge_cloud_size = out->size();

  timing_.total_duration =
    utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.start_computation_time);

  if (print_ros_info_ || debug_info_) {
    RCLCPP_INFO_STREAM(get_logger(), "-- Computation: " << 1e3 / timing_.total_duration << " [Hz]");
  }

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(),
      "--------------- summary ---------------");
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Original cloud size: " << result_statistics_.point_cloud_size);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Edge cloud size: " << result_statistics_.edge_cloud_size);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Original cluster: " << result_statistics_.original_cluster_size);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by minimum returns: " << result_statistics_.cluster_removal.minimum_return);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by maximum returns: " << result_statistics_.cluster_removal.maximum_return);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by plane fitting failure: " << result_statistics_.cluster_removal.plane_fitting);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by plane fitting ourliers: "
      << result_statistics_.cluster_removal.plane_outliers);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by payload boundary point: "
      << result_statistics_.cluster_removal.boundary_point_check);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by payload ring points: "
      << result_statistics_.cluster_removal.minimum_ring_points);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by edge points: " << result_statistics_.cluster_removal.no_edge_check);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by line fitting: " << result_statistics_.cluster_removal.line_fitting);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by pose optimization: " << result_statistics_.cluster_removal.pose_optimization);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Removed by decoding: " << result_statistics_.cluster_removal.decoding_failure);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "-- Remaining Cluster: " << result_statistics_.remaining_cluster_size);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "---------------------------------------");
  }

  if (debug_time_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "--------------- Timing ---------------");
    RCLCPP_DEBUG_STREAM(get_logger(),
      "computation_time: " << 1e3 / timing_.total_duration << " [Hz]");
    RCLCPP_DEBUG_STREAM(get_logger(),
      "edging_and_clustering_time: " << timing_.edging_and_clustering_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "to_pcl_vector_time: " << timing_.to_pcl_vector_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "fill_in_time: " << timing_.fill_in_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "point_check_time: " << timing_.point_check_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "line_fitting_time: " << timing_.line_fitting_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "organize_points_time: " << timing_.organize_points_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "pca_time: " << timing_.pca_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "split_edge_time: " << timing_.split_edge_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "pose_optimization_time: " << timing_.pose_optimization_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "store_template_time: " << timing_.store_template_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "payload_decoding_time: " << timing_.payload_decoding_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "computation_time: " << timing_.total_duration);
    RCLCPP_DEBUG_STREAM(get_logger(), "---------------------------------------");
  }

  return out;
}

/*
 * A function to
 * (1) calculate the depth gradient and
 *     the intensity gradient at a point of a pointcloud
 * (2) group detected 'edge' into different group
 */
void LidarTag::gradientAndGroupEdges(
  const std::vector<std::vector<LidarPoints_t>> & ordered_buff,
  std::vector<std::vector<LidarPoints_t>> & edge_buff, std::vector<ClusterFamily_t> & cluster_buff)
{
  int n = params_.num_points_for_plane_feature;

  // TODO: if suddently partial excluded, it will cause errors
  for (int i = beam_num_ - 1; i >= 0; --i) {
    int size = ordered_buff[i].size();
    for (int j = 0; j < size; j++) {
      // edge_flag:
      // 0 means no edge point,
      // 1 means the left side point is the edge point,
      // 2 means the right side point is the edge point,
      // 3 means two side points are edge points
      // 4 forced corner
      int edge_flag = 0;

      if (params_.use_borders_as_corners && (j == 0 || j == size - 1)) {
        edge_flag = 4;
      } else if (j + n - 2 >= 0 && j < size - n) {
        edge_flag = LidarTag::getEdgePoints(ordered_buff, i, j, n);
      }

      // TODO: delete this when the lidartag development finishes. Is is used to pinpoint the reasons why some pointclouds are not detected
      /*if (j == params_.debug_scan_id &&
        i == params_.debug_ring_id)
      {
        int x = 0;
      } */

      if (edge_flag == 0) {
        continue;
      }

      // KL: we do not want duplicates
      if (edge_flag == 1 || edge_flag == 3) {
        clusterClassifier(ordered_buff[i][j], cluster_buff);
        const auto & point1 = ordered_buff[i][j].point;
        const auto & Point1L = ordered_buff[i][j - 1].point;
        const auto & Point1R = ordered_buff[i][j + 1].point;
        double DepthGrad1 = std::abs(
          (Point1L.getVector3fMap() - point1.getVector3fMap()).norm() -
          (point1.getVector3fMap() - Point1R.getVector3fMap()).norm());

        // push the detected point that is an edge into a buff
        LidarPoints_t lidar_points = {ordered_buff[i][j].point, ordered_buff[i][j].index, 1,
                                      DepthGrad1, 0};
        edge_buff[i].push_back(lidar_points);
      }
      else if (edge_flag == 2 || edge_flag == 3) {
        clusterClassifier(ordered_buff[i][j + n - 1], cluster_buff);
        const auto & point2 = ordered_buff[i][j + n - 1].point;
        const auto & Point2L = ordered_buff[i][j + n - 2].point;
        const auto & Point2R = ordered_buff[i][j + n].point;
        double DepthGrad2 = std::abs(
          (Point2L.getVector3fMap() - point2.getVector3fMap()).norm() -
          (point2.getVector3fMap() - Point2R.getVector3fMap()).norm());
        // push the detected point that is an edge into a buff
        LidarPoints_t lidar_points = {ordered_buff[i][j + n - 1].point,
                                      ordered_buff[i][j + n - 1].index, 1, DepthGrad2, 0};

        // When n=1 and flag=3 the edges are duplicated
        if (edge_flag != 3 || n != 1) {
          edge_buff[i].push_back(lidar_points);
        }
      }
      else if (edge_flag == 4) {
        clusterClassifier(ordered_buff[i][j], cluster_buff);
        double DepthGrad2 = 10.0;
        LidarPoints_t lidar_points = {ordered_buff[i][j].point,
                                      ordered_buff[i][j].index, 1, DepthGrad2, 0};
        edge_buff[i].push_back(lidar_points);

      }
    }
  }

  // this is a one pass algorithm prone to miss some edges -> revisit
  if (params_.optional_fix_cluster) {
    fixClusters(edge_buff, cluster_buff);
  }
}

/* <consecutive n points from ring i index j>
 * A function to
 * (1) kick out points are not near to each other
 * (2) find edge points from two side points of these points
 */
int LidarTag::getEdgePoints(
  const std::vector<std::vector<LidarPoints_t>> & ordered_buff, int i, int j, int n)
{
  const auto & point = ordered_buff[i][j].point;
  if (
    std::abs(point.x) > distance_bound_ || std::abs(point.y) > distance_bound_ ||
    std::abs(point.z) > distance_bound_)
  {
    return 0;
  }

  double point_resolution = 2 * M_PI / lidar_system_.ring_average_table[i].average;
  double near_bound = std::max(
    0.1, params_.nearby_factor * point.getVector3fMap().norm() * std::sin(point_resolution));

  for (int k = 0; k < n - 1; k++) {
    const auto & point1 = ordered_buff[i][j + k].point;
    const auto & point2 = ordered_buff[i][j + k + 1].point;
    double distance = (point1.getVector3fMap() - point2.getVector3fMap()).norm();

    if (distance > near_bound) {
      return 0;
    }
  }

  const auto & point2 = ordered_buff[i][j + n - 1].point;
  const auto & PointL = ordered_buff[i][j - 1].point;
  const auto & PointR = ordered_buff[i][j + 1].point;
  const auto & Point2L = ordered_buff[i][j + n - 2].point;
  const auto & Point2R = ordered_buff[i][j + n].point;

  double DepthGrad1 = std::abs(
    (PointL.getVector3fMap() - point.getVector3fMap()).norm() -
    (point.getVector3fMap() - PointR.getVector3fMap()).norm());

  double DepthGrad2 = std::abs(
      (Point2L.getVector3fMap() - point2.getVector3fMap()).norm() -
      (point2.getVector3fMap() - Point2R.getVector3fMap()).norm());

  if (i == params_.debug_ring_id) {

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = lidar_frame_;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(5.0);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    int distance = std::max(DepthGrad1, DepthGrad2) * 100;
    //std::stringstream stream;
    //stream << std::fixed << std::setprecision(2) << distance;

    marker.header.stamp = clock_->now();
    marker.id = j;
    marker.text = distance > 100 ? "x" : to_string(distance);
    marker.ns = "distance_" + to_string(i);
    marker.pose.position.x = ordered_buff[i][j].point.x;
    marker.pose.position.y = ordered_buff[i][j].point.y;
    marker.pose.position.z = ordered_buff[i][j].point.z;
    ordered_pointcloud_markers_.markers.push_back(marker);
  }

  if (DepthGrad1 > params_.depth_bound) {
    if (DepthGrad2 > params_.depth_bound) {
      return 3;
    } else {
      return 1;
    }
  } else {
    if (DepthGrad2 > params_.depth_bound) {
      return 2;
    } else {
      return 0;
    }
  }
}

/* <All Clusters>
 * A function to
 * (1) remove invalid cluster based on the index is too far or not
 * (2) fill in the points between index of edges
 * (3) after filling, if the points are too less (based on the analyzed system
 *     and given distant of the cluster), then remove this cluster
 * (4) Adaptive thresholding (Maximize and minimize intensity) by comparing
 *     with the average value
 */
void LidarTag::fillInCluster(
  const std::vector<std::vector<LidarPoints_t>> & ordered_buff,
  std::vector<ClusterFamily_t> & cluster_buff)
{
  std::ofstream fplanefit;

  if (log_data_) {
    std::string path(outputs_path_);
    fplanefit.open(path + "/planeFit.csv");

    if (!fplanefit.is_open()) {
      cout << "Could not open planeFit.csv" << endl;
      exit(0);
    }
  }

  result_statistics_.original_cluster_size = cluster_buff.size();
  result_statistics_.remaining_cluster_size = cluster_buff.size();

  int ring_average = 0;
  for (int j = 0; j < beam_num_; ++j) {
    ring_average = std::max(ring_average, lidar_system_.ring_average_table[j].average);
  }

  // tbb::parallel_for(int(0), (int)cluster_buff.size(), [&](int i) {
  for (int i = 0; i < cluster_buff.size(); ++i) {
    ClusterFamily_t & cluster = cluster_buff[i];
    // In a cluster
    // tbb::parallel_for(int(0), (int)cluster_buff.size(), [&](int i) {
    if (debug_time_) {
      timing_.timing = std::chrono::steady_clock::now();
    }

    for (int j = 0; j < beam_num_; ++j) {
      int max_index = cluster.max_min_index_of_each_ring[j].max;
      int min_index = cluster.max_min_index_of_each_ring[j].min;

      // TODO: delete this when the lidartag development finishes. Is is used to pinpoint the reasons why some pointclouds are not detected
      /*if (cluster.cluster_id == params_.debug_cluster_id &&
        j == params_.debug_ring_id)
      {
        int x = 0;
      } */

      // no points on this ring
      if ((std::abs(min_index - 1e5) < 1e-5) || std::abs(max_index + 1) < 1e-5) {
        continue;
      }

      // A cluster can't cover 180 FoV of a LiDAR!
      // If max and min indices are larger than this,
      // that means the special case happened
      // The special case is when first point of a ring is in this cluster
      // so the indices are not consecutive
      double fill_in_gap = ring_average / 2;
      // cout << "(i, j) = (" << i << ", " << j << ")" << endl;
      // cout << "fill_in_gap: " << fill_in_gap << endl;
      // cout << "max index: " << max_index << endl;
      // cout << "min index: " << min_index << endl;

      // if (max_index-min_index < _filling_gap_max_index) {
      if (max_index - min_index < fill_in_gap) {
        // remove minimum index itself
        // (it has been in the cloud already)
        for (int k = min_index + 1; k < max_index; ++k) {
          // cout << "k1: " << k << endl;
          if (isWithinClusterHorizon(ordered_buff[j][k], cluster, 0)) {
            cluster.data.push_back(ordered_buff[j][k]);
          }
        }
        cluster.special_case = 0;
      } else {
        for (int k = 0; k < min_index; ++k) {
          // cout << "k2: " << k << endl;

          if (isWithinClusterHorizon(ordered_buff[j][k], cluster, 0)) {
            cluster.data.push_back(ordered_buff[j][k]);
          }
        }
        for (int k = max_index; k < ordered_buff[j].size(); ++k) {
          // cout << "k3: " << k << endl;

          if (isWithinClusterHorizon(ordered_buff[j][k], cluster, 0)) {
            cluster.data.push_back(ordered_buff[j][k]);
          }
        }

        cluster.special_case = 1;
      }
    }

    if (debug_time_) {
      timing_.fill_in_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
      timing_.timing = std::chrono::steady_clock::now();
    }

    // Mark cluster as invalid if too few points in cluster
    auto min_returns =
      min_returns_per_grid_ * std::pow((std::sqrt(tag_family_) + 2 * black_border_), 2);

    if ((cluster.data.size() + cluster.edge_points.size()) < min_returns) {
      result_statistics_.cluster_removal.minimum_return++;
      result_statistics_.remaining_cluster_size--;

      if (mark_cluster_validity_) {
        cluster.valid = false;
        cluster.detail_valid = LidartagErrorCode::ClusterMinPointsCriteria;
      }
      // tbb::task::self().cancel_group_execution();
      continue;
    }

    // Mark cluster as invalid if too many points in cluster
    if (params_.cluster_check_max_points)
    {
      cluster.expected_points = maxPointsCheck(cluster);
    }    

    if (cluster.valid == false) {
      // tbb::task::self().cancel_group_execution();
      continue;
    }

    if (debug_time_) {
      timing_.point_check_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
      timing_.timing = std::chrono::steady_clock::now();
    }

    // Mark cluster as invalid if unable to perform plane fitting
    if (!plane_fitting_) {
      continue;
    } else {
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      if (!rejectWithPlanarCheck(cluster, inliers, coefficients, fplanefit)) {
        if (mark_cluster_validity_) {
          cluster.valid = false;
          cluster.detail_valid = LidartagErrorCode::PlanarCheckCriteria;
        }
        // tbb::task::self().cancel_group_execution();
        continue;
      }
      inlier_size_ = inliers->indices.size();
      // Mark cluster as invalid if too many outliers in plane fitting
      auto percentage_inliers =
        inliers->indices.size() /
        double(cluster.data.size() + cluster.edge_points.size());

      cluster.percentages_inliers = percentage_inliers;

      if (debug_info_) {
        RCLCPP_DEBUG_STREAM(get_logger(), "==== _planeOutliers ====");
        float distance = std::sqrt(
          pow(cluster.average.x, 2) + pow(cluster.average.y, 2) +
          pow(cluster.average.z, 2));
        RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
        RCLCPP_DEBUG_STREAM(get_logger(),
          "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
      }

      if (percentage_inliers < (1.0 - params_.max_outlier_ratio)) {
        if (debug_info_) {
          RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
        }
        // tbb::task::self().cancel_group_execution();
        result_statistics_.cluster_removal.plane_outliers++;
        result_statistics_.remaining_cluster_size--;

        if (mark_cluster_validity_) {
          cluster.valid = false;
          cluster.detail_valid = LidartagErrorCode::PlanarOutliersCriteria;
          continue;
        }
      }

      if (debug_info_) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << true);
      }

      // Remove all outliers from cluster
      auto outliers_indices = utils::complementOfSet(
        inliers->indices, (cluster.data.size() + cluster.edge_points.size()));

      int edge_inlier = cluster.edge_points.size();

      for (int k = 0; k < outliers_indices.size(); ++k) {
        int out_index = outliers_indices[k];

        if (out_index < cluster.edge_points.size()) {
          edge_inlier -= 1;
          cluster.edge_points[out_index].valid = 0;
        } else {
          cluster.data[out_index - cluster.edge_points.size()].valid = 0;
        }
      }

      cluster.edge_inliers = edge_inlier;

      if (debug_time_) {
        timing_.plane_fitting_removal_time +=
          utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
        timing_.timing = std::chrono::steady_clock::now();
      }
    }

    if (!LidarTag::adaptiveThresholding(cluster)) {
      // removal has been done inside the function
      if (mark_cluster_validity_) {
        cluster.valid = false;
      }
      // cluster_buff.erase(cluster_buff.begin()+i);
      // i--;
    } else {
      if (print_ros_info_ || debug_info_) {
        RCLCPP_INFO_STREAM(get_logger(), "--ID: " << cluster.cluster_id);
        RCLCPP_INFO_STREAM(get_logger(), "---rotation: " <<  cluster.rkhs_decoding.rotation_angle);
      }
    }
    // }
  }
  //}, tbb::auto_partitioner());

  if (log_data_) {
    fplanefit.close();
  }
}

/* <A cluster>
 * A function to
 * (1) do adaptive thresholding (Maximize and minimize intensity) by comparing
 *     with the average value and
 * (2) sort points with ring number and re-index with current cluster into
 *     tag_edges vector so as to do regression boundary lines
 * (3) It will *remove* if linefitting fails
 */
bool LidarTag::adaptiveThresholding(ClusterFamily_t & cluster)
{
  if (debug_time_) {
    timing_.timing = std::chrono::steady_clock::now();
  }

  organizeDataPoints(cluster);

  if (debug_time_) {
    timing_.organize_points_time +=
      utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
    timing_.timing = std::chrono::steady_clock::now();
  }

  if (params_.use_intensity_channel && !LidarTag::detectPayloadBoundries(cluster)) {
    // removal has been done inside the function
    if (debug_time_) {
      timing_.line_fitting_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
    }
    return false;
  } else {
    if (debug_time_) {
      timing_.timing = std::chrono::steady_clock::now();
    }

    estimatePrincipleAxis(cluster);

    if (debug_time_) {
      timing_.pca_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
      timing_.timing = std::chrono::steady_clock::now();
    }
    // Fit line of the cluster
    if (!transformSplitEdges(cluster)) {
      result_statistics_.cluster_removal.line_fitting++;
      result_statistics_.remaining_cluster_size--;
      return false;
    }

    if (debug_time_) {
      timing_.split_edge_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
      timing_.timing = std::chrono::steady_clock::now();
    }

    if (!pose_optimization_) {
      cluster.pose_tag_to_lidar.homogeneous = Eigen::Matrix4f::Identity(4, 4);
      storeTemplatePts(cluster);

      return true;
    } else {
      int status = optimizePose(cluster);
      cluster.pose_estimation_status = status;

      if (status < 0) {
        cluster.detail_valid = LidartagErrorCode::OptimizationErrorCriteria;

        return false;
      } else {
        if (debug_time_) {
          timing_.pose_optimization_time +=
            utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
          timing_.timing = std::chrono::steady_clock::now();
        }

        // cout << "about to store template points" << endl;
        storeTemplatePts(cluster);
        if (debug_time_) {
          timing_.store_template_time +=
            utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
          timing_.timing = std::chrono::steady_clock::now();
        }

        if (!id_decoding_) {
          assignClusterPose(cluster.pose_tag_to_lidar, cluster.pose, 0);
          return true;
        } else {
          if (!LidarTag::decodePayload(cluster)) {
            result_statistics_.cluster_removal.decoding_failure++;
            result_statistics_.remaining_cluster_size--;
            cluster.detail_valid = LidartagErrorCode::DecodingErrorCriteria;
            return false;
          } else {
            if (debug_time_) {
              timing_.payload_decoding_time +=
                utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), timing_.timing);
              timing_.timing = std::chrono::steady_clock::now();
            }
            assignClusterPose(
              cluster.pose_tag_to_lidar, cluster.pose, cluster.rkhs_decoding.rotation_angle);
            return true;
          }
        }
      }
    }
  }
}

/* <A cluster>
 * A function to assign the pose to the cluster using the results of
 * L1-optimization and RKHS decoding results
 */
void LidarTag::assignClusterPose(
  const Homogeneous_t & h_tl, Homogeneous_t & h_lt, const int & ang_num)
{
  float rotation_angle = -ang_num * 90;
  Eigen::Vector3f trans_v = Eigen::Vector3f::Zero(3);
  Eigen::Vector3f rot_v;
  rot_v << rotation_angle, 0, 0;
  Eigen::Matrix4f H = utils::computeTransformation(rot_v, trans_v);

  h_lt.homogeneous = (H * h_tl.homogeneous).inverse();
  h_lt.rotation = h_lt.homogeneous.topLeftCorner(3, 3);
  h_lt.translation = h_lt.homogeneous.topRightCorner(3, 1);
  Eigen::Vector3f euler = h_lt.rotation.eulerAngles(0, 1, 2);  // x,y,z convention
  h_lt.roll = euler[0];
  h_lt.pitch = euler[1];
  h_lt.yaw = euler[2];
}

/* <A cluster>
 * A function to store transformed points
 */
void LidarTag::storeTemplatePts(ClusterFamily_t & cluster)
{
  cluster.rkhs_decoding.initial_template_points =
    cluster.initial_pose.homogeneous * cluster.merged_data_h;
  cluster.rkhs_decoding.initial_template_points.bottomRows(1) = cluster.merged_data.bottomRows(1);

  cluster.rkhs_decoding.template_points =
    cluster.pose_tag_to_lidar.homogeneous * cluster.merged_data_h;
  cluster.rkhs_decoding.template_points.bottomRows(1) = cluster.merged_data.bottomRows(1);
}

/* A function to publish pose of tag to the robot
 */
void LidarTag::tagToRobot(
  const int & cluster_id, const Eigen::Vector3f & normal_vec, Homogeneous_t & pose,
  tf2::Transform & transform, const PointXYZRI & ave,
  lidartag_msgs::msg::LidarTagDetectionArray & lidartag_pose_array)
{
  Eigen::Vector3f x(1, 0, 0);
  Eigen::Vector3f y(0, 1, 0);
  Eigen::Vector3f z(0, 0, 1);
  pose.rotation = utils::qToR(normal_vec).cast<float>();
  pose.translation << ave.x, ave.y, ave.z;

  pose.yaw = utils::rad2Deg(acos(normal_vec.dot(y)));
  pose.pitch = -utils::rad2Deg(acos(normal_vec.dot(x)));
  pose.roll = utils::rad2Deg(acos(normal_vec.dot(z)));

  pose.homogeneous.topLeftCorner(3, 3) = pose.rotation;
  pose.homogeneous.topRightCorner(3, 1) = pose.translation;
  pose.homogeneous.row(3) << 0, 0, 0, 1;

  transform.setOrigin(tf2::Vector3(ave.x, ave.y, ave.z));

  // rotate to fit iamge frame
  Eigen::Vector3f qr(0, std::sqrt(2) / 2, 0);
  float qr_w = std::sqrt(2) / 2;

  Eigen::Vector3f qi_camera_frame = normal_vec + 2 * qr_w * (qr.cross(normal_vec)) +
                                    2 * qr.cross(qr.cross(normal_vec));  // 0 is q.w of normalvec
  float qw_camera_frame = 0;                                             // 0 is q.w of normalvec

  Eigen::Vector3f q_i = qi_camera_frame;
  double q_w = qw_camera_frame;
  double norm =
    std::sqrt(std::pow(q_i(0), 2) + std::pow(q_i(1), 2) + std::pow(q_i(2), 2) + std::pow(q_w, 2));

  q_i = (q_i / norm).eval();
  q_w = q_w / norm;
  tf2::Quaternion q(q_i(0), q_i(1), q_i(2), q_w);
  transform.setRotation(q);

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = point_cloud_header_.stamp;
  transform_msg.header.frame_id = lidar_frame_;
  transform_msg.child_frame_id = to_string(cluster_id)+"_rotated";
  transform_msg.transform = tf2::toMsg(transform);

  broadcaster_.sendTransform(transform_msg);

  tf2::Quaternion q2(normal_vec(0), normal_vec(1), normal_vec(2), 0);
  transform.setRotation(q2);
  transform_msg.header.stamp = point_cloud_header_.stamp;
  transform_msg.header.frame_id = lidar_frame_;
  transform_msg.child_frame_id = "LidarTag-ID" + to_string(cluster_id);
  transform_msg.transform = tf2::toMsg(transform);

  broadcaster_.sendTransform(transform_msg);

  // publish lidar tag pose
  lidartag_msgs::msg::LidarTagDetection lidartag_msg; //single message
  lidartag_msg.id = cluster_id;
  lidartag_msg.size = payload_size_;
  geometry_msgs::msg::Quaternion geo_q;
  geo_q.x = q_i(0);
  geo_q.y = q_i(1);
  geo_q.z = q_i(2);
  geo_q.w = q_w;

  lidartag_msg.pose.position.x = ave.x;
  lidartag_msg.pose.position.y = ave.y;
  lidartag_msg.pose.position.z = ave.z;
  lidartag_msg.pose.orientation = geo_q;
  lidartag_msg.header = point_cloud_header_;
  lidartag_msg.header.frame_id = std::string("lidartag_") + to_string(cluster_id);

  lidartag_pose_array.header = point_cloud_header_;
  lidartag_pose_array.frame_index = 0;
  lidartag_pose_array.detections.push_back(lidartag_msg);
}

/* <A cluster>
 * 1. Convert datatype from PCL to Eigen type for pose optimization.
 * 2. Merge edge points and data points into merged_data and merged_data_h.
 * 3. Compute mean of the cluster, including the mean of intensity
 * 4. Organize "data" points only for boundry detection. When detecting
 *    boundaries, we don't care about PoI.
 */
void LidarTag::organizeDataPoints(ClusterFamily_t & cluster)
{
  cluster.ordered_points_ptr.resize(beam_num_);

  cluster.merged_data.resize(4, cluster.inliers);
  cluster.merged_data_h.resize(4, cluster.inliers);
  int eigenpc_index = 0;
  Eigen::Vector4f cur_vec;
  float x_mean = 0;
  float y_mean = 0;
  float z_mean = 0;
  float i_mean = 0;

  for (int i = 0; i < cluster.edge_points.size(); ++i) {
    if (cluster.edge_points[i].valid != 1) {
      continue;
    }

    pointXYZRIToEigenVector(cluster.edge_points[i].point, cur_vec);
    cluster.merged_data.col(eigenpc_index) = cur_vec;
    cluster.merged_data_h.col(eigenpc_index) << cur_vec(0), cur_vec(1), cur_vec(2), 1;

    x_mean += cur_vec[0];
    y_mean += cur_vec[1];
    z_mean += cur_vec[2];
    i_mean += cur_vec[3];
    eigenpc_index += 1;
  }

  for (int i = 0; i < cluster.data.size(); ++i) {
    if (cluster.data[i].valid != 1) {
      continue;
    }

    pointXYZRIToEigenVector(cluster.data[i].point, cur_vec);
    cluster.merged_data.col(eigenpc_index) = cur_vec;
    cluster.merged_data_h.col(eigenpc_index) << cur_vec(0), cur_vec(1), cur_vec(2), 1;

    x_mean += cur_vec[0];
    y_mean += cur_vec[1];
    z_mean += cur_vec[2];
    i_mean += cur_vec[3];
    eigenpc_index += 1;

    LidarPoints_t * ClusterPointPtr = &cluster.data[i];
    cluster.ordered_points_ptr[ClusterPointPtr->point.ring].push_back(ClusterPointPtr);
  }

  cluster.average.x = x_mean / cluster.inliers;
  cluster.average.y = y_mean / cluster.inliers;
  cluster.average.z = z_mean / cluster.inliers;
  cluster.average.intensity = i_mean / cluster.inliers;
  cluster.rkhs_decoding.num_points = cluster.inliers;
  cluster.rkhs_decoding.ave_intensity = cluster.average.intensity;

  for (int ring = 0; ring < beam_num_; ++ring) {
    sort(cluster.ordered_points_ptr[ring].begin(),
      cluster.ordered_points_ptr[ring].end(), utils::compareIndex);
  }
}

/* <A cluster>
 * A function to extract the payload points from a valid cluster.
 * Let's say we have 10 points on the left boundary (line) of the tag and 9
 * points on the right boundary (line) of the tag. It is seperated into two
 * parts.
 * TODO: should use medium instead of max points
 *  (i) Find the max points that we have on a ring in this cluster by
 *      exmaming the average points on the first 1/2 rings int((10+9)/4)
 * (ii) For another half of the rings, we just find the start index and add the
 *      average number of points to the payload points
 */
void LidarTag::extractPayloadWOThreshold(ClusterFamily_t & cluster)
{
  int last_round_length = 0;  // Save to recover a missing ring
  PointXYZRI average{0, 0, 0, 0};

  for (int ring = 0; ring < beam_num_; ++ring) {
    if (
      cluster.payload_right_boundary_ptr[ring] == 0 && cluster.payload_left_boundary_ptr[ring] == 0)
    {
      continue;
    } else if (
      cluster.payload_right_boundary_ptr[ring] != 0 &&
      cluster.payload_left_boundary_ptr[ring] != 0)
    {
      cluster.payload_boundary_ptr.push_back(cluster.payload_right_boundary_ptr[ring]);
      cluster.payload_boundary_ptr.push_back(cluster.payload_left_boundary_ptr[ring]);
      int StartIndex = cluster.payload_left_boundary_ptr[ring]->index;
      int EndIndex = cluster.payload_right_boundary_ptr[ring]->index;
      last_round_length = EndIndex - StartIndex;

      for (int j = 0; j < cluster.ordered_points_ptr[ring].size(); ++j) {
        if (cluster.ordered_points_ptr[ring][j]->index == StartIndex) {
          // Remove index itself because itself is not the part of a
          // payload
          for (int k = j + 1; k < j + (EndIndex - StartIndex); ++k) {  // since start from 0
            if (k >= cluster.ordered_points_ptr[ring].size()) {
              break;  // make sure the index is valid
            }

            cluster.payload.push_back(cluster.ordered_points_ptr[ring][k]);
            average.x += cluster.ordered_points_ptr[ring][k]->point.x;
            average.y += cluster.ordered_points_ptr[ring][k]->point.y;
            average.z += cluster.ordered_points_ptr[ring][k]->point.z;
          }

          break;
        }
      }
    }

    cluster.average.x = average.x / cluster.payload.size();
    cluster.average.y = average.y / cluster.payload.size();
    cluster.average.z = average.z / cluster.payload.size();
  }
}

/*
 * Compare current point's gradient intensity with next points
 * gradient intensity
 * Example:
 *          o: white, x: black
 *
 *            a9876543210
 *          ooooxxxxxxxooooo
 *           ^^^^     ^^^^
 *           ||||     ||||
 * Gradient from 1-2 sould be large; the same as the gradient from 0-2
 * Gradient from 9-8 sould be large; the same as the gradient from a-8
 *
 * [Note]
 * 1. At least two points have to be on the white boundaries.
 * 2. Therefore, five points on a ring is required to be counted as a
 *    boundary point.
 *    The extreme case:
 *    Two white points on each side.
 *     ooxoo
 */
bool LidarTag::detectPayloadBoundries(ClusterFamily_t & cluster)
{
  cluster.payload_right_boundary_ptr.resize(beam_num_);
  cluster.payload_left_boundary_ptr.resize(beam_num_);
  bool boundary_flag = true;
  bool ring_point_flag = true;

  // Initialization
  cluster.tag_edges.upper_ring = beam_num_;
  cluster.tag_edges.lower_ring = 0;

  double detection_threshold = payload_intensity_threshold_ * cluster.average.intensity / 2;

  int boundary_piont_count = 0;  // Check there're sufficient points
  int num_valid_rings = 0;

  bool found_right_ring_boundary;
  bool found_left_ring_boundary;

  // a ring should at least have three points to have intensity gradients
  // from left to right and from right to right
  int minimum_ring_points = 5;

  for (int ring = 0; ring < beam_num_; ++ring) {
    found_right_ring_boundary = false;
    found_left_ring_boundary = false;

    if (cluster.ordered_points_ptr[ring].size() < minimum_ring_points) {
      continue;
    }

    // [Left]
    // Find edges from the left of a ring
    // Start from 1 becasue we take another point on the left into account
    // size -1 because we compare with next point
    for (int P = 1; P < ceil((cluster.ordered_points_ptr[ring].size() - 1) / 2); ++P) {
      // (1) By knowing it from white to black on the left calucate the
      // intensity gradient
      // (2) Since have thresholded already, we could also choose > 255
      // (3) To determin if p if the edge:
      //     1. compare with p+1 (to its right)

      if (
        (cluster.ordered_points_ptr[ring][P]->point.intensity -
           cluster.ordered_points_ptr[ring][P + 1]->point.intensity >
         detection_threshold) &&
        (cluster.ordered_points_ptr[ring][P - 1]->point.intensity -
           cluster.ordered_points_ptr[ring][P + 1]->point.intensity >
         detection_threshold))
      {
        cluster.payload_left_boundary_ptr[ring] = cluster.ordered_points_ptr[ring][P];
        boundary_piont_count++;
        found_left_ring_boundary = true;

        break;
      }
    }

    // [Right]
    // Find edges from the right a ring
    // -1-2: -1 of index and another -2 is becasue we take into account another
    // two points on the right -1  : because we compare with left point
    for (int p = cluster.ordered_points_ptr[ring].size() - 2;
      p > floor((cluster.ordered_points_ptr[ring].size() - 1) / 2); --p)
    {
      // (1) By knowing it from white to black on the left to calucate the
      // intensity gradient
      // (2) Since have thresholded already, we could also choose > 255
      // cluster.payload_right_boundary_ptr[ring] =
      // cluster.ordered_points_ptr[ring][P];
      if (
        (cluster.ordered_points_ptr[ring][p]->point.intensity -
           cluster.ordered_points_ptr[ring][p - 1]->point.intensity >
         detection_threshold) &&
        (cluster.ordered_points_ptr[ring][p + 1]->point.intensity -
           cluster.ordered_points_ptr[ring][p - 1]->point.intensity >
         detection_threshold))
      {
        cluster.payload_right_boundary_ptr[ring] = cluster.ordered_points_ptr[ring][p];
        boundary_piont_count++;
        found_right_ring_boundary = true;

        break;
      }
    }

    if (found_left_ring_boundary && found_right_ring_boundary) {
      num_valid_rings++;
    }
  }

  cluster.boundary_pts = boundary_piont_count;
  cluster.boundary_rings = num_valid_rings;

  // reject if points are too less (can't even get decoded!)
  // if (boundary_piont_count < int(sqrt(tag_family_))*2 ||
  //     num_valid_rings < int(sqrt(tag_family_))) {
  if (boundary_piont_count < int(sqrt(tag_family_)) * 2) {
    result_statistics_.cluster_removal.boundary_point_check++;
    result_statistics_.remaining_cluster_size--;
    boundary_flag = false;
    cluster.detail_valid = LidartagErrorCode::DecodingPointsCriteria;
  } else if (num_valid_rings < std::min(int(sqrt(tag_family_)), params_.minimum_ring_boundary_points)) {
    result_statistics_.cluster_removal.minimum_ring_points++;
    result_statistics_.remaining_cluster_size--;
    ring_point_flag = false;
    cluster.detail_valid = LidartagErrorCode::DecodingRingsCriteria;
  }

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== detectPayloadBoundries ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));

    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "Boundary threshold : " << detection_threshold);
    RCLCPP_DEBUG_STREAM(get_logger(), "Boundary_piont_count : " << boundary_piont_count);
    RCLCPP_DEBUG_STREAM(get_logger(), "Num_valid_rings: " << num_valid_rings);
    RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << boundary_flag && ring_point_flag);
  }

  return boundary_flag && ring_point_flag;
}

Homogeneous_t LidarTag::estimatePose(ClusterFamily_t & cluster)
{
  Homogeneous_t pose;
  // translation  min sign
  pose.translation << -cluster.average.x, -cluster.average.y, -cluster.average.z;

  // rotation//
  Eigen::Vector3f x_axis(1, 0, 0);
  Eigen::Vector3f edge_direction(0, 0, 1);
  Eigen::Vector3f base_vector1 = utils::crossProduct(x_axis, edge_direction);

  Eigen::Vector3f normal_vector = cluster.normal_vector;
  Eigen::Vector3f edge_vector = estimateEdgeVector(cluster);
  Eigen::Vector3f base_vector2 = utils::crossProduct(normal_vector, edge_vector);

  Eigen::Matrix3f V, W;
  V.col(0) = normal_vector;
  V.col(1) = edge_vector;
  V.col(2) = base_vector2;
  W.col(0) = x_axis;
  W.col(1) = edge_direction;
  W.col(2) = base_vector1;

  pose.rotation = W * V.transpose();

  // euler angles
  Eigen::Vector3f euler_angles = pose.rotation.eulerAngles(2, 1, 0);
  pose.roll = euler_angles[2];
  pose.pitch = euler_angles[1];
  pose.yaw = euler_angles[0];

  pose.homogeneous.topLeftCorner(3, 3) = pose.rotation;
  pose.homogeneous.topRightCorner(3, 1) = pose.translation;
  pose.homogeneous.row(3) << 0, 0, 0, 1;

  if (debug_info_) {
    std::cout << "=============================================================" << std::endl;
    std::cout << "estimate euler angle: \n"
              << pose.roll * 180 / M_PI << "  " << pose.pitch * 180 / M_PI << "  "
              << pose.yaw * 180 / M_PI << std::endl;
    std::cout << "estimate transformation: \n" << pose.homogeneous << std::endl;
  }

  return pose;
}

Eigen::Vector3f LidarTag::estimateEdgeVector(ClusterFamily_t & cluster)
{
  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== estimateEdgeVector ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (std::size_t i = 0; i < beam_num_; ++i) {
    if (!cluster.payload_left_boundary_ptr[i]) {
      continue;
    }

    pcl::PointXYZ p(
      cluster.payload_left_boundary_ptr[i]->point.x, cluster.payload_left_boundary_ptr[i]->point.y,
      cluster.payload_left_boundary_ptr[i]->point.z);
    cloud->points.push_back(p);
  }

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Cloud points: " << cloud->points.size());
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Inliers size " << inliers->indices.size ());
  }

  if (inliers->indices.size() == 0) {
    if (debug_info_) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not estimate a LINE model for the given dataset.");
    }
  }

  Eigen::Vector3f edge_vector(
    coefficients->values[3], coefficients->values[4], coefficients->values[5]);
  edge_vector.normalize();

  if (edge_vector(2) < 0) {
    edge_vector = -edge_vector;
  }

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Edge vector: " << edge_vector.transpose());
  }

  visualizeVector(edge_vector, cluster.average, 0);
  return edge_vector;
}

/* <A cluster>
  * A function to estimate the four corners of a tag using 4x line RANSAC
  */
bool LidarTag::estimateCornersUsingLinesRANSAC(ClusterFamily_t & cluster,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud2,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud3, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud4,
  Eigen::Vector3f & intersection1, Eigen::Vector3f & intersection2,
  Eigen::Vector3f & intersection3, Eigen::Vector3f & intersection4)
{
  // To do: function this part
  // get fitted line for points on the 1th side of the tag
  Eigen::Vector4f line1;
  Eigen::Vector4f line2;
  Eigen::Vector4f line3;
  Eigen::Vector4f line4;
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
  line_cloud1->reserve(point_cloud_size_);
  line_cloud1->clear();
  line_cloud2->reserve(point_cloud_size_);
  line_cloud2->clear();
  line_cloud3->reserve(point_cloud_size_);
  line_cloud3->clear();
  line_cloud4->reserve(point_cloud_size_);
  line_cloud4->clear();

  if (!LidarTag::getLines(cloud1, line1, line_cloud1)) {
    if (debug_info_) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = LidartagErrorCode::Line1EstimationCriteria;
    return false;
  }
  if (!LidarTag::getLines(cloud2, line2, line_cloud2)) {
    if (debug_info_) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = LidartagErrorCode::Line2EstimationCriteria;
    return false;
  }
  if (!LidarTag::getLines(cloud3, line3, line_cloud3)) {
    if (debug_info_) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = LidartagErrorCode::Line3EstimationCriteria;
    return false;
  }
  if (!LidarTag::getLines(cloud4, line4, line_cloud4)) {
    if (debug_info_) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = LidartagErrorCode::Line4EstimationCriteria;
    return false;
  }

  // get intersections of four sides
  intersection1 = getintersec(line1, line2);
  intersection2 = getintersec(line2, line3);
  intersection3 = getintersec(line3, line4);
  intersection4 = getintersec(line1, line4);

  auto eigen2pcd = [](Eigen::Vector3f p){
    pcl::PointXYZ p2;
    p2.x = p.x();
    p2.y = p.y();
    p2.z = p.z();
    return p2;
  };

  if (!estimateTargetSize(cluster, intersection1, intersection2, intersection3, intersection4)) {
    cluster.detail_valid = LidartagErrorCode::TagSizeEstimationCriteria;
    return false;
  }

  return true;
}

/* <A cluster>
  * A function to estimate the four corners of a tag using 1x rectangle fiting with
  * optional RANSAC
  */
bool LidarTag::estimateCornersUsingRectangleFitting(ClusterFamily_t & cluster,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud2,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud3, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud4,
  Eigen::Vector3f & intersection1, Eigen::Vector3f & intersection2,
  Eigen::Vector3f & intersection3, Eigen::Vector3f & intersection4)
{
  rectangle_estimator_->setInputPoints(cloud1, cloud2, cloud3, cloud4);

  if (!rectangle_estimator_->estimate()) {
    cluster.detail_valid = LidartagErrorCode::RectangleEstimationCirteria;

    cloud1->width = cloud1->size();
    cloud2->width = cloud2->size();
    cloud3->width = cloud3->size();
    cloud4->width = cloud4->size();

    cloud1->height = 1;
    cloud2->height = 1;
    cloud3->height = 1;
    cloud4->height = 1;

    return false;
  }

  std::vector<Eigen::Vector2d> corners = rectangle_estimator_->getCorners();
  assert(corners.size() == 4);

  auto corner_2d23d = [](auto & corner2d, auto & corner3d) {
    corner3d.x() = corner2d.x();
    corner3d.y() = corner2d.y();
    corner3d.z() = 0.f;
  };

  corner_2d23d(corners[0], intersection1);
  corner_2d23d(corners[1], intersection2);
  corner_2d23d(corners[2], intersection3);
  corner_2d23d(corners[3], intersection4);

  if (!estimateTargetSize(cluster, intersection1, intersection2, intersection3, intersection4)) {
    cluster.detail_valid = LidartagErrorCode::TagSizeEstimationCriteria;
    return false;
  }

  return true;
}

/* [Edge points and principal axes]
 * A function to transform the edge points to the tag frame and split into 4
 * groups for each group of points, do line fitting and get four corner points
 * from intersection of lines estimate the tagsize according to the mean of
 * distance between consecutive two corner points get the initial pose by
 * associating four corner points with the corners of template
 * TODO: make this modular
 */
bool LidarTag::transformSplitEdges(ClusterFamily_t & cluster)
{
  // TODO: delete this when the lidartag development finishes. Is is used to pinpoint the reasons why some pointclouds are not detected
  /*if (cluster.cluster_id == params_.debug_cluster_id) {
    int x = 0;
  }*/

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<PointXYZRI>::Ptr transformed_pc(new pcl::PointCloud<PointXYZRI>);
  transformed_pc->reserve(point_cloud_size_);
  transformed_pc->clear();

  pcl::PointCloud<PointXYZRI>::Ptr transformed_pc_tag(new pcl::PointCloud<PointXYZRI>);
  transformed_pc_tag->reserve(point_cloud_size_);
  transformed_pc_tag->clear();

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_edge_pc(new pcl::PointCloud<pcl::PointXYZ>);
  transformed_edge_pc->reserve(point_cloud_size_);
  transformed_edge_pc->clear();

  pcl::PointCloud<pcl::PointXYZ>::Ptr before_transformed_edge_pc(
    new pcl::PointCloud<pcl::PointXYZ>);
  before_transformed_edge_pc->resize(cluster.edge_points.size());

  // separate edge points into 4 groups
  sensor_msgs::msg::PointCloud2 before_transformed_edge_pc_msg;

  for (int i = 0; i < cluster.edge_points.size(); i++) {
    before_transformed_edge_pc->points[i].x = cluster.edge_points[i].point.x;
    before_transformed_edge_pc->points[i].y = cluster.edge_points[i].point.y;
    before_transformed_edge_pc->points[i].z = cluster.edge_points[i].point.z;
  }

  pcl::toROSMsg(*before_transformed_edge_pc, before_transformed_edge_pc_msg);
  before_transformed_edge_pc_msg.header = point_cloud_header_;
  before_transformed_edge_pc_msg.header.frame_id = lidar_frame_;
  beforetransformed_edge_pc_pub_->publish(before_transformed_edge_pc_msg);

  for (int i = 0; i < cluster.edge_points.size(); ++i) {
    if (cluster.edge_points[i].valid != 1) {
      continue;
    }

    Eigen::Vector3f edge_point(
      cluster.edge_points[i].point.x - cluster.average.x,
      cluster.edge_points[i].point.y - cluster.average.y,
      cluster.edge_points[i].point.z - cluster.average.z);
    Eigen::Matrix<float, 3, 3, Eigen::DontAlign> transform_matrix = cluster.principal_axes;

    transform_matrix = (transform_matrix.transpose()).eval();

    Eigen::Vector3f transformed_edge_point = transform_matrix * edge_point;
    pcl::PointXYZ p;
    p.x = transformed_edge_point(0);
    p.y = transformed_edge_point(1);
    p.z = 0;
    transformed_edge_pc->push_back(p);
    LidarPoints_t group_point;
    group_point.point = cluster.edge_points[i].point;

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

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== transformSplitEdges ====");
    float distance = std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(),
      "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
  }

  int num_edge_points = 2;

  if (
    cloud1->size() < num_edge_points || cloud2->size() < num_edge_points ||
    cloud3->size() < num_edge_points || cloud4->size() < num_edge_points) {

    if (debug_info_) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = LidartagErrorCode::CornerEstimationMinPointsCriteria;
    return false;
  }

  Eigen::Vector3f intersection1, intersection2, intersection3, intersection4;

  if (!params_.use_rectangle_model && !estimateCornersUsingLinesRANSAC(
    cluster, cloud1, cloud2, cloud3, cloud4,
    intersection1, intersection2, intersection3, intersection4))
  {
    return false;
  }
  else if(params_.use_rectangle_model && !estimateCornersUsingRectangleFitting(
    cluster, cloud1, cloud2, cloud3, cloud4,
    intersection1, intersection2, intersection3, intersection4))
  {
    return false;
  }


  // The data from these functions is only to visualize the new groups
  auto cloud_to_edge_group = [&](
    pcl::PointCloud<pcl::PointXYZ> & cloud,
    pcl::PointCloud<LidarPoints_t> & edges,
    int intensity)
  {
    edges.clear();

    LidarPoints_t group_point = edges[0];
    group_point.point.intensity = intensity;
    group_point.valid = 1;

    edges.clear();
    Eigen::Vector3f translation = Eigen::Vector3f(cluster.average.x, cluster.average.y, cluster.average.z);

    for (int i = 0; i < cloud.size(); ++i) {
      Eigen::Vector3f point(cloud[i].x, cloud[i].y, cloud[i].z);
      point = cluster.principal_axes*point + translation;
      group_point.point.x = point.x();
      group_point.point.y = point.y();
      group_point.point.z = point.z();
      group_point.point.intensity = 0;
      edges.push_back(group_point);
    }
  };

  if(params_.rectangle_fix_point_groups) {
    cloud_to_edge_group(*cloud1, cluster.edge_group1, 0);
    cloud_to_edge_group(*cloud2, cluster.edge_group2, 85);
    cloud_to_edge_group(*cloud3, cluster.edge_group3, 170);
    cloud_to_edge_group(*cloud4, cluster.edge_group4, 255);
  }


  sensor_msgs::msg::PointCloud2 transformed_edge_pc_msg;
  pcl::toROSMsg(*transformed_edge_pc, transformed_edge_pc_msg);
  transformed_edge_pc_msg.header = point_cloud_header_;
  transformed_edge_pc_msg.header.frame_id = lidar_frame_;
  transformed_edge_pc_pub_->publish(transformed_edge_pc_msg);

  std::vector<Eigen::VectorXf> intersection_list{
    intersection1, intersection2, intersection3, intersection4};

  publishIntersections(intersection_list);

  // associate four intersections with four coners of the template
  Eigen::MatrixXf payload_vertices(3, 4);
  payload_vertices.col(0) = cluster.principal_axes * intersection1;
  payload_vertices.col(1) = cluster.principal_axes * intersection2;
  payload_vertices.col(2) = cluster.principal_axes * intersection3;
  payload_vertices.col(3) = cluster.principal_axes * intersection4;

  Eigen::MatrixXf ordered_payload_vertices = getOrderedCorners(payload_vertices, cluster);
  Eigen::MatrixXf vertices = Eigen::MatrixXf::Zero(3, 5);

  utils::formGrid(vertices, 0, 0, 0, cluster.tag_size);

  // This re estimated the rotation since the principal axis are only an estimation !
  // Using the rectangle model over line ransac can provide better results
  Eigen::Matrix3f R;
  utils::fitGridNew(vertices, R, ordered_payload_vertices);

  // used for visualization for corner points
  PointXYZRI showpoint;
  PointXYZRI showpoint_tag;
  PointXYZRI refined_center = cluster.average;


  for (int i = 0; i < 4; ++i) {
    showpoint.intensity = 50;
    showpoint.x = ordered_payload_vertices.col(i)(0);
    showpoint.y = ordered_payload_vertices.col(i)(1);
    showpoint.z = ordered_payload_vertices.col(i)(2);

    refined_center.x += 0.25f * showpoint.x;
    refined_center.y += 0.25f * showpoint.y;
    refined_center.z += 0.25f * showpoint.z;

    showpoint_tag.x = showpoint.x + cluster.average.x;
    showpoint_tag.y = showpoint.y + cluster.average.y;
    showpoint_tag.z = showpoint.z + cluster.average.z;
    transformed_pc->push_back(showpoint);
    transformed_pc_tag->push_back(showpoint_tag);
  }

  cluster.initial_corners = *transformed_pc_tag;
  LidarTag::publishPointcloud(transformed_pc, lidar_frame_, string("transpts"));
  LidarTag::publishPointcloud(transformed_pc_tag, lidar_frame_, string("transptstag"));

  // save initial lidar to tag pose matrix
  cluster.initial_pose.rotation = R;

  if (params_.refine_cluster_with_intersections) {
    cluster.initial_pose.translation << -refined_center.x, -refined_center.y, -refined_center.z;
  }
  else {
    cluster.initial_pose.translation << -cluster.average.x, -cluster.average.y, -cluster.average.z;
  }

  cluster.initial_pose.translation = R * cluster.initial_pose.translation;
  Eigen::Vector3f euler_angles = cluster.initial_pose.rotation.eulerAngles(0, 1, 2);
  cluster.initial_pose.roll = euler_angles[0];
  cluster.initial_pose.pitch = euler_angles[1];
  cluster.initial_pose.yaw = euler_angles[2];
  cluster.initial_pose.homogeneous.topLeftCorner(3, 3) = cluster.initial_pose.rotation;
  cluster.initial_pose.homogeneous.topRightCorner(3, 1) = cluster.initial_pose.translation;
  cluster.initial_pose.homogeneous.row(3) << 0, 0, 0, 1;

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Initial rotation matrix: \n" << R);
    RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << true);
  }

  return true;
}

/* [Grouped edge points]
 * A function to line fitting 4 lines of the tag
 */
bool LidarTag::getLines(
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f & line,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & line_cloud)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setMaxIterations(100);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Inliers size: " << inliers->indices.size());
  }

  if (inliers->indices.size() == 0) {
    if (debug_info_) {
      RCLCPP_WARN_STREAM(get_logger(), "PCL: Could not estimate a LINE model for this cluster");
      PCL_ERROR("Could not estimate a LINE model for the given dataset.");
    }
    return false;
  }

  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*line_cloud);
  line << coefficients->values[0], coefficients->values[1], coefficients->values[3],
    coefficients->values[4];

  return true;
}

/* [Unordered corner points]
 * A function to reorder the undered corner points from PCA
 */
Eigen::MatrixXf LidarTag::getOrderedCorners(
  Eigen::MatrixXf & payload_vertices, ClusterFamily_t & cluster)
{
  double max_z = payload_vertices.col(0)(2);
  int index_max_z = 0;

  for (int i = 0; i < 4; ++i) {
    if (payload_vertices.col(i)(2) >= max_z) {
      max_z = payload_vertices.col(i)(2);
      index_max_z = i;
    }
  }

  Eigen::Vector3f top(
    payload_vertices.col(index_max_z)(0), payload_vertices.col(index_max_z)(1),
    payload_vertices.col(index_max_z)(2));

  int next_index = index_max_z + 1;

  if (next_index > 3) {
    next_index = next_index - 4;
  }

  Eigen::Vector3f next(
    payload_vertices.col(next_index)(0), payload_vertices.col(next_index)(1),
    payload_vertices.col(next_index)(2));
  Eigen::Vector3f cross_vector = top.cross(next);
  Eigen::Vector3f to_center(
    payload_vertices.row(0).mean() + cluster.average.x,
    payload_vertices.row(1).mean() + cluster.average.y,
    payload_vertices.row(2).mean() + cluster.average.z);
  double direction = cross_vector.dot(to_center);
  Eigen::MatrixXf output(3, 4);
  int output_index;
  int order;

  if (direction > 0) {
    order = 1;
  } else {
    order = -1;
  }

  for (int j = 0; j < 4; ++j) {
    output_index = index_max_z + j * order;

    if (output_index >= 4 || output_index < 0) {
      output_index = output_index - 4 * order;
    }

    output.col(j) = payload_vertices.col(output_index);
  }

  return output;
}
/* [four corner points]
 * A function to compute tag size according to the corner points of the tag
 * [Note] The given four points are assumed in either cw or ccw.
 * TODO: can be faster using std::lower_bound
 */
bool LidarTag::estimateTargetSize(
  ClusterFamily_t & cluster, const Eigen::Vector3f & point1, const Eigen::Vector3f & point2,
  const Eigen::Vector3f & point3, const Eigen::Vector3f & point4)
{
  double distance1 = (point1 - point2).norm();
  double distance2 = (point2 - point3).norm();
  double distance3 = (point3 - point4).norm();
  double distance4 = (point4 - point1).norm();
  double mean_distance = (distance1 + distance2 + distance3 + distance4) / 4;
  double gap = 1e5;
  double gap_temp;
  double tagsize;
  int size_num = 0;
  bool status = true;

  for (int i = 0; i < tag_size_list_.size(); ++i) {
    gap_temp = abs(mean_distance - tag_size_list_[i]);

    if (gap_temp < gap) {
      gap = gap_temp;
      tagsize = tag_size_list_[i];
      size_num = i;
    }
  }

  // float tagsize_tunable = 0.1;
  if (gap > tagsize_tunable_ * tagsize) {
    // if (gap > 10) {
    status = false;
  } else {
    cluster.tag_size = tagsize;
    cluster.rkhs_decoding.size_num = size_num;
    status = true;
  }

  // TODO: set a threshold to invalid the tagsize
  if (debug_info_) {
            RCLCPP_DEBUG_STREAM(get_logger(), "==== estimateTargetSize ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));

    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "Estimated side1 legth: " << distance1);
    RCLCPP_DEBUG_STREAM(get_logger(), "Estimated side2 legth: " << distance2);
    RCLCPP_DEBUG_STREAM(get_logger(), "Estimated side3 legth: " << distance3);
    RCLCPP_DEBUG_STREAM(get_logger(), "Estimated side4 legth: " << distance4);
    RCLCPP_DEBUG_STREAM(get_logger(), "Estimated size: " << mean_distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Chosen size: " << tagsize);
    RCLCPP_DEBUG_STREAM(get_logger(), "Gap : " << gap);

    if (gap > tagsize_tunable_ * tagsize)
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << status);
    else
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << status);
  }
  return status;
}
/* [two lines]
 * A function to compute the intersection of two lines
 */
Eigen::Vector3f LidarTag::getintersec(const Eigen::Vector4f & line1, const Eigen::Vector4f & line2)
{
  float c = line1(0);
  float d = line1(1);
  float a = line1(2);
  float b = line1(3);
  float p = line2(0);
  float q = line2(1);
  float m = line2(2);
  float n = line2(3);
  assert((a * n - b * m) != 0);
  float x = (a * n * p - a * m * q + a * d * m - b * c * m) / (a * n - b * m);
  float y = (b * n * p - b * c * n + a * d * n - b * m * q) / (a * n - b * m);
  Eigen::Vector3f intersection(x, y, 0);

  return intersection;
}

/*
 * A function to estimate a cluster's principle axis
 */
// Eigen::MatrixXf
void LidarTag::estimatePrincipleAxis(ClusterFamily_t & cluster)
{
  Eigen::MatrixXf ave = Eigen::MatrixXf::Ones(3, cluster.inliers);
  ave.row(0) *= cluster.average.x;
  ave.row(1) *= cluster.average.y;
  ave.row(2) *= cluster.average.z;

  Eigen::MatrixXf centered_data = cluster.merged_data.topRows(3) - ave;
  cv::Mat cv_centered_data, cv_W, cv_U, cv_Vt;
  cv::eigen2cv(centered_data, cv_centered_data);
  cv::SVD::compute(cv_centered_data, cv_W, cv_U, cv_Vt);
  Eigen::Matrix3f U;
  cv::cv2eigen(cv_U, U);
  // cluster.principal_axes = svd.matrixU();
  cluster.principal_axes = U;

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== estimatePrincipleAxis ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
  }
}


/*
 * A function to transform from a customized type (LiDARpoints_t)
 * of vector of vector (edge_buff)
 * into a standard type (PointXYZRI) of pcl vector (out)
 */
void LidarTag::buffToPclVector(
  const std::vector<std::vector<LidarPoints_t>> & edge_buff, pcl::PointCloud<PointXYZRI>::Ptr out)
{
  for (int ringnumber = 0; ringnumber < beam_num_; ++ringnumber) {
    if (edge_buff.at(ringnumber).size() != 0) {
      for (int i = 0; i < edge_buff.at(ringnumber).size(); ++i) {
        out->push_back(edge_buff[ringnumber][i].point);
      }
    }
  }
}

/*
 * A function to draw a line between two points
 */
void LidarTag::assignLine(
  visualization_msgs::msg::Marker & marker, visualization_msgs::msg::MarkerArray mark_array,
  const uint32_t shape, const string ns, const double r, const double g, const double b,
  const PointXYZRI point1, const PointXYZRI point2, const int count)
{

  marker.header.frame_id = lidar_frame_;
  marker.header.stamp = current_scan_time_;
  // marker.ns = string("Boundary_") + to_string(cluster.cluster_id);
  marker.ns = ns;
  marker.id = count;
  marker.type = shape;

  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = point1.x;
  p.y = point1.y;
  p.z = point1.z;
  marker.points.push_back(p);
  p.x = point2.x;
  p.y = point2.y;
  p.z = point2.z;
  marker.points.push_back(p);

  mark_array.markers.push_back(marker);
}

/*
 * A function to transform an eigen type of vector to pcl point type
 */
void LidarTag::eigenVectorToPointXYZRI(const Eigen::Vector4f & vector, PointXYZRI & point)
{
  point.x = vector[0];
  point.y = vector[1];
  point.z = vector[2];
  // point.intensity = Vector[3]; // TODO: is this okay?
}

/*
 * A function to transform a pcl point type to an eigen vector
 */
void LidarTag::pointXYZRIToEigenVector(const PointXYZRI & point, Eigen::Vector4f & Vector)
{
  Vector[0] = point.x;
  Vector[1] = point.y;
  Vector[2] = point.z;
  Vector[3] = point.intensity;
}

void LidarTag::printStatistics(const std::vector<ClusterFamily_t> & cluster_buff)
{
  // XXX: timings are all in milliseconds
  auto valid_clusters = getValidClusters(cluster_buff);
  RCLCPP_INFO_STREAM(get_logger(), "[Writing CSV] Remaining Clusters: " << valid_clusters.size());

  std::ofstream fstats;
  std::ofstream ftiming;
  std::ofstream fpose;
  std::ofstream fclusters;
  std::ofstream fdecoding;
  std::ofstream fcorners;

  // fstats
  if (iter_ == 0) {
    fstats.open(outputs_path_ + "/stats.csv", ios::trunc);
    if (!fstats.is_open()) {
      cout << "Could not open stats.txt: " << outputs_path_ << "\n Currently at: " << __LINE__
           << endl;
      exit(0);
    }
    fstats << "iter, pc size, PoI size, clusters size, "
           << "minimum return, maximum return, "
           << "plane fitting, plane outliers, "
           << "boundary points, ring points, no edges, line fitting, "
           << "pose optimization, decoding fail, "
           << "remaining" << endl;
  } else {
    fstats.open(outputs_path_ + "/stats.csv", std::ofstream::out | std::ofstream::app);
    if (!fstats.is_open()) {
      cout << "Could not open stats.txt: " << outputs_path_ << "\n Currently at: " << __LINE__
           << endl;
      exit(0);
    }
  }
  // Summary:
  // - Original point cloud size
  // - Edge cloud size (analyzing depth discontinuity)
  // - cluster size
  // - Removal by point check (didnt meet points falling in area check)
  // - Removal by boundary point check (sqrt(tag_family)*2)
  // - Removal by no_edge_check (not used)
  // - Removal by minimum return (decoding pts + boundary_pts)
  // - Number of remaining clusters after all removals

  fstats << iter_ << ",";
  fstats << result_statistics_.point_cloud_size << ",";
  fstats << result_statistics_.edge_cloud_size << ",";
  fstats << result_statistics_.original_cluster_size << ",";
  fstats << result_statistics_.cluster_removal.minimum_return << ",";
  fstats << result_statistics_.cluster_removal.maximum_return << ",";
  fstats << result_statistics_.cluster_removal.plane_fitting << ",";
  fstats << result_statistics_.cluster_removal.plane_outliers << ",";
  fstats << result_statistics_.cluster_removal.boundary_point_check << ",";
  fstats << result_statistics_.cluster_removal.minimum_ring_points << ",";
  fstats << result_statistics_.cluster_removal.no_edge_check << ",";
  fstats << result_statistics_.cluster_removal.line_fitting << ",";
  fstats << result_statistics_.cluster_removal.pose_optimization << ",";
  fstats << result_statistics_.cluster_removal.decoding_failure << ",";
  fstats << result_statistics_.remaining_cluster_size << endl;
  fstats.close();
  fstats << std::endl;

  // Timing
  if (debug_time_) {
    RCLCPP_INFO(get_logger(), "debug time");
    if (iter_ == 0) {
      ftiming.open(outputs_path_ + "/timing_all.txt", ios::trunc);
      if (!ftiming.is_open()) {
        cout << "Could not open timing_all.txt: " << outputs_path_ << "\n Currently at " << __FILE__
             << " at " << __LINE__ << endl;
        exit(0);
      }

      ftiming << "iter, duration, PoI_clustering, "
              << "to_pcl, fill_in_time, point_check, plane_fitting, "
              << "line_fitting, avage_edge_points, pca, "
              << "split_edge, pose_optimization, store_template, "
              << "payload_decoding" << endl;
    } else {
      ftiming.open(outputs_path_ + "/timing_all.txt", std::ofstream::out | std::ofstream::app);
      if (!ftiming.is_open()) {
        cout << "Could not open timing_all.txt: " << outputs_path_ << "\n Currently at " << __FILE__
             << " at " << __LINE__ << endl;
        exit(0);
      }
    }
    ftiming << iter_ << ",";
    ftiming << timing_.total_duration << ",";
    ftiming << timing_.edging_and_clustering_time << ",";
    ftiming << timing_.to_pcl_vector_time << ",";
    ftiming << timing_.fill_in_time << ",";
    ftiming << timing_.point_check_time << ",";
    ftiming << timing_.plane_fitting_removal_time << ",";
    ftiming << timing_.line_fitting_time << ",";
    ftiming << timing_.organize_points_time << ",";
    ftiming << timing_.pca_time << ",";
    ftiming << timing_.split_edge_time << ",";
    ftiming << timing_.pose_optimization_time << ",";
    ftiming << timing_.store_template_time << ",";
    ftiming << timing_.payload_decoding_time << endl;
    ftiming.close();
  } else {
    if (iter_ == 0) {
      ftiming.open(outputs_path_ + "/timing_computation_only.txt", ios::trunc);
      if (!ftiming.is_open()) {
        cout << "Could not open computation_time.txt: " << outputs_path_ << "\n Currently at "
             << __FILE__ << " at " << __LINE__ << endl;
        exit(0);
      }
      ftiming << "iter, duration" << endl;
    } else {
      ftiming.open(
        outputs_path_ + "/timing_computation_only.txt", std::ofstream::out | std::ofstream::app);
      if (!ftiming.is_open()) {
        cout << "Could not open computation_time.txt: " << outputs_path_ << "\n Currently at "
             << __FILE__ << " at " << __LINE__ << endl;
        exit(0);
      }
    }

    ftiming << iter_ << ", ";
    ftiming << timing_.total_duration << endl;
    ftiming.close();
  }

  // decoding
  if (debug_decoding_time_) {
    if (iter_ == 0) {
      fdecoding.open(outputs_path_ + "/decoding_analysis.txt", ios::trunc);
      if (!fdecoding.is_open()) {
        cout << "Could not open decoding_analysis.txt: " << outputs_path_ << "\n Currently at "
             << __FILE__ << " at " << __LINE__ << endl;
        exit(0);
      }
      fdecoding << "iter, original, matrix, vector, "
                << "tbb original, tbb vec, "
                << "manual scdl tbb vec, tbb scdl tbb vec, "
                << "tbb kd tree" << endl;
    } else {
      fdecoding.open(
        outputs_path_ + "/decoding_analysis.txt", std::ofstream::out | std::ofstream::app);
      if (!fdecoding.is_open()) {
        cout << "Could not open decoding_analysis.txt: " << outputs_path_ << "\n Currently at "
             << __FILE__ << " at " << __LINE__ << endl;
        exit(0);
      }
    }
    fdecoding << iter_ << ", ";
    fdecoding << time_decoding_.original << ", ";
    fdecoding << time_decoding_.matrix << ", ";
    fdecoding << time_decoding_.vectorization << ", ";
    fdecoding << time_decoding_.tbb_original << ", ";
    fdecoding << time_decoding_.tbb_vectorization << ", ";
    fdecoding << time_decoding_.manual_scheduling_tbb_vectorization << ", ";
    fdecoding << time_decoding_.tbb_scheduling_tbb_vectorization << ", ";
    fdecoding << time_decoding_.tbb_kd_tree << endl;
    fdecoding.close();
  }

  // pose and clustering
  if (iter_ == 0) {
    for (int i = 0; i < num_tag_sizes_; ++i) {
      std::string _tag_file_path =
        outputs_path_ + "tag_size" + std::to_string(tag_size_list_[i]) + "pose.txt";
      fpose.open(_tag_file_path, std::ofstream::trunc);
      if (!fpose.is_open()) {
        cout << "Could not open fpose file: " << _tag_file_path << "Currently at :" << __LINE__
             << endl;
        exit(0);
      }
      // fpose << "iter, id, rot_num, x, y, z, r, p, y";
      fpose << "iter, id, rot_num, x, y, z, r11, r12, r13, ";
      fpose << "r21, r22, r23, r31, r32, r33" << endl;
      fpose << std::endl;
      fpose.close();
    }

    for (int i = 0; i < num_tag_sizes_; ++i) {
      std::string _corners_file_path =
        outputs_path_ + "tag_size" + std::to_string(tag_size_list_[i]) + "corners.csv";
      fcorners.open(_corners_file_path, std::ofstream::trunc);
      if (!fcorners.is_open()) {
        cout << "Could not open fcorners file: " << _corners_file_path
             << "Currently at :" << __LINE__ << endl;
        exit(0);
      }
      // fpose << "iter, id, rot_num, x, y, z, r, p, y";
      fcorners << "iter, 1x, 1y, 1z, 2x, 2y, 2z, 3x, 3y, 3z, 4x, 4y, 4z" << endl;
      fcorners << std::endl;
      fcorners.close();
    }

    fclusters.open(outputs_path_ + "/clusters.csv", std::ofstream::trunc);
    if (!fclusters.is_open()) {
      cout << "Could not open cluster file: " << outputs_path_ << "Currently at :" << __LINE__
           << endl;
      exit(0);
    }
    fclusters << "It is recorded if there is any valid cluster" << endl;
    fclusters << "iter, valid cluster size, valid cluter points" << endl;
  } else {
    fclusters.open(outputs_path_ + "/clusters.csv", std::ofstream::out | std::ofstream::app);
    if (!fclusters.is_open()) {
      cout << "Could not open cluster file: " << outputs_path_ << "Currently at :" << __LINE__
           << endl;
      exit(0);
    }
  }

  if (valid_clusters.size() > 0) {
    fclusters << iter_ << ",";
    fclusters << valid_clusters.size() << ",";
    for (const auto cluster_idx : valid_clusters) {
      std::string _tag_file_path = outputs_path_ + "tag_size" +
                                   std::to_string(cluster_buff[cluster_idx].tag_size) + "pose.txt";
      fpose.open(_tag_file_path, std::ofstream::out | std::ofstream::app);
      fpose << iter_ << ",";
      fpose << cluster_buff[cluster_idx].rkhs_decoding.id << ",";
      fpose << cluster_buff[cluster_idx].rkhs_decoding.rotation_angle << ",";
      fpose << cluster_buff[cluster_idx].pose.translation(0) << ", "
            << cluster_buff[cluster_idx].pose.translation(1) << ", "
            << cluster_buff[cluster_idx].pose.translation(2) << ", ";

      fpose << cluster_buff[cluster_idx].pose.rotation(0, 0) << ", "
            << cluster_buff[cluster_idx].pose.rotation(0, 1) << ", "
            << cluster_buff[cluster_idx].pose.rotation(0, 2) << ", "
            << cluster_buff[cluster_idx].pose.rotation(1, 0) << ", "
            << cluster_buff[cluster_idx].pose.rotation(1, 1) << ", "
            << cluster_buff[cluster_idx].pose.rotation(1, 2) << ", "
            << cluster_buff[cluster_idx].pose.rotation(2, 0) << ", "
            << cluster_buff[cluster_idx].pose.rotation(2, 1) << ", "
            << cluster_buff[cluster_idx].pose.rotation(2, 2) << "";
      // fpose << cluster_buff[cluster_idx].pose.roll << ",";
      // fpose << cluster_buff[cluster_idx].pose.pitch << ",";
      // fpose << cluster_buff[cluster_idx].pose.yaw << ",";
      fpose << std::endl;
      fclusters << cluster_buff[cluster_idx].data.size() << ",";
      fpose.close();

      Eigen::Vector4f corner_lidar1(
        0, cluster_buff[cluster_idx].tag_size / 2, cluster_buff[cluster_idx].tag_size / 2, 1);
      Eigen::Vector4f corner_lidar2(
        0, cluster_buff[cluster_idx].tag_size / 2, -cluster_buff[cluster_idx].tag_size / 2, 1);
      Eigen::Vector4f corner_lidar3(
        0, -cluster_buff[cluster_idx].tag_size / 2, -cluster_buff[cluster_idx].tag_size / 2, 1);
      Eigen::Vector4f corner_lidar4(
        0, -cluster_buff[cluster_idx].tag_size / 2, cluster_buff[cluster_idx].tag_size / 2, 1);
      Eigen::Vector4f corner_tag1 = cluster_buff[cluster_idx].pose.homogeneous * corner_lidar1;
      Eigen::Vector4f corner_tag2 = cluster_buff[cluster_idx].pose.homogeneous * corner_lidar2;
      Eigen::Vector4f corner_tag3 = cluster_buff[cluster_idx].pose.homogeneous * corner_lidar3;
      Eigen::Vector4f corner_tag4 = cluster_buff[cluster_idx].pose.homogeneous * corner_lidar4;
      std::string _corners_file_path = outputs_path_ + "tag_size" +
                                       std::to_string(cluster_buff[cluster_idx].tag_size) +
                                       "corners.csv";
      fcorners.open(_corners_file_path, std::ofstream::out | std::ofstream::app);
      fcorners << iter_ << ",";
      fcorners << corner_tag1(0) << ",";
      fcorners << corner_tag1(1) << ",";
      fcorners << corner_tag1(2) << ",";
      fcorners << corner_tag2(0) << ",";
      fcorners << corner_tag2(1) << ",";
      fcorners << corner_tag2(2) << ",";
      fcorners << corner_tag3(0) << ",";
      fcorners << corner_tag3(1) << ",";
      fcorners << corner_tag3(2) << ",";
      fcorners << corner_tag4(0) << ",";
      fcorners << corner_tag4(1) << ",";
      fcorners << corner_tag4(2) << ",";
      fcorners << std::endl;
      fcorners.close();
    }
  }
  fclusters << std::endl;
  fclusters.close();

  iter_++;
}

std::vector<int> LidarTag::getValidClusters(const std::vector<ClusterFamily_t> & cluster_buff)
{
  std::vector<int> valid_clusters{};

  for (int i = 0; i < cluster_buff.size(); ++i) {
    if (cluster_buff[i].valid) {
      valid_clusters.push_back(i);
    }
  }

  std::sort(valid_clusters.begin(), valid_clusters.end(), [&](int idx1, int idx2) {
    return cluster_buff[idx1].data.size() < cluster_buff[idx2].data.size();
  });

  return valid_clusters;
}

void LidarTag::detectionArrayPublisher(
  const ClusterFamily_t & cluster,
  lidartag_msgs::msg::LidarTagDetectionArray & detections_array)
{
  lidartag_msgs::msg::LidarTagDetection detection;
  detection.header = point_cloud_header_;

  detection.id = cluster.cluster_id;
  detection.size = cluster.tag_size;
  pcl::PointCloud<PointXYZRI>::Ptr clusterPC(new pcl::PointCloud<PointXYZRI>);
  for (int i = 0; i < cluster.data.size(); ++i) {
    clusterPC->push_back(cluster.data[i].point);
  }

  Eigen::Matrix4d homogeneus = cluster.pose.homogeneous.cast<double>();
  Eigen::Isometry3d isometry;
  isometry.matrix() = homogeneus;
  detection.pose = tf2::toMsg(isometry);

 // Tag in tag coordinates (counter clock-wise)
  std::vector<Eigen::Vector2f> vertex = {
    Eigen::Vector2f{-0.75f, -0.75f}, Eigen::Vector2f{0.75f, -0.75f},
    Eigen::Vector2f{0.75f, 0.75f}, Eigen::Vector2f{-0.75f, 0.75f}};

  detection.vertices.resize(4);

  // Calculate the tag's boundary corners based on the detection's pose and geometry
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector2f & v = vertex[i];
    Eigen::Vector4f corner_lidar(
      0.f, v[0] * cluster.tag_size / 2.f, v[1] * cluster.tag_size / 2.f, 1.f);

    Eigen::Vector4f tag_boundary_corner = cluster.pose.homogeneous * corner_lidar;
    geometry_msgs::msg::Point & p = detection.vertices[i];
    p.x = tag_boundary_corner.x();
    p.y = tag_boundary_corner.y();  //_payload_size
    p.z = tag_boundary_corner.z();
  }


  sensor_msgs::msg::PointCloud2 pcs_waited_to_pub;
  pcl::toROSMsg(*clusterPC, pcs_waited_to_pub);
  detection.points = pcs_waited_to_pub;
  detections_array.detections.push_back(detection);
}
void LidarTag::keyboardEventOccurred(
  const pcl::visualization::KeyboardEvent & event, void * nothing)
{
  if (event.getKeySym() == "space" && event.keyDown()) {
    loop_ = true;
  }

}

void LidarTag::visualiseClusterBuff(vector<ClusterFamily_t> & cluster_buff)
{
  RCLCPP_INFO(get_logger(), "visualiseClusterBuff start");
  cluster_buff_time_stamp_sec_ = point_cloud_header_.stamp.sec;
  cluster_buff_time_stamp_nsec_ = point_cloud_header_.stamp.nanosec;
  cout << fixed;
  std::cout << "time stamp: sec = " << cluster_buff_time_stamp_sec_ << endl;
  std::cout << "time stamp: nsec = " << cluster_buff_time_stamp_nsec_ << endl;

  pcl::visualization::PCLVisualizer viewer("Cluster buff visualization");
  int v1(0);
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;
  pcl::PointCloud<LidarPoints_t>::Ptr cluster_pc(new pcl::PointCloud<LidarPoints_t>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZ>);
  std::stringstream ss;
  std::string index;
  std::string pc_size;
  ss << 0;
  index = "cluster buff index = " + ss.str();
  pc_size = "cluster point cloud size = " + ss.str();
  *cluster_pc = cluster_buff[0].data;
  std::cout << "0:size = " << cluster_pc->points.size() << std::endl;
  output_pc->points.resize(cluster_pc->points.size());
  for (size_t i = 0; i < cluster_pc->points.size(); i++) {
    output_pc->points[i].x = cluster_pc->points[i].point.x;
    output_pc->points[i].y = cluster_pc->points[i].point.y;
    output_pc->points[i].z = cluster_pc->points[i].point.z;
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color(
    output_pc, 20, 180, 20);
  viewer.addPointCloud(output_pc, cluster_color, "cluster", v1);
  viewer.addText(index, 10, 80, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "index", v1);
  viewer.addText(pc_size, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "pc_size", v1);
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.registerKeyboardCallback(&LidarTag::keyboardEventOccurred, *this, (void *)NULL);
  int index_num = 0;
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
    if (loop_) {
      if (index_num + 1 > cluster_buff.size()) {
        RCLCPP_INFO(get_logger(), "[ClusterBuff_visualization] exceeded clusterbuff size");
        return;
      }
      index_num++;
      *cluster_pc = cluster_buff[index_num].data;
      output_pc->points.resize(cluster_pc->points.size());
      if (output_pc->points.size() < 1000) {
        continue;
      }
      // std::cout << "index number = " << index_num << endl << "size = " <<
      // output_pc->points.size() << std::endl;
      for (size_t i = 0; i < cluster_pc->points.size(); i++) {
        output_pc->points[i].x = cluster_pc->points[i].point.x;
        output_pc->points[i].y = cluster_pc->points[i].point.y;
        output_pc->points[i].z = cluster_pc->points[i].point.z;
      }
      viewer.updatePointCloud(output_pc, cluster_color, "cluster");
      ss.str("");
      ss << index_num;
      index = "cluster buff index = " + ss.str();
      viewer.updateText(index, 10, 80, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "index");
      ss.str("");
      ss << output_pc->points.size();
      pc_size = "cluster point cloud size = " + ss.str();
      viewer.updateText(pc_size, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "pc_size");
    }

    loop_ = false;
  }

  viewer.close();
  cluster_pc->clear();
  output_pc->clear();
}

void LidarTag::publishLidartagCluster(const vector<ClusterFamily_t> & cluster_buff)
{
  pcl::PointCloud<LidarPoints_t>::Ptr cluster_pc(new pcl::PointCloud<LidarPoints_t>);
  pcl::PointCloud<LidarPoints_t>::Ptr cluster_ep_pc(new pcl::PointCloud<LidarPoints_t>);
  pcl::PointCloud<LidarPoints_t>::Ptr cluster_tr_ep_pc(new pcl::PointCloud<LidarPoints_t>);
  pcl::PointCloud<PointXYZRI>::Ptr output_pc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr output_ep_pc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr output_tr_ep_pc(new pcl::PointCloud<PointXYZRI>);
  sensor_msgs::msg::PointCloud2 output_data_msg;
  sensor_msgs::msg::PointCloud2 output_ep_msg;
  sensor_msgs::msg::PointCloud2 output_tr_ep_msg;
  geometry_msgs::msg::PointStamped point;

  int index_num = 0;
  while (true) {
    if (index_num + 1 > cluster_buff.size()) {
      return;
    }
    int points_size =
      cluster_buff[index_num].data.size() + cluster_buff[index_num].edge_points.size();
    if (
      points_size < params_.cluster_min_points_size || points_size > params_.cluster_max_points_size ||
      index_num < params_.cluster_min_index || index_num > params_.cluster_max_index) {
      index_num++;
      continue;
    }
    *cluster_pc = cluster_buff[index_num].data;
    *cluster_ep_pc = cluster_buff[index_num].edge_points;
    *cluster_tr_ep_pc = cluster_buff[index_num].transformed_edge_points;
    output_pc->points.resize(cluster_pc->points.size());
    output_ep_pc->points.resize(cluster_ep_pc->points.size());
    output_tr_ep_pc->points.resize(cluster_tr_ep_pc->points.size());
    point.header.stamp = clock_->now();
    point.header.frame_id = lidar_frame_;
    point.point.x = cluster_buff[index_num].average.x;
    point.point.y = cluster_buff[index_num].average.y;
    point.point.z = cluster_buff[index_num].average.z;
    // std::cout << "index number = " << index_num << endl << "size = " <<
    // output_pc->points.size() << std::endl;
    for (size_t i = 0; i < cluster_pc->points.size(); i++) {
      output_pc->points[i].x = cluster_pc->points[i].point.x;
      output_pc->points[i].y = cluster_pc->points[i].point.y;
      output_pc->points[i].z = cluster_pc->points[i].point.z;
      output_pc->points[i].ring = cluster_pc->points[i].point.ring;
      output_pc->points[i].intensity = cluster_pc->points[i].point.intensity;
    }
    for (size_t i = 0; i < cluster_ep_pc->points.size(); i++) {
      output_ep_pc->points[i].x = cluster_ep_pc->points[i].point.x;
      output_ep_pc->points[i].y = cluster_ep_pc->points[i].point.y;
      output_ep_pc->points[i].z = cluster_ep_pc->points[i].point.z;
      output_ep_pc->points[i].ring = cluster_ep_pc->points[i].point.ring;
      output_ep_pc->points[i].intensity = cluster_ep_pc->points[i].point.intensity;
    }
    for (size_t i = 0; i < cluster_tr_ep_pc->points.size(); i++) {
      output_tr_ep_pc->points[i].x = cluster_tr_ep_pc->points[i].point.x;
      output_tr_ep_pc->points[i].y = cluster_tr_ep_pc->points[i].point.y;
      output_tr_ep_pc->points[i].z = cluster_tr_ep_pc->points[i].point.z;
      output_tr_ep_pc->points[i].ring = cluster_tr_ep_pc->points[i].point.ring;
      output_tr_ep_pc->points[i].intensity = cluster_tr_ep_pc->points[i].point.intensity;
    }
    pcl::toROSMsg(*output_pc, output_data_msg);
    pcl::toROSMsg(*output_ep_pc, output_ep_msg);
    pcl::toROSMsg(*output_tr_ep_pc, output_tr_ep_msg);
    output_data_msg.header.frame_id = lidar_frame_;
    output_ep_msg.header.frame_id = lidar_frame_;
    output_tr_ep_msg.header.frame_id = lidar_frame_;
    output_data_msg.header = point_cloud_header_;
    output_ep_msg.header = point_cloud_header_;
    output_tr_ep_msg.header = point_cloud_header_;
    lidartag_cluster_pub_->publish(output_data_msg);
    lidartag_cluster_edge_points_pub_->publish(output_ep_msg);
    lidartag_cluster_transformed_edge_points_pub_->publish(output_tr_ep_msg);
    average_point_pub_->publish(point);
    //publishClusterInfo(cluster_buff[index_num]); Disabled due to the absence of jsk packages in ros2
    index_num++;
  }
  cluster_pc->clear();
  cluster_ep_pc->clear();
  cluster_tr_ep_pc->clear();
  output_pc->clear();
  output_ep_pc->clear();
  output_tr_ep_pc->clear();
}

void LidarTag::publishIntersections(const std::vector<Eigen::VectorXf> intersection_list)
{
  visualization_msgs::msg::MarkerArray intersection_marker_array;
  intersection_marker_array.markers.resize(4);
  int index = 0;

  for (auto intersection : intersection_list) {
    intersection_marker_array.markers[index].header.frame_id = lidar_frame_;
    intersection_marker_array.markers[index].header.stamp = clock_->now();
    intersection_marker_array.markers[index].ns = "intersection_marker";
    intersection_marker_array.markers[index].id = index;
    intersection_marker_array.markers[index].type = visualization_msgs::msg::Marker::CUBE;
    intersection_marker_array.markers[index].action = visualization_msgs::msg::Marker::ADD;
    intersection_marker_array.markers[index].lifetime = rclcpp::Duration::from_seconds(sleep_time_for_vis_ * 10);
    intersection_marker_array.markers[index].scale.x = 0.1;
    intersection_marker_array.markers[index].scale.y = 0.1;
    intersection_marker_array.markers[index].scale.z = 0.1;
    intersection_marker_array.markers[index].pose.position.x = intersection[0];
    intersection_marker_array.markers[index].pose.position.y = intersection[1];
    intersection_marker_array.markers[index].pose.position.z = 0;
    intersection_marker_array.markers[index].pose.orientation.x = 0;
    intersection_marker_array.markers[index].pose.orientation.y = 0;
    intersection_marker_array.markers[index].pose.orientation.z = 0;
    intersection_marker_array.markers[index].pose.orientation.w = 1;
    intersection_marker_array.markers[index].color.r = 0.0f;
    intersection_marker_array.markers[index].color.g = 1.0f;
    intersection_marker_array.markers[index].color.b = 0.0f;
    intersection_marker_array.markers[index].color.a = 1.0f;
    index++;
  }
  intersection_marker_array_pub_->publish(intersection_marker_array);
}

void LidarTag::printClusterResult(const std::vector<ClusterFamily_t> & cluster_buff)
{
  std::ofstream fresult;
  if (iter_ == 0) {
    fresult.open(outputs_path_ + "/results.csv", std::ofstream::out);
    if (!fresult.is_open()) {
      std::cout << "Could not open results.csv: " << outputs_path_ << "\n Crrently at: " << __LINE__
                << std::endl;
      exit(0);
    }
    fresult << "valid,id,x,y,z,roll,pitch,yaw,tag_size";
    fresult << std::endl;

  } else {
    fresult.open(outputs_path_ + "/results.csv", std::ofstream::out | std::ofstream::app);
    if (!fresult.is_open()) {
      cout << "Could not open results.csv: " << outputs_path_ << "\n Currently at: " << __LINE__
           << endl;
      exit(0);
    }
  }
  for (auto cluster : cluster_buff) {
    if (cluster.valid) {
      fresult << cluster.valid << ",";
      fresult << cluster.cluster_id << ",";
      fresult << cluster.pose.translation(0) << ",";
      fresult << cluster.pose.translation(1) << ",";
      fresult << cluster.pose.translation(2) << ",";
      fresult << cluster.pose.roll << ",";
      fresult << cluster.pose.pitch << ",";
      fresult << cluster.pose.yaw << ",";
      //   fresult << cluster.average.x << ", ";
      //   fresult << cluster.average.y << ", ";
      //   fresult << cluster.average.z << ", ";
      fresult << cluster.tag_size;
      fresult << std::endl;
    }
  }
  fresult.close();
  fresult << std::endl;
}

void LidarTag::addOrderedPointcloudMarkers(std::vector<std::vector<LidarPoints_t>> & ordered_buff)
{
  for(int ring_id = 0; ring_id < ordered_buff.size(); ring_id++) {

    if(ring_id != params_.debug_ring_id)
      continue;

    for(int index = 0; index < ordered_buff[ring_id].size(); index++) {

      auto & point = ordered_buff[ring_id][index];

      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = lidar_frame_;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime = rclcpp::Duration::from_seconds(sleep_time_for_vis_ * 10);
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;

      marker.header.stamp = clock_->now();
      marker.id = index;
      marker.text = to_string(index);
      marker.ns = "ring_" + to_string(ring_id);
      marker.pose.position.x = point.point.x;
      marker.pose.position.y = point.point.y;
      marker.pose.position.z = point.point.z;
      ordered_pointcloud_markers_.markers.push_back(marker);

    }
  }
}

};  // namespace BipedLab
