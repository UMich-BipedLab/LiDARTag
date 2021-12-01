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

#include <Eigen/Dense>                        // SVD
#include <unsupported/Eigen/CXX11/Tensor>     // tensor output
#include <unsupported/Eigen/MatrixFunctions>  // matrix exponential

#include <rclcpp/rclcpp.hpp> // package
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "lidartag.h"
#include "apriltag_utils.h"
#include "utils.h"
#include "ultra_puck.h"
#include "utils.h"

#include <math.h>   /* sqrt, pow(a,b) */
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* clock_t, clock, CLOCKS_PER_SEC */
#include <unistd.h>
#include <algorithm>  // std::sort
#include <fstream>    // log files
#include <nlopt.hpp>
#include <thread>


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
  }
}
}  // namespace

namespace BipedLab
{
LiDARTag::LiDARTag(const rclcpp::NodeOptions & options) :
  Node("lidar_tag_node", options), broadcaster_(*this), _point_cloud_received(0),
  _clock(this->get_clock()), _pub_frame("velodyne"), // what frame of the published pointcloud should be 
  _stop(0) // Just a switch for exiting this program while using valgrind
{
  LiDARTag::_getParameters();
  // fbuff.open(_outputs_path + "/cluster_buff.csv", std::ios::out);
  // if (!fbuff.is_open()) {
  //         cout << "Could not open cluster_buff.txt: " <<
  //         _outputs_path
  //         << "\n Currently at: " << __LINE__ << endl;
  //         exit(0);
  // }
  // fbuff << "ros time stamp, pc time stamp sec, pc time stamp nsec,
  // index number, cluster size, edge size, inlier size,
  // percentage_inliers, detail_valid, valid, " << endl;

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  if (_id_decoding) {
    cout << "\033[1;32m\n\n===== loading tag family ===== \033[0m\n";
    LiDARTag::_initDecoder();
  }

  if (_decode_method != 0 && _decode_method != 1 && _decode_method != 2) {
    RCLCPP_ERROR(get_logger(), "Please use 0, 1 or 2 for decode_method in the launch file");
    RCLCPP_INFO_STREAM(get_logger(), "currently using: "<< _decode_method);
  }
  cout << "\033[1;32m=========================== \033[0m\n";
  cout << "\033[1;32m=========================== \033[0m\n";

  RCLCPP_INFO(get_logger(),"ALL INITIALIZED!");

  _lidar1_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    _pointcloud_topic, 50, std::bind(&LiDARTag::_pointCloudCallback, this, std::placeholders::_1));
  _edge_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("WholeEdgedPC", 10);
  _transformed_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("TransformedPoints", 10);
  _transformed_points_tag_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("TransformedPointsTag", 10);
  _edge1_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("EdgeGroup1", 10);
  _edge2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("EdgeGroup2", 10);
  _edge3_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("EdgeGroup3", 10);
  _edge4_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("EdgeGroup4", 10);
  _boundary_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("BoundaryPts", 10);
  _cluster_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("DetectedPC", 10);
  // _index_pub =
  //     this->create_publisher<sensor_msgs::msg::PointCloud2>("IndexPC", 10);
  _payload_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("Associated_pattern_3d", 10);
  _payload3d_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("Template_points_3d", 10);
  _tag_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("Template_points", 10);
  _ini_tag_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("Initial_Template_points", 10);
  _boundary_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("BoundaryMarker", 10);
  _id_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("IDMarkers", 10);
  _cluster_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("ClusterMarker", 10);
  _payload_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("PayloadEdges", 10);
  _payload_grid_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("Grid", 10);
  _payload_grid_line_pub = this->create_publisher<visualization_msgs::msg::Marker>("GridLine", 10);
  _ideal_frame_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("IdealFrame", 10);
  _tag_frame_pub = this->create_publisher<visualization_msgs::msg::Marker>("TagFrame", 10);
  _edge_vector_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("EdgeVector", 10);
  _lidartag_pose_pub = this->create_publisher<lidartag_msgs::msg::LidarTagDetectionArray>("LiDARTagPose", 1);
  _clustered_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("ClusterEdgePC", 5);
  _detectionArray_pub =
    this->create_publisher<lidartag_msgs::msg::LidarTagDetectionArray>(_lidartag_detection_topic, 10);
  _lidartag_cluster_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidartag_cluster_points", 10);
  _lidartag_cluster_edge_points_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidartag_cluster_edge_points", 10);
  _lidartag_cluster_transformed_edge_points_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidartag_cluster_trasformed_edge_points", 10);
  //_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("intersection_marker", 10); // KL: seemes unused
  detail_valid_marker_array_pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("detail_valid_marker", 10);
  //detail_valid_text_pub = this->create_publisher<jsk_rviz_plugins::OverlayText>("detail_valid_text", 10);
  _intersection_marker_array_pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("intesection_markers", 10);
  _line_cloud1_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("line_cloud1", 10);
  _line_cloud2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("line_cloud2", 10);
  _line_cloud3_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("line_cloud3", 10);
  _line_cloud4_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("line_cloud4", 10);
  _cloud1_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud1", 10);
  _cloud2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud2", 10);
  _cloud3_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud3", 10);
  _cloud4_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud4", 10);
  _transformed_edge_pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_edge_pc", 10);
  _average_point_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("average_point", 10);
  _before_transformed_edge_pc_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("before_transformed_edge_pc", 10);
  _corners_array_pub = this->create_publisher<lidartag_msgs::msg::CornersArray>("corners_array", 10);
  _left_corners_pub = this->create_publisher<visualization_msgs::msg::Marker>("left_corner", 10);
  _right_corners_pub = this->create_publisher<visualization_msgs::msg::Marker>("right_corner", 10);
  _down_corners_pub = this->create_publisher<visualization_msgs::msg::Marker>("down_corner", 10);
  _top_corners_pub = this->create_publisher<visualization_msgs::msg::Marker>("top_corner", 10);
  _boundary_corners_array_pub =
    this->create_publisher<lidartag_msgs::msg::CornersArray>("boundary_corners_array", 10);
  _left_boundary_corners_pub =
    this->create_publisher<visualization_msgs::msg::Marker>("left_boundary_corner", 10);
  _right_boundary_corners_pub =
    this->create_publisher<visualization_msgs::msg::Marker>("right_boundary_corner", 10);
  _down_boundary_corners_pub =
    this->create_publisher<visualization_msgs::msg::Marker>("down_boundary_corner", 10);
  _top_boundary_corners_pub = this->create_publisher<visualization_msgs::msg::Marker>("top_boundary_corner", 10);
  _colored_cluster_buff_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cluster_buff", 10);
  _ps_cluster_buff__pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_buff_points_size_markers", 10);
  _in_cluster_buff__pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_buff_index_number_markers", 10);
  _boundary_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("boundary_points", 10);
            
  // Eigen::Matrix3d lie_group;
  // lie_group << 1, 0, 0,
  //             0, 0, -1,
  //             0, 1, 0;
  // Eigen::Vector3d lie_algebra = utils::Log_SO3(lie_group);
  // Eigen::Matrix3d lie_test_result = utils::Exp_SO3(lie_algebra);
  // cout << "rotation: \n" << lie_group << endl;
  // cout << "lie_algebra: \n" << lie_algebra <<endl;
  // cout << "lie_test_result: \n" << lie_test_result << endl;
  // double sign_test = -51;
  // double sign_result = utils::get_sign(sign_test);
  // cout << "sign_test" << sign_test << endl;
  // cout << "sign result" << sign_result << endl;
  // Eigen::Matrix4f test;
  // Eigen::Matrix4f test_addition;
  // test_addition << 1, -2, 3, -4, // 4
  //        2,  3, -4, 5, // 8
  //        3, 4, -5, 6, // 6
  //         4, 5, 6, 7; // 11
  // test << 1, -2, 3, -4, // 4
  //        -2,  3, -4, 5, // 8
  //        -3, -4, -5, 6, // 6
  //         4, -5, -6, 7; // 11
  // // Eigen::Vector4f ans = ;
  // Eigen::Vector4f vect(1,2,5,2);
  // Eigen::Matrix4f test_abs = test.cwiseAbs();
  // Eigen::Matrix4f result = test_abs.colwise() - vect;
  // cout << "matrix: \n" << test << endl;
  // cout << "addition: \n" << test_addition << endl;
  // cout << "addition_result\n" << test + test_addition << endl;
  // cout << "2*matrix: \n" << 2*test << endl;
  // cout << "abs matrix: \n" << test_abs << endl;
  // cout << "result: \n" << result << endl;
  // cout << "ans > 0: \n" << (result.array() > 0 ) << endl;
  // cout << "ans.cwisemax(0): \n" << result.array().cwiseMax(0) << endl;
  // cout << "ans.cwisemax(0).sum: \n" <<
  // result.array().cwiseMax(0).rowwise().sum() << endl;

  // _LiDAR_system.angle_list.insert(10);
  // _LiDAR_system.angle_list.insert(10.0049);
  // _LiDAR_system.angle_list.insert(10.006);
  // _LiDAR_system.angle_list.insert(10.0061);
  // _LiDAR_system.angle_list.insert(11);
  // _LiDAR_system.angle_list.insert(12);
  // _LiDAR_system.angle_list.insert(11);
  // cout << "Elements of set in sorted order: \n";
  // for (auto it : _LiDAR_system.angle_list)
  //     cout << it << " ";

  // cout << endl;

  // Eigen::MatrixXf convex_hull;
  // Eigen::MatrixXf P(Eigen::MatrixXf::Random(4, 100));
  // utils::constructConvexHull(P, convex_hull);
  // std::cout << "area: " << utils::computePolygonArea(convex_hull) <<
  // std::endl;

  // exit(0);

  RCLCPP_INFO(get_logger(), "Waiting for pointcloud data");
  
  // set parameter callback
  _set_param_res = add_on_set_parameters_callback(std::bind(&LiDARTag::paramCallback, this, std::placeholders::_1));
}

            // Exam the minimum distance of each point in a ring
LiDARTag::~LiDARTag()
{
  _extraction_thread->join();
}

rcl_interfaces::msg::SetParametersResult LiDARTag::paramCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  LidarTagParams param = _lidartag_params;
  
  try {
    UPDATE_LIDARTAG_PARAM(param, cluster_max_index);
    UPDATE_LIDARTAG_PARAM(param, cluster_min_index);
    UPDATE_LIDARTAG_PARAM(param, cluster_max_points_size);
    UPDATE_LIDARTAG_PARAM(param, cluster_min_points_size);

    // transaction succeeds, now assign values
    _lidartag_params = param;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

/*
 * Main loop
 */
void LiDARTag::_mainLoop()
{
  // Exam the minimum distance of each point in a ring
  RCLCPP_INFO(get_logger(), "Analyzing LiDAR Device");
  LiDARTag::_analyzeLiDARDevice();

  RCLCPP_INFO(get_logger(), "Start points of interest extraction");
  // ROS_INFO_STREAM("Tag_size:" << _payload_size);
  // ros::Rate r(10); // 10 hz
  //rclcpp::Duration duration(_sleep_time_for_vis); deprecates (the use)
  
  clock_t StartAve = clock();
  pcl::PointCloud<PointXYZRI>::Ptr clusterpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr clusteredgepc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr payloadpc(new pcl::PointCloud<PointXYZRI>);
  // pcl::PointCloud<PointXYZRI>::Ptr indexpc(
  //         new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr boundarypc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr tagpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr ini_tagpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr payload3dpc(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr edge_group1(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr edge_group2(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr edge_group3(new pcl::PointCloud<PointXYZRI>);
  pcl::PointCloud<PointXYZRI>::Ptr edge_group4(new pcl::PointCloud<PointXYZRI>);
  clusterpc->reserve(_point_cloud_size);
  clusteredgepc->reserve(_point_cloud_size);
  // indexpc->reserve(_point_cloud_size);
  tagpc->reserve(_point_cloud_size);
  ini_tagpc->reserve(_point_cloud_size);
  edge_group1->reserve(_point_cloud_size);
  edge_group2->reserve(_point_cloud_size);
  edge_group3->reserve(_point_cloud_size);
  edge_group4->reserve(_point_cloud_size);
  // XXX Tunalbe
  payloadpc->reserve(_point_cloud_size);
  payload3dpc->reserve(_point_cloud_size);
  boundarypc->reserve(_point_cloud_size);
  int valgrind_check = 0;

  // std::vector<std::vector<pcl::PointCloud<LiDARPoints_t>>> matData;
  //_thread_vec = std::make_shared<ThreadPool>(_num_threads);
  tbb::task_scheduler_init tbb_init(_num_threads);
  // _thread_vec(_num_threads);
  // ROS_INFO_STREAM("")

  int curr_frame = 0;
  int frame_of_interest = 9;
  int accumulated_scan = 1;
  std::vector<std::vector<LiDARPoints_t>> ordered_buff(_beam_num);
        
  while (rclcpp::ok()) {
    _lidartag_pose_array.detections.clear();
    detectionsToPub.detections.clear();
    pub_corners_array_.corners.clear();
    _boundary_corners_array_.corners.clear();
    if (_debug_time) {
      _timing = {std::chrono::steady_clock::now(),
                 std::chrono::steady_clock::now(),
                 std::chrono::steady_clock::now(),
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0};
    }

    if (_debug_decoding_time) {
      _time_decoding = {std::chrono::steady_clock::now(), 0, 0, 0, 0, 0, 0, 0, 0};
    }

    // Try to take a pointcloud from the buffer
    if (_num_accumulation == 1) {
      ordered_buff = LiDARTag::_getOrderBuff();
      if (ordered_buff.empty()) {
        continue;
      }
    } else {
      std::vector<std::vector<LiDARPoints_t>> ordered_buff_cur = LiDARTag::_getOrderBuff();
      if (ordered_buff_cur.empty()) {
        continue;
      }
      if (accumulated_scan < _num_accumulation) {
        for (int ring = 0; ring < _beam_num; ++ring) {
          ordered_buff[ring].insert(
            ordered_buff[ring].end(), ordered_buff_cur[ring].begin(), ordered_buff_cur[ring].end());
        }
        accumulated_scan++;
        _point_cloud_size += _point_cloud_size;
        continue;
      }
      accumulated_scan = 1;
    }

    // cout << "Frame: " << curr_frame << endl;
    // if (++curr_frame < frame_of_interest) {
    //     continue;
    // } else if (curr_frame > frame_of_interest) {
    //     break;
    // }

    // A vector of clusters
    std::vector<ClusterFamily_t> clusterbuff;

    pcl::PointCloud<PointXYZRI>::Ptr extracted_poi_pc =
      LiDARTag::_lidarTagDetection(ordered_buff, clusterbuff);

    if (_log_data) {
      printClusterResult(clusterbuff);
      _printStatistics(clusterbuff);
      // writeClusterBuff(clusterbuff, fbuff);
    }
    if (_pcl_visualize_cluster) {
      visualiseClusterBuff(clusterbuff);
    }
    clusterpc->clear();
    clusteredgepc->clear();
    payloadpc->clear();
    payload3dpc->clear();
    tagpc->clear();
    ini_tagpc->clear();
    boundarypc->clear();
    // indexpc->clear();
    edge_group1->clear();
    edge_group2->clear();
    edge_group3->clear();
    edge_group4->clear();
    for (int ring = 0; ring < _beam_num; ++ring) {
      std::vector<LiDARPoints_t>().swap(ordered_buff[ring]);
    }
    _point_cloud_size = 0;

    // // assigning indices clockwise and
    // // rings start from bottom to down (topmost is 32, for example)
    // for(int ring_number =_beam_num-1; ring_number >= 0; --ring_number) {
    //     for (int index=0; index<_np_ring; index++) {
    //         indexpc->push_back(ordered_buff[ring_number][index].point);
    //     }
    // }
    // publish detectionArray
    // LiDARTag::_detectionArrayPublisher(clusterbuff);
    // Prepare results for rviz
    visualization_msgs::msg::MarkerArray cluster_markers;
    LiDARTag::_clusterToPclVectorAndMarkerPublisher(
      clusterbuff, clusterpc, clusteredgepc, payloadpc, payload3dpc, tagpc, ini_tagpc, edge_group1,
      edge_group2, edge_group3, edge_group4, boundarypc, cluster_markers);
    // add clusters to mat files
    // LiDARTag::_saveTemporalCluster(clusterbuff, matData);

    // publish lidartag poses
    _lidartag_pose_pub->publish(_lidartag_pose_array);

    //publish lidartag corners
    // publish results for rviz
    LiDARTag::_plotIdealFrame();
    LiDARTag::_publishPC(extracted_poi_pc, _pub_frame, string("wholeedge"));
    LiDARTag::_publishPC(clusteredgepc, _pub_frame, string("clusteredgepc"));
    LiDARTag::_publishPC(clusterpc, _pub_frame, string("Cluster"));
    // LiDARTag::_publishPC(
    //         indexpc, _pub_frame, string("Indexcolumn"));
    LiDARTag::_publishPC(edge_group1, _pub_frame, string("edgegroup1"));
    LiDARTag::_publishPC(edge_group2, _pub_frame, string("edgegroup2"));
    LiDARTag::_publishPC(edge_group3, _pub_frame, string("edgegroup3"));
    LiDARTag::_publishPC(edge_group4, _pub_frame, string("edgegroup4"));
    LiDARTag::_publishPC(boundarypc, _pub_frame, string("boundarypc"));

    if (_collect_dataset) {
      if (_result_statistics.remaining_cluster_size == 1) {
        LiDARTag::_publishPC(payloadpc, _pub_frame, string("Payload"));
        LiDARTag::_publishPC(payload3dpc, _pub_frame, string("Payload3D"));
        LiDARTag::_publishPC(tagpc, _pub_frame, string("Target"));
        LiDARTag::_publishPC(ini_tagpc, _pub_frame, string("InitialTarget"));
      } else if (_result_statistics.remaining_cluster_size > 1)
        cout << "More than one!! " << endl;
      else
        cout << "Zero!! " << endl;
    } else {
      LiDARTag::_publishPC(payloadpc, _pub_frame, string("Payload"));
      LiDARTag::_publishPC(payload3dpc, _pub_frame, string("Payload3D"));
      LiDARTag::_publishPC(tagpc, _pub_frame, string("Target"));
      LiDARTag::_publishPC(ini_tagpc, _pub_frame, string("InitialTarget"));
    }
    // exit(0);
    if (_sleep_to_display) {
      rclcpp::sleep_for(std::chrono::milliseconds(int(1000*_sleep_time_for_vis)));
    }


    if (_debug_time) {
      _timing.total_time =
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.start_total_time);
    }
    // ROS_INFO_STREAM("Hz (total): " << 1e3/_timing.total_time);
    // cout << "\033[1;31m====================================== \033[0m\n";
    if (_valgrind_check) {
      valgrind_check++;
      if (valgrind_check > 0) {
        RCLCPP_ERROR(get_logger(), "valgrind out");
        _stop = 1;
        break;
      }
    }
  }  // ros::ok()
     // LiDARTag::_saveMatFiles(matData);
}

/*
 * A function to get all parameters from a roslaunch
 * if not get all parameters then it will use hard-coded parameters
 */
void LiDARTag::_getParameters() {

  std::string tag_size_string;
        
  this->declare_parameter<double>("distance_threshold");
  this->declare_parameter<std::string>("lidartag_detection_topic");
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
  this->declare_parameter<int>("beam_number");
  this->declare_parameter<int>("tag_family");
  this->declare_parameter<int>("tag_hamming_distance");
  this->declare_parameter<int>("max_decode_hamming");
  this->declare_parameter<int>("black_border");
  this->declare_parameter<double>("distance_bound");
  this->declare_parameter<double>("intensity_bound");
  this->declare_parameter<double>("depth_bound");
  this->declare_parameter<int>("fine_cluster_threshold");
  this->declare_parameter<double>("vertical_fov");
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
  this->declare_parameter<std::string>("tag_size_list");
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
  this->declare_parameter<double>("clearance");

  bool GotPubFrame = this->get_parameter("frame_name", _pub_frame);
  bool GotThreshold = this->get_parameter("distance_threshold", _distance_threshold);
  bool GotPublishTopic = this->get_parameter("lidartag_detection_topic", _lidartag_detection_topic);
  bool GotSleepToDisplay = this->get_parameter("sleep_to_display", _sleep_to_display);
  bool GotSleepTimeForVis = this->get_parameter("sleep_time_for_visulization", _sleep_time_for_vis);
  bool GotValgrindCheck = this->get_parameter("valgrind_check", _valgrind_check);
  bool GotFakeTag = this->get_parameter("fake_data", _fake_tag);
  bool GotCSV = this->get_parameter("write_csv", _write_CSV);
  bool GotMarkValidity = this->get_parameter("mark_cluster_validity", _mark_cluster_validity);
  bool GotPlaneFitting = this->get_parameter("plane_fitting", _plane_fitting);
  bool GotOptPose = this->get_parameter("optimize_pose", _pose_optimization);
  bool GotDecodeId = this->get_parameter("decode_id", _id_decoding);
  bool GotAssignId = this->get_parameter("assign_id", _assign_id);
  bool GotRingState = this->get_parameter("has_ring", _has_ring);
  bool GotRingEstimation = this->get_parameter("estimate_ring", _ring_estimation);
  bool GotAdaptiveThresholding = this->get_parameter("adaptive_thresholding", _adaptive_thresholding);
  bool GotCollectData = this->get_parameter("collect_data", _collect_dataset);
  bool GotLidarTopic = this->get_parameter("pointcloud_topic", _pointcloud_topic);
  bool GotBeamNum = this->get_parameter("beam_number", _beam_num);
  bool GotSize = this->get_parameter("tag_size", _payload_size);

  bool GotTagFamily = this->get_parameter("tag_family", _tag_family);
  bool GotTagHamming = this->get_parameter("tag_hamming_distance", _tag_hamming_distance);
  bool GotMaxDecodeHamming = this->get_parameter("max_decode_hamming", _max_decode_hamming);
  bool GotBlackBorder = this->get_parameter("black_border", _black_border);

  bool GotDistanceBound = this->get_parameter("distance_bound", _distance_bound);
  bool GotIntensityBound = this->get_parameter("intensity_bound", _intensity_threshold);
  bool GotDepthBound = this->get_parameter("depth_bound", _depth_threshold);
  bool GotFineClusterThreshold = this->get_parameter("fine_cluster_threshold", _fine_cluster_threshold);
  bool GotVerticalFOV = this->get_parameter("vertical_fov", _vertical_fov);
  bool GotFillInGapThreshold = this->get_parameter("fill_in_gap_threshold", _filling_gap_max_index);
  // bool GotFillInMaxPointsThreshold =
  //     this->get_parameter(
  //             "fill_in_max_points_threshold",
  //             fill_in_max_points_threshold);
  bool GotPointsThresholdFactor =
    this->get_parameter("points_threshold_factor", _points_threshold_factor);
  bool GotLineIntensityBound = this->get_parameter("line_intensity_bound", _line_intensity_bound);
  bool GotPayloadIntensityThreshold =
    this->get_parameter("payload_intensity_threshold", _payload_intensity_threshold);

  bool GotLatestModel = this->get_parameter("latest_model", _latest_model);
  bool GotWeightPath = this->get_parameter("weight_path", _weight_path);

  bool GotMaxPointsOnPayload = this->get_parameter("max_points_on_payload", _max_point_on_payload);
  bool GotXYZRI = this->get_parameter("xyzri", _XYZRI);
  bool GotMinPerGrid = this->get_parameter("min_retrun_per_grid", _min_returns_per_grid);
  bool GotOptimizationMethod = this->get_parameter("optimization_solver", _optimization_solver);
  bool GotDecodeMethod = this->get_parameter("decode_method", _decode_method);
  bool GotDecodeMode = this->get_parameter("decode_mode", _decode_mode);
  bool GotGridViz = this->get_parameter("grid_viz", _grid_viz);

  // bool GotStatsFilePath =
  //     this->get_parameter("stats_file_path", _stats_file_path);
  // bool GotClusterFilePath =
  //     this->get_parameter("cluster_file_path", _cluster_file_path);
  // bool GotPoseFilePath =
  //     this->get_parameter("pose_file_path", _pose_file_path);
  bool GotOutPutPath = this->get_parameter("outputs_path", _outputs_path);
  bool GotLibraryPath = this->get_parameter("library_path", _library_path);
  bool GotNumCodes = this->get_parameter("num_codes", _num_codes);

  bool GotDistanceToPlaneThreshold =
    this->get_parameter("distance_to_plane_threshold", _distance_to_plane_threshold);
  bool GotMaxOutlierRatio = this->get_parameter("max_outlier_ratio", _max_outlier_ratio);
  bool GotNumPoints =
    this->get_parameter("num_points_for_plane_feature", _num_points_for_plane_feature);
  bool GotNearBound = this->get_parameter("nearby_factor", _nearby_factor);
  bool GotNumPointsRing = this->get_parameter("number_points_ring", _np_ring);
  bool GotCoefficient = this->get_parameter("linkage_tunable", _linkage_tunable);
  bool GotTagSizeList = this->get_parameter("tag_size_list", tag_size_string);
  bool GotDerivativeMethod = this->get_parameter("euler_derivative", _derivative_method);
  bool GotNumThreads = this->get_parameter("num_threads", _num_threads);
  bool GotPrintInfo = this->get_parameter("print_info", _print_ros_info);
  bool GotDebuginfo = this->get_parameter("debug_info", _debug_info);
  bool GotDebugtime = this->get_parameter("debug_time", _debug_time);
  bool GotDebugDecodingtime = this->get_parameter("debug_decoding_time", _debug_decoding_time);
  bool GotLogData = this->get_parameter("log_data", _log_data);
  bool GotOptimizePercent = this->get_parameter("optimize_percentage", _optimization_percent);
  bool GotCalibration = this->get_parameter("calibration", _calibration);
  bool GotMinimumRingPoints =
    this->get_parameter("minimum_ring_boundary_points", _minimum_ring_boundary_points);
  bool GotUpbound = this->get_parameter("optimize_up_bound", _opt_ub);
  bool GotLowbound = this->get_parameter("optimize_low_bound", _opt_lb);
  bool GotNumAccumulation = this->get_parameter("num_accumulation", _num_accumulation);
  bool GotCoaTunable = this->get_parameter("coa_tunable", _coa_tunable);
  bool GotTagsizeTunable = this->get_parameter("tagsize_tunable", _tagsize_tunable);
  bool GotMaxClusterIndex = this->get_parameter("cluster_max_index", _lidartag_params.cluster_max_index);
  bool GotMinClusterIndex = this->get_parameter("cluster_min_index", _lidartag_params.cluster_min_index);
  bool GotMaxClusterPointsSize =
    this->get_parameter("cluster_max_points_size", _lidartag_params.cluster_max_points_size);
  bool GotMinClusterPointsSize =
    this->get_parameter("cluster_min_points_size", _lidartag_params.cluster_min_points_size);
  bool GotVisualizeCluster = this->get_parameter("pcl_visualize_cluster", _pcl_visualize_cluster);
  bool GotClearance = this->get_parameter("clearance", _clearance);
  
  std::istringstream is(tag_size_string); 
  _tag_size_list.assign( std::istream_iterator<double>( is ), std::istream_iterator<double>() );

  bool Pass = utils::checkParameters(
    65, GotFakeTag, GotLidarTopic, GotBeamNum, GotOptPose, GotDecodeId, GotPlaneFitting,
    GotAssignId, GotCSV, GotOutPutPath, GotDistanceBound, GotIntensityBound, GotDepthBound,
    GotTagFamily, GotTagHamming, GotMaxDecodeHamming, GotFineClusterThreshold, GotVerticalFOV,
    GotFillInGapThreshold, GotMaxOutlierRatio, GotPointsThresholdFactor, GotLineIntensityBound,
    GotDistanceToPlaneThreshold, GotAdaptiveThresholding, GotCollectData, GotSleepToDisplay,
    GotSleepTimeForVis, GotValgrindCheck, GotPayloadIntensityThreshold, GotLatestModel,
    GotWeightPath, GotBlackBorder, GotMaxPointsOnPayload, GotXYZRI, GotMinPerGrid, GotDecodeMethod,
    GotDecodeMode, GotOptimizationMethod, GotGridViz, GotPublishTopic, GotThreshold, GotNumPoints,
    GotNearBound, GotNumPointsRing, GotCoefficient, GotTagSizeList, GotNumThreads, GotPrintInfo,
    GotOptimizePercent, GotDebuginfo, GotDebugtime, GotLogData, GotDebugDecodingtime,
    GotLibraryPath, GotNumCodes, GotCalibration, GotMinimumRingPoints, GotRingState,
    GotRingEstimation, GotNumAccumulation, GotDerivativeMethod, GotUpbound, GotLowbound,
    GotCoaTunable, GotTagsizeTunable, GotClearance);

  if (!Pass) {
    // TODO: check compleness
    cout << "\033[1;32m=========================== \033[0m\n";
    cout << "use hard-coded parameters\n";
    cout << "\033[1;32m=========================== \033[0m\n";

    // Default value
    _distance_threshold = 10;
    _pointcloud_topic = "/velodyne_points";
    _lidartag_detection_topic = "/LiDARTag/lidar_tag/LiDARTagDetectionArray";
    _beam_num = 32;
    _distance_bound = 7;       // for edge gradient
    _intensity_threshold = 2;  // for edge gradient
    _depth_threshold = 0.5;    // for edge gradient
    _payload_intensity_threshold = 30;
    //_payload_size = 0.15;

    _tag_family = 16;
    _tag_hamming_distance = 5;
    _max_decode_hamming = 2;

    // if the points in a cluster is small than this, it'd get dropped
    _fine_cluster_threshold = 20;
    _vertical_fov = 40;

    // When fill in the cluster,
    // if it the index is too far away then drop it
    // TODO:Need a beteer way of doing it!
    _filling_gap_max_index = 200;
    _filling_max_points_threshold = 4500;

    _line_intensity_bound = 1000;  // To determine the payload edge

    // if the points on a "valid" tag is less than this factor,
    // then remove it  (the higher, the looser)
    _points_threshold_factor = 1.3;
    _adaptive_thresholding = 0;
    _collect_dataset = 1;

    _sleep_to_display = 1;
    _sleep_time_for_vis = 0.005;
    _valgrind_check = 0;
    _black_border = 1;
    _fake_tag = 0;

    _latest_model = "-337931";
    _weight_path = "/weight/";
    _max_point_on_payload = 150;
    _XYZRI = 4;
    _min_returns_per_grid = 3;
    _decode_method = 2;
    _grid_viz = 1;

    _distance_to_plane_threshold = 0.1;
    _max_outlier_ratio = 0.1;
    _num_points_for_plane_feature = 3;
    _np_ring = 10;
    _linkage_tunable = 1.0;
    _optimization_percent = 0.1;
    _tag_size_list = {0.7};
    _debug_info = false;
    _debug_time = true;
    _plane_fitting = true;
    _has_ring = true;
    _ring_estimation = true;
    _derivative_method = true;
    _opt_lb = 1.0;
    _opt_ub = 1.0;
    _coa_tunable = 0.75;
    _tagsize_tunable = 1.1;
  } else {
    cout << "\033[1;32m=========================== \033[0m\n";
    cout << "use parameters from the launch file\n";
    cout << "\033[1;32m=========================== \033[0m\n";
  }
  std::sort(_tag_size_list.begin(), _tag_size_list.end());
  _num_tag_sizes = _tag_size_list.size();
  _payload_size = _tag_size_list.back();

  const auto num_processor = std::thread::hardware_concurrency();
  _num_threads = std::min((int)num_processor, _num_threads);

  _iter = 0;

  // point association threshold (which cluster the point belongs to?)
  // cout << "_linkage_tunable: " << _linkage_tunable << endl;
  _linkage_threshold = _linkage_tunable * _payload_size * _clearance;
  if (_has_ring) {
    _use_ring = true;
  } else {
    if (_ring_estimation) {
      _use_ring = true;
    } else {
      _use_ring = false;
    }
  }
  // cout << "link threshold: " << _linkage_threshold << endl;
  // exit(0);
  _RANSAC_threshold = _payload_size / 10;

  RCLCPP_INFO(get_logger(), "Subscribe to %s\n", _pointcloud_topic.c_str());
  RCLCPP_INFO(get_logger(), "Use %i-beam LiDAR\n", _beam_num);
  RCLCPP_INFO(get_logger(), "Use %i threads\n", _num_threads);
  RCLCPP_INFO(get_logger(), "_intensity_threshold: %f \n", _intensity_threshold);
  RCLCPP_INFO(get_logger(), "_depth_threshold: %f \n", _depth_threshold);
  RCLCPP_INFO(get_logger(), "_payload_size: %f \n", _payload_size);
  RCLCPP_INFO(get_logger(), "_vertical_fov: %f \n", _vertical_fov);
  RCLCPP_INFO(get_logger(), "_fine_cluster_threshold: %i \n", _fine_cluster_threshold);
  RCLCPP_INFO(get_logger(), "_filling_gap_max_index: %i \n", _filling_gap_max_index);
  // RCLCPP_INFO(get_logger(), "_filling_max_points_threshold: %i \n", 
  // _filling_max_points_threshold);
  RCLCPP_INFO(get_logger(), "_points_threshold_factor: %f \n", _points_threshold_factor);
  RCLCPP_INFO(get_logger(), "_adaptive_thresholding: %i \n", _adaptive_thresholding);
  RCLCPP_INFO(get_logger(), "_collect_dataset: %i \n", _collect_dataset);
  RCLCPP_INFO(get_logger(), "_decode_method: %i \n", _decode_method);
  RCLCPP_INFO(get_logger(), "linkage_hreshold_: %f \n", _linkage_threshold);
  RCLCPP_INFO(get_logger(), "_RANSAC_threshold: %f \n", _RANSAC_threshold);
  RCLCPP_INFO(get_logger(), "_num_accumulation: %i \n", _num_accumulation);

  usleep(2e6);
}

/*
 * A function to get pcl ordered_buff
 * from a ros sensor-msgs form of pointcould queue
 * */
std::vector<std::vector<LiDARPoints_t>> LiDARTag::_getOrderBuff()
{
  _point_cloud1_queue_lock.lock();
  // ;boost::mutex::scoped_lock(_point_cloud1_queue_lock);
  if (_point_cloud1_queue.size() == 0) {
    _point_cloud1_queue_lock.unlock();

    // cout << "Pointcloud queue is empty" << endl;
    // cout << "size: " << empty.size() << endl;
    vector<vector<LiDARPoints_t>> empty;
    return empty;
  }
        // ROS_INFO_STREAM("Queue size: " << _point_cloud1_queue.size());

  sensor_msgs::msg::PointCloud2::SharedPtr msg = _point_cloud1_queue.front();
  _point_cloud1_queue.pop();
  _point_cloud1_queue_lock.unlock();
  _current_scan_time = msg->header.stamp;

  // Convert to sensor_msg to pcl type
  pcl::PointCloud<PointXYZRI>::Ptr pcl_pointcloud(new pcl::PointCloud<PointXYZRI>);
  pcl::fromROSMsg(*msg, *pcl_pointcloud);

  if (!_has_ring && !_ring_estimated) {
    std::vector<float> angles;
    _getAngleVector(pcl_pointcloud, angles);
    // cout << "angles_size" <<angles.size() << endl;
    std::ofstream fangles;
    fangles.open(_outputs_path + "/angles.txt", std::ofstream::out | std::ofstream::app);
    if (!fangles.is_open()) {
      cout << "Could not open angles.txt: " << _outputs_path << "\n Currently at: " << __LINE__
           << endl;
      exit(0);
    }
    fangles << std::endl;
    for (int i = 0; i < pcl_pointcloud->size(); ++i) {
      fangles << angles[i] << ",";
    }
    // fangles << angles;
    fangles << std::endl;
    fangles.close();
    _ring_estimated = true;
    vector<vector<LiDARPoints_t>> empty;
    return empty;
  }

  // Ordered pointcloud with respect to its own ring number
  std::vector<std::vector<LiDARPoints_t>> ordered_buff(_beam_num);
  _fillInOrderedPC(pcl_pointcloud, ordered_buff);
  _point_cloud_size = pcl_pointcloud->size();
  // cout << "pc size: " << _point_cloud_size << endl;

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
void LiDARTag::_analyzeLiDARDevice()
{
  _ring_estimated = false;
  _LiDAR_system.point_count_table.resize(100);
  _LiDAR_system.ring_average_table.reserve(_beam_num);

  // Initialize the table
  MaxMin_t max_min{(int)1e5, -1, -1};  // min, ave, max

  for (int j = 0; j < _beam_num; ++j) {
    _LiDAR_system.ring_average_table.push_back(max_min);
  }

  // if (!_has_ring && _ring_estimation) {
  //     while (ros::ok()) {
  //         std::vector<std::vector<LiDARPoints_t>> ordered_buff =
  //             LiDARTag::_getOrderBuff(true);
  //         if (ordered_buff.empty()) {
  //             continue;
  //         }
  //         if (!_has_ring && _ring_estimation && _ring_estimated)
  //             break;
  //     }
  // }

  // Calulate for each scan with a few seconds
  int i = 0;
  int num_scan = 0;
  clock_t begin = clock();
  int accumulated_scan = 1;
  std::vector<std::vector<LiDARPoints_t>> ordered_buff(_beam_num);
  while (rclcpp::ok()) {
    if (_num_accumulation == 1) {
      ordered_buff = LiDARTag::_getOrderBuff();
      if (ordered_buff.empty()) {
        continue;
      }
    } else {
      std::vector<std::vector<LiDARPoints_t>> ordered_buff_cur = LiDARTag::_getOrderBuff();
      if (ordered_buff_cur.empty()) {
        continue;
      }
      if (accumulated_scan < _num_accumulation) {
        for (int ring = 0; ring < _beam_num; ++ring) {
          ordered_buff[ring].insert(
            ordered_buff[ring].end(), ordered_buff_cur[ring].begin(), ordered_buff_cur[ring].end());
        }
        accumulated_scan++;
        continue;
      }
      accumulated_scan = 1;
    }

    // cout << "i: " << i++ << endl;
    LiDARTag::_maxMinPtsInAScan(
      _LiDAR_system.point_count_table[num_scan], _LiDAR_system.max_min_table,
      _LiDAR_system.ring_average_table, ordered_buff);
    num_scan++;
    clock_t end = clock();
    for (int ring = 0; ring < _beam_num; ++ring) {
      std::vector<LiDARPoints_t>().swap(ordered_buff[ring]);
    }
    if ((((double)(end - begin) / CLOCKS_PER_SEC) > 3) || num_scan >= 100) {
      break;
    }
  }
  // cout << "here";
  for (auto i = _LiDAR_system.ring_average_table.begin();
       i != _LiDAR_system.ring_average_table.end(); ++i) {
    (*i).average /= num_scan;
    // cout << "average: " << (*i).average << endl;
  }

  LiDARTag::_pointsPerSquareMeterAtOneMeter();

  // std::vector<int>::iterator it;
  // it = std::unique(_LiDAR_system.angle_list.begin(),
  //         _LiDAR_system.angle_list.end(), angleComparision);

  // Check values of pointtable and max_min_table
  // int k = 0;
  // for (auto i=_LiDAR_system.point_count_table.begin();
  // i!=_LiDAR_system.point_count_table.end(); ++i, ++k){
  //     cout << "Vector[" << k << "]:" << endl;
  //     for (auto j=(*i).begin(); j!=(*i).end(); ++j){
  //         cout << "points: " << *j << endl;
  //     }
  // }

  // k=0;
  // for (auto i=_LiDAR_system.max_min_table.begin();
  // i!=_LiDAR_system.max_min_table.end(); ++i, ++k){
  //     cout << "At scan [" << k << "]" << endl;
  //     cout << "Max: " << (*i).Max << endl;
  //     cout << "Min: " << (*i).Min << endl;
  // }

  // k=0;
  // for (auto i=_LiDAR_system.ring_average_table.begin();
  // i!=_LiDAR_system.ring_average_table.end(); ++i, ++k){
  //     cout << "At ring [" << k << "]" << endl;
  //     cout << "Max: " << (*i).Max << endl;
  //     cout << "Ave: " << (*i).average << endl;
  //     cout << "Min: " << (*i).Min << endl;
  // }
  // exit(0);
}

/*
 * A function to calculate how many points are supposed to be
 * on a cluster at 1 meter away
 */
void LiDARTag::_pointsPerSquareMeterAtOneMeter()
{
  double system_average;
  for (auto i = _LiDAR_system.ring_average_table.begin();
       i != _LiDAR_system.ring_average_table.end(); ++i) {
    system_average += (*i).average;
  }
  system_average /= _LiDAR_system.ring_average_table.size();
  _LiDAR_system.beam_per_vertical_radian = _beam_num / utils::deg2Rad(_vertical_fov);
  _LiDAR_system.point_per_horizontal_radian = system_average / utils::deg2Rad(360.0);
}

/*
 * A function to find maximum points and minimum points in a single scan,
 * i.e. to find extrema within 32 rings
 */
void LiDARTag::_maxMinPtsInAScan(
  std::vector<int> & point_count_table, std::vector<MaxMin_t> & max_min_table,
  std::vector<MaxMin_t> & ring_average_table,
  const std::vector<std::vector<LiDARPoints_t>> & ordered_buff)
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
void LiDARTag::_publishPC(
  const pcl::PointCloud<PointXYZRI>::Ptr & source_pc, const std::string & frame,
  string which_publisher)
{
  utils::tranferToLowercase(which_publisher);  // check letter cases
  sensor_msgs::msg::PointCloud2 pcs_waited_to_pub;      
  pcl::toROSMsg(*source_pc, pcs_waited_to_pub);
  pcs_waited_to_pub.header.frame_id = frame;

  try {
    if (which_publisher == "wholeedge")
      _edge_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "original")
      _original_pc_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "cluster") {
      pcs_waited_to_pub.header = _point_cloud_header;
      pcs_waited_to_pub.header.stamp = _clock->now();
      _cluster_pub->publish(pcs_waited_to_pub);
    }
    // else if (which_publisher=="indexcolumn")
    //     _index_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "payload")
      _payload_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "payload3d")
      _payload3d_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "target")
      _tag_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "edgegroup1")
      _edge1_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "edgegroup2")
      _edge2_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "edgegroup3")
      _edge3_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "edgegroup4")
      _edge4_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "boundarypc")
      _boundary_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "initialtarget")
      _ini_tag_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "clusteredgepc") {
      pcs_waited_to_pub.header.stamp = _current_scan_time;
      pcs_waited_to_pub.header.frame_id = _pub_frame;
      _clustered_points_pub->publish(pcs_waited_to_pub);
    } 
    else if (which_publisher == "transpts")
      _transformed_points_pub->publish(pcs_waited_to_pub);
    else if (which_publisher == "transptstag")
      _transformed_points_tag_pub->publish(pcs_waited_to_pub);
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
void LiDARTag::_pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc)
{
  // flag to make sure it receives a pointcloud
  // at the very begining of the program

  if (!_point_cloud_received) {
    RCLCPP_INFO(get_logger(), "Got the first pointcloud. Starting LidarTag");
    _extraction_thread = std::make_unique<boost::thread>(&LiDARTag::_mainLoop, this);
  }


  _point_cloud_received = 1;
  _point_cloud_header = pc->header;
  // boost::mutex::scoped_lock(_point_cloud1_queue_lock);
  _point_cloud1_queue_lock.lock();
  _point_cloud1_queue.push(pc);
  _point_cloud1_queue_lock.unlock();
}

/*
 * A function to slice the Veloydyne full points to sliced pointed
 * based on ring number
 * */
inline void LiDARTag::_fillInOrderedPC(
  const pcl::PointCloud<PointXYZRI>::Ptr & pcl_pointcloud,
  std::vector<std::vector<LiDARPoints_t>> & ordered_buff)
{
  if (!_has_ring && _ring_estimated) {
    std::string ring_list;
    for (auto && it : _LiDAR_system.angle_list) ring_list += (std::to_string(it) + " ");
    RCLCPP_INFO_STREAM_ONCE(get_logger(),"Estimate Ring List Size: " << _LiDAR_system.angle_list.size());
    RCLCPP_INFO_STREAM_ONCE(get_logger(),"Estimate Ring List: " << ring_list);
    assert(("Ring List Error", _LiDAR_system.angle_list.size() <= _beam_num));
  }

  LiDARPoints_t lidar_point;
  int index[_beam_num] = {0};
  std::set<float>::iterator it;
  for (auto && p : *pcl_pointcloud) {
    if (p.x == 0 && p.y == 0 && p.z == 0) {
      continue;
    }

    if (!_has_ring && _ring_estimated) {
      float angle = _getAnglefromPt(p);
      it = _LiDAR_system.angle_list.find(angle);
      p.ring = std::distance(_LiDAR_system.angle_list.begin(), it);
      // cout << "angle: " << angle << endl;
      // if (p.ring==0) {
      //     cout << "x: " << p.x << endl;
      //     cout << "y: " << p.y << endl;
      //     cout << "z: " << p.z << endl;
      //     cout << "ring: " << p.ring << endl;
      // }

      // cout << "point: " << p.x << ", " << p.y << ", " << p.z << ", " <<
      // p.ring << endl;
    }
    // cout << "point: " << p.x << ", " << p.y << ", " << p.z << ", " << p.ring
    // << ", " <<p.intensity << endl;
    assert(("Ring Estimation Error", p.ring < _beam_num));
    lidar_point.point = p;
    lidar_point.index = index[p.ring];
    lidar_point.valid = 1;
    // cout << "lidar_point: " << lidar_point.point.x << ", " <<
    //     lidar_point.point.y << ", " << lidar_point.point.z << ", " <<
    //     lidar_point.point.ring << endl;
    ordered_buff[p.ring].push_back(lidar_point);
    index[p.ring] += 1;
  }

  // for (auto &&i : ordered_buff){
  //     cout << "r: " << std::distance(ordered_buff.begin(), i) << ", size: "<<
  //     ordered_buff[i].size();
  // }
  // for (int i = 0; i < _beam_num; ++i){
  //     cout << "--------" << endl;
  //     cout << "[buff] r: " << i << ", size: "<< ordered_buff[i].size() <<
  //     endl; cout << "[indx] r: " << i << ", size: "<< index[i] << endl;
  // }
}

/*
 * A function to compute angle between the line from origin to this point
 * and z=0 plane in lidar
 * */
float LiDARTag::_getAnglefromPt(PointXYZRI & point)
{
  float distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
  float theta = utils::rad2Deg(std::atan2(point.z, distance));

  return theta;
  // phi = std::atan2(point.y, point.x);
}

void LiDARTag::_getAngleVector(
  const pcl::PointCloud<PointXYZRI>::Ptr & pcl_pointcloud, std::vector<float> & angles)
{
  for (auto && p : *pcl_pointcloud) {
    if (p.x == 0 && p.y == 0 && p.z == 0) {
      continue;
    }
    float angle = LiDARTag::_getAnglefromPt(p);
    // utils::COUT(p);
    angles.push_back(angle);
    _LiDAR_system.angle_list.insert(angle);
    // if (_LiDAR_system.angle_list.insert(angle).second == true) {
    //     cout << "Inserted: " << angle << endl;
    // } else {
    //     cout << "--Rejected: " << angle << endl;
    // }
  }
}
/*
 * Main function
 */
pcl::PointCloud<PointXYZRI>::Ptr LiDARTag::_lidarTagDetection(
  const std::vector<std::vector<LiDARPoints_t>> & ordered_buff,
  std::vector<ClusterFamily_t> & cluster_buff)
{
  if (_debug_info || _debug_time)
    RCLCPP_INFO_STREAM(get_logger(), "--------------- Begining ---------------");
  else
    RCLCPP_DEBUG_STREAM(get_logger(), "--------------- Begining ---------------");

  pcl::PointCloud<PointXYZRI>::Ptr out(new pcl::PointCloud<PointXYZRI>);
  out->reserve(_point_cloud_size);

  // Buff to store all detected edges
  std::vector<std::vector<LiDARPoints_t>> edge_buff(_beam_num);

  // calculate gradient for depth and intensity as well as
  // group them into diff groups
  _result_statistics = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0};

  _timing.start_computation_time = std::chrono::steady_clock::now();
  LiDARTag::_gradientAndGroupEdges(ordered_buff, edge_buff, cluster_buff);
  if (_debug_time) {
    _timing.edging_and_clustering_time = utils::spendElapsedTimeMilli(
      std::chrono::steady_clock::now(), _timing.start_computation_time);
    _timing.timing = std::chrono::steady_clock::now();
  }

  // transform from a vector of vector (edge_buff) into a pcl vector (out)
  boost::thread BuffToPclVectorThread(
    &LiDARTag::_buffToPclVector, this, boost::ref(edge_buff), out);
  if (_debug_time) {
    _timing.to_pcl_vector_time =
      utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
  }

  LiDARTag::_fillInCluster(ordered_buff, cluster_buff);
  BuffToPclVectorThread.join();
  _result_statistics.point_cloud_size = _point_cloud_size;
  _result_statistics.edge_cloud_size = out->size();
  _timing.total_duration =
    utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.start_computation_time);
  // std::chrono::duration<double> duration =
  //     std::chrono::steady_clock::now() - clock_start;
  // _timing.duration = duration.count();
  // cout << "Computation time : " << 1.0 / duration.count() << " [Hz]" << endl;

  // _timing.duration = duration.count();
  if (_print_ros_info || _debug_info) {
    // RCLCPP_INFO_STREAM(get_logger(), "-- Remaining Cluster: " <<
    //         _result_statistics.remaining_cluster_size);
    // RCLCPP_INFO_STREAM(get_logger(), "edge_flag: " << _edge_flag);
    // RCLCPP_INFO_STREAM(get_logger(), "mark_cluster_validity: " << _mark_cluster_validity);
    // RCLCPP_INFO_STREAM(get_logger(), "cluster buffer size: " << cluster_buff.size());
    RCLCPP_INFO_STREAM(get_logger(), "-- Computation: " << 1e3 / _timing.total_duration << " [Hz]");

  }

  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "--------------- summary ---------------");
    RCLCPP_DEBUG_STREAM(get_logger(), "-- Original cloud size: " << _result_statistics.point_cloud_size);
    RCLCPP_DEBUG_STREAM(get_logger(), "-- Edge cloud size: " << _result_statistics.edge_cloud_size);
    RCLCPP_DEBUG_STREAM(get_logger(), "-- Original cluster: " << _result_statistics.original_cluster_size);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by minimum returns: " << _result_statistics.cluster_removal.minimum_return);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by maximum returns: " << _result_statistics.cluster_removal.maximum_return);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by plane fitting failure: " << _result_statistics.cluster_removal.plane_fitting);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by plane fitting ourliers: "
      << _result_statistics.cluster_removal.plane_outliers);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by payload boundary point: "
      << _result_statistics.cluster_removal.boundary_point_check);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by payload ring points: "
      << _result_statistics.cluster_removal.minimum_ring_points);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by edge points: " << _result_statistics.cluster_removal.no_edge_check);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by line fitting: " << _result_statistics.cluster_removal.line_fitting);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by pose optimization: " << _result_statistics.cluster_removal.pose_optimization);
    RCLCPP_DEBUG_STREAM(get_logger(), 
      "-- Removed by decoding: " << _result_statistics.cluster_removal.decoding_failure);
    RCLCPP_DEBUG_STREAM(get_logger(), "-- Remaining Cluster: " << _result_statistics.remaining_cluster_size);
    RCLCPP_DEBUG_STREAM(get_logger(), "---------------------------------------");
  }
  if (_debug_time) {
    RCLCPP_DEBUG_STREAM(get_logger(), "--------------- Timing ---------------");
    RCLCPP_DEBUG_STREAM(get_logger(), "computation_time: " << 1e3 / _timing.total_duration << " [Hz]");
    RCLCPP_DEBUG_STREAM(get_logger(), "edging_and_clustering_time: " << _timing.edging_and_clustering_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "to_pcl_vector_time: " << _timing.to_pcl_vector_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "fill_in_time: " << _timing.fill_in_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "point_check_time: " << _timing.point_check_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "line_fitting_time: " << _timing.line_fitting_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "organize_points_time: " << _timing.organize_points_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "pca_time: " << _timing.pca_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "split_edge_time: " << _timing.split_edge_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "pose_optimization_time: " << _timing.pose_optimization_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "store_template_time: " << _timing.store_template_time);

    RCLCPP_DEBUG_STREAM(get_logger(), "payload_decoding_time: " << _timing.payload_decoding_time);
    RCLCPP_DEBUG_STREAM(get_logger(), "computation_time: " << _timing.total_duration);
    RCLCPP_DEBUG_STREAM(get_logger(), "---------------------------------------");
    // cout << "payload_decoding_time: " << _timing.payload_decoding_time <<
    // endl; RCLCPP_DEBUG_STREAM(get_logger(), "PointCheck: " << _timing.point_check_time);
    // RCLCPP_DEBUG_STREAM(get_logger(), "LineFitting: " << _timing.line_fitting_time);
    // //RCLCPP_DEBUG_STREAM(get_logger(), "ExtractPayload: " <<
    // _timing.payload_extraction_time); RCLCPP_DEBUG_STREAM(get_logger(), "NormalVector: " <<
    // _timing.normal_vector_time); RCLCPP_DEBUG_STREAM(get_logger(), "PayloadDecoder: " <<
    //         _timing.payload_decoding_time);
    // RCLCPP_DEBUG_STREAM(get_logger(), "_tagToRobot: " << _timing.tag_to_robot_time);
  }

  return out;
}

/*
 * A function to
 * (1) calculate the depth gradient and
 *     the intensity gradient at a point of a pointcloud
 * (2) group detected 'edge' into different group
 */
void LiDARTag::_gradientAndGroupEdges(
  const std::vector<std::vector<LiDARPoints_t>> & ordered_buff,
  std::vector<std::vector<LiDARPoints_t>> & edge_buff, std::vector<ClusterFamily_t> & cluster_buff)
{
  int n = _num_points_for_plane_feature;
  // clock_t start = clock();
  // TODO: if suddently partial excluded, it will cause errors
  for (int i = _beam_num - 1; i >= 0; --i) {
    int size = ordered_buff[i].size();
    for (int j = 1; j < size - n; j++) {
      // edge_flag:
      // 0 means no edge point,
      // 1 means the left side point is the edge point,
      // 2 means the right side point is the edge point,
      // 3 means two side points are edge points
      int edge_flag = LiDARTag::_getEdgePoints(ordered_buff, i, j, n);
      _edge_flag = edge_flag;
      if (edge_flag == 0) {
        //   std::cout << "edge_flag = 0" << std::endl;
        continue;
      }
      if (edge_flag == 1 || edge_flag == 3) {
        _clusterClassifier(ordered_buff[i][j], cluster_buff);
        const auto & point1 = ordered_buff[i][j].point;
        const auto & Point1L = ordered_buff[i][j - 1].point;
        const auto & Point1R = ordered_buff[i][j + 1].point;
        double DepthGrad1 = std::abs(
          (Point1L.getVector3fMap() - point1.getVector3fMap()).norm() -
          (point1.getVector3fMap() - Point1R.getVector3fMap()).norm());

        // push the detected point that is an edge into a buff
        LiDARPoints_t lidar_points = {ordered_buff[i][j].point, ordered_buff[i][j].index, 1,
                                      DepthGrad1, 0};
        edge_buff[i].push_back(lidar_points);
      }
      if (edge_flag == 2 || edge_flag == 3) {
        _clusterClassifier(ordered_buff[i][j + n - 1], cluster_buff);
        const auto & point2 = ordered_buff[i][j + n - 1].point;
        const auto & Point2L = ordered_buff[i][j + n - 2].point;
        const auto & Point2R = ordered_buff[i][j + n].point;
        double DepthGrad2 = std::abs(
          (Point2L.getVector3fMap() - point2.getVector3fMap()).norm() -
          (point2.getVector3fMap() - Point2R.getVector3fMap()).norm());
        // push the detected point that is an edge into a buff
        LiDARPoints_t lidar_points = {ordered_buff[i][j + n - 1].point,
                                      ordered_buff[i][j + n - 1].index, 1, DepthGrad2, 0};
        edge_buff[i].push_back(lidar_points);
      }
    }
  }
}
// for(int i=_beam_num-1; i >= 0; --i) {
//     int size = ordered_buff[i].size();
// 	for(int j=2; j<size-2; j++) {
// 		const auto& point = ordered_buff[i][j].point;
//         if (std::abs(point.x) > _distance_bound ||
//             std::abs(point.y) > _distance_bound ||
//             std::abs(point.z) > _distance_bound) continue;
// 		const auto& PointL = ordered_buff[i][j-1].point;
// 		const auto& PointR = ordered_buff[i][j+1].point;
// 		double DepthGrad =
// std::max((PointL.getVector3fMap()-point.getVector3fMap()).norm(),
//                                     (point.getVector3fMap()-PointR.getVector3fMap()).norm());

//         // double IntenstityGrad = std::max(std::abs(PointL.intensity -
//         point.intensity),
//         //                                  std::abs(point.intensity -
//         PointR.intensity));

// 		// if (IntenstityGrad > _intensity_threshold &&
//         //     DepthGrad > _depth_threshold) {
// 		if (DepthGrad > _depth_threshold) {
//             // Cluster the detected 'edge' into different groups
//             _clusterClassifier(ordered_buff[i][j], cluster_buff);

//             // push the detected point that is an edge into a buff
//             LiDARPoints_t LiDARPoints = {ordered_buff[i][j].point,
//             ordered_buff[i][j].index,
//                                          DepthGrad, 0};
// 			edge_buff[i].push_back(LiDARPoints);
// 		}
// 	}
// }

// double DepthGrad =
// std::abs((PointL.getVector3fMap()-point.getVector3fMap()).norm()-
//                            (point.getVector3fMap()-PointR.getVector3fMap()).norm());

// Remove all clusters with insufficient points
// pcl::PointCloud<PointXYZRI>::Ptr ClusterEdgePC(new
// pcl::PointCloud<PointXYZRI>); for (int i = 0; i < cluster_buff.size(); ++i)
// {
//    if (cluster_buff[i].data.size() >= std::sqrt(_tag_family)*2)
//    {
//       for (const auto & lidar_point : cluster_buff[i].data)
//         {
//             ClusterEdgePC->push_back(lidar_point.point);
//         }
//     }
// }

// cout << "Number of Total Clusters: " << cluster_buff.size() << endl;

// Publish edge points in large clusters
// LiDARTag::_publishPC(ClusterEdgePC, _pub_frame, string("clusteredgepc"));

// clock_t end = clock();
// cout << "extraction hz: " << 1/ (((double) (end - start))/clocks_per_sec) <<
// endl;
//	}

/* <consecutive n points from ring i index j>
 * A function to
 * (1) kick out points are not near to each other
 * (2) find edge points from two side points of these points
 */
int LiDARTag::_getEdgePoints(
  const std::vector<std::vector<LiDARPoints_t>> & ordered_buff, int i, int j, int n)
{
  const auto & point = ordered_buff[i][j].point;
  if (
    std::abs(point.x) > _distance_bound || std::abs(point.y) > _distance_bound ||
    std::abs(point.z) > _distance_bound)
    return 0;

  // cout << "average on " << i << " ring: " <<
  //     _LiDAR_system.ring_average_table[i].average << endl;

  double point_resolution = 2 * M_PI / _LiDAR_system.ring_average_table[i].average;
  double near_bound =
    std::max(0.1, _nearby_factor * point.getVector3fMap().norm() * std::sin(point_resolution));
  // cout << "near bound: " << near_bound << endl;
  // double near_bound =  _nearby_factor;
  for (int k = 0; k < n - 1; k++) {
    const auto & point1 = ordered_buff[i][j + k].point;
    const auto & point2 = ordered_buff[i][j + k + 1].point;
    double distance = (point1.getVector3fMap() - point2.getVector3fMap()).norm();
    // if (distance > near_bound)
    if (distance > near_bound) return 0;
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
  if (DepthGrad1 > _depth_threshold) {
    if (DepthGrad2 > _depth_threshold) {
      return 3;
    } else {
      return 1;
    }
  } else {
    if (DepthGrad2 > _depth_threshold) {
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
void LiDARTag::_fillInCluster(
  const std::vector<std::vector<LiDARPoints_t>> & ordered_buff,
  std::vector<ClusterFamily_t> & cluster_buff)
{
  std::ofstream fplanefit;
  if (_log_data) {
    std::string path(_outputs_path);
    fplanefit.open(path + "/planeFit.csv");
    if (!fplanefit.is_open()) {
      cout << "Could not open planeFit.csv" << endl;
      exit(0);
    }
  }
  // _debug_cluster.boundary_point.clear();
  _result_statistics.original_cluster_size = cluster_buff.size();
  _result_statistics.remaining_cluster_size = cluster_buff.size();

  // tbb::parallel_for(int(0), (int)cluster_buff.size(), [&](int i) {
  for (int i = 0; i < cluster_buff.size(); ++i) {
    // In a cluster
    // tbb::parallel_for(int(0), (int)cluster_buff.size(), [&](int i) {
    if (_debug_time) {
      _timing.timing = std::chrono::steady_clock::now();
    }
    for (int j = 0; j < _beam_num; ++j) {
      int max_index = cluster_buff[i].max_min_index_of_each_ring[j].max;
      int min_index = cluster_buff[i].max_min_index_of_each_ring[j].min;

      // no points on this ring
      if ((std::abs(min_index - 1e5) < 1e-5) || std::abs(max_index + 1) < 1e-5) {
        continue;
      }

      // A cluster can't cover 180 FoV of a LiDAR!
      // If max and min indices are larger than this,
      // that means the special case happened
      // The special case is when first point of a ring is in this cluster
      // so the indices are not consecutive
      double fill_in_gap = _LiDAR_system.ring_average_table[j].average / 2;
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
          if (_isWithinClusterHorizon(ordered_buff[j][k], cluster_buff[i], 0)) {
            cluster_buff[i].data.push_back(ordered_buff[j][k]);
          }
        }
        cluster_buff[i].special_case = 0;
      } else {
        for (int k = 0; k < min_index; ++k) {
          // cout << "k2: " << k << endl;
          if (_isWithinClusterHorizon(ordered_buff[j][k], cluster_buff[i], 0)) {
            cluster_buff[i].data.push_back(ordered_buff[j][k]);
          }
        }
        for (int k = max_index; k < ordered_buff[j].size(); ++k) {
          // cout << "k3: " << k << endl;
          if (_isWithinClusterHorizon(ordered_buff[j][k], cluster_buff[i], 0)) {
            cluster_buff[i].data.push_back(ordered_buff[j][k]);
          }
        }
        cluster_buff[i].special_case = 1;
      }
    }
    if (_debug_time) {
      _timing.fill_in_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
      _timing.timing = std::chrono::steady_clock::now();
    }

    // Mark cluster as invalid if too few points in cluster
    auto min_returns =
      _min_returns_per_grid * std::pow((std::sqrt(_tag_family) + 2 * _black_border), 2);
    if ((cluster_buff[i].data.size() + cluster_buff[i].edge_points.size()) < min_returns) {
      _result_statistics.cluster_removal.minimum_return++;
      _result_statistics.remaining_cluster_size--;

      if (_mark_cluster_validity) {
        cluster_buff[i].valid = false;
        cluster_buff[i].detail_valid = 1;
      }
      // tbb::task::self().cancel_group_execution();
      continue;
    }

    // Mark cluster as invalid if too many points in cluster
    cluster_buff[i].expected_points = _maxPointsCheck(cluster_buff[i]);
    if (cluster_buff[i].valid == false) {
      // tbb::task::self().cancel_group_execution();
      continue;
    }
    if (_debug_time) {
      _timing.point_check_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
      _timing.timing = std::chrono::steady_clock::now();
    }

    // Mark cluster as invalid if unable to perform plane fitting
    if (!_plane_fitting) {
      continue;
    } else {
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      if (!_rejectWithPlanarCheck(cluster_buff[i], inliers, coefficients, fplanefit)) {
        if (_mark_cluster_validity) {
          cluster_buff[i].valid = false;
          cluster_buff[i].detail_valid = 3;
        }
        // tbb::task::self().cancel_group_execution();
        continue;
      }
      inlier_size = inliers->indices.size();
      // Mark cluster as invalid if too many outliers in plane fitting
      auto percentage_inliers =
        inliers->indices.size() /
        double(cluster_buff[i].data.size() + cluster_buff[i].edge_points.size());
      cluster_buff[i].percentages_inliers = percentage_inliers;

      if (_debug_info) {
        RCLCPP_DEBUG_STREAM(get_logger(), "==== _planeOutliers ====");
        float distance = std::sqrt(
          pow(cluster_buff[i].average.x, 2) + pow(cluster_buff[i].average.y, 2) +
          pow(cluster_buff[i].average.z, 2));
        RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
        RCLCPP_DEBUG_STREAM(get_logger(), 
          "Actual Points: " << cluster_buff[i].data.size() + cluster_buff[i].edge_points.size());
      }

      if (percentage_inliers < (1.0 - _max_outlier_ratio)) {
        if (_debug_info)
          RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
        // tbb::task::self().cancel_group_execution();
        _result_statistics.cluster_removal.plane_outliers++;
        _result_statistics.remaining_cluster_size--;
        if (_mark_cluster_validity) {
          cluster_buff[i].valid = false;
          cluster_buff[i].detail_valid = 4;
          continue;
        }
      }
      if (_debug_info)
        RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << true);

      // Remove all outliers from cluster
      auto outliers_indices = utils::complementOfSet(
        inliers->indices, (cluster_buff[i].data.size() + cluster_buff[i].edge_points.size()));
      int edge_inlier = cluster_buff[i].edge_points.size();
      for (int k = 0; k < outliers_indices.size(); ++k) {
        int out_index = outliers_indices[k];
        if (out_index < cluster_buff[i].edge_points.size()) {
          edge_inlier -= 1;
          cluster_buff[i].edge_points[out_index].valid = 0;
        } else {
          cluster_buff[i].data[out_index - cluster_buff[i].edge_points.size()].valid = 0;
        }
      }
      cluster_buff[i].edge_inliers = edge_inlier;
      if (_debug_time) {
        _timing.plane_fitting_removal_time +=
          utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
        _timing.timing = std::chrono::steady_clock::now();
      }
    }

    // utils::removeIndicesFromVector(
    //     cluster_buff[i].data, outliers_indices
    // );
    // If the points in this cluster after filling are still less than the
    // factored threshold, then remove it
    // _timing.timing = clock();
    // if(!_clusterPointsCheck(cluster_buff[i])){
    //     _timing.point_check_time += utils::spendTime(clock(),
    //     _timing.timing);
    //     //cout << "cluster removed" << endl;
    //     cluster_buff[i].valid = 0;
    //     //cluster_buff.erase(cluster_buff.begin()+i);
    //     // _debug_cluster.point_check.push_back(&cluster_buff[i]);
    //     //i--;
    //     _result_statistics.remaining_cluster_size -= 1;
    //     _result_statistics.cluster_removal.removed_by_point_check ++;
    // }
    // Adaptive thresholding (Maximize and minimize intensity) by comparing
    // with the average value
    // else {
    // _timing.point_check_time += utils::spendTime(clock(), _timing.timing);
    // boost::thread BuffToPclVectorThread(&LiDARTag::AdaptiveThresholding,
    // this, boost::ref(cluster_buff[i]));
    if (!LiDARTag::_adaptiveThresholding(cluster_buff[i])) {
      // removal has been done inside the function
      if (_mark_cluster_validity) {
        cluster_buff[i].valid = false;
      }
      // cluster_buff.erase(cluster_buff.begin()+i);
      // i--;
    } else {
      if (_print_ros_info || _debug_info) {
        RCLCPP_INFO_STREAM(get_logger(), "--ID: " << cluster_buff[i].cluster_id);
        RCLCPP_INFO_STREAM(get_logger(), "---rotation: " <<  cluster_buff[i].rkhs_decoding.rotation_angle);

      }
    }
    // }
  }
  //}, tbb::auto_partitioner());

  // Collect histogram data of cluster
  // ofstream
  // clusterHist("/home/cassie/catkin_ws/src/LiDARTag/output/hist.txt");
  // pcl::PointCloud<PointXYZRI>::Ptr ClusterEdgePC(new
  // pcl::PointCloud<PointXYZRI>); for (int i = 0; i < cluster_buff.size(); ++i)
  // {
  //     if (!cluster_buff[i].valid) {
  //         continue;
  //     }
  //     clusterHist << i << " " << cluster_buff[i].data.size() << endl;

  //     for (const auto & lidar_point : cluster_buff[i].data){
  //         ClusterEdgePC->push_back(lidar_point.point);
  //     }
  // }

  // // Publish edge points in large clusters
  // LiDARTag::_publishPC(ClusterEdgePC, _pub_frame, string("clusteredgepc"));
  if (_log_data) fplanefit.close();
}

/* <A cluster>
 * A function to
 * (1) do adaptive thresholding (Maximize and minimize intensity) by comparing
 *     with the average value and
 * (2) sort points with ring number and re-index with current cluster into
 *     tag_edges vector so as to do regression boundary lines
 * (3) It will *remove* if linefitting fails
 */
bool LiDARTag::_adaptiveThresholding(ClusterFamily_t & cluster)
{
  if (_debug_time) {
    _timing.timing = std::chrono::steady_clock::now();
  }
  _organizeDataPoints(cluster);
  if (_debug_time) {
    _timing.organize_points_time +=
      utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
    _timing.timing = std::chrono::steady_clock::now();
  }

  if (!LiDARTag::_detectPayloadBoundries(cluster)) {
    // removal has been done inside the function
    if (_debug_time) {
      _timing.line_fitting_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
    }
    return false;
  } else {
    // TODO: calculate the average of edge points
    //       substract the average from each edge point and calculate PCA

    // _timing.line_fitting_time += utils::spendTime(clock(), _timing.timing);
    // _timing.timing = clock();
    // _extractPayloadWOThreshold(cluster);
    // _timing.payload_extraction_time += utils::spendTime(clock(),
    // _timing.timing); 2 is because this payload points is actually includes
    // black boundary if (cluster.payload.size() <
    // _min_returns_per_grid*std::pow((std::sqrt(_tag_family)+2*_black_border),
    // 2)) {
    //     //_debug_cluster.ExtractPayload.push_back(&cluster);
    //     _result_statistics.cluster_removal.minimum_return ++;
    //     return false;
    // }

    // return true;
    if (_debug_time) {
      _timing.timing = std::chrono::steady_clock::now();
    }
    _estimatePrincipleAxis(cluster);
    // cout << "after PCA" << endl;
    // for (int eigenpc_index = 0; eigenpc_index < cluster.merged_data.cols();
    // ++eigenpc_index){
    //     if (isnan(cluster.merged_data(0, eigenpc_index)) ||
    //             isnan(cluster.merged_data(1, eigenpc_index)) ||
    //             isnan(cluster.merged_data(2, eigenpc_index)) ||
    //             isnan(cluster.merged_data(3, eigenpc_index))) {
    //         std::cout << "eigenpc_index : " <<
    //             eigenpc_index << std::endl;
    //         std::cout << "cluster merged : " <<
    //             cluster.merged_data.col(eigenpc_index) << endl;
    //     }
    // }
    if (_debug_time) {
      _timing.pca_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
      _timing.timing = std::chrono::steady_clock::now();
    }
    // Fit line of the cluster
    if (!_transformSplitEdges(cluster)) {
      _result_statistics.cluster_removal.line_fitting++;
      _result_statistics.remaining_cluster_size--;
      return false;
    }
    // cout << "after split edge poins" << endl;
    // for (int eigenpc_index = 0; eigenpc_index < cluster.merged_data.cols();
    // ++eigenpc_index){
    //     if (isnan(cluster.merged_data(0, eigenpc_index)) ||
    //             isnan(cluster.merged_data(1, eigenpc_index)) ||
    //             isnan(cluster.merged_data(2, eigenpc_index)) ||
    //             isnan(cluster.merged_data(3, eigenpc_index))) {
    //         std::cout << "eigenpc_index : " <<
    //             eigenpc_index << std::endl;
    //         std::cout << "cluster merged : " <<
    //             cluster.merged_data.col(eigenpc_index) << endl;
    //     }
    // }
    if (_debug_time) {
      _timing.split_edge_time +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
      _timing.timing = std::chrono::steady_clock::now();
    }

    if (!_pose_optimization) {
      cluster.pose_tag_to_lidar.homogeneous = Eigen::Matrix4f::Identity(4, 4);
      _storeTemplatePts(cluster);

      return true;
    } else {
      int status = _optimizePose(cluster);
      cluster.pose_estimation_status = status;
      // ROS_INFO_STREAM("status: " << status << std::endl);
      // ROS_INFO_STREAM("pose status: " << pose_status << std::endl);
      if (status < 0) {
        //   cout << "initial "
        //           "_result_statistics.cluster_removal.pose_optimization
        //           = "
        //        <<
        //        _result_statistics.cluster_removal.pose_optimization
        //        << endl;
        //   cout << "initial "
        //           "_result_statistics.remaining_cluster_size = "
        //        <<   _result_statistics.remaining_cluster_size <<
        //        endl;
        //   _result_statistics.cluster_removal.pose_optimization++;
        //   _result_statistics.remaining_cluster_size--;
        //   cout << "after "
        //           "_result_statistics.cluster_removal.pose_optimization
        //           = "
        //        <<
        //        _result_statistics.cluster_removal.pose_optimization
        //        << endl;
        //   cout << "after "
        //           "_result_statistics.remaining_cluster_size = "
        //        <<   _result_statistics.remaining_cluster_size <<
        //        endl;
        // cout << "about to return false after _optimizePose" <<
        // endl;
        //   cout << "pose optimization rejeted" << std::endl;
        cluster.detail_valid = 13;
        return false;
      } else {
        if (_debug_time) {
          _timing.pose_optimization_time +=
            utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
          _timing.timing = std::chrono::steady_clock::now();
        }

        // cout << "about to store template points" << endl;
        _storeTemplatePts(cluster);
        if (_debug_time) {
          _timing.store_template_time +=
            utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
          _timing.timing = std::chrono::steady_clock::now();
        }

        if (!_id_decoding) {
          _assignClusterPose(cluster.pose_tag_to_lidar, cluster.pose, 0);
          return true;
        } else {
          if (!LiDARTag::_decodePayload(cluster)) {
            _result_statistics.cluster_removal.decoding_failure++;
            _result_statistics.remaining_cluster_size--;
            cluster.detail_valid = 14;
            return false;
          } else {
            if (_debug_time) {
              _timing.payload_decoding_time +=
                utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), _timing.timing);
              _timing.timing = std::chrono::steady_clock::now();
            }
            _assignClusterPose(
              cluster.pose_tag_to_lidar, cluster.pose, cluster.rkhs_decoding.rotation_angle);
            return true;
          }
        }
      }
    }
    // LiDARTag::_decodePayload(cluster);
    //     if (_debug_time) {
    //         _timing.payload_decoding_time += utils::spendTime(clock(),
    //         _timing.timing); _timing.timing = clock();
    //     }
    // _assignClusterPose(
    //         cluster.pose_tag_to_lidar,
    //         cluster.pose,
    //         cluster.rkhs_decoding.rotation_angle);
    // return true;
    // ROS_ERROR_STREAM("Decoding ID is not supported yet. Please change
    // ROS_ERROR_STREAM("Decoding ID is not supported yet. Please change decode_id in the launch file to false. Currently is " << _id_decoding);

    // // under developement
    // if (_debug_time) {
    //     _timing.timing = clock();
    // }
    //
    // if (LiDARTag::_decodePayload(cluster)){
    //     if (_debug_time) {
    //         _timing.payload_decoding_time += utils::spendTime(clock(),
    //         _timing.timing); _timing.timing = clock();
    //     }

    //     _estimatePrincipleAxis(cluster);
    //     if (_debug_time) {
    //         _timing.normal_vector_time += utils::spendTime(clock(),
    //         _timing.timing); _timing.timing = clock();
    //     }

    //     LiDARTag::_tagToRobot(cluster.cluster_id, cluster.normal_vector,
    //                         cluster.pose, cluster.transform,
    //                         cluster.average);
    //     if (_debug_time) {
    //         _timing.tag_to_robot_time += utils::spendTime(clock(),
    //         _timing.timing);
    //     }
    //
    //     return true;
    // }
    // else {
    //     return false;
    // }
    // }
    // else {
    //     if (_debug_time) {
    //         _timing.timing = std::chrono::steady_clock::now();
    //     }
    //     _estimatePrincipleAxis(cluster);
    //     if (_debug_time) {
    //         _timing.pca_time +=
    //             utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(),
    //                     _timing.timing);
    //         _timing.timing = std::chrono::steady_clock::now();
    //     }
    //     if (!_transformSplitEdges(cluster)) return false;
    //     if (_debug_time) {
    //         _timing.split_edge_time +=
    //             utils::spendElapsedTimeMilli(
    //                     std::chrono::steady_clock::now(),
    //                     _timing.timing);
    //         _timing.timing = std::chrono::steady_clock::now();
    //     }
    //
    //     if (_optimizePose(cluster)) {
    //         if (_debug_time) {
    //             _timing.pose_optimization_time +=
    //                 utils::spendElapsedTimeMilli(
    //                         std::chrono::steady_clock::now(),
    //                         _timing.timing);
    //             _timing.timing = std::chrono::steady_clock::now();
    //         }
    //         _storeTemplatePts(cluster);
    //         if (_debug_time) {
    //             _timing.store_template_time +=
    //                 utils::spendElapsedTimeMilli(
    //                         std::chrono::steady_clock::now(),
    //                         _timing.timing);
    //         }
    //     } else {
    //         return false;
    //     }

    //     // under developement
    //     // LiDARTag::_decodePayload(cluster);

    //     // // directly assign ID
    //     // string Code(_assign_id);
    //     // uint64_t Rcode = stoull(Code, nullptr, 2);
    //     // BipedAprilLab::QuickDecodeCodeword(tf, Rcode, &cluster.entry);
    //     // cluster.cluster_id = cluster.entry.id;

    //     // // if (cluster.average.y > 0) cluster.cluster_id = 1;
    //     // // else cluster.cluster_id = 2;

    //     // LiDARTag::_tagToRobot(cluster.cluster_id, cluster.normal_vector,
    //     //                     cluster.pose, cluster.transform,
    //     cluster.average);
    // }
  }
}

/* <A cluster>
 * A function to assign the pose to the cluster using the results of
 * L1-optimization and RKHS decoding results
 */
void LiDARTag::_assignClusterPose(
  const Homogeneous_t & H_TL, Homogeneous_t & H_LT, const int & ang_num)
{
  float rotation_angle = -ang_num * 90;
  Eigen::Vector3f trans_v = Eigen::Vector3f::Zero(3);
  Eigen::Vector3f rot_v;
  rot_v << rotation_angle, 0, 0;
  Eigen::Matrix4f H = utils::computeTransformation(rot_v, trans_v);

  H_LT.homogeneous = (H * H_TL.homogeneous).inverse();
  H_LT.rotation = H_LT.homogeneous.topLeftCorner(3, 3);
  H_LT.translation = H_LT.homogeneous.topRightCorner(3, 1);
  Eigen::Vector3f euler = H_LT.rotation.eulerAngles(0, 1, 2);  // x,y,z convention
  H_LT.roll = euler[0];
  H_LT.pitch = euler[1];
  H_LT.yaw = euler[2];
  // cout << "H_TL\n" << H_TL.homogeneous << endl;
  // cout << "H\n" << H << endl;
  // cout << "H_LT\n" << H_LT.homogeneous << endl;
  // cout << "===" << endl;
}

/* <A cluster>
 * A function to store transformed points
 */
void LiDARTag::_storeTemplatePts(ClusterFamily_t & cluster)
{
  cluster.rkhs_decoding.initial_template_points =
    cluster.initial_pose.homogeneous * cluster.merged_data_h;
  cluster.rkhs_decoding.initial_template_points.bottomRows(1) = cluster.merged_data.bottomRows(1);

  cluster.rkhs_decoding.template_points =
    cluster.pose_tag_to_lidar.homogeneous * cluster.merged_data_h;
  cluster.rkhs_decoding.template_points.bottomRows(1) = cluster.merged_data.bottomRows(1);

  // cluster.rkhs_decoding.template_points.topRows(3) =
  //     cluster.merged_data.topRows(3);

  // cluster.rkhs_decoding.template_points =
  //     (cluster.initial_pose.homogeneous *
  //      cluster.rkhs_decoding.template_points).eval();

  // cluster.rkhs_decoding.template_points.bottomRows(1) =
  //     cluster.merged_data.bottomRows(1);

  // cluster.rkhs_decoding.template_points_xyz =
  //     cluster.rkhs_decoding.template_points.topRows(3);

  // cluster.rkhs_decoding.template_points_feat =
  //     cluster.rkhs_decoding.template_points.bottomRows(1);

  // for (int i=0; i<cluster.data.size(); ++i){
  //     if (cluster.data[i].valid != 1) continue;
  //     Eigen::Vector4f p(cluster.data[i].point.x, cluster.data[i].point.y,
  //     cluster.data[i].point.z, 1); Eigen::Vector4f tp =
  //     cluster.initial_pose.homogeneous * p; Eigen::
  //     // PointXYZRI point;
  //     // point.x = tp[0];
  //     // point.y = tp[1];
  //     // point.z = tp[2];
  //     // point.intensity = cluster.data[i].point.intensity;
  //     // cluster.RLHS_decoding.push_back(point);
  //     //cluster.RLHS_decoding.push_back(cluster.data[i].point);
  //     // cout << "Intensity: " << cluster[Key].payload[i]->point.intensity <<
  //     endl;
  // }
  // for (int i=0; i<cluster.edge_points.size(); ++i){
  //     if (cluster.edge_points[i].valid != 1) continue;
  //     Eigen::Vector4f p(cluster.edge_points[i].point.x,
  //     cluster.edge_points[i].point.y, cluster.edge_points[i].point.z, 1);
  //     Eigen::Vector4f tp = cluster.initial_pose.homogeneous * p;
  //     PointXYZRI point;
  //     point.x = tp[0];
  //     point.y = tp[1];
  //     point.z = tp[2];
  //     point.intensity = cluster[Key].edge_points[i].point.intensity;
  //     cluster.RLHS_decoding.push_back(point);
  //     //cluster.RLHS_decoding.push_back(cluster.edge_points[i].point);
  //     // cout << "Intensity: " << cluster[Key].payload[i]->point.intensity <<
  //     endl;
  // }
}

/* A function to publish pose of tag to the robot
 */
void LiDARTag::_tagToRobot(
  const int & cluster_id, const Eigen::Vector3f & normal_vec, Homogeneous_t & pose,
  tf2::Transform & transform, const PointXYZRI & ave)

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
  // Eigen::Vector3f qr(-0.5, 0.5, -0.5);
  // float qr_w = 0.5;

  // Eigen::Vector3f qr(-0.5, -0.5, -0.5);
  // float qr_w = 0.5;
  // Eigen::Vector3f qr(std::sqrt(2)/2, 0, 0);
  // float qr_w = std::sqrt(2)/2;
  // Eigen::Vector3f qi_camera_frame = qr_w*normal_vec + 0*qr +
  // qr.cross(normal_vec); // 0 is q.w of normalvec float qw_camera_frame =
  // qr_w*0 - qr.dot(normal_vec); // 0 is q.w of normalvec
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
  transform_msg.header.stamp = _point_cloud_header.stamp;
  transform_msg.header.frame_id = _pub_frame;
  transform_msg.child_frame_id = to_string(cluster_id)+"_rotated";
  transform_msg.transform = tf2::toMsg(transform);

  broadcaster_.sendTransform(transform_msg);


  tf2::Quaternion q2(normal_vec(0), normal_vec(1), normal_vec(2), 0);
  transform.setRotation(q2);
  transform_msg.header.stamp = _point_cloud_header.stamp;
  transform_msg.header.frame_id = _pub_frame;
  transform_msg.child_frame_id = "LiDARTag-ID" + to_string(cluster_id);
  transform_msg.transform = tf2::toMsg(transform);

  broadcaster_.sendTransform(transform_msg);

  // publish lidar tag pose
  lidartag_msgs::msg::LidarTagDetection lidartag_msg; //single message
  lidartag_msg.id = cluster_id;
  lidartag_msg.size = _payload_size;
  geometry_msgs::msg::Quaternion geo_q;
  geo_q.x = q_i(0);
  geo_q.y = q_i(1);
  geo_q.z = q_i(2);
  geo_q.w = q_w;
  // cout << "R: \n" << pose.rotation << endl;
  // cout << "det(R): \n" << pose.rotation.determinant() << endl;
  // cout << "q: " << q_i(0) << ", "
  //               << q_i(1) << ", "
  //               << q_i(2) << ", "
  //               << q_w << endl;
  lidartag_msg.pose.position.x = ave.x;
  lidartag_msg.pose.position.y = ave.y;
  lidartag_msg.pose.position.z = ave.z;
  lidartag_msg.pose.orientation = geo_q;
  lidartag_msg.header = _point_cloud_header;
  lidartag_msg.header.frame_id = std::string("lidartag_") + to_string(cluster_id);
  lidartag_msg.frame_index = 0/* _point_cloud_header.seq  deprecated */;
  _lidartag_pose_array.header = _point_cloud_header;
  _lidartag_pose_array.frame_index = 0/* _point_cloud_header.seq deprecated */;
  _lidartag_pose_array.detections.push_back(lidartag_msg);
  // cout << "R.T*NV: " << endl << pose.rotation.transpose()*normal_vec << endl;
  // cout << "H: " << endl << pose.homogeneous << endl;

  /*
  Eigen::Vector3f x(1, 0, 0);
  Eigen::Vector3f y(0, 1, 0);
  Eigen::Vector3f z(0, 0, 1);
  Eigen::Matrix3f zSkew;
  zSkew << 0, -z(2), z(1),
           z(2), 0, -z(0),
          -z(1), z(0), 0;
  Eigen::Vector3f u = zSkew*normal_vec;
  //u = x.cross(normal_vec);
  //u = z.cross(normal_vec);
  //u = -z.cross(normal_vec);
  //u = -x.cross(normal_vec);
  //u = -y.cross(normal_vec);
  //u = x.cross(normal_vec);
  u = (u.normalized()).eval();
  float theta = acos(z.dot(normal_vec));
  u = (u*theta).eval();
  Eigen::Matrix3f uSkew;
  uSkew << 0, -u(2), u(1),
          u(2), 0, -u(0),
         -u(1), u(0), 0;
  pose.rotation = uSkew.exp();
  pose.translation << ave.x, ave.y, ave.z;
  pose.yaw = utils::rad2Deg(acos(normal_vec.dot(y)));
  pose.pitch = -utils::rad2Deg(acos(normal_vec.dot(x)));
  pose.roll = utils::rad2Deg(acos(normal_vec.dot(z)));
  pose.homogeneous.topLeftCorner(3,3) = pose.rotation;
  pose.homogeneous.topRightCorner(3,1) = pose.translation;
  pose.homogeneous.row(3) << 0,0,0,1;
  
  static tf::TransformBroadcaster broadcaster_;
  transform.setOrigin(tf::Vector3(ave.x, ave.y, ave.z)); Eigen::Vector3f q_i =
  sin(theta/2)*u; double q_w = std::cos(theta/2); double norm =
  std::sqrt(std::pow(q_i(0), 2) + std::pow(q_i(1), 2) + std::pow(q_i(2), 2) +
  std::pow(q_w, 2)); q_i = (q_i/norm).eval(); q_w = q_w/norm; tf::Quaternion
  q(q_i(0), q_i(1), q_i(2), q_w); transform.setRotation(q);
  broadcaster_.sendTransform(tf::StampedTransform(transform,
  _point_cloud_header.stamp, _pub_frame, to_string(cluster_id)));
  // publish lidar tag pose
  lidartag_msgs::LiDARTagDetection lidartag_msg; //single message
  lidartag_msg.id = cluster_id;
  lidartag_msg.size = _payload_size;
  
  geometry_msgs::Quaternion geo_q;
  geo_q.x = q_i(0);
  geo_q.y = q_i(1);
  geo_q.z = q_i(2);
  geo_q.w = q_w;
  // cout << "R: \n" << pose.rotation << endl;
  // cout << "det(R): \n" << pose.rotation.determinant() << endl;
  // cout << "q: " << q_i(0) << ", "
  //               << q_i(1) << ", "
  //               << q_i(2) << ", "
  //               << q_w << endl;
  lidartag_msg.pose.position.x = ave.x;
  lidartag_msg.pose.position.y = ave.y;
  lidartag_msg.pose.position.z = ave.z;
  lidartag_msg.pose.orientation = geo_q;
  lidartag_msg.header = _point_cloud_header;
  lidartag_msg.header.frame_id = std::string("lidartag_") +
  to_string(cluster_id); _lidartag_pose_array.header = _point_cloud_header;
  _lidartag_pose_array.detections.push_back(lidartag_msg);
  // cout << "R.T*NV: " << endl << pose.rotation.transpose()*normal_vec << endl;
  // cout << "H: " << endl << pose.homogeneous << endl;
  */
}

/* <A cluster>
 * 1. Convert datatype from PCL to Eigen type for pose optimization.
 * 2. Merge edge points and data points into merged_data and merged_data_h.
 * 3. Compute mean of the cluster, including the mean of intensity
 * 4. Organize "data" points only for boundry detection. When detecting
 *    boundaries, we don't care about PoI.
 */
void LiDARTag::_organizeDataPoints(ClusterFamily_t & cluster)
{
  cluster.ordered_points_ptr.resize(_beam_num);

  cluster.merged_data.resize(4, cluster.inliers);
  cluster.merged_data_h.resize(4, cluster.inliers);
  int eigenpc_index = 0;
  Eigen::Vector4f cur_vec;
  float x_mean = 0;
  float y_mean = 0;
  float z_mean = 0;
  float i_mean = 0;
  for (int i = 0; i < cluster.edge_points.size(); ++i) {
    if (cluster.edge_points[i].valid != 1) continue;
    _PointXYZRIToEigenVector(cluster.edge_points[i].point, cur_vec);
    cluster.merged_data.col(eigenpc_index) = cur_vec;
    cluster.merged_data_h.col(eigenpc_index) << cur_vec(0), cur_vec(1), cur_vec(2), 1;
    x_mean += cur_vec[0];
    y_mean += cur_vec[1];
    z_mean += cur_vec[2];
    i_mean += cur_vec[3];
    eigenpc_index += 1;

    // LiDARPoints_t* ClusterPointPtr = &cluster.edge_points[i];
    // ClusterPointPtr->point.intensity =
    //     cluster.edge_points[i].point.intensity /
    //     cluster.max_intensity.intensity;
    // cluster.ordered_points_ptr[
    //     ClusterPointPtr->point.ring].push_back(ClusterPointPtr);

    // if (isnan(cluster.merged_data(0, eigenpc_index)) ||
    //     isnan(cluster.merged_data(1, eigenpc_index)) ||
    //     isnan(cluster.merged_data(2, eigenpc_index)) ||
    //     isnan(cluster.merged_data(3, eigenpc_index))) {
    //     std::cout << "vector : " << cur_vec << std::endl;
    //     std::cout << "edge points : " << endl;
    //     utils::COUT(cluster.edge_points[i].point);
    //     std::cout << "eigenpc_index : " << eigenpc_index << std::endl;
    // }
  }
  for (int i = 0; i < cluster.data.size(); ++i) {
    if (cluster.data[i].valid != 1) continue;
    _PointXYZRIToEigenVector(cluster.data[i].point, cur_vec);
    cluster.merged_data.col(eigenpc_index) = cur_vec;
    cluster.merged_data_h.col(eigenpc_index) << cur_vec(0), cur_vec(1), cur_vec(2), 1;
    x_mean += cur_vec[0];
    y_mean += cur_vec[1];
    z_mean += cur_vec[2];
    i_mean += cur_vec[3];
    eigenpc_index += 1;

    LiDARPoints_t * ClusterPointPtr = &cluster.data[i];
    // ClusterPointPtr->point.intensity =
    //     cluster.data[i].point.intensity /
    //     cluster.max_intensity.intensity;
    cluster.ordered_points_ptr[ClusterPointPtr->point.ring].push_back(ClusterPointPtr);

    // if (isnan(cluster.merged_data(0, eigenpc_index)) ||
    //     isnan(cluster.merged_data(1, eigenpc_index)) ||
    //     isnan(cluster.merged_data(2, eigenpc_index)) ||
    //     isnan(cluster.merged_data(3, eigenpc_index))) {
    //     std::cout << "vector : " << cur_vec << std::endl;
    //     std::cout << "edge points : " << std::endl;
    //     utils::COUT(cluster.data[i].point);
    //     std::cout << "eigenpc_index : " << eigenpc_index << std::endl;
    // }
  }
  // if (eigenpc_index != cluster.inliers) {
  //     cout << "======================" << endl;
  //     cout << "number is wrong" << endl;
  // }
  cluster.average.x = x_mean / cluster.inliers;
  cluster.average.y = y_mean / cluster.inliers;
  cluster.average.z = z_mean / cluster.inliers;
  cluster.average.intensity = i_mean / cluster.inliers;
  cluster.rkhs_decoding.num_points = cluster.inliers;
  cluster.rkhs_decoding.ave_intensity = cluster.average.intensity;

  for (int ring = 0; ring < _beam_num; ++ring) {
    sort(
      cluster.ordered_points_ptr[ring].begin(), cluster.ordered_points_ptr[ring].end(),
      utils::compareIndex);
  }

  // cluster.average.x = cluster.merged_data.row(0).mean();
  // cluster.average.y = cluster.merged_data.row(1).mean();
  // cluster.average.z = cluster.merged_data.row(2).mean();
  // cluster.average.intensity = cluster.merged_data.row(3).mean();

  // cout << "Ave1:" << cluster.average.x << ", "<< cluster.average.y
  //         << ", "<< cluster.average.z << endl;

  // PointXYZRI average{0,0,0,0};
  // for (int k = 0; k < cluster.edge_points.size(); ++k) {
  //     if (cluster.edge_points[k].valid != 1) continue;
  //     average.x += cluster.edge_points[k].point.x;
  //     average.y += cluster.edge_points[k].point.y;
  //     average.z += cluster.edge_points[k].point.z;
  //     average.intensity += cluster.edge_points[k].point.intensity;
  // }
  // for (int k = 0; k < cluster.data.size(); ++k) {
  //     if (cluster.data[k].valid != 1) continue;
  //     average.x += cluster.data[k].point.x;
  //     average.y += cluster.data[k].point.y;
  //     average.z += cluster.data[k].point.z;
  //     average.intensity += cluster.data[k].point.intensity;
  // }
  // cluster.average.x = average.x/ cluster.inliers;
  // cluster.average.y = average.y/ cluster.inliers;
  // cluster.average.z = average.z/ cluster.inliers;
  // cluster.average.intensity = average.intensity/ cluster.inliers;

  // cout << "Ave2:" << cluster.average.x << ", "<< cluster.average.y
  //         << ", "<< cluster.average.z << endl;
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
void LiDARTag::_extractPayloadWOThreshold(ClusterFamily_t & cluster)
{
  int last_round_length = 0;  // Save to recover a missing ring
  PointXYZRI average{0, 0, 0, 0};
  for (int ring = 0; ring < _beam_num; ++ring) {
    // if (cluster.payload_right_boundary_ptr[ring]!=0)
    //     cluster.payload_boundary_ptr.push_back(cluster.payload_right_boundary_ptr[ring]);

    // if (cluster.payload_left_boundary_ptr[ring]!=0)
    //     cluster.payload_boundary_ptr.push_back(cluster.payload_left_boundary_ptr[ring]);

    if (
      cluster.payload_right_boundary_ptr[ring] == 0 && cluster.payload_left_boundary_ptr[ring] == 0)
      continue;
    // cout << "ring" << ring << endl;
    else if (
      cluster.payload_right_boundary_ptr[ring] != 0 &&
      cluster.payload_left_boundary_ptr[ring] != 0) {
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
            if (k >= cluster.ordered_points_ptr[ring].size())
              break;  // make sure the index is valid
            // cout << "j: " << j << endl;
            // cout << "k: " << k << endl;
            // cout << "validation1: " << endl;
            // utils::COUT(cluster.ordered_points_ptr[ring][k]->point);
            //
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
    // else if (last_round_length!=0 &&
    // cluster.payload_right_boundary_ptr[ring]!=0){
    //     int EndIndex = cluster.payload_right_boundary_ptr[ring]->index;

    //     for (int j=cluster.ordered_points_ptr[ring].size()-1; j>0; --j){
    //         if (cluster.ordered_points_ptr[ring][j]->index == EndIndex){
    //             cluster.payload.push_back(cluster.ordered_points_ptr[ring][j]);

    //             for (int k=j-1; k>j-last_round_length; --k){
    //                 if (k<0) break; // make sure the index is valid
    //                 cluster.payload.push_back(cluster.ordered_points_ptr[ring][k]);
    //             }
    //             break;
    //         }
    //     }

    // }
    // else if (last_round_length!=0 &&
    // cluster.payload_left_boundary_ptr[ring]!=0){
    //     int StartIndex = cluster.payload_left_boundary_ptr[ring]->index;

    //     for (int j=0; j<cluster.ordered_points_ptr[ring].size(); ++j){
    //         if (cluster.ordered_points_ptr[ring][j]->index == StartIndex){
    //             cluster.payload.push_back(cluster.ordered_points_ptr[ring][j]);

    //             for (int k=j-1; k<j+last_round_length; ++k){ // since start
    //             from 0
    //                 if (k>=cluster.ordered_points_ptr[ring].size()) break; //
    //                 make sure the index is valid
    //                 cluster.payload.push_back(cluster.ordered_points_ptr[ring][k]);
    //             }
    //             break;
    //         }
    //     }

    // }
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
bool LiDARTag::_detectPayloadBoundries(ClusterFamily_t & cluster)
{
  cluster.payload_right_boundary_ptr.resize(_beam_num);
  cluster.payload_left_boundary_ptr.resize(_beam_num);
  bool boundary_flag = true;
  bool ring_point_flag = true;

  // Initialization
  cluster.tag_edges.upper_ring = _beam_num;
  cluster.tag_edges.lower_ring = 0;

  double detection_threshold = _payload_intensity_threshold * cluster.average.intensity / 2;
  // double detection_threshold =
  //     (cluster.average.intensity - cluster.min_intensity.intensity)/
  //     (_payload_intensity_threshold*cluster.max_intensity.intensity);

  int boundary_piont_count = 0;  // Check there're sufficient points
  int num_valid_rings = 0;

  bool found_right_ring_boundary;
  bool found_left_ring_boundary;

  // a ring should at least have three points to have intensity gradients
  // from left to right and from right to right
  int minimum_ring_points = 5;

  for (int ring = 0; ring < _beam_num; ++ring) {
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
         detection_threshold)) {
        // if ((cluster.ordered_points_ptr[ring][P]->point.intensity -
        //      cluster.ordered_points_ptr[ring][P+1]->point.intensity >
        //      detection_threshold)) {
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
    for (int P = cluster.ordered_points_ptr[ring].size() - 2;
         P > floor((cluster.ordered_points_ptr[ring].size() - 1) / 2); --P) {
      // (1) By knowing it from white to black on the left to calucate the
      // intensity gradient
      // (2) Since have thresholded already, we could also choose > 255
      // cluster.payload_right_boundary_ptr[ring] =
      // cluster.ordered_points_ptr[ring][P];
      if (
        (cluster.ordered_points_ptr[ring][P]->point.intensity -
           cluster.ordered_points_ptr[ring][P - 1]->point.intensity >
         detection_threshold) &&
        (cluster.ordered_points_ptr[ring][P + 1]->point.intensity -
           cluster.ordered_points_ptr[ring][P - 1]->point.intensity >
         detection_threshold)) {
        // if ((cluster.ordered_points_ptr[ring][P]->point.intensity -
        //      cluster.ordered_points_ptr[ring][P-1]->point.intensity >
        //      detection_threshold)) {

        cluster.payload_right_boundary_ptr[ring] = cluster.ordered_points_ptr[ring][P];
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
  // if (boundary_piont_count < int(sqrt(_tag_family))*2 ||
  //     num_valid_rings < int(sqrt(_tag_family))) {
  if (boundary_piont_count < int(sqrt(_tag_family)) * 2) {
    _result_statistics.cluster_removal.boundary_point_check++;
    _result_statistics.remaining_cluster_size--;
    boundary_flag = false;
    cluster.detail_valid = 5;
  } else if (num_valid_rings < std::min(int(sqrt(_tag_family)), _minimum_ring_boundary_points)) {
    _result_statistics.cluster_removal.minimum_ring_points++;
    _result_statistics.remaining_cluster_size--;
    ring_point_flag = false;
    cluster.detail_valid = 6;
  }

  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== _detectPayloadBoundries ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "Boundary threshold : " << detection_threshold);
    RCLCPP_DEBUG_STREAM(get_logger(), "Boundary_piont_count : " << boundary_piont_count);
    RCLCPP_DEBUG_STREAM(get_logger(), "Num_valid_rings: " << num_valid_rings);
    RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << boundary_flag && ring_point_flag);
  }

  return boundary_flag && ring_point_flag;
}

Homogeneous_t LiDARTag::_estimatePose(ClusterFamily_t & cluster)
{
  Homogeneous_t pose;
  // translation  min sign
  pose.translation << -cluster.average.x, -cluster.average.y, -cluster.average.z;

  // rotation//
  Eigen::Vector3f x_axis(1, 0, 0);
  Eigen::Vector3f edge_direction(0, 0, 1);
  Eigen::Vector3f base_vector1 = utils::cross_product(x_axis, edge_direction);

  Eigen::Vector3f normal_vector = cluster.normal_vector;
  Eigen::Vector3f edge_vector = _estimateEdgeVector(cluster);
  Eigen::Vector3f base_vector2 = utils::cross_product(normal_vector, edge_vector);

  Eigen::Matrix3f V, W;
  V.col(0) = normal_vector;
  V.col(1) = edge_vector;
  V.col(2) = base_vector2;
  W.col(0) = x_axis;
  W.col(1) = edge_direction;
  W.col(2) = base_vector1;

  pose.rotation = W * V.transpose();

  // float cos_theta = utils::dot_product(x_axis, normal_vector);
  // float rotation_angle = std::acos(cos_theta);
  // Eigen::Vector3f rotation_axis = utils::cross_product(normal_vector,
  // x_axis); Eigen::AngleAxisf rotation_vector (rotation_angle, rotation_axis);

  // pose.rotation = rotation_vector.toRotationMatrix();

  // euler angles
  Eigen::Vector3f euler_angles = pose.rotation.eulerAngles(2, 1, 0);
  pose.roll = euler_angles[2];
  pose.pitch = euler_angles[1];
  pose.yaw = euler_angles[0];
  //
  pose.homogeneous.topLeftCorner(3, 3) = pose.rotation;
  pose.homogeneous.topRightCorner(3, 1) = pose.translation;
  pose.homogeneous.row(3) << 0, 0, 0, 1;
  if (_debug_info) {
    std::cout << "=============================================================" << std::endl;
    std::cout << "estimate euler angle: \n"
              << pose.roll * 180 / M_PI << "  " << pose.pitch * 180 / M_PI << "  "
              << pose.yaw * 180 / M_PI << std::endl;
    std::cout << "estimate transformation: \n" << pose.homogeneous << std::endl;
  }

  return pose;
}

Eigen::Vector3f LiDARTag::_estimateEdgeVector(ClusterFamily_t & cluster)
{
  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== _estimateEdgeVector ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // cloud->points.resize(_beam_num);
  for (std::size_t i = 0; i < _beam_num; ++i) {
    if (!cluster.payload_left_boundary_ptr[i]) continue;
    pcl::PointXYZ p(
      cluster.payload_left_boundary_ptr[i]->point.x, cluster.payload_left_boundary_ptr[i]->point.y,
      cluster.payload_left_boundary_ptr[i]->point.z);
    cloud->points.push_back(p);
  }
  if (_debug_info) {
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
  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Inliers size " << inliers->indices.size ());
  }

  if (inliers->indices.size() == 0) {
    if (_debug_info) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not estimate a LINE model for the given dataset.");
    }
  }

  Eigen::Vector3f edge_vector(
    coefficients->values[3], coefficients->values[4], coefficients->values[5]);
  edge_vector.normalize();
  if (edge_vector(2) < 0) edge_vector = -edge_vector;
  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Edge vector: " << edge_vector.transpose());
  }

  _visualizeVector(edge_vector, cluster.average, 0);
  return edge_vector;
}

/* [Edge points and principal axes]
 * A function to transform the edge points to the tag frame and split into 4
 * groups for each group of points, do line fitting and get four corner points
 * from intersection of lines estimate the tagsize according to the mean of
 * distance between consecutive two corner points get the initial pose by
 * associating four corner points with the corners of template
 * TODO: make this modular
 */
bool LiDARTag::_transformSplitEdges(ClusterFamily_t & cluster)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<PointXYZRI>::Ptr TransformedPC(new pcl::PointCloud<PointXYZRI>);
  TransformedPC->reserve(_point_cloud_size);
  TransformedPC->clear();

  pcl::PointCloud<PointXYZRI>::Ptr TransformedPCTag(new pcl::PointCloud<PointXYZRI>);
  TransformedPCTag->reserve(_point_cloud_size);
  TransformedPCTag->clear();

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_edge_pc(new pcl::PointCloud<pcl::PointXYZ>);
  transformed_edge_pc->reserve(_point_cloud_size);
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
  before_transformed_edge_pc_msg.header = _point_cloud_header;
  before_transformed_edge_pc_msg.header.frame_id = _pub_frame;
  _before_transformed_edge_pc_pub->publish(before_transformed_edge_pc_msg);

  for (int i = 0; i < cluster.edge_points.size(); ++i) {
    if (cluster.edge_points[i].valid != 1) continue;
    Eigen::Vector3f edge_point(
      cluster.edge_points[i].point.x - cluster.average.x,
      cluster.edge_points[i].point.y - cluster.average.y,
      cluster.edge_points[i].point.z - cluster.average.z);
    Eigen::Matrix<float, 3, 3, Eigen::DontAlign> transform_matrix = cluster.principal_axes;
    // cluster.principal_axes << -0.866, 0.180, -0.466, -0.492, -0.130, 0.861,
    //     0.095, 0.975, 0.201;
    // Eigen::Matrix<float, 3, 3, Eigen::DontAlign> transform_matrix;
    // transform_matrix = cluster.principal_axes;

    transform_matrix = (transform_matrix.transpose()).eval();

    Eigen::Vector3f transformed_edge_point = transform_matrix * edge_point;
    pcl::PointXYZ p;
    p.x = transformed_edge_point(0);
    p.y = transformed_edge_point(1);
    p.z = 0;
    transformed_edge_pc->push_back(p);
    LiDARPoints_t group_point;
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

  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== _transformSplitEdges ====");
    float distance = std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
  }
  int num_edge_points = 3;
  if (
    cloud1->size() < num_edge_points || cloud2->size() < num_edge_points ||
    cloud3->size() < num_edge_points || cloud4->size() < num_edge_points) {
    
    if (_debug_info) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }
    
    cluster.detail_valid = 7;
    return false;
  }

  // sensor_msgs::msg::PointCloud2 cloud1_msg;
  // sensor_msgs::msg::PointCloud2 cloud2_msg;
  // sensor_msgs::msg::PointCloud2 cloud3_msg;
  // sensor_msgs::msg::PointCloud2 cloud4_msg;
  // pcl::toROSMsg(*cloud1, cloud1_msg);
  // pcl::toROSMsg(*cloud2, cloud2_msg);
  // pcl::toROSMsg(*cloud3, cloud3_msg);
  // pcl::toROSMsg(*cloud4, cloud4_msg);
  // cloud1_msg.header = _point_cloud_header;
  // cloud2_msg.header = _point_cloud_header;
  // cloud3_msg.header = _point_cloud_header;
  // cloud4_msg.header = _point_cloud_header;
  // cloud1_msg.header.frame_id = _pub_frame;
  // cloud2_msg.header.frame_id = _pub_frame;
  // cloud3_msg.header.frame_id = _pub_frame;
  // cloud4_msg.header.frame_id = _pub_frame;
  // _cloud1_pub->publish(cloud1_msg);
  // _cloud2_pub->publish(cloud2_msg);
  // _cloud3_pub->publish(cloud3_msg);
  // _cloud4_pub->publish(cloud4_msg);
  // //visualize all transformed points
  // LiDARTag::_publishPC(TransformedPC, _pub_frame, string("transpts"));

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
  line_cloud1->reserve(_point_cloud_size);
  line_cloud1->clear();
  line_cloud2->reserve(_point_cloud_size);
  line_cloud2->clear();
  line_cloud3->reserve(_point_cloud_size);
  line_cloud3->clear();
  line_cloud4->reserve(_point_cloud_size);
  line_cloud4->clear();
  // sensor_msgs::msg::PointCloud2 line_cloud1_msg;
  // sensor_msgs::msg::PointCloud2 line_cloud2_msg;
  // sensor_msgs::msg::PointCloud2 line_cloud3_msg;
  // sensor_msgs::msg::PointCloud2 line_cloud4_msg;

  if (!LiDARTag::_getLines(cloud1, line1, line_cloud1)) {
    if (_debug_info) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = 8;
    return false;
  }
  if (!LiDARTag::_getLines(cloud2, line2, line_cloud2)) {
    if (_debug_info) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = 9;
    return false;
  }
  if (!LiDARTag::_getLines(cloud3, line3, line_cloud3)) {
    if (_debug_info) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = 10;
    return false;
  }
  if (!LiDARTag::_getLines(cloud4, line4, line_cloud4)) {
    if (_debug_info) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);
    }

    cluster.detail_valid = 11;
    return false;
  }

  // get intersections of four sides
  Eigen::Vector3f intersection1 = _getintersec(line1, line2);
  Eigen::Vector3f intersection2 = _getintersec(line2, line3);
  Eigen::Vector3f intersection3 = _getintersec(line3, line4);
  Eigen::Vector3f intersection4 = _getintersec(line1, line4);
  if (!_estimateTargetSize(cluster, intersection1, intersection2, intersection3, intersection4)) {
    cluster.detail_valid = 12;
    return false;
  }
  // pcl::toROSMsg(*line_cloud1, line_cloud1_msg);
  // pcl::toROSMsg(*line_cloud2, line_cloud2_msg);
  // pcl::toROSMsg(*line_cloud3, line_cloud3_msg);
  // pcl::toROSMsg(*line_cloud4, line_cloud4_msg);
  // line_cloud1_msg.header = _point_cloud_header;
  // line_cloud2_msg.header = _point_cloud_header;
  // line_cloud3_msg.header = _point_cloud_header;
  // line_cloud4_msg.header = _point_cloud_header;
  // line_cloud1_msg.header.frame_id = _pub_frame;
  // line_cloud2_msg.header.frame_id = _pub_frame;
  // line_cloud3_msg.header.frame_id = _pub_frame;
  // line_cloud4_msg.header.frame_id = _pub_frame;
  // _line_cloud1_pub->publish(line_cloud1_msg);
  // _line_cloud2_pub->publish(line_cloud2_msg);
  // _line_cloud3_pub->publish(line_cloud3_msg);
  // _line_cloud4_pub->publish(line_cloud4_msg);

  sensor_msgs::msg::PointCloud2 transformed_edge_pc_msg;
  pcl::toROSMsg(*transformed_edge_pc, transformed_edge_pc_msg);
  transformed_edge_pc_msg.header = _point_cloud_header;
  transformed_edge_pc_msg.header.frame_id = _pub_frame;
  _transformed_edge_pc_pub->publish(transformed_edge_pc_msg);

  std::vector<Eigen::VectorXf> intersection_list{intersection1, intersection2, intersection3,
                                                 intersection4};
  publishIntersections(intersection_list);
  _intersection1 = intersection1;
  _intersection2 = intersection2;
  _intersection3 = intersection3;
  _intersection4 = intersection4;
  // associate four intersections with four coners of the template
  Eigen::MatrixXf payload_vertices(3, 4);
  payload_vertices.col(0) = cluster.principal_axes * intersection1;
  payload_vertices.col(1) = cluster.principal_axes * intersection2;
  payload_vertices.col(2) = cluster.principal_axes * intersection3;
  payload_vertices.col(3) = cluster.principal_axes * intersection4;
  //   payload_vertices << -0.0151639, -0.629135, 0.0127609, 0.624599,
  //   -0.178287,
  //       -0.325841, 0.167202, 0.319495, 0.728342, -0.0662743, -0.686448,
  //       0.0829155;

  Eigen::MatrixXf ordered_payload_vertices = _getOrderedCorners(payload_vertices, cluster);
  Eigen::MatrixXf Vertices = Eigen::MatrixXf::Zero(3, 5);
  //   ordered_payload_vertices << -0.0151639, -0.629135, 0.0127609, 0.624599,
  //       -0.178287, -0.325841, 0.167202, 0.319495, 0.728342, -0.0662743,
  //       -0.686448, 0.0829155;
  // Vertices << 0, 0, 0, 0, 0,
  //             0, 0.515, 0.515, -0.515, -0.515,
  //             0, 0.515, -0.515, -0.515, 0.515;
  utils::formGrid(Vertices, 0, 0, 0, cluster.tag_size);
  Eigen::Matrix3f R;
  std::vector<Eigen::MatrixXf> mats;
  mats = utils::fitGrid_new(Vertices, R, ordered_payload_vertices);
  _U = mats.front();
  mats.erase(mats.begin());
  _V = mats.front();
  mats.erase(mats.begin());
  _r = mats.front();
  mats.erase(mats.begin());
  mats.clear();
  // R << -0.467, 0.861, 0.201
  //    , -0.632, -0.484, 0.606
  //     , 0.619, 0.156, 0.767;
  _payload_vertices = ordered_payload_vertices;
  _Vertices = Vertices;
  _ordered_payload_vertices = ordered_payload_vertices;
  _R = R;
  // used for visualization for corner points
  PointXYZRI showpoint;
  PointXYZRI showpoint_tag;
  Eigen::MatrixXf::Index col;

  // Eigen::IOFormat mat_format(Eigen::StreamPrecision, 0, ", ", ";\n", "", "",
  //                            "[", "]");
  // std::cout << "ordered payload vertices = " << std::endl;
  // std::cout << ordered_payload_vertices.format(mat_format) << std::endl;

  ordered_payload_vertices.row(1).minCoeff(&col);
  utils::eigen2Corners(ordered_payload_vertices.col(col), _tag_corners.right);

  ordered_payload_vertices.row(1).maxCoeff(&col);
  utils::eigen2Corners(ordered_payload_vertices.col(col), _tag_corners.left);

  ordered_payload_vertices.row(2).minCoeff(&col);
  utils::eigen2Corners(ordered_payload_vertices.col(col), _tag_corners.down);

  ordered_payload_vertices.row(2).maxCoeff(&col);
  utils::eigen2Corners(ordered_payload_vertices.col(col), _tag_corners.top);

  for (int i = 0; i < 4; ++i) {
    showpoint.intensity = 50;
    showpoint.x = ordered_payload_vertices.col(i)(0);
    showpoint.y = ordered_payload_vertices.col(i)(1);
    showpoint.z = ordered_payload_vertices.col(i)(2);

    showpoint_tag.x = showpoint.x + cluster.average.x;
    showpoint_tag.y = showpoint.y + cluster.average.y;
    showpoint_tag.z = showpoint.z + cluster.average.z;
    TransformedPC->push_back(showpoint);
    TransformedPCTag->push_back(showpoint_tag);
  }
  // if (!_debug_info) {
  //     showpoint.intensity = 50;
  //     showpoint.x = ordered_payload_vertices.col(3)(0);
  //     showpoint.y = ordered_payload_vertices.col(3)(1);
  //     showpoint.z = ordered_payload_vertices.col(3)(2);

  //     showpoint_tag.x = showpoint.x + cluster.average.x;
  //     showpoint_tag.y = showpoint.y + cluster.average.y;
  //     showpoint_tag.z = showpoint.z + cluster.average.z;
  //     TransformedPC->push_back(showpoint);
  //     TransformedPCTag->push_back(showpoint_tag);
  // }
  LiDARTag::_publishPC(TransformedPC, _pub_frame, string("transpts"));
  LiDARTag::_publishPC(TransformedPCTag, _pub_frame, string("transptstag"));

  // save initial lidar to tag pose matrix
  cluster.initial_pose.rotation = R;
  cluster.initial_pose.translation << -cluster.average.x, -cluster.average.y, -cluster.average.z;
  cluster.initial_pose.translation = R * cluster.initial_pose.translation;
  Eigen::Vector3f euler_angles = cluster.initial_pose.rotation.eulerAngles(0, 1, 2);
  cluster.initial_pose.roll = euler_angles[0];
  cluster.initial_pose.pitch = euler_angles[1];
  cluster.initial_pose.yaw = euler_angles[2];
  cluster.initial_pose.homogeneous.topLeftCorner(3, 3) = cluster.initial_pose.rotation;
  cluster.initial_pose.homogeneous.topRightCorner(3, 1) = cluster.initial_pose.translation;
  cluster.initial_pose.homogeneous.row(3) << 0, 0, 0, 1;
  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Initial rotation matrix: \n" << R);
    RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << true);
  }
  return true;
}

/* [Grouped edge points]
 * A function to line fitting 4 lines of the tag
 */
bool LiDARTag::_getLines(
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
  seg.setDistanceThreshold(0.02);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Inliers size: " << inliers->indices.size());
  }
  if (inliers->indices.size() == 0) {
    if (_debug_info) {
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
Eigen::MatrixXf LiDARTag::_getOrderedCorners(
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
bool LiDARTag::_estimateTargetSize(
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
  for (int i = 0; i < _tag_size_list.size(); ++i) {
    gap_temp = abs(mean_distance - _tag_size_list[i]);
    if (gap_temp < gap) {
      gap = gap_temp;
      tagsize = _tag_size_list[i];
      size_num = i;
    }
  }

  // float tagsize_tunable = 0.1;
  if (gap > _tagsize_tunable * tagsize) {
    // if (gap > 10) {
    status = false;
  } else {
    cluster.tag_size = tagsize;
    cluster.rkhs_decoding.size_num = size_num;
    status = true;
  }

  // TODO: set a threshold to invalid the tagsize
  if (_debug_info) {
            RCLCPP_DEBUG_STREAM(get_logger(), "==== _estimateTargetSize ====");
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
    if (gap > _tagsize_tunable * tagsize)
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << status);
    else
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << status);
  }
  return status;
}
/* [two lines]
 * A function to compute the intersection of two lines
 */
Eigen::Vector3f LiDARTag::_getintersec(Eigen::Vector4f line1, Eigen::Vector4f line2)
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
void LiDARTag::_estimatePrincipleAxis(ClusterFamily_t & cluster)
{
  // Eigen::MatrixXf EigenPC(3, cluster.inliers);
  // int eigenpc_index = 0;
  // for (int i=0; i<cluster.edge_points.size(); ++i){
  //     if (cluster.edge_points[i].valid != 1) continue;
  //     EigenPC(0,eigenpc_index) = cluster.edge_points[i].point.x -
  //     cluster.average.x; EigenPC(1,eigenpc_index) =
  //     cluster.edge_points[i].point.y - cluster.average.y;
  //     EigenPC(2,eigenpc_index) = cluster.edge_points[i].point.z -
  //     cluster.average.z; eigenpc_index +=1;
  // }
  // for (int i=0; i<cluster.data.size(); ++i){
  //     if (cluster.data[i].valid != 1) continue;
  //     EigenPC(0,eigenpc_index) = cluster.data[i].point.x - cluster.average.x;
  //     EigenPC(1,eigenpc_index) = cluster.data[i].point.y - cluster.average.y;
  //     EigenPC(2,eigenpc_index) = cluster.data[i].point.z - cluster.average.z;
  //     eigenpc_index +=1;
  // }
  // Eigen::JacobiSVD<Eigen::MatrixXf> svd(EigenPC, Eigen::ComputeFullU);
  Eigen::MatrixXf ave = Eigen::MatrixXf::Ones(3, cluster.inliers);
  ave.row(0) *= cluster.average.x;
  ave.row(1) *= cluster.average.y;
  ave.row(2) *= cluster.average.z;

  Eigen::MatrixXf centered_data = cluster.merged_data.topRows(3) - ave;
  //   Eigen::JacobiSVD<Eigen::MatrixXf> svd(centered_data,
  //   Eigen::ComputeFullU);
  cv::Mat cv_centered_data, cv_W, cv_U, cv_Vt;
  cv::eigen2cv(centered_data, cv_centered_data);
  cv::SVD::compute(cv_centered_data, cv_W, cv_U, cv_Vt);
  Eigen::Matrix3f U;
  cv::cv2eigen(cv_U, U);
  // cluster.principal_axes = svd.matrixU();
  cluster.principal_axes = U;

  // Eigen::Matrix4f  m1 = MatrixXd::Random(4,4);
  // Eigen::Matrix4f  m1;
  // m1 << 1, 2, 3, 4,
  //       5, 6, 7, 8,
  //       9, 10,11,12,
  //       13, 14,15,16;
  // Eigen::MatrixXf m2 = Eigen::MatrixXf::Ones(4,4);
  // m2.row(0) *= 1;
  // m2.row(1) *= 2;
  // m2.row(2) *= 3;
  // m2.row(3) *= 4;
  // cout << "m1: \n" << m1 << endl;
  // cout << "m2: \n" << m2 << endl;
  // cout << "m1 - [1;2;3;4]: \n" << m1 -m2<< endl;
  // cout<< "m1.row(0) - 1: " << m1.row(0) - 1*Eigen::MatrixXf::Ones(1, 1) <<
  // endl; cout<< "m1.row(1) - 2: " << m1.row(1) - 2*Eigen::MatrixXf::Ones(1, 2)
  // << endl; cout<< "m1.row(2) - 3: " << m1.row(2) - 3*Eigen::MatrixXf::Ones(1,
  // 4) << endl; cout<< "m1.row(3) - 4: " << m1.row(3) -
  // 4*Eigen::MatrixXf::Ones(1, 4) << endl;

  // // cout<< "1: " << Eigen::MatrixXf::Ones(1, 4) << endl;
  // m1.row(0) -= 1*Eigen::MatrixXf::Ones(1, 4);
  // m1.row(1) -= 2*Eigen::MatrixXf::Ones(1, 4);
  // m1.row(2) -= 3*Eigen::MatrixXf::Ones(1, 4);
  // m1.row(3) -= 4*Eigen::MatrixXf::Ones(1, 4);
  // cout << "m1: \n" << m1 << endl;

  // ROS_INFO_STREAM("principal_axes: " << svd.matrixU());
  // ROS_INFO_STREAM("inner_01: " <<
  // svd.matrixU().col(0).adjoint()*svd.matrixU().col(1));
  // ROS_INFO_STREAM("inner_02: " <<
  // svd.matrixU().col(0).adjoint()*svd.matrixU().col(2));
  // ROS_INFO_STREAM("inner_12: " <<
  // svd.matrixU().col(1).adjoint()*svd.matrixU().col(2));

  if (_debug_info) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== _estimatePrincipleAxis ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
    // Eigen::VectorXf sv = svd.singularValues();
    // RCLCPP_DEBUG_STREAM(get_logger(), "Singular values: " << sv[0] << ", " << sv[1] << ", "
    //                                      << sv[2]);
  }

  // flip through xy plane
  // Eigen::Matrix3f m;
  //  m << 0, 1, 0,
  //       0, 0, 1,
  //       1, 0, 0;

  // m << 0,1,0,
  //      1,0,0,
  //      0,0,1;

  // m << 0,0,1,
  //      1,0,0,
  //      0,1,0;
  // m << 0,0,1,
  //      0,1,0,
  //      1,0,0;

  // m << 1,0,0,
  //      0,0,1,
  //      0,1,0;
  // cout << "Normal Vector1: \n" << normal_vector_robot << endl;
  // normal_vector_robot = (m*normal_vector_robot).eval();
  // cout << "Normal Vector2: \n" << normal_vector_robot << endl;

  // Make sure consistency
  // Eigen::Vector3f position(cluster.average.x, cluster.average.y,
  // cluster.average.z); if(position.transpose()*normal_vector_robot < 0)
  // normal_vector_robot = (-normal_vector_robot).eval();
  // if (normal_vector_robot(0)>=0) normal_vector_robot =
  // (-normal_vector_robot).eval();
  // cout << position << "Normal Vector3: \n" << normal_vector_robot <<
  // endl;************

  // Coordinate transform
  // Eigen::Matrix<float,3,1,Eigen::DontAlign> normal_vector_tag;
  // normal_vector_tag << normal_vector_robot(2), normal_vector_robot(0),
  // normal_vector_robot(1); normal_vector_tag << normal_vector_robot(0),
  // normal_vector_robot(2), normal_vector_robot(1);

  // return normal_vector_robot;***************

  // pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud (new
  // pcl::PointCloud<pcl::PointXYZ>);

  // for (int i=0; i<cluster.payload.size(); ++i){
  //     pcl::PointXYZ point = {cluster.payload[i]->point.x,
  //                            cluster.payload[i]->point.y,
  //                            cluster.payload[i]->point.z};
  //     cout << "point: " << point << endl;
  //     Cloud->push_back(point);
  // }
  // pcl::PointCloud<pcl::Normal>::Ptr normals (new
  // pcl::PointCloud<pcl::Normal>);

  // // pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // // ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  // // ne.setMaxDepthChangeFactor(0.02f);
  // // ne.setNormalSmoothingSize(10.0f);
  // // ne.setInputCloud(Cloud);
  // // ne.compute(*normals);
  // // cout << "NV: " << *normals << endl;

  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> NE;
  // NE.setInputCloud (Cloud);
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new
  // pcl::search::KdTree<pcl::PointXYZ> ()); NE.setSearchMethod (tree);

  // // Output datasets
  // pcl::PointCloud<pcl::Normal>::Ptr CloudNormals (new
  // pcl::PointCloud<pcl::Normal>); NE.setRadiusSearch (1); NE.compute
  // (*CloudNormals); cout << "normals: " << *CloudNormals << endl; std::cout <<
  // "cloud_normals->points.size (): " << CloudNormals->points.size () <<
  // std::endl;
  //
  // return  CloudNormals;
}


/*
 * A function to transform from a customized type (LiDARpoints_t)
 * of vector of vector (edge_buff)
 * into a standard type (PointXYZRI) of pcl vector (out)
 */
void LiDARTag::_buffToPclVector(
  const std::vector<std::vector<LiDARPoints_t>> & edge_buff, pcl::PointCloud<PointXYZRI>::Ptr Out)
{
  for (int ringnumber = 0; ringnumber < _beam_num; ++ringnumber) {
    if (edge_buff.at(ringnumber).size() != 0) {
      for (int i = 0; i < edge_buff.at(ringnumber).size(); ++i) {
        Out->push_back(edge_buff[ringnumber][i].point);
      }
    }
  }
}

/*
 * A function to draw a line between two points
 */
void LiDARTag::_assignLine(
  visualization_msgs::msg::Marker & Marker, visualization_msgs::msg::MarkerArray MarkArray,
  const uint32_t Shape, const string ns, const double r, const double g, const double b,
  const PointXYZRI point1, const PointXYZRI point2, const int count)
{

  Marker.header.frame_id = _pub_frame;
  Marker.header.stamp = _current_scan_time;
  // Marker.ns = string("Boundary_") + to_string(cluster.cluster_id);
  Marker.ns = ns;
  Marker.id = count;
  Marker.type = Shape;

  Marker.action = visualization_msgs::msg::Marker::ADD;
  Marker.pose.orientation.x = 0.0;
  Marker.pose.orientation.y = 0.0;
  Marker.pose.orientation.z = 0.0;
  Marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  Marker.scale.x = 1;

  // Set the color -- be sure to set alpha to something non-zero!
  Marker.color.r = r;
  Marker.color.g = g;
  Marker.color.b = b;
  Marker.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = point1.x;
  p.y = point1.y;
  p.z = point1.z;
  Marker.points.push_back(p);
  p.x = point2.x;
  p.y = point2.y;
  p.z = point2.z;
  Marker.points.push_back(p);
  MarkArray.markers.push_back(Marker);
}

/*
 * A function to transform an eigen type of vector to pcl point type
 */
void LiDARTag::_eigenVectorToPointXYZRI(const Eigen::Vector4f & Vector, PointXYZRI & point)
{
  point.x = Vector[0];
  point.y = Vector[1];
  point.z = Vector[2];
  // point.intensity = Vector[3]; // TODO: is this okay?
}

/*
 * A function to transform a pcl point type to an eigen vector
 */
void LiDARTag::_PointXYZRIToEigenVector(const PointXYZRI & point, Eigen::Vector4f & Vector)
{
  Vector[0] = point.x;
  Vector[1] = point.y;
  Vector[2] = point.z;
  Vector[3] = point.intensity;
}

/* [not used] [not finished]
 * A function to group edges
 */
template <typename Container>
void LiDARTag::_freeVec(Container & c)
{
  while (!c.empty()) {
    if (c.back() != nullptr) {
      delete c.back();
      c.back() = nullptr;
    }
    c.pop_back();
  }
}

void LiDARTag::_freePCL(pcl::PointCloud<LiDARPoints_t *> & vec)
{
  while (!vec.empty()) delete vec.back(), vec.erase(vec.end());
}

void LiDARTag::_freeTagLineStruc(TagLines_t & tag_edges)
{
  LiDARTag::_freeVec(tag_edges.upper_line);
  LiDARTag::_freeVec(tag_edges.lower_line);
  LiDARTag::_freeVec(tag_edges.left_line);
  LiDARTag::_freeVec(tag_edges.right_line);

  LiDARTag::_freeVec(tag_edges.bottom_left);
  LiDARTag::_freeVec(tag_edges.bottom_right);
  LiDARTag::_freeVec(tag_edges.top_left);
  LiDARTag::_freeVec(tag_edges.top_right);
}

void LiDARTag::_printStatistics(const std::vector<ClusterFamily_t> & cluster_buff)
{
  // XXX: timings are all in milliseconds
  auto valid_clusters = _getValidClusters(cluster_buff);
  RCLCPP_INFO_STREAM(get_logger(), "[Writing CSV] Remaining Clusters: " << valid_clusters.size());


  std::ofstream fstats;
  std::ofstream ftiming;
  std::ofstream fpose;
  std::ofstream fclusters;
  std::ofstream fdecoding;
  std::ofstream fcorners;

  // fstats
  if (_iter == 0) {
    fstats.open(_outputs_path + "/stats.csv", ios::trunc);
    if (!fstats.is_open()) {
      cout << "Could not open stats.txt: " << _outputs_path << "\n Currently at: " << __LINE__
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
    fstats.open(_outputs_path + "/stats.csv", std::ofstream::out | std::ofstream::app);
    if (!fstats.is_open()) {
      cout << "Could not open stats.txt: " << _outputs_path << "\n Currently at: " << __LINE__
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

  fstats << _iter << ",";
  fstats << _result_statistics.point_cloud_size << ",";
  fstats << _result_statistics.edge_cloud_size << ",";
  fstats << _result_statistics.original_cluster_size << ",";
  fstats << _result_statistics.cluster_removal.minimum_return << ",";
  fstats << _result_statistics.cluster_removal.maximum_return << ",";
  fstats << _result_statistics.cluster_removal.plane_fitting << ",";
  fstats << _result_statistics.cluster_removal.plane_outliers << ",";
  fstats << _result_statistics.cluster_removal.boundary_point_check << ",";
  fstats << _result_statistics.cluster_removal.minimum_ring_points << ",";
  fstats << _result_statistics.cluster_removal.no_edge_check << ",";
  fstats << _result_statistics.cluster_removal.line_fitting << ",";
  fstats << _result_statistics.cluster_removal.pose_optimization << ",";
  fstats << _result_statistics.cluster_removal.decoding_failure << ",";
  fstats << _result_statistics.remaining_cluster_size << endl;
  fstats.close();
  fstats << std::endl;

  // Timing
  if (_debug_time) {
    RCLCPP_INFO(get_logger(), "debug time");
    if (_iter == 0) {
      ftiming.open(_outputs_path + "/timing_all.txt", ios::trunc);
      if (!ftiming.is_open()) {
        cout << "Could not open timing_all.txt: " << _outputs_path << "\n Currently at " << __FILE__
             << " at " << __LINE__ << endl;
        exit(0);
      }
      ftiming << "iter, duration, PoI_clustering, "
              << "to_pcl, fill_in_time, point_check, plane_fitting, "
              << "line_fitting, avage_edge_points, pca, "
              << "split_edge, pose_optimization, store_template, "
              << "payload_decoding" << endl;
    } else {
      ftiming.open(_outputs_path + "/timing_all.txt", std::ofstream::out | std::ofstream::app);
      if (!ftiming.is_open()) {
        cout << "Could not open timing_all.txt: " << _outputs_path << "\n Currently at " << __FILE__
             << " at " << __LINE__ << endl;
        exit(0);
      }
    }
    ftiming << _iter << ",";
    ftiming << _timing.total_duration << ",";
    ftiming << _timing.edging_and_clustering_time << ",";
    ftiming << _timing.to_pcl_vector_time << ",";
    ftiming << _timing.fill_in_time << ",";
    ftiming << _timing.point_check_time << ",";
    ftiming << _timing.plane_fitting_removal_time << ",";
    ftiming << _timing.line_fitting_time << ",";
    ftiming << _timing.organize_points_time << ",";
    ftiming << _timing.pca_time << ",";
    ftiming << _timing.split_edge_time << ",";
    ftiming << _timing.pose_optimization_time << ",";
    ftiming << _timing.store_template_time << ",";
    ftiming << _timing.payload_decoding_time << endl;
    ftiming.close();
  } else {
    if (_iter == 0) {
      ftiming.open(_outputs_path + "/timing_computation_only.txt", ios::trunc);
      if (!ftiming.is_open()) {
        cout << "Could not open computation_time.txt: " << _outputs_path << "\n Currently at "
             << __FILE__ << " at " << __LINE__ << endl;
        exit(0);
      }
      ftiming << "iter, duration" << endl;
    } else {
      ftiming.open(
        _outputs_path + "/timing_computation_only.txt", std::ofstream::out | std::ofstream::app);
      if (!ftiming.is_open()) {
        cout << "Could not open computation_time.txt: " << _outputs_path << "\n Currently at "
             << __FILE__ << " at " << __LINE__ << endl;
        exit(0);
      }
    }
    ftiming << _iter << ", ";
    ftiming << _timing.total_duration << endl;
    ftiming.close();
  }

  // decoding
  if (_debug_decoding_time) {
    if (_iter == 0) {
      fdecoding.open(_outputs_path + "/decoding_analysis.txt", ios::trunc);
      if (!fdecoding.is_open()) {
        cout << "Could not open decoding_analysis.txt: " << _outputs_path << "\n Currently at "
             << __FILE__ << " at " << __LINE__ << endl;
        exit(0);
      }
      fdecoding << "iter, original, matrix, vector, "
                << "tbb original, tbb vec, "
                << "manual scdl tbb vec, tbb scdl tbb vec, "
                << "tbb kd tree" << endl;
    } else {
      fdecoding.open(
        _outputs_path + "/decoding_analysis.txt", std::ofstream::out | std::ofstream::app);
      if (!fdecoding.is_open()) {
        cout << "Could not open decoding_analysis.txt: " << _outputs_path << "\n Currently at "
             << __FILE__ << " at " << __LINE__ << endl;
        exit(0);
      }
    }
    fdecoding << _iter << ", ";
    fdecoding << _time_decoding.original << ", ";
    fdecoding << _time_decoding.matrix << ", ";
    fdecoding << _time_decoding.vectorization << ", ";
    fdecoding << _time_decoding.tbb_original << ", ";
    fdecoding << _time_decoding.tbb_vectorization << ", ";
    fdecoding << _time_decoding.manual_scheduling_tbb_vectorization << ", ";
    fdecoding << _time_decoding.tbb_scheduling_tbb_vectorization << ", ";
    fdecoding << _time_decoding.tbb_kd_tree << endl;
    fdecoding.close();
  }

  // pose and clustering
  if (_iter == 0) {
    for (int i = 0; i < _num_tag_sizes; ++i) {
      std::string _tag_file_path =
        _outputs_path + "tag_size" + std::to_string(_tag_size_list[i]) + "pose.txt";
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

    for (int i = 0; i < _num_tag_sizes; ++i) {
      std::string _corners_file_path =
        _outputs_path + "tag_size" + std::to_string(_tag_size_list[i]) + "corners.csv";
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

    fclusters.open(_outputs_path + "/clusters.csv", std::ofstream::trunc);
    if (!fclusters.is_open()) {
      cout << "Could not open cluster file: " << _outputs_path << "Currently at :" << __LINE__
           << endl;
      exit(0);
    }
    fclusters << "It is recorded if there is any valid cluster" << endl;
    fclusters << "iter, valid cluster size, valid cluter points" << endl;
  } else {
    fclusters.open(_outputs_path + "/clusters.csv", std::ofstream::out | std::ofstream::app);
    if (!fclusters.is_open()) {
      cout << "Could not open cluster file: " << _outputs_path << "Currently at :" << __LINE__
           << endl;
      exit(0);
    }
  }

  if (valid_clusters.size() > 0) {
    fclusters << _iter << ",";
    fclusters << valid_clusters.size() << ",";
    for (const auto cluster_idx : valid_clusters) {
      std::string _tag_file_path = _outputs_path + "tag_size" +
                                   std::to_string(cluster_buff[cluster_idx].tag_size) + "pose.txt";
      fpose.open(_tag_file_path, std::ofstream::out | std::ofstream::app);
      fpose << _iter << ",";
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
      std::string _corners_file_path = _outputs_path + "tag_size" +
                                       std::to_string(cluster_buff[cluster_idx].tag_size) +
                                       "corners.csv";
      fcorners.open(_corners_file_path, std::ofstream::out | std::ofstream::app);
      fcorners << _iter << ",";
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

  _iter++;
}

std::vector<int> LiDARTag::_getValidClusters(const std::vector<ClusterFamily_t> & cluster_buff)
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

// void LiDARTag::_saveTemporalCluster(const std::vector<ClusterFamily_t>
// &t_cluster, std::vector<std::vector<pcl::PointCloud<LiDARPoints_t>>>&
// matData)
// {
//     //if first time retrieve the data from cluster to mat file
//     if (matData.empty()){
//         int num_cluster = t_cluster.size();
//         for(int i = 0 ; i < num_cluster; ++i){
//             std::vector<pcl::PointCloud<LiDARPoints_t>> target;
//             target.push_back(t_cluster[i].data);
//             matData.push_back(target);
//         }
//     }

//     //check if cluster nums stay consistent with previous recognized nums
//     if (t_cluster.size() != matData.size()){
//         printf("Num of targets not consistent! \n" );
//         exit(0);
//     }

//     //append new pointclouds to matData
//     for(int i = 0; i < t_cluster.size(); ++i){
//         matData[i].push_back(t_cluster[i].data);
//     }
// }

// void
// LiDARTag::_saveMatFiles(std::vector<std::vector<pcl::PointCloud<LiDARPoints_t>>>&
// matData){

//     int num_files = matData.size();
//     for(auto pcs : matData){
//         int depth = pcs.size();
//         //find the max num of points in the temporal series
//         vector<pcl::PointCloud<LiDARPoints_t>>::iterator max_itr=
//         max_element(pcs.begin(),pcs.end(),[](pcl::PointCloud<LiDARPoints_t>
//         &A, pcl::PointCloud<LiDARPoints_t> &B){
//             return A.size() < B.size();
//         });
//         int length = (*max_itr).size();

//         //open matfiles
//         std::string path = _mat_file_path + "pc" +
//         std::to_string(num_files--); MATFile *pmat = matOpen(path.c_str(),
//         "w");

//         if (pmat == NULL) {
//             printf("Error creating file %s\n", path.c_str());
//             printf("(Do you have write permission in this directory?)\n");
//             exit(0);
//         }

//         const mwSize dims[]={depth,length,5};
//         mxArray* plhs = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS,
//         mxREAL);

//         double *ptr = mxGetDoubles(plhs);
//         //assign pointclouds to mat files
//         for(int i = 0; i < depth; ++i){
//             int j = 0;
//             for(const auto &p : pcs[i]){
//                 ptr[i*length*5+j*5+0] = (double)p.point.x;
//                 ptr[i*length*5+j*5+1] = (double)p.point.y;
//                 ptr[i*length*5+j*5+2] = (double)p.point.z;
//                 ptr[i*length*5+j*5+3] = (double)p.point.intensity;
//                 ptr[i*length*5+j*5+4] = (double)p.point.ring;
//                 j++;
//             }
//             //filling all empty cells as 0
//             for(;j<length;++j){
//                 ptr[i*length*5+j*5+0] = (double)0.0;
//                 ptr[i*length*5+j*5+1] = (double)0.0;
//                 ptr[i*length*5+j*5+2] = (double)0.0;
//                 ptr[i*length*5+j*5+3] = (double)0.0;
//                 ptr[i*length*5+j*5+4] = (double)0.0;
//             }
//         }

//         if(matPutVariable(pmat, "pc", plhs)){
//             printf("Error saving varible to file %s\n",path.c_str());
//         };

//         mxDestroyArray(plhs);

//         if (matClose(pmat) != 0) {
//             printf("Error closing file %s\n",path.c_str());
//             exit(-1);
//         }
//     }

// }
void LiDARTag::_detectionArrayPublisher(const ClusterFamily_t & cluster)
{
  // if(cluster.average.x < 0) return;

  lidartag_msgs::msg::LidarTagDetection detection;
  detection.header = _point_cloud_header;
  detection.frame_index = 0 /* _point_cloud_header.seq deprecated */;

  // TODO: here we can only assign the id and tag size according to the
  // distance.
  // That only applies for certain calibration scene configuration
  // imperical threshold for the static scene
  // double distance_threshold = 7.0;
  // std::cout << "Lidartag target depth: " << cluster.average.x << std::endl;
  if (_calibration) {
    if (cluster.average.x > _distance_threshold)
      detection.id = 1;
    else
      detection.id = 3;
  }

  detection.size = detection.id == 1 ? 1.215 : 0.8;

  pcl::PointCloud<PointXYZRI>::Ptr clusterPC(new pcl::PointCloud<PointXYZRI>);
  for (int i = 0; i < cluster.data.size(); ++i) {
    clusterPC->push_back(cluster.data[i].point);
  }
  // std::cout << "LiDARTag cluster size" << cluster.data.size() << std::endl;

  sensor_msgs::msg::PointCloud2 pcs_waited_to_pub;      
  pcl::toROSMsg(*clusterPC, pcs_waited_to_pub);
  detection.points = pcs_waited_to_pub;
  detectionsToPub.detections.push_back(detection);
  // if (detectionsToPub.detections.size() > 2)
  // ROS_INFO_STREAM("LiDARTag Got wrong tags");   
}
void LiDARTag::keyboardEventOccurred(
  const pcl::visualization::KeyboardEvent & event, void * nothing)
{
  if (event.getKeySym() == "space" && event.keyDown()) loop = true;
}

void LiDARTag::visualiseClusterBuff(vector<ClusterFamily_t> & cluster_buff)
{
  RCLCPP_INFO(get_logger(), "visualiseClusterBuff start");
  cluster_buff_time_stamp_sec = _point_cloud_header.stamp.sec;
  cluster_buff_time_stamp_nsec = _point_cloud_header.stamp.nanosec;
  cout << fixed;
  std::cout << "time stamp: sec = " << cluster_buff_time_stamp_sec << endl;
  std::cout << "time stamp: nsec = " << cluster_buff_time_stamp_nsec << endl;

  pcl::visualization::PCLVisualizer viewer("Cluster buff visualization");
  int v1(0);
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;
  pcl::PointCloud<LiDARPoints_t>::Ptr cluster_pc(new pcl::PointCloud<LiDARPoints_t>);
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
  viewer.registerKeyboardCallback(&LiDARTag::keyboardEventOccurred, *this, (void *)NULL);
  int index_num = 0;
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
    if (loop) {
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
    loop = false;
  }
  viewer.close();
  cluster_pc->clear();
  output_pc->clear();
}

void LiDARTag::writeClusterBuff(vector<ClusterFamily_t> & cluster_buff, std::ofstream & fbuff)
{
  RCLCPP_INFO(get_logger(), "writeClusterBuff start");
  cluster_buff_time_stamp_sec = _point_cloud_header.stamp.sec;
  cluster_buff_time_stamp_nsec = _point_cloud_header.stamp.nanosec;
  cout << fixed;
  std::cout << "time stamp: sec = " << cluster_buff_time_stamp_sec << endl;
  std::cout << "time stamp: nsec = " << cluster_buff_time_stamp_nsec << endl;

  pcl::PointCloud<LiDARPoints_t>::Ptr cluster_pc(new pcl::PointCloud<LiDARPoints_t>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc(new pcl::PointCloud<pcl::PointXYZ>);
  *cluster_pc = cluster_buff[0].data;
  output_pc->points.resize(cluster_pc->points.size());
  for (size_t i = 0; i < cluster_pc->points.size(); i++) {
    output_pc->points[i].x = cluster_pc->points[i].point.x;
    output_pc->points[i].y = cluster_pc->points[i].point.y;
    output_pc->points[i].z = cluster_pc->points[i].point.z;
  }
  int index_num = 0;
  while (true) {
    if (index_num + 1 > cluster_buff.size()) {
      return;
    }
    index_num++;
    *cluster_pc = cluster_buff[index_num].data;
    output_pc->points.resize(cluster_pc->points.size());
    if (
      output_pc->points.size() < 1400 || 1600 < output_pc->points.size() || index_num < 50 ||
      80 < index_num) {
      continue;
    }

    // std::cout << "index number = " << index_num << endl << "size = " <<
    // output_pc->points.size() << std::endl;
    for (size_t i = 0; i < cluster_pc->points.size(); i++) {
      output_pc->points[i].x = cluster_pc->points[i].point.x;
      output_pc->points[i].y = cluster_pc->points[i].point.y;
      output_pc->points[i].z = cluster_pc->points[i].point.z;
    }
    fbuff << fixed << std::setprecision(10);  // TODO:
    fbuff << _clock->now().nanoseconds() << ",";
    fbuff << cluster_buff_time_stamp_sec << ",";
    fbuff << cluster_buff_time_stamp_nsec << ",";
    fbuff << index_num << ",";
    fbuff << output_pc->points.size() << ",";
    fbuff << cluster_buff[index_num].edge_points.size() << ",";
    fbuff << inlier_size << ",";
    fbuff << cluster_buff[index_num].percentages_inliers << ",";
    fbuff << cluster_buff[index_num].detail_valid << ",";
    fbuff << cluster_buff[index_num].valid;
    fbuff << std::endl;
  }
  pose_status = 0;
  cluster_pc->clear();
  output_pc->clear();
}

void LiDARTag::publishLidartagCluster(const vector<ClusterFamily_t> & cluster_buff)
{
  pcl::PointCloud<LiDARPoints_t>::Ptr cluster_pc(new pcl::PointCloud<LiDARPoints_t>);
  pcl::PointCloud<LiDARPoints_t>::Ptr cluster_ep_pc(new pcl::PointCloud<LiDARPoints_t>);
  pcl::PointCloud<LiDARPoints_t>::Ptr cluster_tr_ep_pc(new pcl::PointCloud<LiDARPoints_t>);
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
      points_size < _lidartag_params.cluster_min_points_size || points_size > _lidartag_params.cluster_max_points_size ||
      index_num < _lidartag_params.cluster_min_index || index_num > _lidartag_params.cluster_max_index) {
      index_num++;
      continue;
    }
    *cluster_pc = cluster_buff[index_num].data;
    *cluster_ep_pc = cluster_buff[index_num].edge_points;
    *cluster_tr_ep_pc = cluster_buff[index_num].transformed_edge_points;
    output_pc->points.resize(cluster_pc->points.size());
    output_ep_pc->points.resize(cluster_ep_pc->points.size());
    output_tr_ep_pc->points.resize(cluster_tr_ep_pc->points.size());
    point.header.stamp = _clock->now();
    point.header.frame_id = _pub_frame;
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
    output_data_msg.header.frame_id = _pub_frame;
    output_ep_msg.header.frame_id = _pub_frame;
    output_tr_ep_msg.header.frame_id = _pub_frame;
    output_data_msg.header = _point_cloud_header;
    output_ep_msg.header = _point_cloud_header;
    output_tr_ep_msg.header = _point_cloud_header;
    _lidartag_cluster_pub->publish(output_data_msg);
    _lidartag_cluster_edge_points_pub->publish(output_ep_msg);
    _lidartag_cluster_transformed_edge_points_pub->publish(output_tr_ep_msg);
    _average_point_pub->publish(point);
    //publishClusterInfo(cluster_buff[index_num]); KL: jsk has not been ported yet
    index_num++;
  }
  cluster_pc->clear();
  cluster_ep_pc->clear();
  cluster_tr_ep_pc->clear();
  output_pc->clear();
  output_ep_pc->clear();
  output_tr_ep_pc->clear();
}
void LiDARTag::publishIntersections(const std::vector<Eigen::VectorXf> intersection_list)
{
  visualization_msgs::msg::MarkerArray intersection_marker_array;
  intersection_marker_array.markers.resize(4);
  int index = 0;
  rclcpp::Duration d(3.0);

  for (auto intersection : intersection_list) {
    intersection_marker_array.markers[index].header.frame_id = _pub_frame;
    intersection_marker_array.markers[index].header.stamp = _clock->now();
    intersection_marker_array.markers[index].ns = "intersection_marker";
    intersection_marker_array.markers[index].id = index;
    intersection_marker_array.markers[index].type = visualization_msgs::msg::Marker::CUBE;
    intersection_marker_array.markers[index].action = visualization_msgs::msg::Marker::ADD;
    intersection_marker_array.markers[index].lifetime = d;
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
  _intersection_marker_array_pub->publish(intersection_marker_array);
}
void LiDARTag::printClusterResult(const std::vector<ClusterFamily_t> & cluster_buff)
{
  std::ofstream fresult;
  if (_iter == 0) {
    fresult.open(_outputs_path + "/results.csv", std::ofstream::out);
    if (!fresult.is_open()) {
      std::cout << "Could not open results.csv: " << _outputs_path << "\n Crrently at: " << __LINE__
                << std::endl;
      exit(0);
    }
    fresult << "valid,id,x,y,z,roll,pitch,yaw,tag_size";
    fresult << std::endl;

  } else {
    fresult.open(_outputs_path + "/results.csv", std::ofstream::out | std::ofstream::app);
    if (!fresult.is_open()) {
      cout << "Could not open results.csv: " << _outputs_path << "\n Currently at: " << __LINE__
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
};  // namespace BipedLab
