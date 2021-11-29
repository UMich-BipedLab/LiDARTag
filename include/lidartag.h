/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.brucerobot.com/
 */



#pragma once
#ifndef LIDARTAG_H
#define LIDARTAG_H

#include <queue>          // std::queue
#include <string>
#include <fstream>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2/transform_datatypes.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// threadings
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <tbb/tbb.h>

// To trasnform to pcl format
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include "pcl/PCLPointCloud2.h"

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h> 

#include <lidartag_msgs/msg/lidar_tag_detection.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>
#include "types.h"
#include "thread_pool.h"

// #include "mat.h"
// #include "matrix.h"
// #include "tensorflow_ros_test/lib.h"

namespace BipedLab{
    class LiDARTag: public rclcpp::Node{
        public:
            LiDARTag(const rclcpp::NodeOptions & options);
            ~LiDARTag();

        private:
            /*****************************************************
             * Variables
             *****************************************************/
            // Flags for functioning
            int _adaptive_thresholding; // Use adaptive thresholding or not
            int _collect_dataset; // To collect dataset (only publish one tag)
            int _sleep_to_display; // Sleep for visualization
            double _sleep_time_for_vis; // Sleep for how long?
            int _valgrind_check; // Debugging with Valgrind
            int _fake_tag;
            int _decode_method; // Which decode methods to use?


            int _optimization_solver;
            int _decode_mode; 
            int _grid_viz; // visualize remapping grid
            bool _mark_cluster_validity;
            bool _plane_fitting; // whether perform plane fitting 
            bool _pose_optimization; // optimize pose or not
            bool _id_decoding; // decode id or not
            bool _write_CSV; // Write CSV files 
            bool _calibration;
            bool _has_ring; // data has ring_num or not
            bool _ring_estimation; // need to estimate ring_num or not
            bool _ring_estimated;
            bool _use_ring; // use ring information or not
            int _num_accumulation; // Accumuate # of scans as a full scan of lidar 
            int _iter; //iterations of frame
            std::string _assign_id; // Directly assign Id, mainly for calibration usage

            std::unique_ptr<boost::thread> _extraction_thread;

            // ROS
            //rclcpp::NodeHandle _nh; // TODO KL: Delete after it works
            tf2_ros::TransformBroadcaster broadcaster_;
            
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _lidar1_sub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _original_pc_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _edge_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _transformed_points_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _transformed_points_tag_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _edge1_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _edge2_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _edge3_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _edge4_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _boundary_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _cluster_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _payload_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _payload3d_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tag_pub;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _ini_tag_pub;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _cluster_marker_pub;  // cluster markers
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _boundary_marker_pub; // cluster boundary
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _id_marker_pub;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _payload_marker_pub; // payload boundary
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _payload_grid_pub; // grid visualization
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _payload_grid_line_pub; // grid visualization
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _ideal_frame_pub;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _tag_frame_pub;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _edge_vector_pub;
            rclcpp::Publisher<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr _lidartag_pose_pub; // Publish LiDAR pose
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _clustered_points_pub; // Points after minor clusters removed
            rclcpp::Publisher<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr _detectionArray_pub;

            // Flag
            int _point_cloud_received; // check if a scan of point cloud has received or not
            int _stop; // Just a switch for exiting this program while using valgrind

            // rclcpp::Publisher DebugPointCheckPub_; // Debug
            // rclcpp::Publisher DebugBoundaryPointPub_; // Debug
            // rclcpp::Publisher DebugNoEdgePub_; // Debug
            // rclcpp::Publisher DebugExtractPayloadPub_; // Debug

            //rclcpp::Publisher TextPub_; // publish text
            // visualization_msgs::msg::MarkerArray _markers; 

            // Queue for pc data
            std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> _point_cloud1_queue;


            // LiDAR parameters
            rclcpp::Time _current_scan_time; // store current time of the lidar scan
            std::string _pointcloud_topic; // subscribe channel
            std::string _pub_frame; // publish under what frame?
            std::string lidartag_detection_topic;
            // Overall LiDAR system parameters
            LiDARSystem_t _LiDAR_system;
            int _beam_num;
            double _vertical_fov;

            // PointCould data (for a single scan)
            int _point_cloud_size;
            std_msgs::msg::Header _point_cloud_header;

            // Edge detection parameters
            double _intensity_threshold;
            double _depth_threshold;

            // If on-board processing is limited, limit range of points
            double _distance_bound;
            double _distance_threshold;

            // fiducial marker parameters
            double _payload_size; // physical payload size 
            int _tag_family; // what tag family ie tag16h5, this should be 16
            int _tag_hamming_distance; // what tag family ie tag16h5, this should be 5
            int _max_decode_hamming; // max hamming while decoding 
            int _black_border; // black boarder of the fiducial marker
            int _num_codes;
            int _num_tag_sizes;
            std::vector<std::vector<Eigen::MatrixXf>> _function_dic;
            std::vector<std::vector<Eigen::MatrixXf>> _function_dic_xyz; // for kd tree
            std::vector<std::vector<Eigen::VectorXf>> _function_dic_feat; // for kdtree
            std::vector<std::vector<std::string>> _rkhs_function_name_list;
            
            // Cluster parameters
            double _linkage_threshold;
            double _RANSAC_threshold;
            int _fine_cluster_threshold; // TODO: REPLACE WITH TAG PARAMETERS
            int _filling_gap_max_index; // TODO: CHECK
            int _filling_max_points_threshold; //TODO: REMOVE
            double _points_threshold_factor; // TODO: CHECK
            double _distance_to_plane_threshold;
            double _max_outlier_ratio;
            int _num_points_for_plane_feature;
            double _nearby_factor;
            int _minimum_ring_boundary_points;
            int _np_ring;
            double _linkage_tunable;
            std::vector<double> _tag_size_list;
            double _optimization_percent;

            bool _print_ros_info;
            bool _debug_info;
            bool _debug_time;
            bool _debug_decoding_time;
            bool _log_data;
            bool _derivative_method;
            // Payload 
            double _line_intensity_bound; // TODO: not sure but may okay to remove it for releasing
            double _payload_intensity_threshold;
            double _opt_lb;
            double _opt_ub;
            double _coa_tunable;
            double _tagsize_tunable;
            int _min_returns_per_grid;
            GrizTagFamily_t *tf;
            lidartag_msgs::msg::LidarTagDetectionArray _lidartag_pose_array; // an array of apriltags
            lidartag_msgs::msg::LidarTagDetectionArray detectionsToPub;



            // threadings 
            int _num_threads;
            std::shared_ptr<ThreadPool> _thread_vec;


            // lock
            boost::mutex _refine_lock;
            boost::mutex _buff_to_pcl_vector_lock;
            boost::mutex _point_cloud1_queue_lock;


            // NN
            // LoadWeight *NNptr_;
            std::string _latest_model;
            std::string _weight_path;
            int _max_point_on_payload;
            int _XYZRI; // input channels

            // Debug
            Debug_t _debug_cluster;
            Timing_t _timing;
            TimeDecoding_t _time_decoding;

            // Statistics
            Statistics_t _result_statistics;
            std::string  _stats_file_path;
            std::string  _cluster_file_path;
            std::string  _pose_file_path;
            std::string  _outputs_path;
            std::string  _library_path;

            /*****************************************************
             * Functions
             *****************************************************/

            /* [Main loop]
             * main loop of process
             */
            void _mainLoop();


            /* [basic ros]
             * A function to get all parameters from a roslaunch
             * if not get all parameters then it will use hard-coded parameters
             */
            void _getParameters();


            /* [basic ros]
             * A function of ros spin
             * reason: in order to put it into background as well as able to run other tasks
             */
            inline void _rosSpin();


            /* [basic ros]
             * A function to push the received pointcloud into a queue in the class
             */
            inline void _pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);


            /* [Transform/Publish]
             * A function to transform pcl msgs to ros msgs and then publish
             * WhichPublisher should be a string of "organized" or "original" 
             * regardless lowercase and uppercase
             */
            void _publishPC(const pcl::PointCloud<PointXYZRI>::Ptr &t_source_PC, 
                            const std::string &t_frame_name, 
                            std::string t_which_publisher);

            /* [Type transformation]
             * A function to transform from a customized type (LiDARpoints_t) of vector of 
             * vector (EdgeBuff) into a standard type (PointXYZRI) of pcl vector (out)
             */
            void _buffToPclVector(const std::vector<std::vector<LiDARPoints_t>> &t_edge_buff,
                                 pcl::PointCloud<PointXYZRI>::Ptr t_out);

            /* [Pre-Processing] 
             * A function to slice the Veloydyne full points to sliced pointed
             * based on ring number
             */
            inline void _fillInOrderedPC(const pcl::PointCloud<PointXYZRI>::Ptr    &t_pcl_pointcloud, 
                                         std::vector< std::vector<LiDARPoints_t> > &t_ordered_buff);
            /* 
            * A function to compute angle between the line from origin to this point 
            * and z=0 plane in lidar
            * */
            float _getAnglefromPt(PointXYZRI &t_point);

            void _getAngleVector(
                        const pcl::PointCloud<PointXYZRI>::Ptr &pcl_pointcloud,
                        std::vector<float> &angles);
            /* [Type transformation]
             * A function to get pcl OrderedBuff from a ros sensor-msgs form of
             * pointcould queue
             */
            std::vector<std::vector<LiDARPoints_t>> _getOrderBuff();
            // void GetOrderBuff(std::vector<std::vector<PointXYZRI>>& OrderedBuff);


            /* [LiDAR analysis]
             * A function to get a LiDAR system parameters such as max, min points per scan
             * and how many points per ring
             */
            void _analyzeLiDARDevice();
            

            /* [LiDAR analysis]
             * A function to find maximum points and minimum points in a single scan, i.e. to
             * find extrema within 32 rings
             */
            void _maxMinPtsInAScan(std::vector<int>      &t_point_count_table, 
                                   std::vector<MaxMin_t> &t_max_min_table,
                                   std::vector<MaxMin_t> &t_ring_average_table,
                                   const std::vector<std::vector<LiDARPoints_t>> &t_ordered_buff);


            /* [LiDAR analysis]
             * A function to calculate how many points are supposed to be on a cluster at 1
             * meter away
             */
            void _pointsPerSquareMeterAtOneMeter();


            /*
             * A function to get a number of points on a given-distance tag or object
             * from LiDAR analysis
             */
            int _areaPoints(const double &t_distance, const double &t_obj_width, const double &t_obj_height);



            /* [LiDARTag detection]
             * Given lidar pointcloud, this function performs 
             * lidartag detection and decode the corresponding id
             */
            pcl::PointCloud<PointXYZRI>::Ptr
            _lidarTagDetection(const std::vector<std::vector<LiDARPoints_t>> &t_ordered_buff, 
                               std::vector<ClusterFamily_t> &t_cluster_buff);


            /* [Edge detection and clustering]
             * A function to 
             * (1) calculate the depth gradient and the intensity gradient at a point of a pointcloud
             * (2) group detected 'edge' into different group
             */
            void _gradientAndGroupEdges(const std::vector<std::vector<LiDARPoints_t>> &t_ordered_buff, 
                                       std::vector<std::vector<LiDARPoints_t>> &t_edge_buff,
                                       std::vector<ClusterFamily_t> &t_cluster_buff);

            /* [Edge detection from n consecutive points]
            *<consecutive n points from ring i index j>
            * A function to 
            * (1) kick out points are not near to each other
            * (2) find edge points from two side points of these points
            * Return value : 0 mean no edge point, 1 mean the left side point is the edge point
            * Return value : 2 mean the right side point is the edge point, 3 mean two side points are edge points
            */
            int _getEdgePoints(const std::vector<std::vector<LiDARPoints_t>>& OrderedBuff, 
                                        int i, int j, int n);


            /* [Clustering-Linkage]
             * A function to cluster a single edge point into a new cluster or an existing cluster 
             */
            void _clusterClassifier(const LiDARPoints_t &point, std::vector<ClusterFamily_t> &t_cluster_buff);


            /* [Clustering-Update]
             * A function update some information about a cluster if this point belongs to
             * this cluster; if not belonging to this cluster then return and create a new
             * one 
             */
            void _updateCluster(const LiDARPoints_t &t_point, 
                                ClusterFamily_t &t_old_cluster, 
                                TestCluster_t *t_new_cluster);

            /* [Adaptive-Clustering]
             * A function that determines if a point is within a given cluster adaptively
             * based on the ring number and range of the point. 
             */
            bool _isWithinCluster(const LiDARPoints_t &point, ClusterFamily_t &cluster);

            /* [Adaptive-Clustering]
             * A function that determines if a point is within a given cluster horizontally and adaptively
             * based on the range of the point. 
             */
            bool _isWithinClusterHorizon(const LiDARPoints_t &point, ClusterFamily_t &cluster, double threshold);

            /* [Clustering-Validation] <For all Clusters>
             * A function to 
             * (1) remove invalid cluster based on the index is too far or not
             * (2) fill in the points between index of edges 
             * (3) after filling, if the points are too less (based on the analyzed system
             *     and given distant of the cluster), then remove this cluster
             * (4) Adaptive thresholding (Maximize and minimize intensity) by comparing
             *     with the average value
             */
            void _fillInCluster(const std::vector<std::vector<LiDARPoints_t>> &t_ordered_buff, 
                                std::vector<ClusterFamily_t> &t_cluster_buff);


            /* [Clustering-Validation] <For "a" cluster>
             * A valid cluster, valid tag, the points from the original point cloud that belong to the cluster 
             * could be estimated from the LiDAR system
             * Therefore, if the points on the tag is too less, which means it is not a valid
             * tag where it might just a shadow of a valid tag
             */
            bool _clusterPointsCheck(ClusterFamily_t &t_cluster);


            /* [Clustering-Validation] <A cluster> TODO:RENAME
             * A function to 
             * (1) do adaptive thresholding (Maximize and minimize intensity) by comparing 
             *     with the average value and  
             * (2) sort points with ring number and re-index with current cluster into 
             *     tag_edges vector so as to do regression boundary lines
             * (3) It will *remove* if linefitting fails
             */
            bool _adaptiveThresholding(ClusterFamily_t &t_cluster);


            /* [Clustering-Validation] <A cluster>
             * A function to fit 4 lines of a payload in a cluster by
             * (1) finding the edges of the payload (how to find is stated below)
             * (2) rejecting and removing the cluster if one of the line is too short 
             */
            bool _detectPayloadBoundries(ClusterFamily_t &t_cluster);


            /* [Payload extraction] <A cluster>
             * A function to extract the payload points from a valid cluster.
             * Let's say we have 10 points on the left boundary (line) of the tag and 9 points on the right boundary
             * (line) of the tag.
             * It is separated into two parts.
             * TODO: should use medium instead of max points
             *  (i) Find the max points that we have on a ring in this cluster by
             *      exmaming the average points on the first 1/2 rings int((10+9)/4)
             * (ii) For another half of the rings, we just find the start index and add the
             *      average number of points to the payload points
             */
            void _extractPayloadWOThreshold(ClusterFamily_t &t_cluster);

            /* <A cluster>
            * A function to calculate the average point of valid edge points
            */   
            void _organizeDataPoints(ClusterFamily_t &t_cluster);

            /* [Edge points and principal axes]
            * A function to transform the edge points to the tag frame
            */
            bool _transformSplitEdges(ClusterFamily_t &t_cluster);

            /* <A cluster>
            * A function to store transformed points
            */ 
            void _storeTemplatePts(ClusterFamily_t &t_Cluster);

            
            /* [Unordered corner points]
            * A function to reorder the undered corner points from PCA
            */
            Eigen::MatrixXf _getOrderedCorners(Eigen::MatrixXf &t_payload_vertices, ClusterFamily_t &t_Cluster);

            /* [two lines]
            * A function to compute the intersection of two lines
            */            
            Eigen::Vector3f _getintersec(Eigen::Vector4f t_line1, Eigen::Vector4f t_line2);

            /* [four corner points]
            * A function to compute tag size according to the corner points of the tag
            */
            bool _estimateTargetSize(
                    ClusterFamily_t &t_cluster, 
                    const Eigen::Vector3f &point1, 
                    const Eigen::Vector3f &point2, 
                    const Eigen::Vector3f &point3,
                    const Eigen::Vector3f &point4);

            /* [A set of 2D points]
            * A function to transform the edge points to the tag frame
            */
            bool _getLines(pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud, Eigen::Vector4f &t_line);

            /* [Type transformation]
             * A function to transform an eigen type of vector to pcl point type
             */
            void _eigenVectorToPointXYZRI(const Eigen::Vector4f &t_vector, PointXYZRI &t_point);

            /* [Type transformation]
             * A function to transform a pcl point type to an eigen vector
             */
            void _PointXYZRIToEigenVector(const PointXYZRI &point, Eigen::Vector4f &Vector);


            /* [Normal vector]
             * A function to estimate the normal vector of a potential payload
             */
            //Eigen::MatrixXf 
            void _estimatePrincipleAxis(ClusterFamily_t &cluster);

            /* [pose]
             * A function to estimate the pose of a potential payload
             */
            Homogeneous_t _estimatePose(ClusterFamily_t &cluster);

            /*[oritented vector]
            */
            Eigen::Vector3f _estimateEdgeVector(ClusterFamily_t &Cluster);
            /* [pose]
             * A function to optimize the pose of a potential payload with  L1 optimization
             */
            int _optimizePose(ClusterFamily_t &cluster);
            bool _optimizePoseGrad(ClusterFamily_t &cluster);
            /* [Pose: tag to robot]
             * A function to publish pose of tag to the robot
             */
            void _tagToRobot(const int &t_cluster_id, const Eigen::Vector3f &t_normal_vec, 
                Homogeneous_t &t_pose, 
                tf2::Transform &t_transform, const PointXYZRI &t_ave);

            /* [Payload decoding]
             * A function to decode payload with different means
             * 0: Naive decoding
             * 1: Weighted Gaussian
             * 2: Deep learning
             * 3: Gaussian Process
             * 4: ?!
             */
            bool _decodePayload(ClusterFamily_t &t_cluster);


            /* [Decoder]
             * A function to determine a codeword on a payload using equal weight
             * methods
             */
            void _getCodeNaive(std::string &t_code, pcl::PointCloud<LiDARPoints_t*> t_payload);


            /* [Decoder]
             * Decode using Weighted Gaussian weight
             * return  0: normal
             * return -1: not enough return
             * return -2: fail corner detection
             */
            int _getCodeWeightedGaussian(std::string &Code, Homogeneous_t &t_pose,
                                         int &t_payload_points,
                                         const PointXYZRI &ave, 
                                         const pcl::PointCloud<LiDARPoints_t*> &t_payload, 
                                         const std::vector<LiDARPoints_t*> &t_payload_boundary_ptr);

            /* [Decoder]
             * 1) Transfrom the payload points to 3D-shape pc
             * 2) Compute inner product
             * 3) 
             */
            int _getCodeRKHS(RKHSDecoding_t &rkhs_decoding, const double &tag_size);

          
            Eigen::MatrixXf _construct3DShapeMarker(RKHSDecoding_t &rkhs_decoding,
                                                    const double &ell);

            float computeFunctionInnerProduct(
                    const Eigen::MatrixXf &pc1, 
                    const Eigen::MatrixXf &pc2, 
                    const float &ell);


            void computeFunctionOriginalInnerProduct(
                    const Eigen::MatrixXf &pc1,
                    const float &num_pc1,
                    const Eigen::MatrixXf &pc2,
                    const float &num_pc2,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell,
                    float &score);


            void computeFunctionMatrixInnerProduct(
                    const Eigen::MatrixXf &pc1,
                    const float &num_pc1,
                    const Eigen::MatrixXf &pc2,
                    const float &num_pc2,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell, 
                    float &score);


            void computeFunctionVectorInnerProduct(
                    const Eigen::MatrixXf &pc1, 
                    const float &num_pc1, 
                    const Eigen::MatrixXf &pc2, 
                    const float &num_pc2, 
                    const float &geo_sig, 
                    const float &feature_ell, 
                    const float &geo_ell, 
                    float &score);

            void _assignClusterPose(
                    const Homogeneous_t &H_TL, 
                    Homogeneous_t &H_LT,
                    const int &rotation_angle);


            void singleTask(const Eigen::ArrayXf &x_ary,
                    const Eigen::ArrayXf &y_ary,
                    const Eigen::ArrayXf &z_ary,
                    const Eigen::ArrayXf &i_ary,
                    const Eigen::MatrixXf &pc1_j,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell,
                    float &score);

            void singleTaskFixedSize(
                    const Eigen::ArrayXf &x_ary,
                    const Eigen::ArrayXf &y_ary,
                    const Eigen::ArrayXf &z_ary,
                    const Eigen::ArrayXf &i_ary,
                    const Eigen::MatrixXf &pc1_j,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell,
                    float &score);


            void multipleTasks(
                    const Eigen::ArrayXf &x_ary,
                    const Eigen::ArrayXf &y_ary,
                    const Eigen::ArrayXf &z_ary,
                    const Eigen::ArrayXf &i_ary,
                    const Eigen::MatrixXf &pc1_j,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell,
                    float &score);


            void computeFunctionVectorInnerProductThreading(
                    const Eigen::MatrixXf &pc1, 
                    const int &num_pc1, 
                    const Eigen::MatrixXf &pc2, 
                    const int &num_pc2, 
                    const float &geo_sig,
                    const float &feature_ell, 
                    const float &geo_ell, 
                    float &score);

            void test(
                    const Eigen::ArrayXf &x_ary,
                    const Eigen::ArrayXf &y_ary,
                    const Eigen::ArrayXf &z_ary,
                    const Eigen::ArrayXf &i_ary,
                    const Eigen::MatrixXf &pc1_j,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell,
                    float &score);

            void computeFunctionVectorInnerProductTBBThreadingNoScheduling(
                    const Eigen::MatrixXf &pc1, 
                    const int &num_pc1, 
                    const Eigen::MatrixXf &pc2, 
                    const int &num_pc2, 
                    const float &geo_sig,
                    const float &feature_ell, 
                    const float &geo_ell, 
                    float &score);
            void computeFunctionVectorInnerProductTBBThreadingManualScheduling(
                    const Eigen::MatrixXf &pc1, 
                    const int &num_pc1, 
                    const Eigen::MatrixXf &pc2, 
                    const int &num_pc2, 
                    const float &geo_sig,
                    const float &feature_ell, 
                    const float &geo_ell, 
                    float &score);

            void computeFunctionVectorInnerProductTBBThreadingTBBScheduling(
                    const Eigen::MatrixXf &pc1, 
                    const int &num_pc1, 
                    const Eigen::MatrixXf &pc2, 
                    const int &num_pc2, 
                    const float &geo_sig,
                    const float &feature_ell, 
                    const float &geo_ell, 
                    float &score);

            void computeFunctionOriginalInnerProductTBB(
                    const Eigen::MatrixXf &pc1,
                    const float &num_pc1,
                    const Eigen::MatrixXf &pc2,
                    const float &num_pc2,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell, 
                    float &score);

            void computeFunctionOriginalInnerProductKDTree(
                    const Eigen::MatrixXf &pc1,
                    const int &num_pc1,
                    const Eigen::MatrixXf &pc2,
                    const int &num_pc2,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell,
                    float &score);

            void computeFunctionInnerProductModes(
                    const int mode,
                    const Eigen::MatrixXf &pc1,
                    const float &num_pc1,
                    const Eigen::MatrixXf &pc2,
                    const float &num_pc2,
                    const float &geo_sig,
                    const float &feature_ell,
                    const float &geo_ell,
                    float &score);






        //     Eigen::VectorXf d_px_euler(double x11, double y11, double z11, double rpy11, double rpy12, double rpy13);
        //     Eigen::VectorXf d_py_euler(double x11, double y11, double z11, double rpy11, double rpy12, double rpy13);
        //     Eigen::VectorXf d_pz_euler(double x11, double y11, double z11, double rpy11, double rpy12, double rpy13);


            /* [Decoder]
             * Create hash table of chosen tag family
             */
            void _initDecoder();


            /* [Decoder]
             * Feed a code to test if initialized correctly
             */
            void _testInitDecoder();


            /* [ros-visualization]
             * A function to prepare for visualization results in rviz
             */
            void _clusterToPclVectorAndMarkerPublisher(const std::vector<ClusterFamily_t> &t_cluster,
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
                                                       visualization_msgs::msg::MarkerArray &t_marker_array);
            void _plotIdealFrame();
            void _plotTagFrame(const ClusterFamily_t &t_cluster);
            visualization_msgs::msg::Marker _visualizeVector(Eigen::Vector3f edge_vector, PointXYZRI centriod, int t_ID);
            /* [accumulating temporal cluster]
            * A function to save temporal clusters data
            */
            // void _saveTemporalCluster(const std::vector<ClusterFamily_t> &t_cluster, std::vector<std::vector<pcl::PointCloud<LiDARPoints_t>>> &matData);
            
            /* [save points to mat files]
            * A function to save points as .mat data
            */
            // void _saveMatFiles(std::vector<std::vector<pcl::PointCloud<LiDARPoints_t>>>& matData);
            
            // [A function to put clusterFamily to LidarTagDetectionArray]
            void _detectionArrayPublisher(const ClusterFamily_t &Cluster);

            
            /* [Drawing]
             * A function to draw lines in rviz
             */
            void _assignLine(visualization_msgs::msg::Marker &marker, 
                             visualization_msgs::msg::MarkerArray t_mark_array,
                             const uint32_t shape, const std::string ns,
                             const double r, const double g, const double b,
                             const PointXYZRI t_point1, const PointXYZRI t_point2, 
                             const int t_count);


            /* [Drawing]
             * A function to assign markers in rviz
             */
            void _assignMarker(visualization_msgs::msg::Marker &t_marker, const uint32_t t_shape, 
                               const std::string t_namespace,
                               const double r, const double g, const double b,
                               const PointXYZRI &t_point, const int t_count, 
                               const double t_size, const std::string Text="");

            void _assignVectorMarker(visualization_msgs::msg::Marker &t_marker, const uint32_t t_shape, 
                               const std::string t_namespace,
                               const double r, const double g, const double b,
                               const int t_count, const double t_size, Eigen::Vector3f t_edge_vector, const PointXYZRI &t_centriod, 
                               const std::string Text="");

            void _printStatistics(
                    const std::vector<ClusterFamily_t>& ClusterBuff);

            std::vector<int> _getValidClusters(const std::vector<ClusterFamily_t>& ClusterBuff);

            void _maxPointsCheck(ClusterFamily_t& Cluster);

            bool _rejectWithPlanarCheck( ClusterFamily_t &Cluster, 
                pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients,
                std::ostream &fplanefit);

            void _initFunctionDecoder();


            ///
            // void  _calculateCost(Eigen::Vector3f q, double & costx, double & costy, double & costz);
            // double _checkCost(double point, double cons1, double cons2);
            // double _costfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data);

            /*****************************************************
             * not used 
             *****************************************************/

            // Clean up
            void _freeUp(std::vector<ClusterFamily_t> &ClusterBuff);
            void _freeCluster(ClusterFamily_t &Cluster);
            template<typename Container>
            void _freeVec(Container& c); 
            void _freePCL(pcl::PointCloud<LiDARPoints_t*> &vec);
            void _freeTagLineStruc(TagLines_t &TagEdges);
    }; // GrizTag
} // namespace
#endif
