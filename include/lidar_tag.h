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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h> // Marker
#include <visualization_msgs/MarkerArray.h> // Marker

#include <boost/thread/mutex.hpp>

// To trasnform to pcl format
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h> 

#include "lidartag_msgs/LiDARTagDetection.h"
#include "lidartag_msgs/LiDARTagDetectionArray.h"
#include "types.h"

// #include "tensorflow_ros_test/lib.h"

namespace BipedLab{
    class LiDARTag{
        public:
            LiDARTag();

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
            int _grid_viz; // visualize remapping grid
            bool _id_decoding; // decode id or not
            bool _write_CSV; // Write CSV files 
            std::string _assign_id; // Directly assign Id, mainly for calibration usage

            // ROS
            ros::NodeHandle _nh;
            ros::Subscriber _LiDAR1_sub;
            ros::Publisher _original_pc_pub;
            ros::Publisher _edge_pub;
            ros::Publisher _cluster_pub;
            ros::Publisher _payload_pub;

            ros::Publisher _cluster_marker_pub;  // cluster markers
            ros::Publisher _boundary_marker_pub; // cluster boundary
            ros::Publisher _payload_marker_pub; // payload boundary
            ros::Publisher _payload_grid_pub; // grid visualization
            ros::Publisher _payload_grid_line_pub; // grid visualization
            ros::Publisher _lidartag_pose_pub; // Publish LiDAR pose

            // Flag
            int _point_cloud_received; // check if a scan of point cloud has received or not
            int _stop; // Just a switch for exiting this program while using valgrind

            // ros::Publisher DebugPointCheckPub_; // Debug
            // ros::Publisher DebugBoundaryPointPub_; // Debug
            // ros::Publisher DebugNoEdgePub_; // Debug
            // ros::Publisher DebugExtractPayloadPub_; // Debug

            //ros::Publisher TextPub_; // publish text
            // visualization_msgs::MarkerArray _markers; 


            // Queue for pc data
            std::queue<sensor_msgs::PointCloud2ConstPtr> _point_cloud1_queue;

            // lock
            boost::mutex _refine_lock;
            boost::mutex _buff_to_pcl_vector_lock;
            boost::mutex _point_cloud1_queue_lock;

            // LiDAR parameters
            ros::Time _current_scan_time; // store current time of the lidar scan
            std::string _pointcloud_topic; // subscribe channel
            std::string _pub_frame; // publish under what frame?

            // Overall LiDAR system parameters
            LiDARSystem_t _LiDAR_system;
            int _beam_num;
            int _vertical_fov;

            // PointCould data (for a single scan)
            int _point_cloud_size;
            std_msgs::Header _point_cloud_header;

            // Edge detection parameters
            double _intensity_threshold;
            double _depth_threshold;

            // If on-board processing is limited, limit range of points
            double _distance_bound;

            // fiducial marker parameters
            double _payload_size; // physical payload size 
            int _tag_family; // what tag family ie tag16h5, this should be 16
            int _tag_hamming_distance; // what tag family ie tag16h5, this should be 5
            int _max_decode_hamming; // max hamming while decoding 
            int _black_border; // black boarder of the fiducial marker
            
            // Cluster parameters
            double _threshold;
            double _RANSAC_threshold;
            int _fine_cluster_threshold; // TODO: REPLACE WITH TAG PARAMETERS
            int _filling_gap_max_index; // TODO: CHECK
            int _filling_max_points_threshold; //TODO: REMOVE
            double _points_threshold_factor; // TODO: CHECK

            // Payload 
            double _line_intensity_bound; // TODO: not sure but may okay to remove it for releasing
            double _payload_intensity_threshold;
            int _min_returns_per_grid;
            GrizTagFamily_t *tf;
            lidartag_msgs::LiDARTagDetectionArray _lidartag_pose_array; // an array of apriltags


            // NN
            // LoadWeight *NNptr_;
            std::string _latest_model;
            std::string _weight_path;
            int _max_point_on_payload;
            int _XYZRI; // input channels

            // Debug
            Debug_t _debug_cluster;
            Timing_t _timing;

            // Statistics
            Statistics_t _result_statistics;


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
             * A function to make sure the program has received at least one pointcloud 
             * from subscribed channel
             * at the very start of this program
             */
            inline void _waitForPC();


            /* [basic ros]
             * A function to push the received pointcloud into a queue in the class
             */
            inline void _pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pc);


            /* [Transform/Publish]
             * A function to transform pcl msgs to ros msgs and then publish
             * WhichPublisher should be a string of "organized" or "original" regardless
             * lowercase and uppercase
             */
            void _publishPC(const pcl::PointCloud<PointXYZRI>::Ptr      &t_source_PC, 
                            const std::string &t_frame_name, std::string t_which_publisher);

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




            /* [Type transformation]
             * A function to transform an eigen type of vector to pcl point type
             */
            void _eigenVectorToPointXYZRI(const Eigen::Vector4f &t_vector, PointXYZRI &t_point);


            /* [Normal vector]
             * A function to estimate the normal vector of a potential payload
             */
            Eigen::MatrixXf _estimateNormalVector(ClusterFamily_t &cluster);


            /* [Pose: tag to robot]
             * A function to publish pose of tag to the robot
             */
			void _tagToRobot(const int &t_cluster_id, const Eigen::Vector3f &t_normal_vec, 
                             Homogeneous_t &t_pose, 
							 tf::Transform &t_transform, const PointXYZRI &t_ave);


            /* [Payload decoding]
             * A function to decode payload with different means
             * 0: Naive decoding
             * 1: Weighted Gaussian
             * 2: Deep learning
             * 3: Gaussian Process
             * 4: ?!
             */
            bool _decodPayload(ClusterFamily_t &t_cluster);


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
                                                       pcl::PointCloud<PointXYZRI>::Ptr t_out_payload,
                                                       visualization_msgs::MarkerArray &t_marker_array);


            /* [Drawing]
             * A function to draw lines in rviz
             */
            void _assignLine(visualization_msgs::Marker &marker, 
                             visualization_msgs::MarkerArray t_mark_array,
                             const uint32_t shape, const std::string ns,
                             const double r, const double g, const double b,
                             const PointXYZRI t_point1, const PointXYZRI t_point2, 
                             const int t_count);


            /* [Drawing]
             * A function to assign markers in rviz
             */
            void _assignMarker(visualization_msgs::Marker &t_marker, const uint32_t t_shape, 
                               const std::string t_namespace,
                               const double r, const double g, const double b,
                               const PointXYZRI &t_point, const int t_count, 
                               const double t_size, const std::string Text="");


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
