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


#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/common/intersections.h>
#include <pcl/features/integral_image_normal.h>

#include <Eigen/Dense> // SVD
#include <unsupported/Eigen/MatrixFunctions> // matrix exponential
#include <unsupported/Eigen/CXX11/Tensor> // tensor output

#include <ros/package.h> // package
#include "lidar_tag.h"
#include "apriltag_utils.h"
#include "utils.h"
#include "tag49h14.h"
#include "tag16h5.h"

#include <unistd.h>
#include <algorithm>    // std::sort
#include <math.h>        /* sqrt, pow(a,b) */
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <stdlib.h>     /* srand, rand */
#include <fstream> // log files


/* CONSTANT */
#define SQRT2 1.41421356237
#define MAX_INTENSITY 255
#define MIN_INTENSITY 0

using namespace std;

namespace BipedLab {
    LiDARTag::LiDARTag():
        _nh("~"),
        _point_cloud_received(0),
		_pub_frame("velodyne"), // what frame of the published pointcloud should be 
        _stop(0) // Just a switch for exiting this program while using valgrind
    {
            LiDARTag::_getParameters();
            cout << "\033[1;32m\n\n===== loading tag family ===== \033[0m\n";
            LiDARTag::_initDecoder();

            if (_decode_method!=0 && _decode_method!=1){
                ROS_ERROR("Please use 0 or 1 for decode_method in the launch file");
                ROS_INFO_STREAM("currently using: "<< _decode_method);
            }
            cout << "\033[1;32m=========================== \033[0m\n";
            cout << "\033[1;32m=========================== \033[0m\n";

            ROS_INFO("ALL INITIALIZED!");
            _LiDAR1_sub = _nh.subscribe<sensor_msgs::PointCloud2>(_pointcloud_topic, 50, 
                                                                 &LiDARTag::_pointCloudCallback, this);
			_edge_pub = _nh.advertise<sensor_msgs::PointCloud2>("EdgedPC", 10);
			_cluster_pub = _nh.advertise<sensor_msgs::PointCloud2>("DetectedPC", 10);
			_payload_pub = _nh.advertise<sensor_msgs::PointCloud2>("Payload", 10);
            _boundary_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>( "BoundaryMarker", 10);
			_cluster_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("ClusterMarker", 10);
			_payload_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("PayloadEdges", 10);
			_payload_grid_pub = _nh.advertise<visualization_msgs::MarkerArray>("Grid", 10);
			_payload_grid_line_pub = _nh.advertise<visualization_msgs::Marker>("GridLine", 10);
            _lidartag_pose_pub = _nh.advertise<lidartag_msgs::LiDARTagDetectionArray>("LiDARTagPose", 1);

            boost::thread RosSpin(&LiDARTag::_rosSpin, this); // put ros spin into a background thread

            ROS_INFO("Waiting for pointcloud data");
            LiDARTag::_waitForPC();

            // Exam the minimum distance of each point in a ring
            ROS_INFO("Analyzing system");
            LiDARTag::_analyzeLiDARDevice();
            boost::thread ExtractionSpin(&LiDARTag::_mainLoop, this);
            ExtractionSpin.join();
            RosSpin.join();
    }


    /*
     * A function to get all parameters from a roslaunch
     * if not get all parameters then it will use hard-coded parameters
     */
    void LiDARTag::_getParameters(){
        bool GotSleepToDisplay = ros::param::get("sleep_to_display", _sleep_to_display);
        bool GotSleepTimeForVis = ros::param::get("sleep_time_for_visulization", _sleep_time_for_vis);
        bool GotValgrindCheck = ros::param::get("valgrind_check", _valgrind_check);
        bool GotFakeTag = ros::param::get("fake_data", _fake_tag);

        bool GotCSV = ros::param::get("write_csv", _write_CSV);
        bool GotDecodeId = ros::param::get("decode_id", _id_decoding);
        bool GotAssignId = ros::param::get("assign_id", _assign_id);

        bool GotAdaptiveThresholding = ros::param::get("adaptive_thresholding", _adaptive_thresholding);
        bool GotCollectData = ros::param::get("collect_data", _collect_dataset);

        bool GotLidarTopic = ros::param::get("pointcloud_topic", _pointcloud_topic);
        bool GotBeamNum = ros::param::get("beam_number", _beam_num);
        bool GotSize = ros::param::get("tag_size", _payload_size);

        bool GotTagFamily = ros::param::get("tag_family", _tag_family);
        bool GotTagHamming = ros::param::get("tag_hamming_distance", _tag_hamming_distance);
        bool GotMaxDecodeHamming = ros::param::get("max_decode_hamming", _max_decode_hamming);
        bool GotBlackBorder = ros::param::get("black_border", _black_border);

        bool GotDistanceBound = ros::param::get("distance_bound", _distance_bound);
        bool GotIntensityBound = ros::param::get("intensity_bound", _intensity_threshold);
        bool GotDepthBound = ros::param::get("depth_bound", _depth_threshold);
        bool GotFineClusterThreshold = ros::param::get("fine_cluster_threshold", _fine_cluster_threshold);
        bool GotVerticalFOV = ros::param::get("vertical_fov", _vertical_fov);
        bool GotFillInGapThreshold = ros::param::get("fill_in_gap_threshold", _filling_gap_max_index);
        bool GotFillInMaxPointsThreshold = ros::param::get("fill_in_max_points_threshold", _filling_max_points_threshold);
        bool GotPointsThresholdFactor = ros::param::get("points_threshold_factor", _points_threshold_factor);
        bool GotLineIntensityBound = ros::param::get("line_intensity_bound", _line_intensity_bound);
        bool GotPayloadIntensityThreshold = ros::param::get("payload_intensity_threshold", _payload_intensity_threshold);

        bool GotLatestModel = ros::param::get("latest_model", _latest_model);
        bool GotWeightPath = ros::param::get("weight_path", _weight_path);

        bool GotMaxPointsOnPayload = ros::param::get("max_points_on_payload", _max_point_on_payload);
        bool GotXYZRI = ros::param::get("xyzri", _XYZRI);
        bool GotMinPerGrid = ros::param::get("min_retrun_per_grid", _min_returns_per_grid);
        bool GotDecodeMethod = ros::param::get("decode_method", _decode_method);
        bool GotGridViz = ros::param::get("grid_viz", _grid_viz);

        bool Pass = utils::checkParameters(33, GotFakeTag, GotLidarTopic, GotBeamNum, 
                                            GotDecodeId, GotAssignId, GotCSV,
                                            GotDistanceBound, GotIntensityBound, GotDepthBound, 
                                            GotSize, GotTagFamily, GotTagHamming, GotMaxDecodeHamming,
                                            GotFineClusterThreshold, GotVerticalFOV, 
                                            GotFillInGapThreshold, GotFillInMaxPointsThreshold, 
                                            GotPointsThresholdFactor, GotLineIntensityBound, 
                                            GotAdaptiveThresholding, GotCollectData, GotSleepToDisplay, 
                                            GotSleepTimeForVis, GotValgrindCheck, 
                                            GotLatestModel, GotWeightPath,
                                            GotMaxPointsOnPayload, GotXYZRI, GotMinPerGrid, GotDecodeMethod, 
                                            GotGridViz);

        if (!Pass){
            // TODO: check compleness
            cout << "\033[1;32m=========================== \033[0m\n";
            cout << "use hard-coded parameters\n";
            cout << "\033[1;32m=========================== \033[0m\n";

            // Default value
            _pointcloud_topic = "/velodyne_points";
            _beam_num = 32;
            _distance_bound = 7; // for edge gradient
            _intensity_threshold = 2; // for edge gradient
            _depth_threshold = 0.5; // for edge gradient
            _payload_intensity_threshold = 30;
            _payload_size = 0.15;

            _tag_family = 16;
            _tag_hamming_distance = 5;
            _max_decode_hamming = 2;

            _fine_cluster_threshold = 20; // if the points in a cluster is small than this, it'd get dropped 
            _vertical_fov = 40;

            // When fill in the cluster, if it the index is too far away then drop it
            // TODO:Need a beteer way of doing it!
            _filling_gap_max_index = 200; 
            _filling_max_points_threshold = 4500; 

            _line_intensity_bound = 1000; // To determine the payload edge

            // if the points on a "valid" tag is less than this factor, then remove
            // it  (the higher, the looser)
            _points_threshold_factor = 1.3; 
            _adaptive_thresholding = 0;
            _collect_dataset = 1;
    
            _sleep_to_display = 1;
            _sleep_time_for_vis = 0.05;
            _valgrind_check = 0;
            _black_border = 2;
            _fake_tag = 0;

            _latest_model = "-337931";
            _weight_path = "/weight/";
            _max_point_on_payload = 150;
            _XYZRI = 4 ;
            _min_returns_per_grid = 3;
            _decode_method = 2;
            _grid_viz = 1;
        }
        else{
            cout << "\033[1;32m=========================== \033[0m\n";
            cout << "use parameters from the launch file\n";
            cout << "\033[1;32m=========================== \033[0m\n";
        }

        _threshold = _payload_size/4; // point association threshold (which cluster the point belongs to?)
        _RANSAC_threshold = _payload_size/10;

        ROS_INFO("Subscribe to %s\n", _pointcloud_topic.c_str());
        ROS_INFO("Use %i-beam LiDAR\n", _beam_num);
        ROS_INFO("_intensity_threshold: %f \n", _intensity_threshold);
        ROS_INFO("_depth_threshold: %f \n", _depth_threshold);
        ROS_INFO("_payload_size: %f \n", _payload_size);
        ROS_INFO("_vertical_fov: %i \n", _vertical_fov);
        ROS_INFO("_fine_cluster_threshold: %i \n", _fine_cluster_threshold);
        ROS_INFO("_filling_gap_max_index: %i \n", _filling_gap_max_index);
        ROS_INFO("_filling_max_points_threshold: %i \n", _filling_max_points_threshold);
        ROS_INFO("_points_threshold_factor: %f \n", _points_threshold_factor);
        ROS_INFO("_adaptive_thresholding: %i \n", _adaptive_thresholding);
        ROS_INFO("_collect_dataset: %i \n", _collect_dataset);
        ROS_INFO("_decode_method: %i \n", _decode_method);
        ROS_INFO("Threshold_: %f \n", _threshold);
        ROS_INFO("_RANSAC_threshold: %f \n", _RANSAC_threshold);

        usleep(2e6);
    } 


    /*
     * Main loop
     */
    void LiDARTag::_mainLoop(){
        ROS_INFO("Start edge extraction");
        //ros::Rate r(10); // 10 hz
        ros::Duration duration(_sleep_time_for_vis);
        int Count = 0; // just to caculate average speed
        clock_t StartAve = clock();
        pcl::PointCloud<PointXYZRI>::Ptr ClusterPC(new pcl::PointCloud<PointXYZRI>);
        pcl::PointCloud<PointXYZRI>::Ptr PayloadPC(new pcl::PointCloud<PointXYZRI>);
        ClusterPC->reserve(_point_cloud_size);

        // XXX Tunalbe
        PayloadPC->reserve(_point_cloud_size);
        int valgrind_check = 0;
        while (ros::ok()) {
            _lidartag_pose_array.detections.clear();
            _timing = {clock(), clock(), clock(), 0,0,0,0,0,0,0,0,0};

            // Try to take a pointcloud from the buffer
            std::vector<std::vector<LiDARPoints_t>> OrderedBuff = LiDARTag::_getOrderBuff();
            if (OrderedBuff.empty()){
                continue;
            }

            // A vector of clusters
            vector<ClusterFamily_t> ClusterBuff; 

            pcl::PointCloud<PointXYZRI>::Ptr ExtractedEdgesPC = LiDARTag::_lidarTagDetection(OrderedBuff, 
                                                                                             ClusterBuff);
            ClusterPC->clear();
            PayloadPC->clear();
            
            // Prepare results for rviz
            visualization_msgs::MarkerArray ClusterMarkers;
            LiDARTag::_clusterToPclVectorAndMarkerPublisher(ClusterBuff, ClusterPC, PayloadPC, ClusterMarkers); 

            // publish lidartag poses
            _lidartag_pose_pub.publish(_lidartag_pose_array);

            // publish results for rviz
            LiDARTag::_publishPC(ExtractedEdgesPC, _pub_frame, string("edge"));
            LiDARTag::_publishPC(ClusterPC, _pub_frame, string("Cluster"));
            if (_collect_dataset){
                if (_result_statistics.remaining_cluster_size==1)
                    LiDARTag::_publishPC(PayloadPC, _pub_frame, string("Payload"));
                else if (_result_statistics.remaining_cluster_size>1)
                    cout << "More than one!! " << endl;
                else 
                    cout << "Zero!! " << endl;
            }
            else
                LiDARTag::_publishPC(PayloadPC, _pub_frame, string("Payload"));

            //exit(0);
            if (_sleep_to_display) duration.sleep(); 
            Count++;
            _timing.total_time = utils::spendTime(clock(), _timing.start_total_time);
            ROS_INFO_STREAM("Hz (total): " << 1e3/_timing.total_time);
            cout << "\033[1;31m====================================== \033[0m\n";
            if (_valgrind_check){
                valgrind_check++;
                if (valgrind_check > 0) {
                    _stop = 1;
                    break;
                }
            }
        }
    }


    /* 
     * A function to get pcl OrderedBuff from a ros sensor-msgs form of pointcould queue
     * */
    std::vector<std::vector<LiDARPoints_t>> LiDARTag::_getOrderBuff(){
        _point_cloud1_queue_lock.lock();
        if (_point_cloud1_queue.size()==0) {
            _point_cloud1_queue_lock.unlock();
            //ros::spinOnce();
            //cout << "Pointcloud queue is empty" << endl;
            //cout << "size: " << Empty.size() << endl;
            vector<vector<LiDARPoints_t>> Empty;
            return Empty;
        }
        ROS_INFO_STREAM("Queue size: " << _point_cloud1_queue.size());

        sensor_msgs::PointCloud2ConstPtr msg = _point_cloud1_queue.front();
        _current_scan_time = msg->header.stamp;

        // Convert to sensor_msg to pcl type
        pcl::PointCloud<PointXYZRI>::Ptr PclPointcloud(new pcl::PointCloud<PointXYZRI>);
        pcl::fromROSMsg(*msg, *PclPointcloud);
        _point_cloud1_queue.pop();
        _point_cloud1_queue_lock.unlock();

        std::vector<std::vector<LiDARPoints_t>> OrderedBuff(_beam_num);

        // Ordered pointcloud with respect to its own ring number
        _fillInOrderedPC(PclPointcloud, OrderedBuff);
        _point_cloud_size = PclPointcloud->size();

        return OrderedBuff;
    }


    /*
     * A function to get a LiDAR system parameters such as max, min points per scan
     * and how many points per ring
     * The data format is:
     *
     * (A) PointCountTable:
     * PointCountTable[Scan][Ring]
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
     * (B) MaxMinTable:
     * ----------------------------------------
     *   1      2      3 .... scan
     *  Max    Max    Max
     *  Min    Min    Min
     * ---------------------------------------
     */
    void LiDARTag::_analyzeLiDARDevice(){
        clock_t begin = clock();
        int NumberOfScan = 0;
        _LiDAR_system.point_count_table.resize(50);
        _LiDAR_system.ring_average_table.reserve(_beam_num);

        // Initialize the table
        MaxMin_t max_min{100000, -1, -1}; // min, ave, max

        for (int j=0; j<_beam_num; ++j){
            _LiDAR_system.ring_average_table.push_back(max_min);
        }

        // Calulate for each scan with a few seconds 
        while (ros::ok()){
            std::vector<std::vector<LiDARPoints_t>> OrderedBuff = LiDARTag::_getOrderBuff();
            if (OrderedBuff.empty()){
                continue;
            }
            LiDARTag::_maxMinPtsInAScan(_LiDAR_system.point_count_table[NumberOfScan], 
                                       _LiDAR_system.max_min_table, 
                                       _LiDAR_system.ring_average_table, 
                                       OrderedBuff);
            NumberOfScan++;
            clock_t end = clock();
            if (((double) (end-begin)/ CLOCKS_PER_SEC) > 3){
                break;
            }
        }
        for (auto i=_LiDAR_system.ring_average_table.begin(); i!=_LiDAR_system.ring_average_table.end(); ++i){
            (*i).average /= NumberOfScan;
        }
        
        LiDARTag::_pointsPerSquareMeterAtOneMeter();




        // Check values of pointtable and maxmintable
        // int k = 0;
        // for (auto i=_LiDAR_system.PointCountTable.begin(); i!=_LiDAR_system.PointCountTable.end(); ++i, ++k){
        //     cout << "Vector[" << k << "]:" << endl;
        //     for (auto j=(*i).begin(); j!=(*i).end(); ++j){
        //         cout << "points: " << *j << endl;
        //     }
        // }

        // k=0;
        // for (auto i=_LiDAR_system.MaxMinTable.begin(); i!=_LiDAR_system.MaxMinTable.end(); ++i, ++k){
        //     cout << "At scan [" << k << "]" << endl;
        //     cout << "Max: " << (*i).Max << endl;
        //     cout << "Min: " << (*i).Min << endl;
        // }

        // k=0;
        // for (auto i=_LiDAR_system.ring_average_table.begin(); i!=_LiDAR_system.ring_average_table.end(); ++i, ++k){
        //     cout << "At ring [" << k << "]" << endl;
        //     cout << "Max: " << (*i).Max << endl;
        //     cout << "Ave: " << (*i).average << endl;
        //     cout << "Min: " << (*i).Min << endl;
        // }
        // exit(0);
    }


    /*
     * A function to calculate how many points are supposed to be on a cluster at 1
     * meter away
     */
    void LiDARTag::_pointsPerSquareMeterAtOneMeter(){
        double system_average;

        for (auto i=_LiDAR_system.ring_average_table.begin(); 
                  i!=_LiDAR_system.ring_average_table.end(); ++i){
            system_average += (*i).average;
        }
        system_average /= _LiDAR_system.ring_average_table.size();
        _LiDAR_system.beam_per_vertical_radian = _beam_num / utils::deg2Rad(_vertical_fov);
        _LiDAR_system.point_per_horizontal_radian = system_average / utils::deg2Rad(360);
    }


    /*
     * A function to get a number of points on a given-distance tag or object
     */
    int LiDARTag::_areaPoints(const double &Distance, const double &ObjWidth, const double &ObjHeight){
        double WAngle = ObjWidth * (1 + SQRT2) / abs(Distance);

        if (WAngle>=1) return (int) 1e6; // return big number to reject the cluster

        double HAngle = ObjHeight * (1 + SQRT2) / abs(Distance);
        if (HAngle>=1) return (int) 1e6; // return big number to reject the cluster

        double HorizontalAngle = asin(WAngle); // in radian
        double VerticalAngle = asin(HAngle); // in radian
        int NumOfVerticalRing = floor(VerticalAngle * _LiDAR_system.beam_per_vertical_radian);
        int NumOfHorizontalPoints = floor(HorizontalAngle * _LiDAR_system.point_per_horizontal_radian);

        // use 3 instead of 2 becasue of we assume the tag would be put in the dense
        // region of LiDAR (LiDAR is denser in the middle)
        int Area = floor(3 * (NumOfVerticalRing * NumOfHorizontalPoints) / (1 + SQRT2)); 

        //cout << "distance: " << abs(Distance) << endl;
        //cout << "HorizontalAngle: " << HorizontalAngle << endl;
        //cout << "VerticalAngle: " << VerticalAngle << endl;

        //cout << "NumOfVerticalRing: " << NumOfVerticalRing << endl;
        //cout << "NumOfHorizontalPoints: " << NumOfHorizontalPoints << endl;
        //cout << "Area: " << Area << endl;

        return Area;
    }

    
    /* 
     * A function to find maximum points and minimum points in a single scan, i.e. to
     * find extrema within 32 rings
     */
    void LiDARTag::_maxMinPtsInAScan(std::vector<int> &PointCountTable, 
                                    std::vector<MaxMin_t> &MaxMinTable,
                                    std::vector<MaxMin_t> &RingAverageTable,
                                    const std::vector<std::vector<LiDARPoints_t>>& OrderedBuff){

        // every scan should have different largest/ smallest numbers
        int Largest = -1;
        int Smallest = 100000;
        MaxMin_t max_min;

        int i = 0;
        for (auto Ring=OrderedBuff.begin(); Ring!=OrderedBuff.end(); ++Ring){
            int RingSize = (*Ring).size();
            PointCountTable.push_back(RingSize);

            // first time (we had initialized with -1)
            if (RingAverageTable[i].average<0) {
                RingAverageTable[i].average = RingSize; 
            }
            else{
                RingAverageTable[i].average = (RingAverageTable[i].average + RingSize);
            }

            if (RingAverageTable[i].max<RingSize){
                RingAverageTable[i].max = RingSize;
            }

            if (RingAverageTable[i].min>RingSize){
                RingAverageTable[i].min = RingSize;
            }
            
            if (RingSize > Largest) {
                Largest = RingSize;
                max_min.max = RingSize;
            }

            if (RingSize < Smallest) {
                Smallest = RingSize;
                max_min.min = RingSize;
            }
            i++;
        }
        MaxMinTable.push_back(max_min);
    }


    /*
     * A function to transfer pcl msgs to ros msgs and then publish
     * WhichPublisher should be a string of "organized" or "original" regardless
     * lowercase and uppercase
     */
	void LiDARTag::_publishPC(const pcl::PointCloud<PointXYZRI>::Ptr &SourcePC, 
                              const std::string &Frame, string WhichPublisher){
        utils::tranferToLowercase(WhichPublisher); // check letter cases
		sensor_msgs::PointCloud2 PCsWaitedToPub;      
		pcl::toROSMsg(*SourcePC, PCsWaitedToPub);
		PCsWaitedToPub.header.frame_id = Frame;	

        try { 
            if (WhichPublisher=="edge") _edge_pub.publish(PCsWaitedToPub);
            else if (WhichPublisher=="original") _original_pc_pub.publish(PCsWaitedToPub);
            else if (WhichPublisher=="cluster") _cluster_pub.publish(PCsWaitedToPub);
            else if (WhichPublisher=="payload") _payload_pub.publish(PCsWaitedToPub);
            else {
                throw "No such Publisher exists";
            }
        } 
        catch (const char* msg){
            cout << "\033[1;31m========================= \033[0m\n";
            cerr << msg << endl;
            cout << "\033[1;31m========================= \033[0m\n";
            exit(-1);
        }
	}


    /* [basic ros]
     * A function to push the received pointcloud into a queue in the class
     */
    void LiDARTag::_pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pc){
        _point_cloud_received = 1; // flag to make sure it receives a pointcloud at the very begining of the program
        _point_cloud_header = pc->header;
        boost::mutex::scoped_lock(_point_cloud1_queue_lock);
        _point_cloud1_queue.push(pc);
    }


    /*
     * A function to make sure the program has received at least one pointcloud at
     * the very start of this program
     */
    inline 
    void LiDARTag::_waitForPC(){
        while (ros::ok()){
            if (_point_cloud_received) {
                ROS_INFO("Got pointcloud data");
                return;
            }
            ros::spinOnce();
        }
    }


    /* 
     * A function to slice the Veloydyne full points to sliced pointed based on ring
     * number
     * */
	inline
    void LiDARTag::_fillInOrderedPC(const pcl::PointCloud<PointXYZRI>::Ptr &PclPointcloud, 
                                  std::vector<std::vector<LiDARPoints_t>>& OrderedBuff) {
        LiDARPoints_t LiDARPoint;
        int index [_beam_num] = {0};
		for (auto && p : *PclPointcloud) {
            LiDARPoint.point = p;
            LiDARPoint.index = index[p.ring];
            index[p.ring] += 1;
			OrderedBuff[p.ring].push_back(LiDARPoint);
		}
    }


    /*
     * Main function 
     */
	pcl::PointCloud<PointXYZRI>::Ptr
	LiDARTag::_lidarTagDetection(const std::vector<std::vector<LiDARPoints_t>>& OrderedBuff, 
                                 std::vector<ClusterFamily_t> &ClusterBuff){

        _timing.start_computation_time = clock();
		pcl::PointCloud<PointXYZRI>::Ptr Out(new pcl::PointCloud<PointXYZRI>);
		Out->reserve(_point_cloud_size);
        std::vector<std::vector<LiDARPoints_t>> EdgeBuff(_beam_num); // Buff to store all detected edges

        // calculate gradient for depth and intensity as well as group them into diff groups
        _result_statistics = {{0, 0, 0, 0, 0, 0}, 0, 0, 0, 0};
        _timing.timing = clock();
        LiDARTag::_gradientAndGroupEdges(OrderedBuff, EdgeBuff, ClusterBuff);
        _timing.edgingand_clustering_time = utils::spendTime(clock(), _timing.timing);

        _timing.timing = clock();
        // transform from a vector of vector (EdgeBuff) into a pcl vector (out)
        boost::thread BuffToPclVectorThread(&LiDARTag::_buffToPclVector, this, boost::ref(EdgeBuff), Out);
        _timing.to_pcl_vector_time = utils::spendTime(clock(), _timing.timing);


        LiDARTag::_fillInCluster(OrderedBuff, ClusterBuff); 
        BuffToPclVectorThread.join();
        _result_statistics.point_cloud_size = _point_cloud_size;
        _result_statistics.edge_cloud_size = Out->size();

        _timing.computation_time = utils::spendTime(clock(), _timing.start_computation_time);
        ROS_DEBUG_STREAM("--------------- summary ---------------");
        ROS_DEBUG_STREAM("Original cloud size: " << _result_statistics.point_cloud_size);
        ROS_DEBUG_STREAM("--Edge cloud size: " << _result_statistics.edge_cloud_size);
        ROS_DEBUG_STREAM("Original cluster: " << _result_statistics.original_cluster_size);
        ROS_DEBUG_STREAM("--Point check Removed: " << _result_statistics.cluster_removal.removed_by_point_check);
        ROS_DEBUG_STREAM("--Minimum return removed: " << _result_statistics.cluster_removal.minimum_return);
        ROS_DEBUG_STREAM("--Boundary point Removed: " << _result_statistics.cluster_removal.boundary_point_check);
        ROS_DEBUG_STREAM("--No Edge Removed: " << _result_statistics.cluster_removal.no_edge_check);
        ROS_DEBUG_STREAM("--Not Enough for decode Removed: " << _result_statistics.cluster_removal.decoder_not_return);
        ROS_DEBUG_STREAM("--Corners Removed: " << _result_statistics.cluster_removal.decoder_fail_corner);
        ROS_DEBUG_STREAM("--Decode-Fail Removed: " << _result_statistics.cluster_removal.decode_fail);
        ROS_DEBUG_STREAM("--Remaining Cluster: " << _result_statistics.remaining_cluster_size);
        ROS_DEBUG_STREAM("computation_time Hz: " << 1/_timing.computation_time*1e3);
        ROS_DEBUG_STREAM("--------------- Timing ---------------");
        ROS_DEBUG_STREAM("computation_time: " << _timing.computation_time);
        ROS_DEBUG_STREAM("edgingand_clustering_time: " << _timing.edgingand_clustering_time);
        ROS_DEBUG_STREAM("PointCheck: " << _timing.point_check_time);
        ROS_DEBUG_STREAM("LineFitting: " << _timing.line_fitting_time);
        ROS_DEBUG_STREAM("ExtractPayload: " << _timing.payload_extraction_time);
        ROS_DEBUG_STREAM("NormalVector: " << _timing.normal_vector_time);
        ROS_DEBUG_STREAM("PayloadDecoder: " << _timing.payload_decoding_time);
        ROS_DEBUG_STREAM("_tagToRobot: " << _timing.tag_to_robot_time);

		return Out;
	}


    /* 
     * A function to 
     * (1) calculate the depth gradient and the intensity gradient at a point of a pointcloud
     * (2) group detected 'edge' into different group
     */
	void LiDARTag::_gradientAndGroupEdges(const std::vector<std::vector<LiDARPoints_t>>& OrderedBuff, 
                                        std::vector<std::vector<LiDARPoints_t>>& EdgeBuff,
                                        std::vector<ClusterFamily_t> &ClusterBuff) {

        // clock_t start = clock();
        // TODO: if suddently partial excluded, it will cause errors
		for(int i=0; i<_beam_num; i++) {
			for(int j=2; j<OrderedBuff[i].size()-2; j++) {
				const auto& point = OrderedBuff[i][j].point;
                if (std::abs(point.x) > _distance_bound ||
                    std::abs(point.y) > _distance_bound || 
                    std::abs(point.z) > _distance_bound) continue;
				const auto& PointL = OrderedBuff[i][j-2].point;
				const auto& PointR = OrderedBuff[i][j+2].point;

				double DepthGrad = std::max((PointL.getVector3fMap()-point.getVector3fMap()).norm(),
                                            (point.getVector3fMap()-PointR.getVector3fMap()).norm());

                // double IntenstityGrad = std::max(std::abs(PointL.intensity - point.intensity),
                //                                  std::abs(point.intensity - PointR.intensity));

				// if (IntenstityGrad > _intensity_threshold &&
                //     DepthGrad > _depth_threshold) {
				if (DepthGrad > _depth_threshold) {

                    // Cluster the detected 'edge' into different groups
                    _clusterClassifier(OrderedBuff[i][j], ClusterBuff);
                    
                    // push the detected point that is an edge into a buff
                    LiDARPoints_t LiDARPoints = {OrderedBuff[i][j].point, OrderedBuff[i][j].index,
                                                 DepthGrad, 0};
					EdgeBuff[i].push_back(LiDARPoints);
				}
			}
		}
        // clock_t end = clock();
        // cout << "extraction hz: " << 1/ (((double) (end - start))/clocks_per_sec) << endl;
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
	void LiDARTag::_fillInCluster(const std::vector<std::vector<LiDARPoints_t>>& OrderedBuff, 
                                 std::vector<ClusterFamily_t> &ClusterBuff){

        _result_statistics.original_cluster_size = ClusterBuff.size();
        _result_statistics.remaining_cluster_size = ClusterBuff.size();
        for (int i=0; i<ClusterBuff.size(); ++i){
            // In a cluster 
            for (int j=0; j<_beam_num; ++j){
                //cout << "\033[1;31m============== \033[0m\n";
                // cout << "(i, j): " << i << ", " << j << endl;
                int MaxIndex = ClusterBuff[i].max_min_index_of_each_ring[j].max;
                int MinIndex = ClusterBuff[i].max_min_index_of_each_ring[j].min;
                //cout << "MaxInex: " << MaxIndex << endl;
                //cout << "MinInex: " << MinIndex << endl;
                
                // If the differnce of index is too far, remove it 
                if (MaxIndex-MinIndex>_filling_gap_max_index) continue;

                // push points between index into the cluster 
                for (int k=MinIndex+1; k < MaxIndex; ++k){ // remove minimum index itself (it has been in the cloud already)
                    ClusterBuff[i].data.push_back(OrderedBuff[j][k]);
                    //cout << "point added:" << k << endl;
                }
            }

            // If the points in this cluster after filling are still less than the
            // factored threshold, then remove it
            _timing.timing = clock();
            if(!_clusterPointsCheck(ClusterBuff[i])){ 
                _timing.point_check_time += utils::spendTime(clock(), _timing.timing);
                //cout << "cluster removed" << endl;
			    ClusterBuff[i].valid = 0;	
                //ClusterBuff.erase(ClusterBuff.begin()+i);
                // _debug_cluster.point_check.push_back(&ClusterBuff[i]);
                //i--;
                _result_statistics.remaining_cluster_size -= 1;
                _result_statistics.cluster_removal.removed_by_point_check ++;
            }
            // Adaptive thresholding (Maximize and minimize intensity) by comparing
            // with the average value
            else {
                _timing.point_check_time += utils::spendTime(clock(), _timing.timing);
                //boost::thread BuffToPclVectorThread(&LiDARTag::AdaptiveThresholding, this, boost::ref(ClusterBuff[i]));
                if(!LiDARTag::_adaptiveThresholding(ClusterBuff[i])) {
                    ClusterBuff[i].valid = 0;	
                    _result_statistics.remaining_cluster_size -= 1;
                    // ClusterBuff.erase(ClusterBuff.begin()+i);
                    // i--;
                }
            }
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
    bool LiDARTag::_adaptiveThresholding(ClusterFamily_t &Cluster){
        Cluster.ordered_points_ptr.resize(_beam_num);
        Cluster.payload_right_boundary_ptr.resize(_beam_num);
        Cluster.payload_left_boundary_ptr.resize(_beam_num);

        for (int k=0; k<Cluster.data.size(); ++k){
            LiDARPoints_t* ClusterPointPtr = &Cluster.data[k];
            ClusterPointPtr->point.intensity = Cluster.data[k].point.intensity/Cluster.max_intensity.intensity;
            // Calucate "biased" average intensity value
            // if (ClusterPointPtr->point.intensity > MIN_INTENSITY)
            //     Cluster.accumulate_intensity_of_each_ring[ClusterPointPtr->point.ring] += ClusterPointPtr->point.intensity;
            // else 
            //     Cluster.accumulate_intensity_of_each_ring[ClusterPointPtr->point.ring] -= MAX_INTENSITY; // Punish if it is black

            Cluster.ordered_points_ptr[ClusterPointPtr->point.ring].push_back(ClusterPointPtr);
        }

        // Fit line of the cluster
        _timing.timing = clock();
        if (!LiDARTag::_detectPayloadBoundries(Cluster)){
            _timing.line_fitting_time += utils::spendTime(clock(), _timing.timing);

            return false;
        }
        else {
            _timing.line_fitting_time += utils::spendTime(clock(), _timing.timing);
            _timing.timing = clock();
            _extractPayloadWOThreshold(Cluster);
            _timing.payload_extraction_time += utils::spendTime(clock(), _timing.timing);
            
            // 2 is because this payload points is actually includes black
            // boundary 
            if (Cluster.payload.size() < _min_returns_per_grid*std::pow((std::sqrt(_tag_family)+2*_black_border), 2)) {
                //_debug_cluster.ExtractPayload.push_back(&Cluster);
                _result_statistics.cluster_removal.minimum_return ++;
                return false;
            }

            // return true;
            if (_id_decoding){
                _timing.timing = clock();
                if (LiDARTag::_decodPayload(Cluster)){
                    _timing.payload_decoding_time += utils::spendTime(clock(), _timing.timing);

                    _timing.timing = clock();
                    Cluster.normal_vector = _estimateNormalVector(Cluster); 
                    _timing.normal_vector_time += utils::spendTime(clock(), _timing.timing);

                    _timing.timing = clock();
                    LiDARTag::_tagToRobot(Cluster.cluster_id, Cluster.normal_vector, 
                                        Cluster.pose, Cluster.transform, Cluster.average);
                    _timing.tag_to_robot_time += utils::spendTime(clock(), _timing.timing);
                    return true;
                }
                else {
                    return false;
                }
            }
            else {
                LiDARTag::_decodPayload(Cluster);

                // directly assign ID
                string Code(_assign_id);
                uint64_t Rcode = stoull(Code, nullptr, 2);
                BipedAprilLab::QuickDecodeCodeword(tf, Rcode, &Cluster.entry);
                Cluster.cluster_id = Cluster.entry.id;

                Cluster.normal_vector = _estimateNormalVector(Cluster); 
                LiDARTag::_tagToRobot(Cluster.cluster_id, Cluster.normal_vector, 
                                    Cluster.pose, Cluster.transform, Cluster.average);
                return true;
            }
        }
    } 

    /* A function to publish pose of tag to the robot
     */
    void LiDARTag::_tagToRobot(const int &cluster_id, const Eigen::Vector3f &NormalVec, 
                               Homogeneous_t &pose, 
                               tf::Transform &transform, const PointXYZRI &Ave){
        Eigen::Vector3f x(1, 0, 0);
        Eigen::Vector3f y(0, 1, 0);
        Eigen::Vector3f z(0, 0, 1);
        pose.rotation = utils::qToR(NormalVec).cast<float> ();
        pose.translation << Ave.x, Ave.y, Ave.z;

        pose.yaw = utils::rad2Deg(acos(NormalVec.dot(y)));
        pose.pitch = -utils::rad2Deg(acos(NormalVec.dot(x)));
        pose.roll = utils::rad2Deg(acos(NormalVec.dot(z)));

        pose.homogeneous.topLeftCorner(3,3) = pose.rotation;
        pose.homogeneous.topRightCorner(3,1) = pose.translation;
        pose.homogeneous.row(3) << 0,0,0,1;

        static tf::TransformBroadcaster Broadcaster_;
        transform.setOrigin(tf::Vector3(Ave.x, Ave.y, Ave.z));

        // rotate to fit iamge frame
        Eigen::Vector3f qr(0, std::sqrt(2)/2, 0);
        float qr_w = std::sqrt(2)/2;
        // Eigen::Vector3f qr(-0.5, 0.5, -0.5);
        // float qr_w = 0.5;
        
        // Eigen::Vector3f qr(-0.5, -0.5, -0.5);
        // float qr_w = 0.5;
        // Eigen::Vector3f qr(std::sqrt(2)/2, 0, 0);
        // float qr_w = std::sqrt(2)/2;
        // Eigen::Vector3f qiCameraFrame = qr_w*NormalVec + 0*qr + qr.cross(NormalVec); // 0 is q.w of normalvec
        // float qwCameraFrame = qr_w*0 - qr.dot(NormalVec); // 0 is q.w of normalvec
        Eigen::Vector3f qiCameraFrame = NormalVec + 2*qr_w*(qr.cross(NormalVec)) + 2*qr.cross(qr.cross(NormalVec)); // 0 is q.w of normalvec
        float qwCameraFrame = 0; // 0 is q.w of normalvec
         

        Eigen::Vector3f q_i = qiCameraFrame; 
        double q_w = qwCameraFrame;
        double norm = std::sqrt(std::pow(q_i(0), 2) + std::pow(q_i(1), 2) + std::pow(q_i(2), 2) + std::pow(q_w, 2));
        q_i = (q_i/norm).eval();
        q_w = q_w/norm;
        tf::Quaternion q(q_i(0), q_i(1), q_i(2), q_w);
        transform.setRotation(q);
        Broadcaster_.sendTransform(tf::StampedTransform(transform, _point_cloud_header.stamp, 
                                                        _pub_frame, to_string(cluster_id)+"_rotated"));


        tf::Quaternion q2(NormalVec(0), NormalVec(1), NormalVec(2), 0);
        transform.setRotation(q2);
        Broadcaster_.sendTransform(tf::StampedTransform(transform, _point_cloud_header.stamp, 
                                                        _pub_frame, "LiDARTag-ID" + to_string(cluster_id)));

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
        lidartag_msg.pose.position.x = Ave.x;
        lidartag_msg.pose.position.y = Ave.y;
        lidartag_msg.pose.position.z = Ave.z;
        lidartag_msg.pose.orientation = geo_q;
        lidartag_msg.header = _point_cloud_header;
        lidartag_msg.header.frame_id = std::string("lidartag_") + to_string(cluster_id);
        _lidartag_pose_array.header = _point_cloud_header;
        _lidartag_pose_array.detections.push_back(lidartag_msg);
        // cout << "R.T*NV: " << endl << pose.rotation.transpose()*NormalVec << endl;
        // cout << "H: " << endl << pose.homogeneous << endl;

        /*
        Eigen::Vector3f x(1, 0, 0);
        Eigen::Vector3f y(0, 1, 0);
        Eigen::Vector3f z(0, 0, 1);
        Eigen::Matrix3f zSkew;
        zSkew << 0, -z(2), z(1),
                 z(2), 0, -z(0),
                -z(1), z(0), 0;
        Eigen::Vector3f u = zSkew*NormalVec;
        //u = x.cross(NormalVec);
        //u = z.cross(NormalVec);
        //u = -z.cross(NormalVec);
        //u = -x.cross(NormalVec);
        //u = -y.cross(NormalVec);
        //u = x.cross(NormalVec);

        u = (u.normalized()).eval();
        float theta = acos(z.dot(NormalVec));
        u = (u*theta).eval();
        Eigen::Matrix3f uSkew;
        uSkew << 0, -u(2), u(1),
                u(2), 0, -u(0),
               -u(1), u(0), 0;

        pose.rotation = uSkew.exp();
        pose.translation << Ave.x, Ave.y, Ave.z;
        pose.yaw = utils::rad2Deg(acos(NormalVec.dot(y)));
        pose.pitch = -utils::rad2Deg(acos(NormalVec.dot(x)));
        pose.roll = utils::rad2Deg(acos(NormalVec.dot(z)));

        pose.homogeneous.topLeftCorner(3,3) = pose.rotation;
        pose.homogeneous.topRightCorner(3,1) = pose.translation;
        pose.homogeneous.row(3) << 0,0,0,1;

        static tf::TransformBroadcaster Broadcaster_;
        transform.setOrigin(tf::Vector3(Ave.x, Ave.y, Ave.z));
        Eigen::Vector3f q_i = sin(theta/2)*u;
        double q_w = std::cos(theta/2);
        double norm = std::sqrt(std::pow(q_i(0), 2) + std::pow(q_i(1), 2) + std::pow(q_i(2), 2) + std::pow(q_w, 2));
        q_i = (q_i/norm).eval();
        q_w = q_w/norm;
        tf::Quaternion q(q_i(0), q_i(1), q_i(2), q_w);
        transform.setRotation(q);
        Broadcaster_.sendTransform(tf::StampedTransform(transform, _point_cloud_header.stamp, 
                                                        _pub_frame, to_string(cluster_id)));

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
        lidartag_msg.pose.position.x = Ave.x;
        lidartag_msg.pose.position.y = Ave.y;
        lidartag_msg.pose.position.z = Ave.z;
        lidartag_msg.pose.orientation = geo_q;
        lidartag_msg.header = _point_cloud_header;
        lidartag_msg.header.frame_id = std::string("lidartag_") + to_string(cluster_id);
        _lidartag_pose_array.header = _point_cloud_header;
        _lidartag_pose_array.detections.push_back(lidartag_msg);
        // cout << "R.T*NV: " << endl << pose.rotation.transpose()*NormalVec << endl;
        // cout << "H: " << endl << pose.homogeneous << endl;
        */
    }



    /* <A cluster>
     * A function to extract the payload points from a valid cluster.
     * Let's say we have 10 points on the left boundary (line) of the tag and 9 points on the right boundary
     * (line) of the tag.
     * It is seperated into two parts.
     * TODO: should use medium instead of max points
     *  (i) Find the max points that we have on a ring in this cluster by
     *      exmaming the average points on the first 1/2 rings int((10+9)/4)
     * (ii) For another half of the rings, we just find the start index and add the
     *      average number of points to the payload points
     */
    void LiDARTag::_extractPayloadWOThreshold(ClusterFamily_t &Cluster){
        int LastRoundLength = 0; // Save to recover a missing ring
        PointXYZRI average{0,0,0,0};
        for(int Ring=0; Ring<_beam_num; ++Ring){

            // if (Cluster.payload_right_boundary_ptr[Ring]!=0)
            //     Cluster.payload_boundary_ptr.push_back(Cluster.payload_right_boundary_ptr[Ring]);

            // if (Cluster.payload_left_boundary_ptr[Ring]!=0)
            //     Cluster.payload_boundary_ptr.push_back(Cluster.payload_left_boundary_ptr[Ring]);

            if (Cluster.payload_right_boundary_ptr[Ring]==0 && 
                Cluster.payload_left_boundary_ptr[Ring]==0) continue;
            // cout << "Ring" << Ring << endl;
            else if (Cluster.payload_right_boundary_ptr[Ring]!=0 && 
                Cluster.payload_left_boundary_ptr[Ring]!=0){
                Cluster.payload_boundary_ptr.push_back(Cluster.payload_right_boundary_ptr[Ring]);
                Cluster.payload_boundary_ptr.push_back(Cluster.payload_left_boundary_ptr[Ring]);
                int StartIndex = Cluster.payload_left_boundary_ptr[Ring]->index;
                int EndIndex = Cluster.payload_right_boundary_ptr[Ring]->index;
                LastRoundLength = EndIndex - StartIndex;

                
                for (int j=0; j<Cluster.ordered_points_ptr[Ring].size(); ++j){
                    if (Cluster.ordered_points_ptr[Ring][j]->index == StartIndex){
                        // Remove index itself because itself is not the part of a
                        // payload
                        for (int k=j+1; k<j+(EndIndex-StartIndex); ++k){ // since start from 0
                            if (k>=Cluster.ordered_points_ptr[Ring].size()) break; // make sure the index is valid
                            // cout << "j: " << j << endl;
                            // cout << "k: " << k << endl;
                            // cout << "validation1: " << endl;
                            // utils::COUT(Cluster.ordered_points_ptr[Ring][k]->point);
                            //
                            Cluster.payload.push_back(Cluster.ordered_points_ptr[Ring][k]);
                            average.x += Cluster.ordered_points_ptr[Ring][k]->point.x;
                            average.y += Cluster.ordered_points_ptr[Ring][k]->point.y;
                            average.z += Cluster.ordered_points_ptr[Ring][k]->point.z;
                        }
                        break;
                    }
                }
            }
            Cluster.average.x = average.x/ Cluster.payload.size();
            Cluster.average.y = average.y/ Cluster.payload.size();
            Cluster.average.z = average.z/ Cluster.payload.size();
            // else if (LastRoundLength!=0 && Cluster.payload_right_boundary_ptr[Ring]!=0){
            //     int EndIndex = Cluster.payload_right_boundary_ptr[Ring]->index;

            //     for (int j=Cluster.ordered_points_ptr[Ring].size()-1; j>0; --j){
            //         if (Cluster.ordered_points_ptr[Ring][j]->index == EndIndex){
            //             Cluster.payload.push_back(Cluster.ordered_points_ptr[Ring][j]);

            //             for (int k=j-1; k>j-LastRoundLength; --k){
            //                 if (k<0) break; // make sure the index is valid
            //                 Cluster.payload.push_back(Cluster.ordered_points_ptr[Ring][k]);
            //             }
            //             break;
            //         }
            //     }

            // }
            // else if (LastRoundLength!=0 && Cluster.payload_left_boundary_ptr[Ring]!=0){
            //     int StartIndex = Cluster.payload_left_boundary_ptr[Ring]->index;

            //     for (int j=0; j<Cluster.ordered_points_ptr[Ring].size(); ++j){
            //         if (Cluster.ordered_points_ptr[Ring][j]->index == StartIndex){
            //             Cluster.payload.push_back(Cluster.ordered_points_ptr[Ring][j]);

            //             for (int k=j-1; k<j+LastRoundLength; ++k){ // since start from 0
            //                 if (k>=Cluster.ordered_points_ptr[Ring].size()) break; // make sure the index is valid
            //                 Cluster.payload.push_back(Cluster.ordered_points_ptr[Ring][k]);
            //             }
            //             break;
            //         }
            //     }

            // }
        }
    }


    bool LiDARTag::_detectPayloadBoundries(ClusterFamily_t &Cluster){
        // return true;
        bool FitOkay = false; // Meaning reject this cluster and will be removed

        // Initialization
        Cluster.tag_edges.upper_ring = _beam_num;
        Cluster.tag_edges.lower_ring = 0;
        double AverageIntensity = ((Cluster.max_intensity.intensity + 
                                   Cluster.min_intensity.intensity)/2);
        double DetectionThreshold_ = (AverageIntensity - Cluster.min_intensity.intensity)/
                                      (_payload_intensity_threshold*Cluster.max_intensity.intensity);
        //cout << "Detection Threshold: " << DetectionThreshold_ << endl;

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
         */
        int BoundaryPointCount = 0; // Check there're sufficient points
        float RightMin = 1000;
        float RightMax = -1000;
        float LeftMin = 1000;
        float LeftMax = -1000;
        PointXYZRI PTOP;
        PointXYZRI PDown;
        PointXYZRI PLeft;
        PointXYZRI PRight;
        for (int Ring=0; Ring<_beam_num; ++Ring){
            // skip this ring if doesn't exist  
            if (Cluster.ordered_points_ptr[Ring].size()<int(sqrt(_tag_family))) {
                // cout << "Ring: " << Ring << endl;
                //cout << "size: " << Cluster.ordered_points_ptr[Ring].size() << endl;
                continue;
            }
            // cout << "Ring: " << Ring << endl;
            // cout << "size: " << Cluster.ordered_points_ptr[Ring].size() << endl;
            // cout << "ave intensity: " << AverageIntensity/Cluster.max_intensity.intensity << endl;
            // cout << "det intensity: " << DetectionThreshold_ << endl;
            
            /* [Right]
             * Find edges from the right a ring
             * -1-2: -1 of index and another -2 is becasue we take into account another two points on the right
             * -1  : because we compare with left point
             */
            sort(Cluster.ordered_points_ptr[Ring].begin(), Cluster.ordered_points_ptr[Ring].end(), utils::compareIndex);
            for (int P=Cluster.ordered_points_ptr[Ring].size()-2; 
                 P>floor((Cluster.ordered_points_ptr[Ring].size()-2)/2); 
                 //P>1; 
                 --P){
                /* 
                 * (1) By knowing it from white to black on the left calucate the
                 * intensity gradient
                 * (2) Since have thresholded already, we * could also choose > 255
                 */
                // cout << "p intensity: " << Cluster.ordered_points_ptr[Ring][P]->point.intensity << endl;
                // cout << "PR: " << P << endl;
                // cout << "PointR: " << Cluster.ordered_points_ptr[Ring][P]->index << endl;
                // utils::COUT(Cluster.ordered_points_ptr[Ring][P]->point);
                // Cluster.payload_right_boundary_ptr[Ring] = Cluster.ordered_points_ptr[Ring][P];
                // FitOkay = true;
                // break;
                if ((Cluster.ordered_points_ptr[Ring][P]->point.intensity - 
                     Cluster.ordered_points_ptr[Ring][P-1]->point.intensity>DetectionThreshold_) &&
                    (Cluster.ordered_points_ptr[Ring][P+1]->point.intensity - 
                     Cluster.ordered_points_ptr[Ring][P-1]->point.intensity>DetectionThreshold_)) {

                    Cluster.payload_right_boundary_ptr[Ring] = Cluster.ordered_points_ptr[Ring][P];
                    RightMin = (RightMin < Cluster.ordered_points_ptr[Ring][P]->point.y) ? RightMin : Cluster.ordered_points_ptr[Ring][P]->point.y;
                    RightMax = (RightMax > Cluster.ordered_points_ptr[Ring][P]->point.y) ? RightMax : Cluster.ordered_points_ptr[Ring][P]->point.y;
                    BoundaryPointCount ++;

                    break;
                }
            }


            /* [Left]
             * Find edges from the left of a ring
             * Start from 1 becasue we take another point on the left into account
             * size -1 because we compare with next point
             */
            // for (int P=2; P<Cluster.ordered_points_ptr[Ring].size()-1; ++P){
            for (int P=2; 
                 P<ceil((Cluster.ordered_points_ptr[Ring].size()-1)/2); 
                 // P<Cluster.ordered_points_ptr[Ring].size()-1; 
                 ++P){
                /* 
                 * (1) By knowing it from white to black on the left calucate the
                 * intensity gradient
                 * (2) Since have thresholded already, we * could also choose > 255
                 * (3) To determin if p if the edge: 
                 *     1. compare with p+1 (to its right)
                 */
                //cout << "PL: " << P << endl;
                //Cluster.payload_left_boundary_ptr[Ring] = Cluster.ordered_points_ptr[Ring][P];
                //cout << "PointL: " << Cluster.ordered_points_ptr[Ring][P]->index << endl;
                //// utils::COUT(Cluster.ordered_points_ptr[Ring][P]->point);
                //FitOkay = true;

                //break;
                if ((Cluster.ordered_points_ptr[Ring][P]->point.intensity - 
                     Cluster.ordered_points_ptr[Ring][P+1]->point.intensity>DetectionThreshold_) &&
                    (Cluster.ordered_points_ptr[Ring][P-1]->point.intensity - 
                     Cluster.ordered_points_ptr[Ring][P+1]->point.intensity>DetectionThreshold_)) {

                    Cluster.payload_left_boundary_ptr[Ring] = Cluster.ordered_points_ptr[Ring][P];
                    LeftMin = (LeftMin < Cluster.ordered_points_ptr[Ring][P]->point.y) ? LeftMin : Cluster.ordered_points_ptr[Ring][P]->point.y;
                    LeftMax = (LeftMax > Cluster.ordered_points_ptr[Ring][P]->point.y) ? LeftMax : Cluster.ordered_points_ptr[Ring][P]->point.y;
                    BoundaryPointCount ++;

                    break;
                }
            }
        }

        // reject if points are too less (can't even get decoded!)
        if (BoundaryPointCount < int(sqrt(_tag_family))*2) {
            _result_statistics.cluster_removal.boundary_point_check++;
            // _debug_cluster.BoundaryPoint.push_back(&Cluster);
            return false;
        } else FitOkay = true;

        if (!FitOkay) {
            _result_statistics.cluster_removal.no_edge_check++;
            // _debug_cluster.NoEdge.push_back(&Cluster);
        }

        return FitOkay;
    }





    /* [Normal vector]
     * A function to estimate the normal vector of a potential payload
     */
    Eigen::MatrixXf
    LiDARTag::_estimateNormalVector(ClusterFamily_t &Cluster){
        Eigen::MatrixXf EigenPC(3, Cluster.payload.size());

        for (int i=0; i<Cluster.payload.size(); ++i){
            EigenPC(0,i) = Cluster.payload[i]->point.x - Cluster.average.x;
            EigenPC(1,i) = Cluster.payload[i]->point.y - Cluster.average.y;
            EigenPC(2,i) = Cluster.payload[i]->point.z - Cluster.average.z;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(EigenPC, Eigen::ComputeFullU);
        Eigen::Matrix<float,3,1,Eigen::DontAlign> normal_vector_robot = svd.matrixU().col(2); // Take the last column of U in SVD

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
        if (normal_vector_robot(0)>=0) normal_vector_robot = (-normal_vector_robot).eval();
        // cout << "Normal Vector3: \n" << normal_vector_robot << endl;

        // Coordinate transform
         // Eigen::Matrix<float,3,1,Eigen::DontAlign> normal_vector_tag;
         // normal_vector_tag << normal_vector_robot(2), normal_vector_robot(0), normal_vector_robot(1);
         // normal_vector_tag << normal_vector_robot(0), normal_vector_robot(2), normal_vector_robot(1);

        
        return normal_vector_robot;

        // pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud (new pcl::PointCloud<pcl::PointXYZ>);

        // for (int i=0; i<Cluster.payload.size(); ++i){
        //     pcl::PointXYZ point = {Cluster.payload[i]->point.x, 
        //                            Cluster.payload[i]->point.y, 
        //                            Cluster.payload[i]->point.z};
        //     cout << "point: " << point << endl;
        //     Cloud->push_back(point);
        // }
        // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

        // // pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        // // ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        // // ne.setMaxDepthChangeFactor(0.02f);
        // // ne.setNormalSmoothingSize(10.0f);
        // // ne.setInputCloud(Cloud);
        // // ne.compute(*normals);
        // // cout << "NV: " << *normals << endl;

        // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> NE;
        // NE.setInputCloud (Cloud);
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        // NE.setSearchMethod (tree);

        // // Output datasets
        // pcl::PointCloud<pcl::Normal>::Ptr CloudNormals (new pcl::PointCloud<pcl::Normal>);
        // NE.setRadiusSearch (1);
        // NE.compute (*CloudNormals);
        // cout << "normals: " << *CloudNormals << endl;
        // std::cout << "cloud_normals->points.size (): " << CloudNormals->points.size () << std::endl;
        // 
        // return  CloudNormals;
    }

    /*
     * A function of ros spin
     * reason: in order to put it into background as well as able to run other tasks
     */
    void LiDARTag::_rosSpin(){
        while (ros::ok() && !_stop){
            ros::spinOnce();
        }
    }


    /*
     * A function to cluster a single point into a new cluster or an existing cluster 
     */
    void LiDARTag::_clusterClassifier(const LiDARPoints_t &point, vector<ClusterFamily_t> &ClusterBuff){
        // The first time to cluster the point cloud
        int ValidCluster = 1; // Marker every cluster is valid and will be checked again later 
        if (ClusterBuff.size()==0){
            PointXYZRI top_most_point = point.point;
            top_most_point.z = top_most_point.z + _threshold;
            PointXYZRI bottom_most_point = point.point;
            bottom_most_point.z -= _threshold;

            PointXYZRI front_most_point = point.point;
            front_most_point.x += _threshold;
            PointXYZRI back_most_point = point.point;
            back_most_point.x -= _threshold;

            PointXYZRI right_most_point = point.point;
            right_most_point.y -= _threshold;
            PointXYZRI left_most_point = point.point;
            left_most_point.y += _threshold;
            //cout << "_threshold:" << _threshold << endl;
            //
            //cout << "\033[1;31m============== \033[0m\n";
            //cout << "First created" << endl;
            //cout << "TopMost: " << top_most_point.x << ", " << top_most_point.y << ", " << top_most_point.z << endl;
            //cout << "bottom_most_point: " << bottom_most_point.x << ", " << bottom_most_point.y << ", " << bottom_most_point.z << endl;
            //cout << "Front: " << front_most_point.x << ", " << front_most_point.y << ", " << front_most_point.z << endl;
            //cout << "back: " << back_most_point.x << ", " << back_most_point.y << ", " << back_most_point.z << endl;
            //cout << "Right: " << right_most_point.x << ", " << right_most_point.y << ", " << right_most_point.z << endl;
            //cout << "Left: " << left_most_point.x << ", " << left_most_point.y << ", " << left_most_point.z << endl;

            ClusterFamily_t CurrentCluster = {0, ValidCluster, top_most_point, bottom_most_point, 
                                                               front_most_point, back_most_point, 
                                                               right_most_point, left_most_point,
                                                               point.point};
            MaxMin_t InitialValue = {(int)1e8, 0, -1};
            CurrentCluster.max_min_index_of_each_ring.resize(_beam_num, InitialValue);
            CurrentCluster.max_min_index_of_each_ring[point.point.ring].max = point.index;
            CurrentCluster.max_min_index_of_each_ring[point.point.ring].min = point.index;

            CurrentCluster.max_intensity = point.point;
            CurrentCluster.min_intensity = point.point;

            CurrentCluster.data.push_back(point);
            ClusterBuff.push_back(CurrentCluster);
            return ;
        }
        else {
            // Take a point to go through all the existing cluster to see if this
            // point belongs to any of them 
            // Once it is found belonging to one of the clusters then return.
            // After looping though and couldn't find a belonging group then add it
            // to a new cluster
            TestCluster_t *new_cluster = new TestCluster_t{0};
            //cout << "\033[1;31m============== \033[0m\n";
            //cout << "ClusterBuff size: " << ClusterBuff.size() << endl;
            for (int i=0; i<ClusterBuff.size(); ++i){
                _updateCluster(point, ClusterBuff[i], new_cluster);
                if (!(new_cluster->flag)) {
                    delete new_cluster;
                    return;
                }
            }
            // Add a new cluster 
            if (new_cluster->flag){
                //cout << "new cluster added" << endl;
                int Newcluster_id = ClusterBuff.size();
                PointXYZRI top_most_point = point.point;
                top_most_point.z += _threshold;
                PointXYZRI bottom_most_point = point.point;
                bottom_most_point.z -= _threshold;

                PointXYZRI front_most_point = point.point;
                front_most_point.x += _threshold;
                PointXYZRI back_most_point = point.point;
                back_most_point.x -= _threshold;

                PointXYZRI right_most_point = point.point;
                right_most_point.y -= _threshold;
                PointXYZRI left_most_point = point.point;
                left_most_point.y += _threshold;

                //cout << "TopMost: " << top_most_point.x << ", " << top_most_point.y << ", " << top_most_point.z << endl;
                //cout << "bottom_most_point: " << bottom_most_point.x << ", " << bottom_most_point.y << ", " << bottom_most_point.z << endl;
                //cout << "Front: " << front_most_point.x << ", " << front_most_point.y << ", " << front_most_point.z << endl;
                //cout << "back: " << back_most_point.x << ", " << back_most_point.y << ", " << back_most_point.z << endl;
                //cout << "Right: " << right_most_point.x << ", " << right_most_point.y << ", " << right_most_point.z << endl;
                //cout << "Left: " << left_most_point.x << ", " << left_most_point.y << ", " << left_most_point.z << endl;


                // Check every time when new marker added 
                // visualization_msgs::MarkerArray CheckArray;
                // visualization_msgs::Marker CheckMarker;
                // LiDARTag::_assignMarker(CheckMarker, visualization_msgs::Marker::CUBE, 
                //                       "Check0", 
                //                       1, 0, 0,
                //                       top_most_point, 0, 0.05);
                // CheckArray.markers.push_back(CheckMarker);

                // LiDARTag::_assignMarker(CheckMarker, visualization_msgs::Marker::CUBE, 
                //                       "Check1", 
                //                       1, 0, 0,
                //                       bottom_most_point, 1, 0.05);
                // CheckArray.markers.push_back(CheckMarker);

                // LiDARTag::_assignMarker(CheckMarker, visualization_msgs::Marker::CUBE, 
                //                       "Check2", 
                //                       1, 0, 0,
                //                       front_most_point, 2, 0.05);
                // CheckArray.markers.push_back(CheckMarker);

                // LiDARTag::_assignMarker(CheckMarker, visualization_msgs::Marker::CUBE, 
                //                       "Check3", 
                //                       1, 0, 0,
                //                       back_most_point, 3, 0.05);
                // CheckArray.markers.push_back(CheckMarker);

                // LiDARTag::_assignMarker(CheckMarker, visualization_msgs::Marker::CUBE, 
                //                       "Check4", 
                //                       1, 0, 0,
                //                       left_most_point, 4, 0.05);
                // CheckArray.markers.push_back(CheckMarker);

                // LiDARTag::_assignMarker(CheckMarker, visualization_msgs::Marker::CUBE, 
                //                       "Check5", 
                //                       1, 0, 0,
                //                       right_most_point, 5, 0.05);
                // CheckArray.markers.push_back(CheckMarker);

                // LiDARTag::_assignMarker(CheckMarker, visualization_msgs::Marker::SPHERE, 
                //                       "Check6", 
                //                       0, 1, 0,
                //                       Point, 5, 0.1);
                // CheckArray.markers.push_back(CheckMarker);

                // _boundary_marker_pub.publish(CheckArray); 
                //utils::pressEnterToContinue();


                new_cluster->new_cluster = {Newcluster_id, ValidCluster, top_most_point, bottom_most_point, 
                                                                      front_most_point, back_most_point, 
                                                                      right_most_point, left_most_point,
                                                                      point.point};

				// To fill in points between initial points and end points later
                MaxMin_t InitialValue = {(int)1e8, 0, -1};
                new_cluster->new_cluster.max_min_index_of_each_ring.resize(_beam_num, InitialValue);
                new_cluster->new_cluster.max_min_index_of_each_ring[point.point.ring].max = point.index;
                new_cluster->new_cluster.max_min_index_of_each_ring[point.point.ring].min = point.index;

                new_cluster->new_cluster.max_intensity = point.point;
                new_cluster->new_cluster.min_intensity = point.point;

                new_cluster->new_cluster.data.push_back(point);
                ClusterBuff.push_back(new_cluster->new_cluster);
            }
            delete new_cluster;
        }
    }


    /*
     * A function update some information about a cluster if this point belongs to
     * this cluster; if not belonging to this cluster then return and create a new
     * one 
     */
    void LiDARTag::_updateCluster(const LiDARPoints_t &point, ClusterFamily_t &OldCluster, TestCluster_t *new_cluster){
        // This point is outside of the current cluster
        if (!(
             (point.point.z < OldCluster.top_most_point.z + _threshold)  && 
             (point.point.x < OldCluster.front_most_point.x + _threshold)  &&  
             (point.point.y < OldCluster.left_most_point.y + _threshold)   &&  
             (OldCluster.back_most_point.x  - _threshold< point.point.x)   &&  
             (OldCluster.right_most_point.y  - _threshold< point.point.y)  &&  
             (OldCluster.bottom_most_point.z  - _threshold< point.point.z))){
            
            // cout << "\033[1;31m============== \033[0m\n";
            // cout << "New flag" << endl;
            // cout << "point: " << point.point.x << ", " << point.point.y << ", " << point.point.z << endl;
            // cout << "TOP.z: " << OldCluster.top_most_point.z << ", " << point.point.z << endl;
            // cout << "Front.x: " << OldCluster.front_most_point.x << ", " << point.point.x << endl;
            // cout << "Left.y: " << OldCluster.left_most_point.y << ", " << point.point.y << endl;
            // cout << "Right.y: " << OldCluster.right_most_point.y << ", " << point.point.y  << endl;
            // cout << "Bottom.z: " << OldCluster.bottom_most_point.z << ", " << point.point.z  << endl;
            // cout << "Back.x: " << OldCluster.back_most_point.x << ", " << point.point.x  << endl;
            // utils::pressEnterToContinue();
            new_cluster->flag = 1;
            return ;
        }
        else{
            new_cluster->flag = 0;
        }

        // This point is inside this cluster 
        if (!new_cluster->flag){
            // update the boundary of the old cluster
            if (point.point.x + _threshold > OldCluster.front_most_point.x) {
                OldCluster.front_most_point = point.point;
                OldCluster.front_most_point.x += _threshold;
            }
            if (point.point.x - _threshold < OldCluster.back_most_point.x) {
                OldCluster.back_most_point = point.point;
                OldCluster.back_most_point.x -= _threshold;
            }

            if (point.point.y + _threshold > OldCluster.left_most_point.y) {
                OldCluster.left_most_point = point.point;
                OldCluster.left_most_point.y += _threshold;
            }
            if (point.point.y  - _threshold< OldCluster.right_most_point.y) {
                OldCluster.right_most_point = point.point;
                OldCluster.right_most_point.y -= _threshold;
            }

            if (point.point.z + _threshold > OldCluster.top_most_point.z) {
                OldCluster.top_most_point = point.point;
                OldCluster.top_most_point.z += _threshold;
            }
            if (point.point.z  - _threshold < OldCluster.bottom_most_point.z) {
                OldCluster.bottom_most_point = point.point;
                OldCluster.bottom_most_point.z -= _threshold;
            }

            // update the average // spend around 5-6 HZ
            OldCluster.average.getVector3fMap() = ((OldCluster.average.getVector3fMap() * OldCluster.data.size() + 
                                                   point.point.getVector3fMap()) / (OldCluster.data.size()+1)).eval();

            // update the max/min index of each ring in this cluster 
            if (OldCluster.max_min_index_of_each_ring[point.point.ring].max < point.index)
                OldCluster.max_min_index_of_each_ring[point.point.ring].max = point.index;

            if (OldCluster.max_min_index_of_each_ring[point.point.ring].min > point.index)
                OldCluster.max_min_index_of_each_ring[point.point.ring].min = point.index;

            // update the max/min intensity of this cluster 
            if (OldCluster.max_intensity.intensity < point.point.intensity)
                OldCluster.max_intensity = point.point;
            
            if (OldCluster.min_intensity.intensity > point.point.intensity)
                OldCluster.min_intensity = point.point;

            OldCluster.data.push_back(point);
        }
    }




    /* 
     * A function to transform from a customized type (LiDARpoints_t) of vector of vector (EdgeBuff) 
     * into a standard type (PointXYZRI) of pcl vector (out)
     */
    void LiDARTag::_buffToPclVector(const std::vector<std::vector<LiDARPoints_t>> &EdgeBuff,
                                 pcl::PointCloud<PointXYZRI>::Ptr Out){
        for (int RingNumber=0; RingNumber<_beam_num; ++RingNumber){
            if (EdgeBuff.at(RingNumber).size()!=0){
                for (int i=0; i<EdgeBuff.at(RingNumber).size(); ++i){
                    Out->push_back(EdgeBuff[RingNumber][i].point);
                }
            }
        }
    }


    /*
     * A function to prepare for displaying results in rviz
     */
    void LiDARTag::_clusterToPclVectorAndMarkerPublisher(const std::vector<ClusterFamily_t> &Cluster,
                                                        pcl::PointCloud<PointXYZRI>::Ptr OutCluster, 
                                                        pcl::PointCloud<PointXYZRI>::Ptr OutPayload, 
                                                        visualization_msgs::MarkerArray &ClusterArray){

        /* initialize random seed for coloring the marker*/
        srand (time(NULL));
        visualization_msgs::MarkerArray BoundMarkArray;
        visualization_msgs::MarkerArray PayloadMarkArray;


        int PointsInClusters = 0;
        for (int Key=0; Key<Cluster.size(); ++Key){
            if (Cluster[Key].valid != 1) continue;
            visualization_msgs::Marker Marker;
            visualization_msgs::Marker BoundaryMarker;
            
            // pick a random color for each cluster 
            double r = (double) rand() / RAND_MAX;
            double g = (double) rand() / RAND_MAX;
            double b = (double) rand() / RAND_MAX;

            // Draw boundary marker of each cluster
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::CUBE, 
                                  "Boundary" + to_string(Cluster[Key].cluster_id), 
                                  r, g, b,
                                  Cluster[Key].top_most_point, 0, 0.02);
            BoundMarkArray.markers.push_back(BoundaryMarker);
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::CUBE, 
                                  "Boundary" + to_string(Cluster[Key].cluster_id), 
                                  r, g, b,
                                  Cluster[Key].bottom_most_point, 1, 0.02);
            BoundMarkArray.markers.push_back(BoundaryMarker);
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::CUBE, 
                                  "Boundary" + to_string(Cluster[Key].cluster_id), 
                                  r, g, b,
                                  Cluster[Key].front_most_point, 2, 0.02);
            BoundMarkArray.markers.push_back(BoundaryMarker);
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::CUBE, 
                                  "Boundary" + to_string(Cluster[Key].cluster_id), 
                                  r, g, b,
                                  Cluster[Key].back_most_point, 3, 0.02);
            BoundMarkArray.markers.push_back(BoundaryMarker);
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::CUBE, 
                                  "Boundary" + to_string(Cluster[Key].cluster_id), 
                                  r, g, b,
                                  Cluster[Key].right_most_point, 4, 0.02);
            BoundMarkArray.markers.push_back(BoundaryMarker);
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::CUBE, 
                                  "Boundary" + to_string(Cluster[Key].cluster_id), 
                                  r, g, b,
                                  Cluster[Key].left_most_point, 5, 0.02);
            BoundMarkArray.markers.push_back(BoundaryMarker);
            



            // Display cluster information
            // how many points are supposed to be on the this tag
            int AvePoints = LiDARTag::_areaPoints(Cluster[Key].average.x, _payload_size, _payload_size);  
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::SPHERE, 
                                  "AveragePoint" + to_string(Cluster[Key].cluster_id), 
                                  1, 0, 0,
                                  Cluster[Key].average, 1, 0.05);
            BoundMarkArray.markers.push_back(BoundaryMarker);

            int SupposedPoints = LiDARTag::_areaPoints(Cluster[Key].average.x, _payload_size, _payload_size);  
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::TEXT_VIEW_FACING, 
                                  "Text" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 1,
                                  Cluster[Key].average, 1, 0.05, 
                                  string(
                                      to_string(Cluster[Key].cluster_id) + ", " + 
                                         "\nAt: " + to_string(Cluster[Key].average.x) + ", " +
                                                  to_string(Cluster[Key].average.y) + ", " +
                                                  to_string(Cluster[Key].average.z) + 
                                         "\nNormal vector: " + to_string(Cluster[Key].normal_vector(0)) + ", " +
                                                               to_string(Cluster[Key].normal_vector(1)) + ", " +
                                                               to_string(Cluster[Key].normal_vector(2)) +
                                         // "\nSupposed points: " + to_string((int)SupposedPoints/_points_threshold_factor) + 
                                         "\nActual points: " + to_string(Cluster[Key].data.size()) + ", " +
                                         "\nPayload points: " + to_string(Cluster[Key].payload_without_boundary) + ", " +
                                         "\nPose_xyz: " + to_string(Cluster[Key].pose.translation[0]) + ", " + 
                                                      to_string(Cluster[Key].pose.translation[1]) + ", " +
                                                      to_string(Cluster[Key].pose.translation[2]) +
                                         "\nPose_rpy: " + to_string(Cluster[Key].pose.roll) + ", " + 
                                                      to_string(Cluster[Key].pose.pitch) + ", " +
                                                      to_string(Cluster[Key].pose.yaw) +
                                         "\nIntensity: " + to_string(Cluster[Key].max_intensity.intensity) + ", " +
                                                           to_string(Cluster[Key].min_intensity.intensity)
                                      )
                                  );
            BoundMarkArray.markers.push_back(BoundaryMarker);
            if (_write_CSV){
                std::fstream file;
                std::string path("/home/brucebot/workspace/griztag/src/griz_tag/results");

                // write poses
                file.open(path + "/LiDARTag.txt", 
                           ios::in|ios::out|ios::app);
                if (file.is_open()){
                    file << Cluster[Key].cluster_id << ", ";
                    file << Cluster[Key].pose.translation[0] << ", ";
                    file << Cluster[Key].pose.translation[1] << ", ";
                    file << Cluster[Key].pose.translation[2] << ", ";
                    file << Cluster[Key].pose.roll << ", ";
                    file << Cluster[Key].pose.pitch << ", ";
                    file << Cluster[Key].pose.yaw << "\n ";
                    file.close();
                }
                else{
                    cout << "FAILED opening LiDARTag.txt" << endl;
                    exit(0);
                }

                // write Timing
                file.open(path + "/Analysis.txt", ios::in|ios::out|ios::app);
                if (file.is_open()){
                    file << _timing.computation_time << ",";
                    file << _timing.edgingand_clustering_time << ","; 
                    file << _timing.point_check_time << ",";
                    file << _timing.line_fitting_time << ",";
                    file << _timing.payload_extraction_time << ","; 
                    file << _timing.normal_vector_time << ",";
                    file << _timing.payload_decoding_time << ",";
                    file << _timing.tag_to_robot_time << ",";
                    file << Cluster[Key].data.size() << ",";
                    file << Cluster[Key].payload_without_boundary << ",";
                    file << _result_statistics.point_cloud_size << ",";
                    file << _result_statistics.edge_cloud_size << ",";
                    file << _result_statistics.original_cluster_size << "\n";
                    file.close();
                }
                else{
                    cout << "FAILED opening Analysis.txt" << endl;
                    exit(0);
                }
            }

            // Normal vector
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::ARROW, 
                                  "NormalVector" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 0,
                                  Cluster[Key].average, 1, 0.01);
            BoundaryMarker.scale.x = 0.15;
            // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?  
            //                                     -Cluster[Key].normal_vector(0) : Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.x = Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.y = Cluster[Key].normal_vector(1);
            BoundaryMarker.pose.orientation.z = Cluster[Key].normal_vector(2);
            BoundaryMarker.pose.orientation.w = 0; 
            BoundMarkArray.markers.push_back(BoundaryMarker);

            /*
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::ARROW, 
                                  "NormalVector1" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 0,
                                  Cluster[Key].average, 1, 0.01);
            BoundaryMarker.scale.x = 0.15;
            // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?  
            //                                     -Cluster[Key].normal_vector(0) : Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.x = Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.y = Cluster[Key].normal_vector(2);
            BoundaryMarker.pose.orientation.z = Cluster[Key].normal_vector(1);
            BoundaryMarker.pose.orientation.w = 0; 
            BoundMarkArray.markers.push_back(BoundaryMarker);

            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::ARROW, 
                                  "NormalVector2" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 0,
                                  Cluster[Key].average, 1, 0.01);
            BoundaryMarker.scale.x = 0.15;
            // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?  
            //                                     -Cluster[Key].normal_vector(0) : Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.x = Cluster[Key].normal_vector(1);
            BoundaryMarker.pose.orientation.y = Cluster[Key].normal_vector(2);
            BoundaryMarker.pose.orientation.z = Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.w = 0; 
            BoundMarkArray.markers.push_back(BoundaryMarker);

            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::ARROW, 
                                  "NormalVector3" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 0,
                                  Cluster[Key].average, 1, 0.01);
            BoundaryMarker.scale.x = 0.15;
            // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?  
            //                                     -Cluster[Key].normal_vector(0) : Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.x = Cluster[Key].normal_vector(1);
            BoundaryMarker.pose.orientation.y = Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.z = Cluster[Key].normal_vector(2);
            BoundaryMarker.pose.orientation.w = 0; 
            BoundMarkArray.markers.push_back(BoundaryMarker);

            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::ARROW, 
                                  "NormalVector4" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 0,
                                  Cluster[Key].average, 1, 0.01);
            BoundaryMarker.scale.x = 0.15;
            // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?  
            //                                     -Cluster[Key].normal_vector(0) : Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.x = Cluster[Key].normal_vector(2);
            BoundaryMarker.pose.orientation.y = Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.z = Cluster[Key].normal_vector(1);
            BoundaryMarker.pose.orientation.w = 0; 
            BoundMarkArray.markers.push_back(BoundaryMarker);

            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::ARROW, 
                                  "NormalVector5" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 0,
                                  Cluster[Key].average, 1, 0.01);
            BoundaryMarker.scale.x = 0.15;
            // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?  
            //                                     -Cluster[Key].normal_vector(0) : Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.x = Cluster[Key].normal_vector(2);
            BoundaryMarker.pose.orientation.y = Cluster[Key].normal_vector(1);
            BoundaryMarker.pose.orientation.z = Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.w = 0; 
            BoundMarkArray.markers.push_back(BoundaryMarker);

            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::ARROW, 
                                  "NormalVector6" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 0,
                                  Cluster[Key].average, 1, 0.01);
            BoundaryMarker.scale.x = 0.15;
            // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?  
            //                                     -Cluster[Key].normal_vector(0) : Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.x = Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.y = Cluster[Key].normal_vector(1);
            BoundaryMarker.pose.orientation.z = Cluster[Key].normal_vector(2);
            BoundaryMarker.pose.orientation.w = 0; 
            BoundMarkArray.markers.push_back(BoundaryMarker);

            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::ARROW, 
                                  "NormalVector7" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 0,
                                  Cluster[Key].average, 1, 0.01);
            BoundaryMarker.scale.x = 0.15;
            // BoundaryMarker.pose.orientation.x = (Cluster[Key].normal_vector(0)>0) ?  
            //                                     -Cluster[Key].normal_vector(0) : Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.x = Cluster[Key].normal_vector(0);
            BoundaryMarker.pose.orientation.y = Cluster[Key].normal_vector(1);
            BoundaryMarker.pose.orientation.z = Cluster[Key].normal_vector(2);
            BoundaryMarker.pose.orientation.w = 0; 
            BoundMarkArray.markers.push_back(BoundaryMarker);
            */



            // LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::SPHERE, 
            //                       "Average" + to_string(Cluster[Key].cluster_id), 
            //                       r, g, b,
            //                       Cluster[Key].average, 5, 0.04);
            // BoundMarkArray.markers.push_back(BoundaryMarker);
            


            //_cluster_pub.publish(BoundaryMarkerList); 
            //LiDARTag::_assignLine(BoundaryMarkerList, visualization_msgs::Marker::LINE_STRIP, 
            //                    r, g, b,
            //                    Cluster[Key], 1);
            //_cluster_pub.publish(BoundaryMarkerList); 

           // _boundary_marker_pub.publish(BoundMarkArray); 
           
            // Draw payload boundary marker
            visualization_msgs::Marker PayloadMarker;
            
            if (_adaptive_thresholding){
                // Upper boundary
                for (int i=0; i<Cluster[Key].tag_edges.upper_line.size(); ++i){
                    LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::SPHERE, 
                                          "PayloadUpperBoundary_" + to_string(Cluster[Key].cluster_id), 
                                          0, 0, 1,
                                          Cluster[Key].tag_edges.upper_line[i]->point, i, 0.015);
                    PayloadMarkArray.markers.push_back(PayloadMarker);
                }

                // Lower boundary
                for (int i=0; i<Cluster[Key].tag_edges.lower_line.size(); ++i){
                    LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::SPHERE, 
                                          "PayloadLowerBoundary_"  + to_string(Cluster[Key].cluster_id), 
                                          0, 0, 1,
                                          Cluster[Key].tag_edges.lower_line[i]->point, i, 0.015);
                    PayloadMarkArray.markers.push_back(PayloadMarker);
                }

                // Left boundary (green)
                for (int i=0; i<Cluster[Key].tag_edges.left_line.size(); ++i){
                    LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::SPHERE, 
                                          "PayloadLeftBoundary_" + to_string(Cluster[Key].cluster_id), 
                                          0, 1, 0,
                                          Cluster[Key].tag_edges.left_line[i]->point, i, 0.015);
                    PayloadMarkArray.markers.push_back(PayloadMarker);
                }

                // Right boundary (red)
                for (int i=0; i<Cluster[Key].tag_edges.right_line.size(); ++i){
                    LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::SPHERE, 
                                          "PayloadRightBoundary_" + to_string(Cluster[Key].cluster_id), 
                                          1, 0, 0,
                                          Cluster[Key].tag_edges.right_line[i]->point, i, 0.015);
                    PayloadMarkArray.markers.push_back(PayloadMarker);
                }
            }
            else {

                // cout << "size1: " << Cluster[Key].payload_boundary_ptr.size() << endl;
                int count = 0;
                for (int i=0; i<Cluster[Key].payload_boundary_ptr.size(); ++i){
                    // cout << "i: " << i << endl;
                    // if (Cluster[Key].payload_boundary_ptr[i].size()==0) continue;
                    // else{
                    //     // cout << "size2: " << Cluster[Key].payload_boundary_ptr[i].size() << endl;
                    //     LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::SPHERE, 
                    //                           "PayloadBoundary_" + to_string(Cluster[Key].cluster_id), 
                    //                           1, 0, 0,
                    //                           Cluster[Key].payload_boundary_ptr[i][0]->point, count, 0.015);
                    //     PayloadMarkArray.markers.push_back(PayloadMarker);
                    //     count ++;

                    //     LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::SPHERE, 
                    //                           "PayloadBoundary_" + to_string(Cluster[Key].cluster_id), 
                    //                           1, 0, 0,
                    //                           Cluster[Key].payload_boundary_ptr[i][1]->point, count, 0.015);
                    //     count ++;
                    //     PayloadMarkArray.markers.push_back(PayloadMarker);
                    // }
                    // for (int k=0; k<Cluster[Key].payload_boundary_ptr[i].size(); ++k){
                        // cout << "(i, k): (" << i << "," << k << ")" << endl;
                        // cout << "size2: " << Cluster[Key].payload_boundary_ptr[i].size() << endl;
                        LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::SPHERE, 
                                              "PayloadBoundary_" + to_string(Cluster[Key].cluster_id), 
                                              1, 0, 0,
                                              Cluster[Key].payload_boundary_ptr[i]->point, i, 0.015);
                        PayloadMarkArray.markers.push_back(PayloadMarker);
                        count ++;
                    // }
                }
            }

            // Text
            /*
            LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::TEXT_VIEW_FACING, 
                                  "PayloadUpperBoundary_text_" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 1,
                                  Cluster[Key].tag_edges.upper_line[0]->point, 5, 0.05,
                                  string("Ring: " + to_string(Cluster[Key].tag_edges.upper_line[0]->point.ring))
                                  );
            PayloadMarkArray.markers.push_back(PayloadMarker);

            LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::TEXT_VIEW_FACING, 
                                  "PayloadLowerBoundary_text_" + to_string(Cluster[Key].cluster_id), 
                                  1, 1, 1,
                                  Cluster[Key].tag_edges.lower_line[0]->point, 5, 0.05,
                                  string("Ring: " + to_string(Cluster[Key].tag_edges.lower_line[0]->point.ring))
                                  );
            PayloadMarkArray.markers.push_back(PayloadMarker);
            */


            // corner points and RANSAC line
            if (_adaptive_thresholding){
                Eigen::Vector4f EigenPoint;
                PointXYZRI point; // just for conversion

                for (int i=0; i<4; ++i){ // 4 corners
                    // Corners
                    if (i!=3){
                        pcl::lineWithLineIntersection(Cluster[Key].line_coeff[i], Cluster[Key].line_coeff[i+1], 
                                                      EigenPoint, 1e-2);
                    }
                    else {
                        pcl::lineWithLineIntersection(Cluster[Key].line_coeff[i], Cluster[Key].line_coeff[0], 
                                                      EigenPoint, 1e-2);
                    }

                    LiDARTag::_eigenVectorToPointXYZRI(EigenPoint, point);
                    LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::SPHERE, 
                                          "Corner_" + to_string(Cluster[Key].cluster_id), 
                                          0, 1, 1,
                                          point, i, 0.02);
                    PayloadMarkArray.markers.push_back(PayloadMarker);

                    // RANSAC
                    LiDARTag::_assignMarker(PayloadMarker, visualization_msgs::Marker::ARROW, 
                                          "RANSAC" + to_string(Cluster[Key].cluster_id), 
                                          1, 1, 0,
                                          point, i, 0.01);
                    double Length = sqrt(pow(Cluster[Key].line_coeff[i][3],2) + 
                                         pow(Cluster[Key].line_coeff[i][4],2) +
                                         pow(Cluster[Key].line_coeff[i][5],2));
                    PayloadMarker.scale.x = 0.15;
                    PayloadMarker.pose.orientation.x = Cluster[Key].line_coeff[i][3]/Length;
                    PayloadMarker.pose.orientation.y = Cluster[Key].line_coeff[i][4]/Length;
                    PayloadMarker.pose.orientation.z = Cluster[Key].line_coeff[i][5]/Length;
                    PayloadMarkArray.markers.push_back(PayloadMarker);
                }
            }

            // Add lines to the tag
            //LiDARTag::_assignLine(PayloadMarker, PayloadMarkArray,
            //                    visualization_msgs::Marker::LINE_STRIP, 
            //                    "left_line_" + to_string(Cluster[Key].cluster_id), 
            //                    r, g, b,
            //                    PointTL, PointTR, 1);
            //PayloadMarkArray.markers.push_back(PayloadMarker);
           

            // Draw all detected-filled cluster points
            for (int i=0; i<Cluster[Key].data.size(); ++i){
                PointsInClusters++;
                OutCluster->push_back(Cluster[Key].data[i].point);
                double Intensity = Cluster[Key].data[i].point.intensity;
                LiDARTag::_assignMarker(Marker, visualization_msgs::Marker::SPHERE, 
                                      to_string(Cluster[Key].cluster_id), 
                                      Intensity, Intensity, Intensity,
                                      Cluster[Key].data[i].point, i, 0.01);
                ClusterArray.markers.push_back(Marker);
            }

            // Add all payload points to a vector and will be publish to another
            // channel for both visualizatoin and creating training datasets
            // cout << "payload size2: " << Cluster[Key].payload.size() << endl;
            for (int i=0; i<Cluster[Key].payload.size(); ++i){
                OutPayload->push_back(Cluster[Key].payload[i]->point);
                // cout << "Intensity: " << Cluster[Key].payload[i]->point.intensity << endl;
            }


        } 
        if (PointsInClusters>_filling_max_points_threshold) {
            cout << "Too many points on a tag" << endl;
            exit(-1);
        }

        //cout << "BoundaryMarkerList size/10: " << BoundaryMarkerList.points.size()/10 << endl;
        _boundary_marker_pub.publish(BoundMarkArray); 
        _cluster_marker_pub.publish(ClusterArray); 
        _payload_marker_pub.publish(PayloadMarkArray); 
    }


    /*
     * A function to draw a line between two points
     */
    void LiDARTag::_assignLine(visualization_msgs::Marker &Marker, visualization_msgs::MarkerArray MarkArray,
                             const uint32_t Shape, const string ns,
                             const double r, const double g, const double b,
                             const PointXYZRI point1, const PointXYZRI point2, const int Count){
        Marker.header.frame_id = _pub_frame;
        Marker.header.stamp = _current_scan_time;
        //Marker.ns = string("Boundary_") + to_string(Cluster.cluster_id);
        Marker.ns = ns;
        Marker.id = Count;
        Marker.type = Shape; 

        Marker.action = visualization_msgs::Marker::ADD;
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

        geometry_msgs::Point p;
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
     * A function to draw a point in rviz
     */
    void LiDARTag::_assignMarker(visualization_msgs::Marker &Marker, const uint32_t Shape, const string NameSpace,
                               const double r, const double g, const double b,
                               const PointXYZRI &point, 
                               const int Count, const double Size, const string Text){
        Marker.header.frame_id = _pub_frame;
        Marker.header.stamp = _current_scan_time;
        Marker.ns = NameSpace;
        Marker.id = Count;
        Marker.type = Shape; 
        Marker.action = visualization_msgs::Marker::ADD;
        Marker.pose.position.x = point.x;
        Marker.pose.position.y = point.y;
        Marker.pose.position.z = point.z;
        Marker.pose.orientation.x = 0.0;
        Marker.pose.orientation.y = 0.0;
        Marker.pose.orientation.z = 0.0;
        Marker.pose.orientation.w = 1.0;
        Marker.text = Text;
        Marker.lifetime = ros::Duration(_sleep_time_for_vis); // should disappear along with updateing rate

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


    /* 
     * A valid cluster, valid tag, the points from the original point cloud that belong to the cluster 
     * could be estimated from the LiDAR system
     * Therefore, if the points on the tag is too less, which means it is not a valid
     * tag where it might just a shadow of a valid tag
     */
    bool LiDARTag::_clusterPointsCheck(ClusterFamily_t &Cluster){ 
        int points = LiDARTag::_areaPoints(Cluster.average.x, _payload_size, _payload_size);
        //cout << "points: " << points << endl;
        //cout << "Data: " << Cluster.data.size() << endl;
        // return true;
        if (Cluster.data.size() < floor(points/ _points_threshold_factor)) return false;
        else return true;
    }


    /*
     * A function to transform an eigen type of vector to pcl point type
     */
    void LiDARTag::_eigenVectorToPointXYZRI(const Eigen::Vector4f &Vector, PointXYZRI &point){
        point.x = Vector[0];
        point.y = Vector[1];
        point.z = Vector[2];
    }


    void LiDARTag::_getCodeNaive(string &Code, pcl::PointCloud<LiDARPoints_t*> payload){
        int topring = 0;
        int bottomring = _beam_num;
        PointXYZRI tl= payload[0]->point;
        PointXYZRI tr= payload[0]->point;
        PointXYZRI br= payload[0]->point;
        PointXYZRI bl= payload[0]->point;


        // Find the size of the payload 
        for (int i=0; i<payload.size(); ++i){
            PointXYZRI point = payload[i]->point;
            if (point.y>tl.y && point.z>tl.z) tl = point;
            if (point.y>bl.y && point.z<bl.z) bl = point;
            if (point.y<tr.y && point.z>tr.z) tr = point;
            if (point.y<br.y && point.z<br.z) br = point;
        }

        vector<int> payload49(_tag_family, 0);
        int d = sqrt(_tag_family);
        float IntervalY = abs( (tl.y+bl.y)/2 - (tr.y+br.y)/2 ) /(d+_black_border-1);
        float IntervalZ = abs( (tl.z+tr.z)/2 - (bl.z+br.z)/2 ) /(d+_black_border-1);


        if (_fake_tag){
            tl.y = 0;
            tl.z = 0;
            IntervalY = 1;
            IntervalZ = 1;
            payload.clear();
            payload.reserve((_tag_family+4*d*_black_border+4*(std::pow(_black_border, 2))));
            float j = 0;
            float k = 0;
            LiDARPoints_t point;
            for (int i=0; i<(_tag_family+4*d*_black_border+4*(std::pow(_black_border, 2))); ++i){
                if (i%(d+2*_black_border)==0 && i!=0) {k++; j=0;}
                LiDARPoints_t *point = new LiDARPoints_t{{0, j, k,0,0}, 0,0,0,0};
                payload.push_back(point);
                // cout << "j,k: " << j << ", " << k << endl;
                j++;
                //cout << "payload[i]" << payload[i]->point.y << ", " << payload[i]->point.z << endl;
                // delete point;
            }
            payload[20]->point.intensity = 100;
            payload[26]->point.intensity = 100;
            payload[27]->point.intensity = 100;
            payload[28]->point.intensity = 100;
            payload[34]->point.intensity = 100;
            payload[36]->point.intensity = 100;
            payload[43]->point.intensity = 100;
            payload[45]->point.intensity = 100;
        }


        // Calcutate Average intensity for thresholding 
        float AveIntensity = 0;
        for (int i=0; i<payload.size(); ++i){
            AveIntensity += payload[i]->point.intensity;
        }
        AveIntensity /= payload.size();
        // cout << "size: " << payload.size() << endl;
        // cout << "AveIntensity: " << AveIntensity << endl;


        // Split into grids
        for (int i=0; i<payload.size(); ++i){
            PointXYZRI *pointPtr = &(payload[i]->point);
            // cout << "i: " << i << endl;
            // cout << "point.y: " << pointPtr->y << endl;
            // cout << "point.z: " << pointPtr->z << endl;
            float DeltaY = abs(pointPtr->y - tl.y);
            float DeltaZ = abs(pointPtr->z - tl.z);

            // if (DeltaY==0 || (DeltaY)) {
            //     if (pointPtr->intensity < AveIntensity) payload49[0] -= 1;
            //     else payload49[0] += 1;
            //     continue;
            // }
            int Y = floor(DeltaY/IntervalY);
            int Z = floor(DeltaZ/IntervalZ);
            // cout << "Y: " << Y << endl;
            // cout << "Z: " << Z << endl;

            // remove black borders
            if (Y>=_black_border && Z>=_black_border && Y<=d+_black_border && Z<=d+_black_border){
                int y = (Y-_black_border)%d; // the yth column (remove the black border)
                int z = (Z-_black_border)%d; // the zth row (remove the black border)

                int k = d*z + y; // index in a 1D vector
                // cout << "y: " << y << endl;
                // cout << "z: " << z << endl;
                // cout << "k: " << k << endl;
                // cout << "intensity: " << pointPtr->intensity << endl;
                if (pointPtr->intensity <= AveIntensity) payload49[k] -= 1;
                else payload49[k] += 1;
                //cout << "payload[k]: " << payload49[k] << endl; 
                //cout << "--" << endl;
            }
            
        }

        // Threshold into black and white
        for (int i=0; i<_tag_family; ++i){
            if (payload49[i]<0) Code += to_string(0);
            else Code += to_string(1);
        }

        for (int i=0; i<_tag_family; ++i){
            if (i%d==0){
                cout << "\n";
                cout << " " << Code[i];
            }
            else{
                cout << " " << Code[i];
            }
        }



        Code += "UL";
        //cout << "\ncodeB: " << Code << endl; 
                              //0101111111111111UL
        //cout << "Code:  " << "0010001100011011UL" << endl;
        cout << "\nCode 1:  \n" << " 0 0 1 0\n 1 1 1 0\n 1 0 1 0\n 0 1 1 1" << endl;
        // exit(-1);
        // Code = "0010001100011011UL";
        //Code = "0b0000011111100000011000011000110011100000111111111UL";
        // Code = "11111100000011000011000110011100000111111111UL"; //1
        //Code = "0001100111110000011001100011111000000110001100011UL"; // 2
        //cout << "codeA: " << Code << endl; 
    }


    /* Decode using Weighted Gaussian weight
     * return  0: normal
     * return -1: not enough return
     * return -2: fail corner detection
     */
    int LiDARTag::_getCodeWeightedGaussian(string &Code, Homogeneous_t &pose,
                            int &payload_points,
                            const PointXYZRI &average, 
                            const pcl::PointCloud<LiDARPoints_t*> &payload, 
                            const std::vector<LiDARPoints_t*> &payload_boundary_ptr){
        /*          p11
         *          .                   p11. . . . . p41        ^ z
         *        .   .                    .  ave  .        y __|
         *  p21 .   .   . p41              .   .   .      
         *        .   .                    .       .
         *          .                   p21. . . . . p31
         *          p31
         *  px2s are just second largest number corresponding to x position
         */

        // For visualization
        visualization_msgs::MarkerArray GridMarkerArray;
        visualization_msgs::Marker GridMarker;

        visualization_msgs::Marker LineStrip;
        LineStrip.header.frame_id = _pub_frame;
        LineStrip.header.stamp = _current_scan_time;
        LineStrip.ns = "boundary" ;
        LineStrip.action = visualization_msgs::Marker::ADD;
        LineStrip.pose.orientation.w= 1.0;
        LineStrip.id = 1;
        LineStrip.type = visualization_msgs::Marker::LINE_STRIP;
        LineStrip.scale.x = 0.002;
        LineStrip.color.b = 1.0;
        LineStrip.color.a = 1.0;



        PointXYZRI p11{0, 0, -1000, 0};
        PointXYZRI p21{0, -1000, 0, 0};
        PointXYZRI p31{0, 0, 1000, 0};
        PointXYZRI p41{0, 1000, 0, 0};

        PointXYZRI p12{0, 0, -1000, 0};
        PointXYZRI p22{0, -1000, 0, 0};
        PointXYZRI p32{0, 0, 1000, 0};
        PointXYZRI p42{0, 1000, 0, 0};


        // Find the largest 
        for (int i=0; i<payload_boundary_ptr.size(); ++i){
            PointXYZRI point = payload_boundary_ptr[i]->point;

            if (point.z>=p11.z) p11 = point;
            if (point.y>=p21.y) p21 = point;
            if (point.z<=p31.z) p31 = point;
            if (point.y<=p41.y) p41 = point;

        }
        if(_grid_viz){
            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("11"), 
                          0, 0, 0,
                          p11, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("21"),
                          0, 0, 0,
                          p21, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("31"), 
                          0, 0, 0,
                          p31, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("41"), 
                          0, 0, 0,
                          p41, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);
        }

        // Find the second large
        for (int i=0; i<payload_boundary_ptr.size(); ++i){
            PointXYZRI point = payload_boundary_ptr[i]->point;
            if (point.z<p11.z && point.z>=p12.z) p12 = point;
            if (point.y<p21.y && point.y>=p22.y) p22 = point;

            if (point.z>p31.z && point.z<=p32.z) p32 = point;
            if (point.y>p41.y && point.y<=p42.y) p42 = point;
        }

        if(_grid_viz){
            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("12"), 
                          0, 0, 0,
                          p12, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("22"),
                          0, 0, 0,
                          p22, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("32"), 
                          0, 0, 0,
                          p32, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("42"), 
                          0, 0, 0,
                          p42, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);
        }

        PointXYZRI p1 = utils::pointsAddDivide(p11, p12, 2);
        PointXYZRI p2 = utils::pointsAddDivide(p21, p22, 2);
        PointXYZRI p3 = utils::pointsAddDivide(p31, p32, 2);
        PointXYZRI p4 = utils::pointsAddDivide(p41, p42, 2);

        // check condition of the detected corners
        // if corners are not between certain angle or distance, 
        // consider the tag is up right =>
        // change way of detection
        int status = utils::checkCorners(_payload_size, p1, p2, p3, p4);
        if (status!=0){
            // the tag is up right
            p1 = {0, 0,-1000, 0};
            p2 = {0, 0, 1000, 0};
            p3 = {0, 0, 1000, 0};
            p4 = {0, 0,-1000, 0};
            for (int i=0; i<payload_boundary_ptr.size(); ++i){
                PointXYZRI point = payload_boundary_ptr[i]->point;

                // left boundary
                if (point.z>=p1.z && point.y > average.y/2) p1 = point;
                if (point.z<=p2.z && point.y > average.y/2) p2 = point;

                // right boundary
                if (point.z<=p3.z && point.y < average.y/2) p3 = point;
                if (point.z>=p4.z && point.y < average.y/2) p4 = point;
            }
        }

        if(_grid_viz){
            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("1"), 
                          1, 1, 1,
                          p1, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("2"),
                          1, 1, 1,
                          p2, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("3"), 
                          1, 1, 1,
                          p3, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Grid_" + string("4"), 
                          1, 1, 1,
                          p4, 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            GridMarkerArray.markers.push_back(GridMarker);
        }


        int d = std::sqrt(_tag_family);
        Eigen:: MatrixXf Vertices = Eigen::MatrixXf::Zero(3,5);
        utils::formGrid(Vertices, 0, 0, 0, _payload_size);
        Eigen::Matrix3f R;
        // Eigen::MatrixXf VerticesOffset = (Vertices.colwise() - utils::toEigen(average));
        // cout << "vertice: " << Vertices << endl;
        // cout << "verticeOffset: " << VerticesOffset << endl;
        // cout << "Average: " << utils::toEigen(average) << endl;
        utils::minus(p1, average);
        utils::minus(p2, average);
        utils::minus(p3, average);
        utils::minus(p4, average);

        utils::fitGrid(Vertices, R, p1, p2, p3, p4);
        Eigen::Vector3f Angle = utils::rotationMatrixToEulerAngles(R);

        if(_grid_viz){
            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Model_" + string("1"), 
                          0, 1, 0,
                          utils::toVelodyne(Vertices.col(1)), 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Model_" + string("2"),
                          0, 1, 0,
                          utils::toVelodyne(Vertices.col(2)), 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Model_" + string("3"), 
                          0, 1, 0,
                          utils::toVelodyne(Vertices.col(3)), 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::CUBE, 
                          "Model_" + string("4"), 
                          0, 1, 0,
                          utils::toVelodyne(Vertices.col(4)), 1, 0.01);
            GridMarkerArray.markers.push_back(GridMarker);

            geometry_msgs::Point p;
            p.x = Vertices(0, 1);
            p.y = Vertices(1, 1);
            p.z = Vertices(2, 1);
            LineStrip.points.push_back(p);
            p.x = Vertices(0, 2);
            p.y = Vertices(1, 2);
            p.z = Vertices(2, 2);
            LineStrip.points.push_back(p);
            p.x = Vertices(0, 3);
            p.y = Vertices(1, 3);
            p.z = Vertices(2, 3);
            LineStrip.points.push_back(p);
            p.x = Vertices(0, 4);
            p.y = Vertices(1, 4);
            p.z = Vertices(2, 4);
            LineStrip.points.push_back(p);
            p.x = Vertices(0, 1);
            p.y = Vertices(1, 1);
            p.z = Vertices(2, 1);
            LineStrip.points.push_back(p);
        }

        // Calcutate Average intensity for thresholding 
        float AveIntensity = 0;
        for (int i=0; i<payload.size(); ++i)
            AveIntensity += payload[i]->point.intensity;

        AveIntensity /= payload.size();

        vector<PayloadVoting_t> Votes(payload.size());
        vector<float> vR(std::pow((d+2*_black_border), 2));
        vector<float> vG(std::pow((d+2*_black_border), 2));
        vector<float> vB(std::pow((d+2*_black_border), 2));

        // pick a random color for each cell
        if(_grid_viz){
            for (int i=0; i<vR.size(); ++i){
                float r = (double) rand() / RAND_MAX;
                float g = (double) rand() / RAND_MAX;
                float b = (double) rand() / RAND_MAX;
                vR[i] = r;
                vG[i] = g;
                vB[i] = b;
            }
        }

        // Split into grids
        for (int i=0; i<payload.size(); ++i){
            float t14, t12;
            Eigen::Vector2f v14, v12;
            PointXYZRI *pointPtr = &(payload[i]->point);
            utils::getProjection(p1, p4, *pointPtr, t14, v14);
            utils::getProjection(p1, p2, *pointPtr, t12, v12);
            Votes[i].p = pointPtr;
            PointXYZRI p; // for visualization
            utils::assignCellIndex(_payload_size, R, p, 
                                   average, d + 2*_black_border, Votes[i]);
            if(_grid_viz){
                LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::SPHERE, 
                              "TransPoints", 
                              vR[Votes[i].cell], vG[Votes[i].cell], vB[Votes[i].cell],
                              p, i, 0.005);
                GridMarkerArray.markers.push_back(GridMarker);
            }

        }
        vector<vector<PayloadVoting_t*>> Grid(std::pow((d+2*_black_border), 2));
        utils::sortPointsToGrid(Grid, Votes);
        int TooLessReturn = 0;
        int PayloadPointCount = 0;
        for (int i=(d+2*_black_border)*_black_border+_black_border; 
             i<(d+2*_black_border)*(_black_border+d)-_black_border; ++i){

            if ((i%(d+2*_black_border)<_black_border) ||
                (i%(d+2*_black_border)>(d+_black_border-1))) continue;

            if (Grid[i].size() < _min_returns_per_grid)  TooLessReturn ++;
            if (TooLessReturn>_max_decode_hamming) return -1;

            float WeightedProb = 0;
            float WeightSum = 0;
            float minIntensity = 10000.;
            float maxIntensity = -1.;
            double r;
            double g;
            double b;

            // pick a random color for each cell
            if(_grid_viz){
                r = (double) rand() / RAND_MAX;
                g = (double) rand() / RAND_MAX;
                b = (double) rand() / RAND_MAX;
            }
            
            for (int j=0; j<Grid[i].size(); ++j){
                if(_grid_viz){
                    LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::SPHERE, 
                            "Point" + to_string(i), 
                            r, g, b,
                            *(Grid[i][j]->p), j, 0.005);
                    GridMarkerArray.markers.push_back(GridMarker);
                    LiDARTag::_assignMarker(GridMarker, visualization_msgs::Marker::SPHERE, 
                            "Center" + to_string(i), 
                            1, 1, 1,
                            Grid[i][j]->centroid, j, 0.005);
                    GridMarkerArray.markers.push_back(GridMarker);
                    LiDARTag::_assignMarker(GridMarker, 
                            visualization_msgs::Marker::TEXT_VIEW_FACING, 
                            "Prob" + to_string(i), 
                            1, 1, 1,
                            *(Grid[i][j]->p), j, 0.003, 
                            to_string((Grid[i][j]->p->intensity)));
                    GridMarkerArray.markers.push_back(GridMarker);
                }
                WeightedProb += Grid[i][j]->weight*Grid[i][j]->p->intensity;
                WeightSum += Grid[i][j]->weight;
            }
            WeightedProb /= WeightSum;
            PayloadPointCount += Grid[i].size();

            if (WeightedProb>0.5) Code += to_string(1);
            else Code += to_string(0);
        }
        payload_points = PayloadPointCount;

        Code += "UL";

        if(_grid_viz){
            _payload_grid_pub.publish(GridMarkerArray); 
            _payload_grid_line_pub.publish(LineStrip); 
        }
        return 0;
    }


    /* [Payload decoding]
     * A function to decode payload with different means
     * 0: Naive decoding
     * 1: Weighted Gaussian
     * 2: Deep learning
     * 3: Gaussian Process
     * 4: ?!
     */
    bool LiDARTag::_decodPayload(ClusterFamily_t &Cluster){
        string Code("");
        bool ValidTag = true;
        string Msg;

        if (_decode_method==0){ // Naive decoder
            LiDARTag::_getCodeNaive(Code, Cluster.payload);
        }
        else if (_decode_method==1){ // Weighted Gaussian
            int status = LiDARTag::_getCodeWeightedGaussian(
                                    Code, Cluster.pose, 
                                    Cluster.payload_without_boundary, // size of actual payload
                                    Cluster.average, 
                                    Cluster.payload, 
                                    Cluster.payload_boundary_ptr);

            if (_id_decoding){
                if (status==-1){
                    ValidTag = false;
                    _result_statistics.cluster_removal.decoder_not_return ++;
                    Cluster.valid = 0;
                    Msg = "Not enough return";
                }
                else if (status==-2){
                    ValidTag = false;
                    _result_statistics.cluster_removal.decoder_fail_corner ++;
                    Cluster.valid = 0;
                    Msg = "Fail corner detection";
                }
            }
        }

        if (ValidTag && _id_decoding){
            uint64_t Rcode = stoull(Code, nullptr, 2);
            BipedAprilLab::QuickDecodeCodeword(tf, Rcode, &Cluster.entry);
            Cluster.cluster_id = Cluster.entry.id;
            ROS_INFO_STREAM("id: " << Cluster.entry.id);
            ROS_DEBUG_STREAM("hamming: " << Cluster.entry.hamming);
            ROS_DEBUG_STREAM("rotation: " << Cluster.entry.rotation);
        }
        else {
            // too big, return as an invalid tag 
            Code = "1111111111111111UL";
            ROS_DEBUG_STREAM("\nCODE: " << Code);
            uint64_t Rcode = stoull(Code, nullptr, 2);
            BipedAprilLab::QuickDecodeCodeword(tf, Rcode, &Cluster.entry);
            Cluster.cluster_id = 8888;
            ROS_DEBUG_STREAM("id: " << Cluster.cluster_id);
            ROS_DEBUG_STREAM("hamming: " << Cluster.entry.hamming);
            ROS_DEBUG_STREAM("rotation: " << Cluster.entry.rotation);
        }
        return ValidTag;
    }



    /* [Decoder]
     * Create hash table of chosen tag family
     */
    void LiDARTag::_initDecoder(){
        string famname = "tag" + to_string(_tag_family) + "h" + to_string(_tag_hamming_distance); 
        if (famname == "tag49h14") tf = tag49h14_create();
        else if (famname == "tag16h5") tf = tag16h5_create();
        else {
            cout << "[ERROR]" << endl;
            cout << "Unrecognized tag family name: "<< famname << ". Use e.g. \"tag16h5\". " << endl;
            cout << "This is line " << __LINE__ << " of file "<<  __FILE__ << 
                " (function " << __func__ << ")"<< endl;
            exit(0);
        }
        tf->black_border = _black_border;
        cout << "Preparing for tags: " << famname << endl;
        BipedAprilLab::QuickDecodeInit(tf, _max_decode_hamming);
    }

    /* 
     *
     */
    void LiDARTag::_testInitDecoder(){
        uint64_t rcode = 0x0001f019cf1cc653UL; 
        QuickDecodeEntry_t entry;
        BipedAprilLab::QuickDecodeCodeword(tf, rcode, &entry);
        cout << "code: " << entry.rcode << endl;
        cout << "id: " << entry.id << endl;
        cout << "hamming: " << entry.hamming << endl;
        cout << "rotation: " << entry.rotation << endl;
        exit(0);
    }



    /* [not used] [not finished]
     * A function to group edges
     */
    template<typename Container>
    void LiDARTag::_freeVec(Container& c) { 
        while(!c.empty()) {
            if (c.back()!=nullptr) {
                delete c.back(); 
                c.back()=nullptr;
            }  
            c.pop_back(); 
        }
    }

    void LiDARTag::_freePCL(pcl::PointCloud<LiDARPoints_t*> &vec){
        while(!vec.empty()) delete vec.back(), vec.erase(vec.end());
    }

    void LiDARTag::_freeTagLineStruc(TagLines_t &tag_edges){
        LiDARTag::_freeVec(tag_edges.upper_line);
        LiDARTag::_freeVec(tag_edges.lower_line);
        LiDARTag::_freeVec(tag_edges.left_line);
        LiDARTag::_freeVec(tag_edges.right_line);

        LiDARTag::_freeVec(tag_edges.bottom_left);
        LiDARTag::_freeVec(tag_edges.bottom_right);
        LiDARTag::_freeVec(tag_edges.top_left);
        LiDARTag::_freeVec(tag_edges.top_right);
    }
} // namespacer
