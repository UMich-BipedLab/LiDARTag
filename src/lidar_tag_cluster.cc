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

#include <ros/package.h> // package
#include "lidar_tag.h"
#include "ultra_puck.h"

#include <iostream>

#define _2SQRT2 2.82842712475

using namespace std;

namespace BipedLab {

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
        if (!_isWithinCluster(point, OldCluster)) {
            
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

    bool LiDARTag::_isWithinCluster(const LiDARPoints_t &point, ClusterFamily_t &cluster)
    {
        auto upper_z_threshold = _threshold;
        auto lower_z_threshold = _threshold;
        const double cluster_cushion  = _2SQRT2;
        
        // Calculate upper z threshold
        auto upper_ring = cluster.top_most_point.ring;
        if (upper_ring + 1 <_beam_num)
        {
            // Calculate the expected z distance between the ring of top_most_point
            // and the next ring e.g. the z distance between ring id 20 and 21
            auto z2 = abs(point.point.x) * UltraPuckV2::EL_TAN.values[upper_ring+1];
            auto z1 = abs(point.point.x) * UltraPuckV2::EL_TAN.values[upper_ring];

            // auto z2 = abs(point.point.x) * tan(UltraPuckV2::el[upper_ring+1]*M_PI/180);
            // auto z1 = abs(point.point.x) * tan(UltraPuckV2::el[upper_ring]*M_PI/180);
            upper_z_threshold += abs(z2 - z1);
        }

        // Calculate lower z threshold
        auto lower_ring = cluster.bottom_most_point.ring;
        if (lower_ring - 1 > 0) 
        {
            // Calculate the expected z distance between the ring of bottom_most_point
            // and the next ring e.g. the z distance between ring id 5 and 4
            auto z2 = abs(point.point.x) * UltraPuckV2::EL_TAN.values[lower_ring-1];
            auto z1 = abs(point.point.x) * UltraPuckV2::EL_TAN.values[lower_ring];

            // auto z2 = abs(point.point.x) * tan(UltraPuckV2::el[lower_ring-1]*M_PI/180);
            // auto z1 = abs(point.point.x) * tan(UltraPuckV2::el[lower_ring]*M_PI/180);
            // assert(z2 - z1 <= 0);
            lower_z_threshold += abs(z2 - z1);
        }

        return (point.point.z < cluster.top_most_point.z + upper_z_threshold)  && 
             (point.point.x < cluster.average.x + cluster_cushion * _payload_size)  &&  
             (cluster.average.x - cluster_cushion * _payload_size < point.point.x)   &&  
             (point.point.y < cluster.average.y + cluster_cushion * _payload_size) && 
             (cluster.average.y - cluster_cushion * _payload_size < point.point.y) && 
             (cluster.bottom_most_point.z - lower_z_threshold < point.point.z);
    } 

}
