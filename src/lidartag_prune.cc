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

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h> 

#include <fstream>
#include <ros/package.h> // package

#include "ultra_puck.h"
#include "lidartag.h"

#define SQRT2 1.41421356237

using namespace std;

namespace BipedLab {
    
    /* 
     * A valid cluster, valid tag, the points from the original point cloud that belong to the cluster 
     * could be estimated from the LiDAR system
     * Therefore, if the points on the tag is too less, which means it is not a valid
     * tag where it might just a shadow of a valid tag
     */
    bool LiDARTag::_clusterPointsCheck(ClusterFamily_t &Cluster){
        
        auto distance = sqrt(pow(Cluster.average.x,2) + pow(Cluster.average.y,2));
        // cout << "distance: " << distance << "X: " << Cluster.average.x << "Y: " << Cluster.average.y << endl;
        int maxPoints = LiDARTag::_areaPoints(distance, _payload_size*SQRT2, _payload_size*SQRT2);
        int minPoints = LiDARTag::_areaPoints(distance, _payload_size/SQRT2, _payload_size/SQRT2);
        // cout << "Cluster Size: " << Cluster.data.size() << " Max Points: " << maxPoints << " Min Points: " << minPoints << endl;
        //cout << "Data: " << Cluster.data.size() << endl;
        // return true;
        // if (Cluster.data.size() < floor(points/ _points_threshold_factor)) return false;
        // else return true;

        if (Cluster.data.size() < minPoints) { // Cluster.data.size() > maxPoints) { // || Cluster.data.size() < minPoints) {
            return false;
        } else {
            return true;
        }
    }

        /*
     * A function to get a number of points on a given-distance tag or object
     */
    int LiDARTag::_areaPoints(const double &Distance, const double &ObjWidth, const double &ObjHeight){
        // double WAngle = ObjWidth * (1 + SQRT2) / abs(Distance);

        // if (WAngle>=1) return (int) 1e6; // return big number to reject the cluster

        // double HAngle = ObjHeight * (1 + SQRT2) / abs(Distance);
        // if (HAngle>=1) return (int) 1e6; // return big number to reject the cluster

        // double HorizontalAngle = asin(WAngle); // in radian
        // double VerticalAngle = asin(HAngle); // in radian
        // int NumOfVerticalRing = floor(VerticalAngle * _LiDAR_system.beam_per_vertical_radian);
        // int NumOfHorizontalPoints = floor(HorizontalAngle * _LiDAR_system.point_per_horizontal_radian);

        // // use 3 instead of 2 becasue of we assume the tag would be put in the dense
        // // region of LiDAR (LiDAR is denser in the middle)
        // int Area = floor(3 * (NumOfVerticalRing * NumOfHorizontalPoints) / (1 + SQRT2)); 

        // cout << "distance: " << Distance << endl;
        // //cout << "HorizontalAngle: " << HorizontalAngle << endl;
        // //cout << "VerticalAngle: " << VerticalAngle << endl;

        int NumOfHorizontalPoints = ceil(ObjWidth / (Distance * tan(0.1 * M_PI / 180)));


        //int NumOfHorizontalPoints = 2 * atan((ObjWidth / 2) / abs(Distance)) * 
        //    _LiDAR_system.point_per_horizontal_radian;
        double HalfVerticalAngle = atan((ObjHeight / 2) / abs(Distance)) * 180 / M_PI;

        int NumOfVerticalRing = 0;
        for (int i = 0; i < UltraPuckV2::beams; ++i)
        {
            if (HalfVerticalAngle > abs(UltraPuckV2::el[i])) 
            {
                NumOfVerticalRing++;
            }
        }
        int Area = NumOfVerticalRing * NumOfHorizontalPoints;

        // cout << "NumOfVerticalRing: " << NumOfVerticalRing << endl;
        // cout << "NumOfHorizontalPoints: " << NumOfHorizontalPoints << endl;
        // cout << "Area: " << Area << endl;
        // cout << "Points / Radian: " << _LiDAR_system.point_per_horizontal_radian << endl;

        return Area;
    }

    /*
     * A function to calculate the upper bound of points that can exist in a cluster
     * based on the payload size
     */ 
    void LiDARTag::_maxPointsCheck(ClusterFamily_t &Cluster) {
        auto distance = sqrt(pow(Cluster.average.x,2) + pow(Cluster.average.y,2));
        auto payload_w = 2*SQRT2*_payload_size;
        auto payload_h = 2*SQRT2*_payload_size;
        
        int NumOfHorizontalPoints = ceil(payload_w / (distance * tan(0.1 * M_PI / 180)));
        double HalfVerticalAngle  = atan((payload_h / 2) / abs(distance)) * 180 / M_PI;

        int NumOfVerticalRing = 0;
        for (int i = 0; i < UltraPuckV2::beams; ++i)
        {
            if (HalfVerticalAngle > abs(UltraPuckV2::el[i])) 
            {
                NumOfVerticalRing++;
            }
        }

        int expected_points = NumOfVerticalRing * NumOfHorizontalPoints;
        // cout << "Expected Points: " << expected_points << endl;
        
        if (Cluster.data.size() > expected_points) {
            _result_statistics.cluster_removal.removed_by_point_check++;
            Cluster.valid = false;
        }
    }

    /*
     * Fit a plane to a cluster. Returns false if unable to estimate a plane.
     * Otherwise, returns the number of inliers and the coefficients of the plane.
     */ 
    bool LiDARTag::_rejectWithPlanarCheck(
        ClusterFamily_t &Cluster, 
        pcl::PointIndices::Ptr inliers,
        pcl::ModelCoefficients::Ptr coefficients,
        std::ostream &fplanefit) {

        // Convert cluster family into pcl point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(Cluster.data.size());
        for (std::size_t i = 0; i < cloud->points.size(); ++i) {
            cloud->points[i].x = Cluster.data[i].point.x;
            cloud->points[i].y = Cluster.data[i].point.y;
            cloud->points[i].z = Cluster.data[i].point.z;
        }

        // Create segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(_distance_to_plane_threshold);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            std::cerr << "Could fit plane model to cluster." << endl;
            return false;
        }
        
        fplanefit << "Successfully fit plane!" << endl;
        fplanefit << "Cluster Size: " << Cluster.data.size() << endl;
        fplanefit << "Inliers     : " << inliers->indices.size() << endl;
        fplanefit << "Outliers    : " << Cluster.data.size() - inliers->indices.size() << endl;

        return true;
    } 

}
