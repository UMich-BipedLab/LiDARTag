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

#include <pcl/common/intersections.h>

#include <ros/package.h> // package
#include "lidar_tag.h"

#include <iostream>
#include <string>

using namespace std;

namespace BipedLab {

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

        // Used to identify between multiple clusters in a single point
        // cloud in the analysis file. The id being reset to 1 each time
        // the function is called is supposed to indicate in the output 
        // file that the proceeding clusters belong to a new payload
        int cluster_pc_id = 1; 
        
        int PointsInClusters = 0;
        for (int Key=0; Key<Cluster.size(); ++Key) {
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
            // int AvePoints = LiDARTag::_areaPoints(Cluster[Key].average.x, _payload_size, _payload_size);  
            LiDARTag::_assignMarker(BoundaryMarker, visualization_msgs::Marker::SPHERE, 
                                  "AveragePoint" + to_string(Cluster[Key].cluster_id), 
                                  1, 0, 0,
                                  Cluster[Key].average, 1, 0.05);
            BoundMarkArray.markers.push_back(BoundaryMarker);

            // int SupposedPoints = LiDARTag::_areaPoints(Cluster[Key].average.x, _payload_size, _payload_size);  
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
                std::string path("/home/cassie/catkin_ws/src/LiDARTag/output");

                // write poses
                // file.open(path + "/LiDARTag.txt", 
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
                //     cout << "FAILED opening LiDARTag.txt" << endl;
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
                //     file << _result_statistics.cluster_removal.removed_by_point_check << ",";
                //     file << _result_statistics.remaining_cluster_size << ",";
                //     file << _result_statistics.cluster_removal.boundary_point_check << ",";
                //     file << _result_statistics.cluster_removal.no_edge_check << "\n";
                //     file.close();
                // }
                // else{
                //     cout << "FAILED opening Analysis.txt" << endl;
                //     exit(0);
                // }
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
           
            // TODO: Too many for loops, could be done in a better way.
            // Some of the loops are inside of the _detectionArrayPublisher
            // Changes NEEDED!

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

            // Publish to a lidartag channel
            _detectionArrayPublisher(Cluster[Key]);

        } 
        if (PointsInClusters>_filling_max_points_threshold) {
            cout << "Too many points on a tag" << endl;
            // exit(-1);
        }
        detectionsToPub.header = _point_cloud_header;
        detectionsToPub.frame_index = _point_cloud_header.seq; 
        //cout << "BoundaryMarkerList size/10: " << BoundaryMarkerList.points.size()/10 << endl;
        _boundary_marker_pub.publish(BoundMarkArray); 
        _cluster_marker_pub.publish(ClusterArray); 
        _payload_marker_pub.publish(PayloadMarkArray);
        // srand(time(0));
        // if (rand()%10 < 7)
        _detectionArray_pub.publish(detectionsToPub); 
    }
}