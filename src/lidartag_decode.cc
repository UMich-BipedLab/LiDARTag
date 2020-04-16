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
#include "lidartag.h"

#include "apriltag_utils.h"
#include "utils.h"
#include "tag49h14.h"
#include "tag16h5.h"

using namespace std;

namespace BipedLab {

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

}
