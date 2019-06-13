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
#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <algorithm> // to use to_lower function 
#include <stdarg.h> // for variadic functions
#include <vector>


#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h> 
#include "lidar_tag.h"


namespace BipedLab {

    //typedef velodyne_pointcloud::PointXYZIR PointXYZRI;
    namespace utils {
        double spendTime(const std::clock_t &t_end, const std::clock_t &t_start);
        std::string tranferToLowercase(std::string &t_data);
        void pressEnterToContinue();
        double deg2Rad(double t_degree);
        double rad2Deg(double t_radian);
        bool isRotationMatrix(Eigen::Matrix3f &t_R);
        Eigen::Vector3f rotationMatrixToEulerAngles(Eigen::Matrix3f &t_R);


        bool checkParameters(int t_n, ...);
        void COUT(const velodyne_pointcloud::PointXYZIR& t_p);
        bool compareIndex(LiDARPoints_t *A, LiDARPoints_t *B);
        uint64_t bitShift(std::string const& t_value);



        void normalizeByAve(std::vector<float> &t_x, std::vector<float> &t_y, 
                std::vector<float> &t_z, std::vector<float> &t_I, 
                const pcl::PointCloud<LiDARPoints_t*> payload);
        void normalize(std::vector<float> &t_x, std::vector<float> &t_y, 
                std::vector<float> &t_z, std::vector<float> &t_I, 
                const pcl::PointCloud<LiDARPoints_t*> t_payload);

        velodyne_pointcloud::PointXYZIR pointsAddDivide (
                const velodyne_pointcloud::PointXYZIR& t_p1, 
                const velodyne_pointcloud::PointXYZIR& t_p2, float t_d=1);

        velodyne_pointcloud::PointXYZIR vectorize (
                const velodyne_pointcloud::PointXYZIR& t_p1, 
                const velodyne_pointcloud::PointXYZIR& t_p2);

        float dot (const velodyne_pointcloud::PointXYZIR& t_p1, 
                   const velodyne_pointcloud::PointXYZIR& t_p2);

        float Norm (const velodyne_pointcloud::PointXYZIR& t_p);

        // a function to determine the step of given two points
        float getStep(const velodyne_pointcloud::PointXYZIR &t_p1, 
                const velodyne_pointcloud::PointXYZIR &t_p2, const int t_d);

        void getProjection(const velodyne_pointcloud::PointXYZIR &t_p1, 
                const velodyne_pointcloud::PointXYZIR &t_p2, 
                const velodyne_pointcloud::PointXYZIR &t_p,
                float &t_k, Eigen::Vector2f &t_v);

        double MVN(const float &t_tag_size, const int &t_d,
                const Eigen::Vector2f &t_X, const Eigen::Vector2f t_mean);

        void assignCellIndex(const float &t_tag_size,
                const Eigen::Matrix3f &t_R, 
                velodyne_pointcloud::PointXYZIR &t_p_reference,
                const velodyne_pointcloud::PointXYZIR &t_average,
                const int t_d, PayloadVoting_t &t_vote);

        void sortPointsToGrid(std::vector<std::vector<PayloadVoting_t*>> &t_grid, 
                              std::vector<PayloadVoting_t> &t_votes);

        Eigen::Vector2f pointToLine(const velodyne_pointcloud::PointXYZIR &t_p1, 
                const velodyne_pointcloud::PointXYZIR &t_p2, 
                const velodyne_pointcloud::PointXYZIR &t_p);

        void formGrid(Eigen::MatrixXf &t_vertices, 
                float t_x, float t_y, float t_z, float t_tag_size);

        void fitGrid(Eigen::MatrixXf &t_vertices, 
                Eigen::Matrix3f &t_R,
                const velodyne_pointcloud::PointXYZIR &t_p1,   
                const velodyne_pointcloud::PointXYZIR &t_p2, 
                const velodyne_pointcloud::PointXYZIR &t_p3, 
                const velodyne_pointcloud::PointXYZIR &t_p4);

        float distance(
                const velodyne_pointcloud::PointXYZIR &t_p1,
                const velodyne_pointcloud::PointXYZIR &t_p2);
        template <class T, class U>
        float getAngle (T a, U b);


        int checkCorners(
                const float t_tag_size,
                const velodyne_pointcloud::PointXYZIR &t_p1,
                const velodyne_pointcloud::PointXYZIR &t_p2,
                const velodyne_pointcloud::PointXYZIR &t_p3,
                const velodyne_pointcloud::PointXYZIR &t_p4);

        velodyne_pointcloud::PointXYZIR toVelodyne(const Eigen::Vector3f &t_p);
        Eigen::Vector3f toEigen(const velodyne_pointcloud::PointXYZIR &t_point);
        void minus(velodyne_pointcloud::PointXYZIR &t_p1, 
                   const velodyne_pointcloud::PointXYZIR &t_p2);

        template <class T>
        T blockMatrix(int t_n, ...);


        // template <class T>
        Eigen::Matrix4d poseToEigenMatrix(const geometry_msgs::Pose &t_pose);
        // Eigen::Matrix4d poseToEigenMatrix(const T &pose);

        template <class T>
        Eigen::Matrix3d qToR(const T &t_pose);

        Eigen::Matrix3d qToR(const Eigen::Vector3f &t_pose);

        // q1q2 = q2q1q2^-1
        Eigen::Matrix3d qMultiplication(const double &t_q1_w, const Eigen::Vector3f &t_q1, 
                                        const double &t_q2_w, const Eigen::Vector3f &t_q2);


        template <class T>
        bool goodNumber(T t_number){
            if (std::isinf(t_number) || std::isnan(t_number))
                return false;
            else return true;
        }



        // std::ostream& operator<<(std::ostream& os, const velodyne_pointcloud::PointXYZIR& p);

    }
} // Bipedlab
#endif
