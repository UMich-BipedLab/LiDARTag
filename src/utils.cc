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


#include "utils.h"
#include <math.h>
#include <iostream>

namespace BipedLab{
    namespace utils {
        // milisecond
        double spendTime(const std::clock_t &t_end, const std::clock_t &t_start){
            return (((double) (t_end - t_start))/CLOCKS_PER_SEC)*1e3;
        }

        std::string tranferToLowercase(std::string &t_data){
            std::transform(t_data.begin(), t_data.end(), t_data.begin(), ::tolower);

            return t_data;
        }

		void pressEnterToContinue() {
			int c;
			printf("Press [Enter] key to continue.\n");
			while(getchar()!='\n'); // option TWO to clean stdin
			getchar(); // wait for ENTER
		}

		double deg2Rad(double t_degree){
            return t_degree*M_PI/180;
		}

		double rad2Deg(double t_radian){
            return t_radian*180/M_PI;
		}

		// Checks if a matrix is a valid rotation matrix.
		bool isRotationMatrix(Eigen::Matrix3f &t_R) {
            Eigen::Matrix3f should_be_identity = t_R*t_R.transpose();
			return  (should_be_identity - Eigen::Matrix3f::Identity()).norm() < 1e-6;
		}

        Eigen::Vector3f rotationMatrixToEulerAngles(Eigen::Matrix3f &t_R) {
			// assert(isRotationMatrix(t_R));
			float sy = std::sqrt(t_R(0,0) * (0,0) +  
                                 t_R(1,0) * (1,0));
		 
			bool singular = sy < 1e-6; 
		 
			float x, y, z;
			if (!singular) {
				x = rad2Deg(std::atan(t_R(2,1)/ t_R(2,2)));
				y = rad2Deg(std::atan(-t_R(2,0)/ sy));
				z = rad2Deg(std::atan(t_R(1,0)/ t_R(0,0)));
			}
			else {
				x = rad2Deg(std::atan(-t_R(1,2)/ t_R(1,1)));
				y = rad2Deg(std::atan(-t_R(2,0)/ sy));
				z = 0;
			}

			return Eigen::Vector3f(x, y, z);
		}


        /*
         * A function to check if get all parameters
         */
        bool checkParameters(int t_n, ...){
			va_list vl_num;
			va_start(vl_num, t_n);
            bool Pass = true;
			
			for (int i=0; i<t_n; ++i){
				bool Got = va_arg(vl_num, int);
                if (Got) continue;
                else{
                   std::cout << "didn't get i: " << i << " in the launch file" << std::endl;
                   Pass = false; 
                   break;
                }
			}
			va_end(vl_num);
			return Pass;
		}

        // Overload operator << for PointXYZRI
        // DO NOT want to change their stucture
		void COUT(const velodyne_pointcloud::PointXYZIR& t_p) {
            std::cout << "x: " << t_p.x << ", y: " << t_p.y << ", z: " << t_p.z << 
                ", ring: " << t_p.ring << ", intensity: " << t_p.intensity << std::endl;
		}

        bool compareIndex(LiDARPoints_t *A, LiDARPoints_t *B){
            return A->index < B->index;
        }

		uint64_t bitShift(std::string const& t_value) {
			uint64_t result = 0;

			char const* p = t_value.c_str();
			char const* q = p + t_value.size();
			while (p < q) {
				result = (result << 1) + (result << 3) + *(p++) - '0';
			}

			return result;
		}

        void normalize(std::vector<float> &x, std::vector<float> &y, 
							  std::vector<float> &z, std::vector<float> &I, 
							  const pcl::PointCloud<LiDARPoints_t*> t_payload){
			// normlize the y,z so the top left is (0,0) and bottom right is (1,1)
			// as well as x axis                         
			//                                           o
			// top left                                 /|
			//        o----_o         LiDAR ---> front o |  back
			//        |     |                          | o
			//        |     |                          |/
			//        o-----o                          o
			//               bottom right
			float front_x = 1e8;
			float back_x = -1e8;
			float bottom_right_y = 1e8;
			float top_left_y = -1e8;
			float bottom_right_z = 1e8;
			float top_left_z = -1e8;

			float max_intensity = -1e8;

            for (int i=0; i<t_payload.size(); ++i){
                if(t_payload[i]->point.x>back_x) back_x = t_payload[i]->point.x;
                if(t_payload[i]->point.x<front_x) front_x = t_payload[i]->point.x;

                if(t_payload[i]->point.y>top_left_y) top_left_y = t_payload[i]->point.y;
                if(t_payload[i]->point.y<bottom_right_y) bottom_right_y = t_payload[i]->point.y;

                if(t_payload[i]->point.z>top_left_z) top_left_z = t_payload[i]->point.z;
                if(t_payload[i]->point.z<bottom_right_z) bottom_right_z = t_payload[i]->point.z;
                if(t_payload[i]->point.intensity>max_intensity) max_intensity = t_payload[i]->point.intensity;
            }

            float dx = std::abs(front_x - back_x);
            float dy = std::abs(top_left_y - bottom_right_y);
            float dz = std::abs(top_left_z - bottom_right_z);
            for (int i=0; i<t_payload.size(); ++i){
				x[i] = (back_x - t_payload[i]->point.x)/8;
				y[i] = (top_left_y - t_payload[i]->point.y)/8;
				z[i] = (top_left_z - t_payload[i]->point.z)/8;
				I[i] = (t_payload[i]->point.intensity)/1.5;
            }
		}


        void normalizeByAve(std::vector<float> &x, std::vector<float> &y, 
                          std::vector<float> &z, std::vector<float> &I, 
                          const pcl::PointCloud<LiDARPoints_t*> t_payload){
			float ave_x = 0;
			float ave_y = 0;
			float ave_z = 0;

			for (int i=0; i<t_payload.size(); ++i){
				ave_x += t_payload[i]->point.x;
				ave_y += t_payload[i]->point.y;
				ave_z += t_payload[i]->point.z;
				x[i] = t_payload[i]->point.x;
				y[i] = t_payload[i]->point.y;
				z[i] = t_payload[i]->point.z;
				I[i] = t_payload[i]->point.intensity;
			}
			ave_x /= t_payload.size();
			ave_y /= t_payload.size();
			ave_z /= t_payload.size();

			for (int i=0; i<t_payload.size(); ++i){
				x[i] = (x[i] - ave_x)/5;
				y[i] = (y[i] - ave_y)/5;
				z[i] = (z[i] - ave_z)/5;
				I[i] /= 1.5;
			}
        }

        velodyne_pointcloud::PointXYZIR pointsAddDivide
            (const velodyne_pointcloud::PointXYZIR& t_p1, 
             const velodyne_pointcloud::PointXYZIR& t_p2, float t_d){
            assert(t_d!=0);
            velodyne_pointcloud::PointXYZIR tmp;
                tmp.x = (t_p1.x+t_p2.x)/t_d;
                tmp.y = (t_p1.y+t_p2.y)/t_d;
                tmp.z = (t_p1.z+t_p2.z)/t_d;
                tmp.intensity = (t_p1.intensity+t_p2.intensity)/t_d;

            return tmp;
        }

        // form vector from p1 to p2. ie p2-p1
        velodyne_pointcloud::PointXYZIR vectorize (
            const velodyne_pointcloud::PointXYZIR& t_p1, 
            const velodyne_pointcloud::PointXYZIR& t_p2){
            velodyne_pointcloud::PointXYZIR tmp;
            tmp.x = (t_p2.x-t_p1.x);
            tmp.y = (t_p2.y-t_p1.y);
            tmp.z = (t_p2.z-t_p1.z);
            tmp.intensity = (t_p2.intensity-t_p1.intensity);

            return tmp;
        }

        float dot (const velodyne_pointcloud::PointXYZIR& t_p1, 
                const velodyne_pointcloud::PointXYZIR& t_p2){
            return t_p1.y*t_p2.y + t_p1.z*t_p2.z;
        }

        float Norm (const velodyne_pointcloud::PointXYZIR& t_p){
            return std::sqrt(std::pow(t_p.y, 2) + std::pow(t_p.z, 2));
        }

        double MVN(const float &t_tag_size, const int &t_d,
                   const Eigen::Vector2f &t_X, const Eigen::Vector2f t_mean){
            Eigen::Matrix2f Sigma;
            Sigma << t_tag_size/t_d/2, 0, 0, t_tag_size/t_d/2;
            double sqrt2pi = std::sqrt(2 * M_PI);
            double QuadForm  = (t_X - t_mean).transpose() * Sigma.inverse() * (t_X - t_mean);
            double Norm = std::pow(sqrt2pi, - 2) *
                          std::pow(Sigma.determinant(), - 0.5); 
            
            return Norm * exp(-0.5 * QuadForm);
        }

        // step between p1 and p2
        float getStep(const velodyne_pointcloud::PointXYZIR &t_p1, 
                const velodyne_pointcloud::PointXYZIR &t_p2, const int t_d){
            return std::sqrt(std::pow((t_p2.y-t_p1.y), 2) + std::pow((t_p2.z-t_p1.z), 2))/t_d;
        }

        // To get the t where p1 + t * v12 is the point that p projects onto line p12
        void getProjection(const velodyne_pointcloud::PointXYZIR &t_p1, 
                const velodyne_pointcloud::PointXYZIR &t_p2, 
                const velodyne_pointcloud::PointXYZIR &t_p,
                float &k, Eigen::Vector2f &t_v){
            // form vector from p1 to p2 and p1 to p
            velodyne_pointcloud::PointXYZIR v12 = vectorize(t_p1, t_p2);
            velodyne_pointcloud::PointXYZIR v1p = vectorize(t_p1, t_p);

            k = std::abs(dot(v12, v1p)/Norm(v12));
            // v = v12;
        }

        void assignCellIndex(const float &t_tag_size, const Eigen::Matrix3f &t_R, 
                         velodyne_pointcloud::PointXYZIR &t_p_reference,
                         const velodyne_pointcloud::PointXYZIR &t_average,
                         const int t_d, PayloadVoting_t &t_vote){
            // R: Payload p -> reference x
            // prepare for Gaussian
            float xOffset = t_vote.p->x - t_average.x;
            float yOffset = t_vote.p->y - t_average.y;
            float zOffset = t_vote.p->z - t_average.z;
            // float x = t_vote.p->x;
            // float y = t_vote.p->y;
            // float z = t_vote.p->z;

            float x = xOffset*t_R(0,0) + yOffset*t_R(0,1) + zOffset*t_R(0,2);
            float y = xOffset*t_R(1,0) + yOffset*t_R(1,1) + zOffset*t_R(1,2);
            float z = xOffset*t_R(2,0) + yOffset*t_R(2,1) + zOffset*t_R(2,2);

            // x = x*t_R(0,0) + y*t_R(0,1) + z*t_R(0,2) + t_average.x;
            // y = x*t_R(1,0) + y*t_R(1,1) + z*t_R(1,2) + t_average.y;
            // z = x*t_R(2,0) + y*t_R(2,1) + z*t_R(2,2) + t_average.z;
            // y,z should range int_ between -3s and 3s
            t_p_reference.x = x;
            t_p_reference.y = y;
            t_p_reference.z = z;
            float ss = t_tag_size/t_d; // scale back to the unit square
            y = std::max(std::min(y, t_d/2*ss), (-t_d/2*ss+(float)0.001)); // don't match to 6
            z = std::max(std::min(z, t_d/2*ss), (-t_d/2*ss+(float)0.001)); // don't match to 6
            int cellIndexT = t_d/2 + std::floor(-y/ss);
            int cellIndexK = t_d/2 + std::floor(-z/ss);

            float cy = (std::ceil(y/ss) - 0.5)*ss; // offset to center of each ceil
            float cz = (std::ceil(z/ss) - 0.5)*ss;

            // which grid it belongs to (in 1-16 vector form)?
            Eigen::Vector2f X(y, z);
            Eigen::Vector2f Mean(cy, cz);
            t_vote.centroid.x = 0;
            t_vote.centroid.y = cy;
            t_vote.centroid.z = cz;
            t_vote.cell = t_d*cellIndexK + cellIndexT;
            t_vote.weight = MVN(t_tag_size, t_d, X, Mean);
        }

        // normalize weight and classify them into grid
        void sortPointsToGrid(std::vector<std::vector<PayloadVoting_t*>> &t_grid, 
                          std::vector<PayloadVoting_t> &t_votes){
            for (int i=0; i<t_votes.size(); ++i)
                t_grid[t_votes[i].cell].push_back(&t_votes[i]);
        } 


        void formGrid(Eigen::MatrixXf &t_vertices, 
                float x, float y, float z, float t_tag_size){
            // define 5 points in reference coord frame: x0,...x4 (x0==(0,0,0))
            Eigen::Vector3f tmp;
            tmp << x, y, z; // 0,0,0

            // center
            t_vertices.col(0) = tmp; // center of ref model

            // p1
            tmp[1] = y + t_tag_size/2;
            tmp[2] = z + t_tag_size/2;
            t_vertices.col(1) = tmp;

            // p2
            tmp[1] = y + t_tag_size/2;
            tmp[2] = z - t_tag_size/2;
            t_vertices.col(2) = tmp;

            // p3
            tmp[1] = y - t_tag_size/2;
            tmp[2] = z - t_tag_size/2;
            t_vertices.col(3) = tmp;

            // p4
            tmp[1] = y - t_tag_size/2;
            tmp[2] = z + t_tag_size/2;
            t_vertices.col(4) = tmp;
        }

        void fitGrid(Eigen::MatrixXf &GridVertices, Eigen::Matrix3f &H,
                     const velodyne_pointcloud::PointXYZIR &t_p1,   
                     const velodyne_pointcloud::PointXYZIR &t_p2, 
                     const velodyne_pointcloud::PointXYZIR &t_p3, 
                     const velodyne_pointcloud::PointXYZIR &t_p4){
            Eigen::MatrixXf payload_vertices(3, 4);
            payload_vertices(0,0) = t_p1.x;
            payload_vertices(1,0) = t_p1.y;
            payload_vertices(2,0) = t_p1.z;

            payload_vertices(0,1) = t_p2.x;
            payload_vertices(1,1) = t_p2.y;
            payload_vertices(2,1) = t_p2.z;

            payload_vertices(0,2) = t_p3.x;
            payload_vertices(1,2) = t_p3.y;
            payload_vertices(2,2) = t_p3.z;

            payload_vertices(0,3) = t_p4.x;
            payload_vertices(1,3) = t_p4.y;
            payload_vertices(2,3) = t_p4.z;

            Eigen::Matrix3f M = GridVertices.rightCols(4)*payload_vertices.transpose();
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            // Eigen::Matrix<float,3,3,Eigen::DontAlign> R = svd.matrixV()*svd.matrixU().transpose();
            Eigen::Matrix<float,3,3,Eigen::DontAlign> R = svd.matrixU()*svd.matrixV().transpose();
            H = R; // H: payload -> ref
        }

        velodyne_pointcloud::PointXYZIR toVelodyne(const Eigen::Vector3f &t_p){ 
            velodyne_pointcloud::PointXYZIR point;
            point.x = t_p[0];
            point.y = t_p[1];
            point.z = t_p[2]; 
            
            return point;
        }

        Eigen::Vector3f toEigen(const velodyne_pointcloud::PointXYZIR &t_point){ 
            Eigen::Vector3f tmp;
            tmp[0] = t_point.x;
            tmp[1] = t_point.y;
            tmp[2] = t_point.z; 
            
            return tmp;
        }

        void minus(velodyne_pointcloud::PointXYZIR &t_p1, 
                const velodyne_pointcloud::PointXYZIR &t_p2){
            t_p1.x = t_p1.x - t_p2.x;
            t_p1.y = t_p1.y - t_p2.y; 
            t_p1.z = t_p1.z - t_p2.z;
        }

        float distance(
                const velodyne_pointcloud::PointXYZIR &t_p1,
                const velodyne_pointcloud::PointXYZIR &t_p2){
            return std::sqrt(std::pow((t_p1.x - t_p2.x), 2) + 
                             std::pow((t_p1.y - t_p2.y), 2) + 
                             std::pow((t_p1.z - t_p2.z), 2));
        }
        
        /*
         * A function to calculate angle between va and vb
         * return: angle in degree
         */
        template <class T, class U>
        float getAngle (T a, U b) {
            return rad2Deg(std::acos(dot(a, b)/ (Norm(a) * Norm(b))));
        }

        /* 
         * Check if 4 four corners are valid
         * return  0: valid corners
         * return -1: incorrect distance
         * return -2: incorrect angle
         */
        int checkCorners(
                const float Tagsize,
                const velodyne_pointcloud::PointXYZIR &t_p1,
                const velodyne_pointcloud::PointXYZIR &t_p2,
                const velodyne_pointcloud::PointXYZIR &t_p3,
                const velodyne_pointcloud::PointXYZIR &t_p4){

            // XXX tunable
            float ratio = 1/3;
            float AngleLowerBound = 75;
            float AngleUpperBound = 105;
            if (distance(t_p1, t_p2) < Tagsize*ratio) return -1;
            if (distance(t_p1, t_p3) < Tagsize*ratio) return -1;
            if (distance(t_p1, t_p4) < Tagsize*ratio) return -1;
            if (distance(t_p2, t_p3) < Tagsize*ratio) return -1;
            if (distance(t_p2, t_p4) < Tagsize*ratio) return -1;
            if (distance(t_p3, t_p4) < Tagsize*ratio) return -1;

            // angle between p12 and p14
            float Angle1 = getAngle<velodyne_pointcloud::PointXYZIR,
                                       velodyne_pointcloud::PointXYZIR>
                                       (vectorize(t_p1, t_p2), vectorize(t_p1, t_p4));
            if ((Angle1<AngleLowerBound) || (AngleUpperBound<Angle1)) return -2;

            // angle between p21 and p23
            float Angle2 = getAngle<velodyne_pointcloud::PointXYZIR,
                                       velodyne_pointcloud::PointXYZIR>
                                       (vectorize(t_p2, t_p1), vectorize(t_p2, t_p3));
            if ((Angle2<AngleLowerBound) || (AngleUpperBound<Angle2)) return -2;


            // angle between p32 and p34
            float Angle3 = getAngle<velodyne_pointcloud::PointXYZIR,
                                       velodyne_pointcloud::PointXYZIR>
                                       (vectorize(t_p3, t_p2), vectorize(t_p3, t_p4));
            if ((Angle3<AngleLowerBound) || (AngleUpperBound<Angle3)) return -2;

            // angle between p43 and p41
            float Angle4 = getAngle<velodyne_pointcloud::PointXYZIR,
                                       velodyne_pointcloud::PointXYZIR>
                                       (vectorize(t_p4, t_p3), vectorize(t_p4, t_p1));
            if ((Angle4<AngleLowerBound) || (AngleUpperBound<Angle4)) return -2;

            return 0;
        }

		template <class T>
		T blockMatrix(int t_n, ...){
			/* Function for creating blockdiagonal given arbitrary number of arguments.  */
			va_list vl_num;
			va_start(vl_num, t_n);
			int cols_now = 0;
			int rows_now = 0;
			
			for (int i=0; i<t_n; ++i){
				T matrix = va_arg(vl_num, T);
				cols_now = cols_now + matrix.cols();
				rows_now = rows_now + matrix.rows();
			}
			va_end(vl_num);
			T Mblock = T::Zero(rows_now, cols_now);
			va_list vl;
			va_start(vl, t_n);
			int rows = 0;
			int cols = 0;
			for (int i=0; i<t_n; ++i){
				T matrix = va_arg(vl, T);
				Mblock.block(rows, cols, matrix.rows(), matrix.cols()) = matrix;
				rows += matrix.rows();
				cols += matrix.cols();
			}
			return Mblock;
		}

        // pose is geometry_msgs pose
		// template <class T>
		// Eigen::Matrix4d poseToEigenMatrix(const T &pose){
		Eigen::Matrix4d poseToEigenMatrix(const geometry_msgs::Pose &t_pose){
            Eigen::Matrix4d matrix_pose = Eigen::Matrix4d::Identity();
            matrix_pose(0, 3) = t_pose.position.x;
            matrix_pose(1, 3) = t_pose.position.y;
            matrix_pose(2, 3) = t_pose.position.z;
            matrix_pose.topLeftCorner(3, 3) << qToR(t_pose);

            return matrix_pose;
		}


        // pose is geometry_msgs pose
		template <class T>
		Eigen::Matrix3d qToR(const T &t_pose){
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            double a = t_pose.orientation.w;
            double b = t_pose.orientation.x;
            double c = t_pose.orientation.y;
            double d = t_pose.orientation.z;
            R(0, 0) = std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2) - std::pow(d, 2);
            R(0, 1) = 2*(b*c - a*d);
            R(0, 2) = 2*(b*d + a*c);

            R(1, 0) = 2*(b*c + a*d);
            R(1, 1) = std::pow(a, 2) - std::pow(b, 2) + std::pow(c, 2) - std::pow(d, 2);
            R(1, 2) = 2*(c*d - a*b);

            R(2, 0) = 2*(b*d - a*c);
            R(2, 1) = 2*(c*d + a*b);
            R(2, 2) = std::pow(a, 2) - std::pow(b, 2) - std::pow(c, 2) + std::pow(d, 2);
            
            return R;
        }

		Eigen::Matrix3d qToR(const Eigen::Vector3f &t_pose){
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            double a = 0; // w
            double b = t_pose(0); // x
            double c = t_pose(1); // y
            double d = t_pose(2); // z
            R(0, 0) = std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2) - std::pow(d, 2);
            R(0, 1) = 2*(b*c - a*d);
            R(0, 2) = 2*(b*d + a*c);

            R(1, 0) = 2*(b*c + a*d);
            R(1, 1) = std::pow(a, 2) - std::pow(b, 2) + std::pow(c, 2) - std::pow(d, 2);
            R(1, 2) = 2*(c*d - a*b);

            R(2, 0) = 2*(b*d - a*c);
            R(2, 1) = 2*(c*d + a*b);
            R(2, 2) = std::pow(a, 2) - std::pow(b, 2) - std::pow(c, 2) + std::pow(d, 2);
            
            return R;
        }


        Eigen::Matrix3d qMultiplication(const double &q1_w, const Eigen::Vector3f &q1, 
                                        const double &q2_w, const Eigen::Vector3f &q2){
        }
    } // utils
} // Bipedlab
