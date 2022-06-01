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

#include <lidartag/lidartag.hpp>
#include <lidartag/utils.hpp>

#include <math.h>
#include <algorithm>  // std::sort, std::stable_sort
#include <iostream>
#include <numeric>  // std::iota

namespace BipedLab
{
namespace utils
{
// second
double spendCPUTime(const std::clock_t & t_end, const std::clock_t & t_start)
{
  return (((double)(t_end - t_start)) / CLOCKS_PER_SEC);
}

double spendCPUHz(const std::clock_t & t_end, const std::clock_t & t_start)
{
  return 1.0 / spendCPUTime(t_end, t_start);
}

void printSpendCPUHz(const std::clock_t & t_end, const std::clock_t & t_start, std::string txt)
{
  std::cout << std::fixed << std::setprecision(2) << txt << spendCPUHz(t_end, t_start) << " [Hz]"
            << std::endl;
}

void printSpendCPUHz(const std::clock_t & t_end, const std::clock_t & t_start)
{
  std::string text = "CPU time used: ";
  printSpendCPUHz(t_end, t_start, text);
}

// std::chrono::steady_clock::time_point clock_start =
// std::chrono::steady_clock::now(); std::chrono::duration<double> duration =
//     std::chrono::steady_clock::now() - clock_start;

double spendElapsedTime(
  const std::chrono::steady_clock::time_point & t_end,
  const std::chrono::steady_clock::time_point & t_start)
{
  std::chrono::duration<double> duration = t_end - t_start;
  return duration.count();
}

double spendElapsedTimeMilli(
  const std::chrono::steady_clock::time_point & t_end,
  const std::chrono::steady_clock::time_point & t_start)
{
  return 1e3 * spendElapsedTime(t_end, t_start);
}

double spendElapsedHz(
  const std::chrono::steady_clock::time_point & t_end,
  const std::chrono::steady_clock::time_point & t_start)
{
  return 1.0 / spendElapsedTime(t_end, t_start);
}

void printSpendElapsedHz(
  const std::chrono::steady_clock::time_point & t_end,
  const std::chrono::steady_clock::time_point & t_start, std::string txt)
{
  std::cout << std::fixed << std::setprecision(2) << txt << spendElapsedHz(t_end, t_start)
            << " [Hz]" << std::endl;
}

void printSpendElapsedHz(
  const std::chrono::steady_clock::time_point & t_end,
  const std::chrono::steady_clock::time_point & t_start)
{
  std::string text = "Elapsed time: ";
  printSpendElapsedHz(t_end, t_start, text);
}

// bool angleComparision (float i, float j) {
//   return std::abs(i-j) < 1e-3;
// }

std::string tranferToLowercase(std::string & t_data)
{
  std::transform(t_data.begin(), t_data.end(), t_data.begin(), ::tolower);

  return t_data;
}

void pressEnterToContinue()
{
  int c;
  printf("Press [Enter] key to continue.\n");
  while (getchar() != '\n')
    ;         // option TWO to clean stdin
  getchar();  // wait for ENTER
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Eigen::Matrix3f & t_R)
{
  Eigen::Matrix3f should_be_identity = t_R * t_R.transpose();
  return (should_be_identity - Eigen::Matrix3f::Identity()).norm() < 1e-6;
}

Eigen::Vector3f rotationMatrixToEulerAngles(Eigen::Matrix3f & t_R)
{
  // assert(isRotationMatrix(t_R));
  float sy = std::sqrt(t_R(0, 0) * (0, 0) + t_R(1, 0) * (1, 0));

  bool singular = sy < 1e-6;

  float x, y, z;
  if (!singular) {
    x = rad2Deg(std::atan(t_R(2, 1) / t_R(2, 2)));
    y = rad2Deg(std::atan(-t_R(2, 0) / sy));
    z = rad2Deg(std::atan(t_R(1, 0) / t_R(0, 0)));
  } else {
    x = rad2Deg(std::atan(-t_R(1, 2) / t_R(1, 1)));
    y = rad2Deg(std::atan(-t_R(2, 0) / sy));
    z = 0;
  }

  return Eigen::Vector3f(x, y, z);
}

/*
 * A function to check if get all parameters
 */
bool checkParameters(const std::vector<bool>& list)
{
  bool pass = true;

  for (int i = 0; i < list.size(); ++i) {
    if (!list[i]) {
      pass = false;
      std::cout << "didn't get i: " << i << " in the launch file" << std::endl;
    }
  }

  return pass;
}

// Overload operator << for PointXYZRI
// DO NOT want to change their stucture
void COUT(const velodyne_pointcloud::PointXYZIR & t_p)
{
  std::cout << "x: " << t_p.x << ", y: " << t_p.y << ", z: " << t_p.z << ", ring: " << t_p.ring
            << ", intensity: " << t_p.intensity << std::endl;
}

bool compareIndex(LidarPoints_t * A, LidarPoints_t * B) { return A->index < B->index; }

uint64_t bitShift(std::string const & t_value)
{
  uint64_t result = 0;

  char const * p = t_value.c_str();
  char const * q = p + t_value.size();
  while (p < q) {
    result = (result << 1) + (result << 3) + *(p++) - '0';
  }

  return result;
}

void normalize(
  std::vector<float> & x, std::vector<float> & y, std::vector<float> & z, std::vector<float> & I,
  const pcl::PointCloud<LidarPoints_t *> t_payload)
{
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

  for (int i = 0; i < t_payload.size(); ++i) {
    if (t_payload[i]->point.x > back_x) back_x = t_payload[i]->point.x;
    if (t_payload[i]->point.x < front_x) front_x = t_payload[i]->point.x;

    if (t_payload[i]->point.y > top_left_y) top_left_y = t_payload[i]->point.y;
    if (t_payload[i]->point.y < bottom_right_y) bottom_right_y = t_payload[i]->point.y;

    if (t_payload[i]->point.z > top_left_z) top_left_z = t_payload[i]->point.z;
    if (t_payload[i]->point.z < bottom_right_z) bottom_right_z = t_payload[i]->point.z;
    if (t_payload[i]->point.intensity > max_intensity)
      max_intensity = t_payload[i]->point.intensity;
  }

  float dx = std::abs(front_x - back_x);
  float dy = std::abs(top_left_y - bottom_right_y);
  float dz = std::abs(top_left_z - bottom_right_z);
  for (int i = 0; i < t_payload.size(); ++i) {
    x[i] = (back_x - t_payload[i]->point.x) / 8;
    y[i] = (top_left_y - t_payload[i]->point.y) / 8;
    z[i] = (top_left_z - t_payload[i]->point.z) / 8;
    I[i] = (t_payload[i]->point.intensity) / 1.5;
  }
}

void normalizeByAve(
  std::vector<float> & x, std::vector<float> & y, std::vector<float> & z, std::vector<float> & I,
  const pcl::PointCloud<LidarPoints_t *> t_payload)
{
  float ave_x = 0;
  float ave_y = 0;
  float ave_z = 0;

  for (int i = 0; i < t_payload.size(); ++i) {
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

  for (int i = 0; i < t_payload.size(); ++i) {
    x[i] = (x[i] - ave_x) / 5;
    y[i] = (y[i] - ave_y) / 5;
    z[i] = (z[i] - ave_z) / 5;
    I[i] /= 1.5;
  }
}

velodyne_pointcloud::PointXYZIR pointsAddDivide(
  const velodyne_pointcloud::PointXYZIR & t_p1, const velodyne_pointcloud::PointXYZIR & t_p2,
  float t_d)
{
  assert(t_d != 0);
  velodyne_pointcloud::PointXYZIR tmp;
  tmp.x = (t_p1.x + t_p2.x) / t_d;
  tmp.y = (t_p1.y + t_p2.y) / t_d;
  tmp.z = (t_p1.z + t_p2.z) / t_d;
  tmp.intensity = (t_p1.intensity + t_p2.intensity) / t_d;

  return tmp;
}

// form vector from p1 to p2. ie p2-p1
velodyne_pointcloud::PointXYZIR vectorize(
  const velodyne_pointcloud::PointXYZIR & t_p1, const velodyne_pointcloud::PointXYZIR & t_p2)
{
  velodyne_pointcloud::PointXYZIR tmp;
  tmp.x = (t_p2.x - t_p1.x);
  tmp.y = (t_p2.y - t_p1.y);
  tmp.z = (t_p2.z - t_p1.z);
  tmp.intensity = (t_p2.intensity - t_p1.intensity);

  return tmp;
}

float dot(
  const velodyne_pointcloud::PointXYZIR & t_p1, const velodyne_pointcloud::PointXYZIR & t_p2)
{
  return t_p1.y * t_p2.y + t_p1.z * t_p2.z;
}

float norm(const velodyne_pointcloud::PointXYZIR & t_p)
{
  return std::sqrt(std::pow(t_p.y, 2) + std::pow(t_p.z, 2));
}

double mvn(
  const float & t_tag_size, const int & t_d, const Eigen::Vector2f & t_X,
  const Eigen::Vector2f t_mean)
{
  Eigen::Matrix2f Sigma;
  Sigma << t_tag_size / t_d / 2, 0, 0, t_tag_size / t_d / 2;
  double sqrt2pi = std::sqrt(2 * M_PI);
  double QuadForm = (t_X - t_mean).transpose() * Sigma.inverse() * (t_X - t_mean);
  double norm = std::pow(sqrt2pi, -2) * std::pow(Sigma.determinant(), -0.5);

  return norm * exp(-0.5 * QuadForm);
}

// step between p1 and p2
float getStep(
  const velodyne_pointcloud::PointXYZIR & t_p1, const velodyne_pointcloud::PointXYZIR & t_p2,
  const int t_d)
{
  return std::sqrt(std::pow((t_p2.y - t_p1.y), 2) + std::pow((t_p2.z - t_p1.z), 2)) / t_d;
}

// To get the t where p1 + t * v12 is the point that p projects onto line p12
void getProjection(
  const velodyne_pointcloud::PointXYZIR & t_p1, const velodyne_pointcloud::PointXYZIR & t_p2,
  const velodyne_pointcloud::PointXYZIR & t_p, float & k, Eigen::Vector2f & t_v)
{
  // form vector from p1 to p2 and p1 to p
  velodyne_pointcloud::PointXYZIR v12 = vectorize(t_p1, t_p2);
  velodyne_pointcloud::PointXYZIR v1p = vectorize(t_p1, t_p);

  k = std::abs(dot(v12, v1p) / norm(v12));
  // v = v12;
}

void assignCellIndex(
  const float & t_tag_size, const Eigen::Matrix3f & t_R,
  velodyne_pointcloud::PointXYZIR & t_p_reference,
  const velodyne_pointcloud::PointXYZIR & t_average, const int t_d, PayloadVoting_t & t_vote)
{
  // R: Payload p -> reference x
  // prepare for Gaussian
  float xOffset = t_vote.p->x - t_average.x;
  float yOffset = t_vote.p->y - t_average.y;
  float zOffset = t_vote.p->z - t_average.z;
  // float x = t_vote.p->x;
  // float y = t_vote.p->y;
  // float z = t_vote.p->z;

  float x = xOffset * t_R(0, 0) + yOffset * t_R(0, 1) + zOffset * t_R(0, 2);
  float y = xOffset * t_R(1, 0) + yOffset * t_R(1, 1) + zOffset * t_R(1, 2);
  float z = xOffset * t_R(2, 0) + yOffset * t_R(2, 1) + zOffset * t_R(2, 2);

  // x = x*t_R(0,0) + y*t_R(0,1) + z*t_R(0,2) + t_average.x;
  // y = x*t_R(1,0) + y*t_R(1,1) + z*t_R(1,2) + t_average.y;
  // z = x*t_R(2,0) + y*t_R(2,1) + z*t_R(2,2) + t_average.z;
  // y,z should range int_ between -3s and 3s
  t_p_reference.x = x;
  t_p_reference.y = y;
  t_p_reference.z = z;
  float ss = t_tag_size / t_d;  // scale back to the unit square
  y = std::max(std::min(y, t_d / 2 * ss),
               (-t_d / 2 * ss + (float)0.001));  // don't match to 6
  z = std::max(std::min(z, t_d / 2 * ss),
               (-t_d / 2 * ss + (float)0.001));  // don't match to 6
  int cellIndexT = t_d / 2 + std::floor(-y / ss);
  int cellIndexK = t_d / 2 + std::floor(-z / ss);

  float cy = (std::ceil(y / ss) - 0.5) * ss;  // offset to center of each ceil
  float cz = (std::ceil(z / ss) - 0.5) * ss;

  // which grid it belongs to (in 1-16 vector form)?
  Eigen::Vector2f X(y, z);
  Eigen::Vector2f mean(cy, cz);
  t_vote.centroid.x = 0;
  t_vote.centroid.y = cy;
  t_vote.centroid.z = cz;
  t_vote.cell = t_d * cellIndexK + cellIndexT;
  t_vote.weight = mvn(t_tag_size, t_d, X, mean);
}

// normalize weight and classify them into grid
void sortPointsToGrid(
  std::vector<std::vector<PayloadVoting_t *>> & t_grid, std::vector<PayloadVoting_t> & t_votes)
{
  for (int i = 0; i < t_votes.size(); ++i) t_grid[t_votes[i].cell].push_back(&t_votes[i]);
}

void formGrid(Eigen::MatrixXf & t_vertices, float x, float y, float z, float t_tag_size)
{
  // define 5 points in reference coord frame: x0,...x4 (x0==(0,0,0))
  Eigen::Vector3f tmp;
  tmp << x, y, z;  // 0,0,0

  // center
  t_vertices.col(0) = tmp;  // center of ref model

  // p1
  tmp[1] = y + t_tag_size / 2;
  tmp[2] = z + t_tag_size / 2;
  t_vertices.col(1) = tmp;

  // p2
  tmp[1] = y + t_tag_size / 2;
  tmp[2] = z - t_tag_size / 2;
  t_vertices.col(2) = tmp;

  // p3
  tmp[1] = y - t_tag_size / 2;
  tmp[2] = z - t_tag_size / 2;
  t_vertices.col(3) = tmp;

  // p4
  tmp[1] = y - t_tag_size / 2;
  tmp[2] = z + t_tag_size / 2;
  t_vertices.col(4) = tmp;
}

void fitGrid(
  Eigen::MatrixXf & grid_vertices, Eigen::Matrix3f & H, const velodyne_pointcloud::PointXYZIR & t_p1,
  const velodyne_pointcloud::PointXYZIR & t_p2, const velodyne_pointcloud::PointXYZIR & t_p3,
  const velodyne_pointcloud::PointXYZIR & t_p4)
{
  Eigen::MatrixXf payload_vertices(3, 4);
  payload_vertices(0, 0) = t_p1.x;
  payload_vertices(1, 0) = t_p1.y;
  payload_vertices(2, 0) = t_p1.z;

  payload_vertices(0, 1) = t_p2.x;
  payload_vertices(1, 1) = t_p2.y;
  payload_vertices(2, 1) = t_p2.z;

  payload_vertices(0, 2) = t_p3.x;
  payload_vertices(1, 2) = t_p3.y;
  payload_vertices(2, 2) = t_p3.z;

  payload_vertices(0, 3) = t_p4.x;
  payload_vertices(1, 3) = t_p4.y;
  payload_vertices(2, 3) = t_p4.z;

  Eigen::Matrix3f M = grid_vertices.rightCols(4) * payload_vertices.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // Eigen::Matrix<float,3,3,Eigen::DontAlign> R =
  // svd.matrixV()*svd.matrixU().transpose();
  Eigen::Matrix<float, 3, 3, Eigen::DontAlign> R = svd.matrixU() * svd.matrixV().transpose();
  H = R;  // H: payload -> ref
}

void fitGridNew(
  const Eigen::MatrixXf & grid_vertices, Eigen::Matrix3f & H,
  const Eigen::MatrixXf & payload_vertices)
{

  Eigen::Matrix3f M = grid_vertices.rightCols(4) * payload_vertices.transpose();
  Eigen::Matrix<float, 3, 3, Eigen::DontAlign> R;

  Eigen::Matrix3f r, U, Vt;
  cv::Mat cv_A, cv_W, cv_U, cv_Vt;
  cv::eigen2cv(M, cv_A);
  cv::SVD::compute(cv_A, cv_W, cv_U, cv_Vt);
  cv::cv2eigen(cv_W, r);
  cv::cv2eigen(cv_U, U);
  cv::cv2eigen(cv_Vt, Vt);

  R = U * Vt;
  double determinant = R.determinant();

  assert(std::abs(std::abs(determinant) - 1) < 0.01);

  if (determinant < 0.0) {
    Vt.row(2) *= -1;
    R = U * Vt;
    determinant = R.determinant();
  }

  assert(std::abs(determinant - 1) < 0.01);

  H = R;  // H: payload -> ref


  /** Eigen Version -> Replaced by OpenCV since sgk-000 reported numerical unstability (unconfirmed)
  Eigen::Matrix3f M =
      grid_vertices.rightCols(4)*payload_vertices.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(
          M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<float,3,3,Eigen::DontAlign> R =
      svd.matrixU()*svd.matrixV().transpose();

  double determinant = R.determinant();

  assert(std::abs(std::abs(determinant) - 1) < 0.01);

  if (determinant < 0.0) {
    Eigen::MatrixXf V = svd.matrixV();
    V.col(2) *= -1;
    R = svd.matrixU()*V.transpose();
    determinant = R.determinant();
  }

  assert(std::abs(determinant - 1) < 0.01);

  H = R;  // H: payload -> ref
  */
}
velodyne_pointcloud::PointXYZIR toVelodyne(const Eigen::Vector3f & t_p)
{
  velodyne_pointcloud::PointXYZIR point;
  point.x = t_p[0];
  point.y = t_p[1];
  point.z = t_p[2];

  return point;
}

Eigen::Vector3f toEigen(const velodyne_pointcloud::PointXYZIR & t_point)
{
  Eigen::Vector3f tmp;
  tmp[0] = t_point.x;
  tmp[1] = t_point.y;
  tmp[2] = t_point.z;

  return tmp;
}

void minus(velodyne_pointcloud::PointXYZIR & t_p1, const velodyne_pointcloud::PointXYZIR & t_p2)
{
  t_p1.x = t_p1.x - t_p2.x;
  t_p1.y = t_p1.y - t_p2.y;
  t_p1.z = t_p1.z - t_p2.z;
}

float distance(
  const velodyne_pointcloud::PointXYZIR & t_p1, const velodyne_pointcloud::PointXYZIR & t_p2)
{
  return std::sqrt(
    std::pow((t_p1.x - t_p2.x), 2) + std::pow((t_p1.y - t_p2.y), 2) +
    std::pow((t_p1.z - t_p2.z), 2));
}

/*
 * A function to calculate angle between va and vb
 * return: angle in degree
 */
template <class T, class U>
float getAngle(T a, U b)
{
  return rad2Deg(std::acos(dot(a, b) / (norm(a) * norm(b))));
}

/*
 * Check if 4 four corners are valid
 * return  0: valid corners
 * return -1: incorrect distance
 * return -2: incorrect angle
 */
int checkCorners(
  const float tag_size, const velodyne_pointcloud::PointXYZIR & t_p1,
  const velodyne_pointcloud::PointXYZIR & t_p2, const velodyne_pointcloud::PointXYZIR & t_p3,
  const velodyne_pointcloud::PointXYZIR & t_p4)
{
  // XXX tunable
  float ratio = 1 / 3;
  float angle_lower_bound = 75;
  float angle_upper_bound = 105;

  if (distance(t_p1, t_p2) < tag_size * ratio) {
    return -1;
  }

  if (distance(t_p1, t_p3) < tag_size * ratio) {
    return -1;
  }
  if (distance(t_p1, t_p4) < tag_size * ratio) {
    return -1;
  }

  if (distance(t_p2, t_p3) < tag_size * ratio) {
    return -1;
  }

  if (distance(t_p2, t_p4) < tag_size * ratio) {
    return -1;
  }

  if (distance(t_p3, t_p4) < tag_size * ratio) {
    return -1;
  }

  // angle between p12 and p14
  float angle_1 = getAngle<velodyne_pointcloud::PointXYZIR, velodyne_pointcloud::PointXYZIR>(
    vectorize(t_p1, t_p2), vectorize(t_p1, t_p4));

  if ((angle_1 < angle_lower_bound) || (angle_upper_bound < angle_1)) {
    return -2;
  }

  // angle between p21 and p23
  float angle_2 = getAngle<velodyne_pointcloud::PointXYZIR, velodyne_pointcloud::PointXYZIR>(
    vectorize(t_p2, t_p1), vectorize(t_p2, t_p3));

  if ((angle_2 < angle_lower_bound) || (angle_upper_bound < angle_2)) {
    return -2;
  }

  // angle between p32 and p34
  float angle_3 = getAngle<velodyne_pointcloud::PointXYZIR, velodyne_pointcloud::PointXYZIR>(
    vectorize(t_p3, t_p2), vectorize(t_p3, t_p4));

  if ((angle_3 < angle_lower_bound) || (angle_upper_bound < angle_3)) {
    return -2;
  }

  // angle between p43 and p41
  float angle_4 = getAngle<velodyne_pointcloud::PointXYZIR, velodyne_pointcloud::PointXYZIR>(
    vectorize(t_p4, t_p3), vectorize(t_p4, t_p1));

  if ((angle_4 < angle_lower_bound) || (angle_upper_bound < angle_4)) {
    return -2;
  }

  return 0;
}

/* Function for creating blockdiagonal given arbitrary number of arguments.  */
template <class T>
T blockMatrix(int t_n, ...)
{
  va_list vl_num;
  va_start(vl_num, t_n);
  int cols_now = 0;
  int rows_now = 0;

  for (int i = 0; i < t_n; ++i) {
    T matrix = va_arg(vl_num, T);
    cols_now = cols_now + matrix.cols();
    rows_now = rows_now + matrix.rows();
  }
  va_end(vl_num);
  T m_block = T::Zero(rows_now, cols_now);
  va_list vl;
  va_start(vl, t_n);
  int rows = 0;
  int cols = 0;
  for (int i = 0; i < t_n; ++i) {
    T matrix = va_arg(vl, T);
    m_block.block(rows, cols, matrix.rows(), matrix.cols()) = matrix;
    rows += matrix.rows();
    cols += matrix.cols();
  }
  return m_block;
}

// pose is geometry_msgs pose
// template <class T>
// Eigen::Matrix4d poseToEigenMatrix(const T &pose){
Eigen::Matrix4d poseToEigenMatrix(const geometry_msgs::msg::Pose &t_pose)
{
  Eigen::Matrix4d matrix_pose = Eigen::Matrix4d::Identity();
  matrix_pose(0, 3) = t_pose.position.x;
  matrix_pose(1, 3) = t_pose.position.y;
  matrix_pose(2, 3) = t_pose.position.z;
  matrix_pose.topLeftCorner(3, 3) << qToR(t_pose);

  return matrix_pose;
}

// pose is geometry_msgs pose
template <class T>
Eigen::Matrix3d qToR(const T & t_pose)
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  double a = t_pose.orientation.w;
  double b = t_pose.orientation.x;
  double c = t_pose.orientation.y;
  double d = t_pose.orientation.z;
  R(0, 0) = std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2) - std::pow(d, 2);
  R(0, 1) = 2 * (b * c - a * d);
  R(0, 2) = 2 * (b * d + a * c);

  R(1, 0) = 2 * (b * c + a * d);
  R(1, 1) = std::pow(a, 2) - std::pow(b, 2) + std::pow(c, 2) - std::pow(d, 2);
  R(1, 2) = 2 * (c * d - a * b);

  R(2, 0) = 2 * (b * d - a * c);
  R(2, 1) = 2 * (c * d + a * b);
  R(2, 2) = std::pow(a, 2) - std::pow(b, 2) - std::pow(c, 2) + std::pow(d, 2);

  return R;
}

Eigen::Matrix3d qToR(const Eigen::Vector3f &t_pose)
{
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

Eigen::Matrix3d qMultiplication(
  const double & q1_w, const Eigen::Vector3f & q1, const double & q2_w, const Eigen::Vector3f & q2)
{
  return Eigen::Matrix3d::Identity();
}

/*
 * Returns all numbers not in set, where the total set is [0,n)
 */
std::vector<int> complementOfSet(const std::vector<int> & set, std::size_t n)
{
  std::size_t curr_idx = 0;
  std::vector<int> complement;

  for (auto i = 0; i < n; i++) {
    if (curr_idx >= set.size()) {
      complement.push_back(i);
    } else if (i < set[curr_idx]) {
      complement.push_back(i);
    } else if (i == set[curr_idx]) {
      curr_idx++;
    }  // Inlier
  }

  return complement;
}

float dotProduct(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

Eigen::Vector3f crossProduct(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2)
{
  Eigen::Vector3f res;
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
  return res;
}

void readDirectory(const std::string & name, std::vector<std::string> & v)
{
  boost::filesystem::path p(name);
  boost::filesystem::directory_iterator start(p);
  boost::filesystem::directory_iterator end;
  std::transform(start, end, std::back_inserter(v), PathLeafString());
  std::sort(v.begin(), v.end());
}

float computeMedian(const Eigen::VectorXf & eigen_vec)
{
  assert(eigen_vec.size() != 0);
  std::vector<float> vec(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());
  assert(vec.size() != 0);
  if (vec.size() % 2 == 0) {
    const auto median_it1 = vec.begin() + vec.size() / 2 - 1;
    const auto median_it2 = vec.begin() + vec.size() / 2;

    std::nth_element(vec.begin(), median_it1, vec.end());
    const auto e1 = *median_it1;

    std::nth_element(vec.begin(), median_it2, vec.end());
    const auto e2 = *median_it2;

    return (e1 + e2) / 2;

  } else {
    const auto median_it = vec.begin() + vec.size() / 2;
    std::nth_element(vec.begin(), median_it, vec.end());

    return *median_it;
  }
}

Eigen::MatrixXf convertXYZIToHomogeneous(const Eigen::MatrixXf & mat_xyzi)
{
  assert(mat_xyzi.rows() == 4 && "The input dimension is wrong, it should be four");
  Eigen::MatrixXf mat_h = mat_xyzi;
  mat_h.row(3).setOnes();

  return mat_h;
}

/* A function takes in rot_v as r, p, y in deg and trans_v as x, y, z in meter.
 * It returns a rigid-body transformation.
 * [Note] The rotation matrix from rpy follows the "XYZ" convention
 */
Eigen::Matrix4f computeTransformation(Eigen::Vector3f rot_v, Eigen::Vector3f trans_v)
{
  Eigen::Matrix4f H = Eigen::Matrix4f::Identity(4, 4);
  H.topLeftCorner(3, 3) = computeRotX(rot_v(0)) * computeRotY(rot_v(1)) * computeRotZ(rot_v(2));
  H.topRightCorner(3, 1) = trans_v;

  return H;
}

double getSign(double x)
{
  double output;
  if (x >= 0) {
    output = 1;
  } else {
    output = -1;
  }
  return output;
}

Eigen::Matrix3f skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3f m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

Eigen::Vector3d unskew(const Eigen::Matrix3f & a_x)
{
  Eigen::Vector3d v(a_x(2, 1), a_x(0, 2), a_x(1, 0));
  return v;
}

Eigen::Matrix3f expSO3(const Eigen::Vector3d & w)
{
  double theta = w.norm();
  Eigen::Matrix3f A = utils::skew(w);
  Eigen::Matrix3f output;
  // cout << Eigen::Matrix3d::Identity() << endl;
  if (theta == 0) {
    output = Eigen::Matrix3f::Identity();
  } else {
    output = Eigen::Matrix3f::Identity() + (std::sin(theta) / theta) * A +
             ((1 - std::cos(theta)) / std::pow(theta, 2)) * A * A;
  }
  return output;
}

Eigen::Vector3d logSO3(const Eigen::Matrix3f & A)
{
  double theta = std::acos((A(0, 0) + A(1, 1) + A(2, 2) - 1) / 2);
  Eigen::Matrix3f A_transpose = A.transpose();
  Eigen::Vector3d output;
  if (theta == 0) {
    output = Eigen::Vector3d::Zero(3);
  } else {
    output = utils::unskew(theta * (A - A_transpose) / (2 * std::sin(theta)));
  }
  return output;
}

// 3D cross product of OA and OB vectors,
// (i.e x-component of their "2D" cross product,
// but remember that it is not defined in "2D").
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
// compute on y-z plane and direction on x or -x axis
double cross(const Eigen::Vector4f & O, const Eigen::Vector4f & A, const Eigen::Vector4f & B)
{
  return (A[1] - O[1]) * (B[2] - O[2]) - (A[2] - O[2]) * (B[1] - O[1]);
}

// comparator of transformed points, used for convex hull
void sortEigenVectorIndices(const Eigen::MatrixXf & mat, Eigen::VectorXi & indices)
{
  // initialize original index locations
  int num_elements = mat.cols();
  std::vector<int> idx(num_elements);
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values
  std::stable_sort(idx.begin(), idx.end(), [&mat](int i1, int i2) {
    return (mat(1, i1) < mat(1, i2)) || (mat(1, i1) == mat(1, i2) && mat(2, i1) < mat(2, i2));
  });
  // Eigen::Transpositions indices(Eigen::Map<Eigen::Matrix<float, num_elements,
  // 1>>(idx)); Eigen::Transpositions indices; Eigen::Map<Eigen::VectorXi>
  // test(idx.data()).cast<int>(); printVector(idx);
  int * ptr = &idx[0];
  Eigen::Map<Eigen::VectorXi> tmp(ptr, num_elements);
  indices = tmp;
}

// Vertices are in a 4xn matrix as [x,y,z,1]
// This function computes the area on y-z plane
float computePolygonArea(const Eigen::MatrixXf & vertices)
{
  // Initialze area
  float area = 0.0;

  int num_pts = vertices.cols();

  // Calculate value of shoelace formula
  int j = num_pts - 1;
  for (int i = 0; i < num_pts; i++) {
    area += (vertices(1, j) + vertices(1, i)) * (vertices(2, j) - vertices(2, i));
    j = i;  // j is previous vertex to i
  }

  // Return absolute value
  return abs(area / 2.0);
}

// Returns a list of points on the convex hull in counter-clockwise order.
void constructConvexHull(const Eigen::MatrixXf & P, Eigen::MatrixXf & convex_hull)
{
  size_t n = P.cols();

  if (n <= 3) {
    convex_hull = P;
  }

  // Eigen::MatrixXf Q(Eigen::MatrixXf::Random(3, 4));
  // Q(1, 0) = 3;
  // Q(1, 1) = -1;
  // Q(1, 2) = -2;
  // Q(1, 3) = 5;
  // Eigen::VectorXi indices;
  // sortEigenVectorIndices(Q.row(1), indices); // sort by y
  // Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm(indices);
  // Eigen::MatrixXf sorted_points = Q * perm;
  // std::cout << "Q: \n" << Q << std::endl;
  // std::cout << "indices: \n" << indices << std::endl;
  // std::cout << "sorted Q: \n" << sorted_points << std::endl;

  // Sort points lexicographically
  Eigen::VectorXi indices;
  sortEigenVectorIndices(P, indices);
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm(indices);
  Eigen::MatrixXf sorted_points = P * perm;

  // std::vector<Eigen::Vector4d> up;
  // std::vector<Eigen::Vector4d> down;
  // up.push_back(sorted_points.col(0));
  // down.push_back(sorted_points.col(0));
  // for (int i = 1; i < sorted_points.cols(); i++) {
  //     Eigen::Vector4d cur_pt = sorted_points.col(i);
  //     if (i == sorted_points.cols() - 1 || cross(left_most, cur_pt,
  //     right_most) > 0) {
  //         while (up.size() >= 2 && cross(up[up.size()-2], up[up.size()-1],
  //         cur_pt) <= 0)
  //             up.pop_back();
  //         up.push_back(cur_pt);
  //     }
  //     if (i == sorted_points.cols() - 1 || cross(left_most, cur_pt,
  //     right_most) <=0) {
  //         while(down.size() >= 2 && cross(down[down.size()-2],
  //         down[down.size()-1], cur_pt) >= 0)
  //             down.pop_back();
  //         down.push_back(cur_pt);
  //     }
  // }
  // up.insert(up.end(), down.begin(), down.end()-1);
  // printVector(up);

  Eigen::Vector4f left_most = sorted_points.col(0);
  Eigen::Vector4f right_most = sorted_points.col(n - 1);

  Eigen::MatrixXf up = Eigen::MatrixXf::Zero(4, n);
  Eigen::MatrixXf down = Eigen::MatrixXf::Zero(4, n);
  up.col(0) = left_most;
  // std::cout << "left p: \n" << left_most << std::endl;
  // std::cout << "right p: \n" <<  right_most << std::endl;
  // std::cout << "1. up: \n" << up << std::endl;
  int k = 0;
  int j = 0;

  for (int i = 0; i < sorted_points.cols(); i++) {
    Eigen::Vector4f cur_pt = sorted_points.col(i);
    if (i == sorted_points.cols() - 1 || cross(left_most, cur_pt, right_most) > 0) {
      // std::cout << "i: " << i << std::endl;
      while (k >= 2 && cross(up.col(k - 2), up.col(k - 1), cur_pt) <= 0) k--;
      up.col(k++) = cur_pt;
      // std::cout << "k: " << k << std::endl;
      // std::cout << "2. up: \n" << up << std::endl;
    }

    if (i == sorted_points.cols() - 1 || cross(left_most, cur_pt, right_most) <= 0) {
      while (j >= 2 && cross(down.col(j - 2), down.col(j - 1), cur_pt) >= 0) j--;
      down.col(j++) = cur_pt;
    }
  }
  convex_hull.resize(up.rows(), k + j - 1);
  convex_hull << up.leftCols(k), down.leftCols(j - 1).rowwise().reverse();
  // std::cout << "sorted P: \n" << sorted_points << std::endl;
  // std::cout << "up: \n" << up << std::endl;
  // std::cout << "down: \n" << down << std::endl;
  // std::cout << "down reversed: \n" << down.leftCols(j -
  // 1).rowwise().reverse() << std::endl; std::cout << "ch: \n" << convex_hull
  // << std::endl; std::cout << "area: " << computePolygonArea(convex_hull) <<
  // std::endl; std::cout << "bool \n: " << up.cast<bool>() << std::endl;
  // std::cout << "bool.col \n: " << up.cast<bool>().colwise() << std::endl;
  // std::cout << "bool.col.all \n: " << up.cast<bool>().colwise().all() <<
  // std::endl; std::cout << "up: \n" << up << std::endl; std::cout << "bool:
  // \n" << up.cast<bool>() << std::endl; std::cout << "test: \n" <<
  // up.cast<bool>().colwise().any() << std::endl; Eigen::MatrixXd res = up("",
  // up.cast<bool>().colwise().any()); std::cout << "res: \n" << res <<
  // std::endl;

  // Eigen::MatrixXd convex_hull = Eigen::MatrixXd::Zero(4, 2 * n);
  // // Build lower hull
  // for (size_t i = 0; i < n; ++i) {
  // 	while (k >= 2 &&
  //            cross(convex_hull.col(k - 2),
  //                  convex_hull.col(k - 1),
  //                  sorted_points.col(i)) <= 0) {
  //         k--;
  //     }
  // 	convex_hull.col(k++) = P.col(i);
  // }

  // // Build upper hull
  // for (size_t i = n - 1, t = k + 1; i > 0; --i) {
  // 	while (k >= t &&
  //            cross(convex_hull.col(k - 2),
  //                  convex_hull.col(k - 1),
  //                  sorted_points.col(i - 1)) <= 0) {
  //         k--;
  //     }
  // 	convex_hull.col(k++) = P.col(i - 1);
  // }
  // std::cout << "convex_hull: \n" << convex_hull.leftCols(k - 1) << std::endl;

  // return convex_hull.leftCols(k - 1);
}

void eigen2Corners(const Eigen::MatrixXf & vector, point & tag_point)
{
  tag_point.x = vector(0, 0);
  tag_point.y = vector(1, 0);
  tag_point.z = vector(2, 0);
}

// float computeMedian(std::vector<float> &vec){
//     assert(vec.size()!=0);
//     if (vec.size() % 2 == 0) {
//         const auto median_it1 = vec.begin() + vec.size() / 2 - 1;
//         const auto median_it2 = vec.begin() + vec.size() / 2;

//         std::nth_element(vec.begin(), median_it1 , vec.end());
//         const auto e1 = *median_it1;

//         std::nth_element(vec.begin(), median_it2 , vec.end());
//         const auto e2 = *median_it2;

//         return (e1 + e2) / 2;

//     } else {
//         const auto median_it = vec.begin() + vec.size() / 2;
//         std::nth_element(vec.begin(), median_it , vec.end());

//         return *median_it;
//     }
// }

// template <typename T>
// T computeMedian(std::vector<T> &vec){
//     assert(vec.size()!=0);
//     if (vec.size() % 2 == 0) {
//         const auto median_it1 = vec.begin() + vec.size() / 2 - 1;
//         const auto median_it2 = vec.begin() + vec.size() / 2;

//         std::nth_element(vec.begin(), median_it1 , vec.end());
//         const T e1 = *median_it1;

//         std::nth_element(vec.begin(), median_it2 , vec.end());
//         const T e2 = *median_it2;

//         return (e1 + e2) / 2;

//     } else {
//         const auto median_it = vec.begin() + vec.size() / 2;
//         std::nth_element(vec.begin(), median_it , vec.end());

//         return *median_it;
//     }
// }

}  // namespace utils
}  // namespace BipedLab
