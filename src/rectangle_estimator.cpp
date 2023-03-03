// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <lidartag/rectangle_estimator.hpp>
#include <algorithm>
#include <iostream>
#include <string>
#include <random>

RectangleEstimator::RectangleEstimator()
{
  use_ransac_ = false;
  filter_by_coefficients_ = true;
  max_ransac_iterations_ = 20;
  max_error_ = 0.05;
  fix_point_groups_ = false;

  augmented_matrix_.resize(111, 6);
}

void RectangleEstimator::setInputPoints(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & points1,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & points2,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & points3,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & points4)
{
  points1_ = points1;
  points2_ = points2;
  points3_ = points3;
  points4_ = points4;

  points.clear();
  points += *points1;
  points += *points2;
  points += *points3;
  points += *points4;
}

bool RectangleEstimator::estimate()
{
  // This implementation assumes  ordered points to detect probably wrong estimations
  //        *
  //   4  /   \ 1
  //     /     \
  //    *       *
  //     \     /
  //   3  \   / 2
  //        *

  bool status;
  preparePointsMatrix();

  if (use_ransac_) {
    status = estimate_ransac();
  }
  else {
    status = estimate_imlp(points1_, points2_, points3_, points4_, estimated_n, estimated_c);
  }

  bool status_tmp = status;

  if (status && filter_by_coefficients_) {
    status = checkCoefficients();
  }

  if(status != status_tmp) {
    bool asd = 0;
  }

  return status;
}

bool RectangleEstimator::estimate_ransac()
{
  int ransac_min_points = 2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr points1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points4(new pcl::PointCloud<pcl::PointXYZ>);

  points1->points.resize(ransac_min_points);
  points2->points.resize(ransac_min_points);
  points3->points.resize(ransac_min_points);
  points4->points.resize(ransac_min_points);

  std::random_device dev;
  std::mt19937 rng(dev());

  auto sample_points = [&](auto & in_points, auto & out_points)
  {
    out_points->clear();
    std::vector<int> index_vector;
    for (int i=0; i < in_points->points.size(); ++i) {
      index_vector.push_back(i);
    }

    std::random_shuffle ( index_vector.begin(), index_vector.end() );

    for(int i = 0; i < ransac_min_points; i++) {
      int input_index = index_vector[i];
      out_points->push_back(in_points->points[input_index]);
    }
  };

  double min_error = 1e12;
  int max_inliers = 0;
  bool status = false;

  Eigen::VectorXd errors1, errors2, errors3, errors4;
  std::vector<Eigen::Vector2d> inlier_vector;

  for (int i = 0; i < max_ransac_iterations_; i++) {

    sample_points(points1_, points1);
    sample_points(points2_, points2);
    sample_points(points3_, points3);
    sample_points(points4_, points4);

    Eigen::Vector4d h_c;
    Eigen::Vector2d h_n;

    bool h_status = estimate_imlp(points1, points2, points3, points4, h_n, h_c);
    int h_inliers = 0;

    double error = getModelErrorAndInliers(h_n, h_c, h_inliers,
      inlier_vector, inlier_vector, inlier_vector, inlier_vector,
      errors1, errors2, errors3, errors4, false);

    if (h_status && (h_inliers > max_inliers || h_inliers == max_inliers && error < min_error)) {
      min_error = error;
      max_inliers = h_inliers;
      estimated_n = h_n;
      estimated_c = h_c;
      status = h_status;

      if (fix_point_groups_) {
        fixPointGroups(errors1, errors2, errors3, errors4);
      }

    }
  }

  if (!status) {
    return false;
  }

  int inliers = 0;
  std::vector<Eigen::Vector2d> inliers1, inliers2, inliers3, inliers4;

  double error = getModelErrorAndInliers(estimated_n, estimated_c, inliers,
    inliers1, inliers2, inliers3, inliers4,
    errors1, errors2, errors3, errors4, true);

  auto eigen_to_pointcloud = [&](std::vector<Eigen::Vector2d> & inliers_vector,
    auto & points)
  {
    points->resize(inliers_vector.size());

    for (int i = 0; i < inliers_vector.size(); i++) {
      points->points[i].x = inliers_vector[i].x();
      points->points[i].y = inliers_vector[i].y();
    }
  };

  eigen_to_pointcloud(inliers1, points1);
  eigen_to_pointcloud(inliers2, points2);
  eigen_to_pointcloud(inliers3, points3);
  eigen_to_pointcloud(inliers4, points4);


  status = estimate_imlp(points1, points2, points3, points4, estimated_n, estimated_c);


  return status;
}

bool RectangleEstimator::estimate_imlp(
  pcl::PointCloud<pcl::PointXYZ>::Ptr points1,
  pcl::PointCloud<pcl::PointXYZ>::Ptr points2,
  pcl::PointCloud<pcl::PointXYZ>::Ptr points3,
  pcl::PointCloud<pcl::PointXYZ>::Ptr points4,
  Eigen::Vector2d & n_out,
  Eigen::Vector4d & c_out
  )
{
  int num_points_1 = points1->points.size();
  int num_points_2 = points2->points.size();
  int num_points_3 = points3->points.size();
  int num_points_4 = points4->points.size();

  int num_points = num_points_1 + num_points_2 + num_points_3 + num_points_4;

  if (num_points_1 < 2 || num_points_2 < 2 || num_points_3 < 2
    || num_points_4 < 2 || num_points < 6 )
  {
    return false;
  }

  augmented_matrix_.resize(num_points, 6);
  augmented_matrix_.setZero();

  int i = 0;

  auto lambda = [&](auto & points, int column)
  {
    for (auto & p : points) {
      this->augmented_matrix_(i, column) = 1.0;
      this->augmented_matrix_(i, 4) = column % 2 == 0 ? p.x : p.y;
      this->augmented_matrix_(i, 5) = column % 2 == 0 ? p.y : -p.x;

      i += 1;
    }
  };

  lambda(points1->points, 0);
  lambda(points2->points, 1);
  lambda(points3->points, 2);
  lambda(points4->points, 3);

  Eigen::HouseholderQR<Eigen::MatrixXd> qr(augmented_matrix_);
  Eigen::MatrixXd temp = qr.matrixQR().triangularView<Eigen::Upper>();
  Eigen::MatrixXd r_matrix = temp.topRows(augmented_matrix_.cols());
  Eigen::MatrixXd r_matrix_sub = r_matrix.block(4, 4, 2, 2);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(r_matrix_sub, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::MatrixXd v = svd.matrixV();
  Eigen::MatrixXd n = v.col(1);

  Eigen::MatrixXd aux1 = r_matrix.block(0, 0, 4, 4);
  Eigen::MatrixXd aux2 = r_matrix.block(0, 4, 4, 2);

  Eigen::MatrixXd c = -(aux1.inverse()) * (aux2) * n;

  Eigen::MatrixXd error1 = aux1 * c + aux2 * n;

  Eigen::VectorXd x;
  x.resize(6);
  x.block(0,0,4,1) = c;
  x.block(4,0,2,1) = n;

  Eigen::VectorXd error2 = augmented_matrix_ * x;

  c_out = c;
  n_out = n;

  return true;
}

std::vector<Eigen::Vector2d> RectangleEstimator::getCorners()
{
  return getCorners(estimated_n, estimated_c);
}

std::vector<Eigen::Vector2d> RectangleEstimator::getCorners(Eigen::Vector2d & n, Eigen::Vector4d & c)
{
  std::vector<Eigen::Vector2d> corners;

  for (int i = 0; i < 4; i++) {
    double c1 = c(i);
    double c2 = c((i + 1) % 4);

    double nx1 = i % 2 == 0 ? n(0) : -n(1);
    double ny1 = i % 2 == 0 ? n(1) : n(0);
    double nx2 = (i +1) % 2 == 0 ? n(0) : -n(1);
    double ny2 = (i +1) % 2 == 0 ? n(1) : n(0);

    double y = (nx2*c1 - nx1*c2) / (nx1*ny2 - nx2*ny1);
    double x = (-c1 - ny1*y) / nx1;

    corners.push_back(Eigen::Vector2d(x, y));
  }

  return corners;
}

Eigen::Vector4d RectangleEstimator::getCCoefficients() const
{
  return estimated_c;
}

Eigen::Vector2d RectangleEstimator::getNCoefficients() const
{
  return estimated_n;
}

void RectangleEstimator::setRANSAC(bool enable)
{
  use_ransac_ = enable;
}

void RectangleEstimator::setFilterByCoefficients(bool enable)
{
  filter_by_coefficients_ = enable;
}

void RectangleEstimator::setMaxIterations(int iterations)
{
  max_ransac_iterations_ = iterations;
}

void RectangleEstimator::setInlierError(double error)
{
  max_error_ = error;
}

void RectangleEstimator::setFixPointGroups(bool enable)
{
  fix_point_groups_ = enable;
}

bool RectangleEstimator::checkCoefficients() const
{
  Eigen::Vector2d reference(1.0, 1.0);
  reference.normalize();
  double max_cos_distance = std::cos(45.0 * M_PI / 180.0);

  return estimated_n.dot(reference) >= max_cos_distance;
}

void RectangleEstimator::preparePointsMatrix()
{
  int num_points = points1_->size() + points2_->size()
    + points3_->size()+ points4_->size();
  points_matrix_.resize(num_points, 2);

  int index = 0;

  auto lambda = [&](auto & points, int column)
  {
    for (auto & p : points) {
      this->points_matrix_(index, 0) = p.x;
      this->points_matrix_(index, 1) = p.y;
      index += 1;
    }
  };

  lambda(points1_->points, 0);
  lambda(points2_->points, 1);
  lambda(points3_->points, 2);
  lambda(points4_->points, 3);
}

double RectangleEstimator::getModelErrorAndInliers(
  Eigen::Vector2d & n, Eigen::Vector4d & c, int & inliers,
  std::vector<Eigen::Vector2d> & inliers1, std::vector<Eigen::Vector2d> & inliers2,
  std::vector<Eigen::Vector2d> & inliers3, std::vector<Eigen::Vector2d> & inliers4,
  Eigen::VectorXd & errors1, Eigen::VectorXd & errors2,
  Eigen::VectorXd & errors3, Eigen::VectorXd & errors4,
  bool add_inliers)
{
  Eigen::Vector2d n1 = n;
  Eigen::Vector2d n2(-n1.y(), n1.x());

  Eigen::Vector2d d0_paralell(-n1.y(), n1.x());
  Eigen::Vector2d d0_perpendicular(n1.x(), n1.y());

  Eigen::Vector2d d1_paralell(-n2.y(), n2.x());
  Eigen::Vector2d d1_perpendicular(n2.x(), n2.y());

  Eigen::Vector2d p0(0, -c(0) / n1.y());
  Eigen::Vector2d p1(0, -c(1) / n2.y());
  Eigen::Vector2d p2(0, -c(2) / n1.y());
  Eigen::Vector2d p3(0, -c(3) / n2.y());

  std::vector<Eigen::Vector2d> corners = getCorners(n, c);
  double t0_1 = (corners[0] - p0).dot(d0_paralell);
  double t0_2 = (corners[3] - p0).dot(d0_paralell);
  double t0_min = std::min(t0_1, t0_2);
  double t0_max = std::max(t0_1, t0_2);
  double t1_1 = (corners[1] - p1).dot(d1_paralell);
  double t1_2 = (corners[0] - p1).dot(d1_paralell);
  double t1_min = std::min(t1_1, t1_2);
  double t1_max = std::max(t1_1, t1_2);
  double t2_1 = (corners[2] - p2).dot(d0_paralell);
  double t2_2 = (corners[1] - p2).dot(d0_paralell);
  double t2_min = std::min(t2_1, t2_2);
  double t2_max = std::max(t2_1, t2_2);
  double t3_1 = (corners[3] - p3).dot(d1_paralell);
  double t3_2 = (corners[2] - p3).dot(d1_paralell);
  double t3_min = std::min(t3_1, t3_2);
  double t3_max = std::max(t3_1, t3_2);

  errors1.resize(points_matrix_.rows());
  errors2.resize(points_matrix_.rows());
  errors3.resize(points_matrix_.rows());
  errors4.resize(points_matrix_.rows());

  auto calculate_line_error = [&](
    Eigen::Vector2d & p0,
    Eigen::Vector2d & d_paralell,
    Eigen::Vector2d & d1_perpendicular,
    double t_min, double t_max,
    Eigen::VectorXd & error,
    std::vector<Eigen::Vector2d> & inliers_vector)
  {
    for(int i = 0; i < points_matrix_.rows(); i++)
    {
      Eigen::Vector2d q = points_matrix_.row(i);

      double t = (q - p0).dot(d_paralell);
      double lambda = std::abs((q - p0).dot(d1_perpendicular));
      double t_error = t <= t_min ? t_min - t : t > t_max ? t - t_max : 0.0;
      double e = std::max(t_error, lambda);
      error(i) = e;

      if(add_inliers && e <= max_error_) {
        inliers_vector.push_back(q);
      }
    }
  };

  calculate_line_error(p0, d0_paralell, d0_perpendicular, t0_min, t0_max, errors1, inliers1);
  calculate_line_error(p1, d1_paralell, d1_perpendicular, t1_min, t1_max, errors2, inliers2);
  calculate_line_error(p2, d0_paralell, d0_perpendicular, t2_min, t2_max, errors3, inliers3);
  calculate_line_error(p3, d1_paralell, d1_perpendicular, t3_min, t3_max, errors4, inliers4);

  Eigen::VectorXd error = errors1.cwiseMin(errors2).cwiseMin(errors3).cwiseMin(errors4);

  inliers = 0;

  for(int i = 0; i < points_matrix_.rows(); i++){
    if (error(i) < max_error_) {
      inliers++;
    }
  }

  return error.mean();
}

void RectangleEstimator::fixPointGroups(
  Eigen::VectorXd & errors1, Eigen::VectorXd & errors2,
  Eigen::VectorXd & errors3, Eigen::VectorXd & errors4)
{
  assert(points.size() == errors1.rows());

  Eigen::VectorXd min_errors = errors1.cwiseMin(errors2).cwiseMin(errors3).cwiseMin(errors4);

  int num_points1 = 0;
  int num_points2 = 0;
  int num_points3 = 0;
  int num_points4 = 0;

  for(int i = 0; i < points.size(); i++){
    if (errors1(i) == min_errors(i)) {
      num_points1++;
    }
    else if (errors2(i) == min_errors(i)) {
      num_points2++;
    }
    else if (errors3(i) == min_errors(i)) {
      num_points3++;
    }
    else if (errors4(i) == min_errors(i)) {
      num_points4++;
    }
    else {
      assert(false);
    }
  }

  if (num_points1 < 3 || num_points2 < 3 || num_points3 < 3 || num_points4 < 3) {
    return;
  }

  points1_->clear();
  points2_->clear();
  points3_->clear();
  points4_->clear();

  for(int i = 0; i < points.size(); i++){
    if (errors1(i) == min_errors(i)) {
      points1_->push_back(points[i]);
    }
    else if (errors2(i) == min_errors(i)) {
      points2_->push_back(points[i]);
    }
    else if (errors3(i) == min_errors(i)) {
      points3_->push_back(points[i]);
    }
    else if (errors4(i) == min_errors(i)) {
      points4_->push_back(points[i]);
    }
    else {
      assert(false);
    }
  }

  assert(points1_->size() >= 2);
  assert(points2_->size() >= 2);
  assert(points3_->size() >= 2);
  assert(points4_->size() >= 2);
}
