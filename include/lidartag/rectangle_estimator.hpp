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

#ifndef RECTANGLE_ESTIMATOR_HPP
#define RECTANGLE_ESTIMATOR_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

class RectangleEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RectangleEstimator();

  void setInputPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr & points1,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & points2,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & points3,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & points4);

  bool estimate();
  bool estimate_ransac();
  bool estimate_imlp(
    pcl::PointCloud<pcl::PointXYZ>::Ptr points1,
    pcl::PointCloud<pcl::PointXYZ>::Ptr points2,
    pcl::PointCloud<pcl::PointXYZ>::Ptr points3,
    pcl::PointCloud<pcl::PointXYZ>::Ptr points4,
    Eigen::Vector2d & n_out,
    Eigen::Vector4d & c_out);

  std::vector<Eigen::Vector2d> getCorners();
  std::vector<Eigen::Vector2d> getCorners(Eigen::Vector2d & n, Eigen::Vector4d & c);

  Eigen::Vector2d getNCoefficients() const;
  Eigen::Vector4d getCCoefficients() const;

  void setRANSAC(bool enable);
  void setFilterByCoefficients(bool enable);
  void setMaxIterations(int iterations);
  void setInlierError(double error);
  void setFixPointGroups(bool enabe);

  void preparePointsMatrix();
  double getModelErrorAndInliers(Eigen::Vector2d & n, Eigen::Vector4d & c, int & inliers,
    std::vector<Eigen::Vector2d> & inliers1, std::vector<Eigen::Vector2d> & inliers2,
    std::vector<Eigen::Vector2d> & inliers3, std::vector<Eigen::Vector2d> & inliers4,
    Eigen::VectorXd & errors1, Eigen::VectorXd & errors2,
    Eigen::VectorXd & errors3, Eigen::VectorXd & errors4,
    bool add_inliers);

protected:

  bool checkCoefficients() const;
  void fixPointGroups(Eigen::VectorXd & errors1, Eigen::VectorXd & errors2,
    Eigen::VectorXd & errors3, Eigen::VectorXd & errors4);

  bool use_ransac_;
  bool filter_by_coefficients_;
  int max_ransac_iterations_;
  double max_error_;
  bool fix_point_groups_;

  Eigen::MatrixXd augmented_matrix_;
  Eigen::MatrixXd points_matrix_;
  Eigen::Vector4d estimated_c;
  Eigen::Vector2d estimated_n;

  pcl::PointCloud<pcl::PointXYZ>::Ptr points1_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points2_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points3_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points4_;

  pcl::PointCloud<pcl::PointXYZ> points;

};

#endif // RECTANGLE_ESTIMATOR_HPP
