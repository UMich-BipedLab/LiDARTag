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


#include <gtest/gtest.h>
#include <lidartag/rectangle_estimator.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <Eigen/Dense>
#include <vector>

constexpr double epsilon = 1e-6;

TEST(lidartag, rectangle_estimation)
{
  RectangleEstimator estimator;

  pcl::PointCloud<pcl::PointXYZ>::Ptr points1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points4(new pcl::PointCloud<pcl::PointXYZ>);

  int num_points_per_side = 6;

  points1->points.reserve(num_points_per_side);
  points2->points.reserve(num_points_per_side);
  points3->points.reserve(num_points_per_side);
  points4->points.reserve(num_points_per_side);

  double min_t = 0.1;
  double max_t = 0.9;

  for(int i = 0; i < num_points_per_side; i++) {

        double t = min_t + (max_t - min_t) * (1.0 * i) / num_points_per_side;

        double tx = t;
        double ty = 1.0 - t;

        points1->points.push_back(pcl::PointXYZ( tx, ty, 0.0));
        points2->points.push_back(pcl::PointXYZ( tx, -ty, 0.0));
        points3->points.push_back(pcl::PointXYZ( -tx, -ty, 0.0));
        points4->points.push_back(pcl::PointXYZ( -tx ,ty, 0.0));
  }

  estimator.setInputPoints(points1, points2, points3, points4);

  estimator.estimate();

  Eigen::Vector4d c = estimator.getCCoefficients();
  Eigen::Vector2d n = estimator.getNCoefficients();

  int inliers = 0;
  std::vector<Eigen::Vector2d> inliers_vector;
  double error = estimator.getModelErrorAndInliers(n, c, inliers,
    inliers_vector, inliers_vector, inliers_vector, inliers_vector, false);

  ASSERT_NEAR(0, error, 1e-3);
  ASSERT_EQ(4*num_points_per_side, inliers);

  double x = 0.5 * std::sqrt(2);

  ASSERT_NEAR(x, n(0), 1e-3);
  ASSERT_NEAR(x, n(1), 1e-3);

  ASSERT_NEAR(-x, c(0), 1e-3);
  ASSERT_NEAR(x, c(1), 1e-3);
  ASSERT_NEAR(x, c(2), 1e-3);
  ASSERT_NEAR(-x, c(3), 1e-3);

  std::vector<Eigen::Vector2d> corners = estimator.getCorners();
  ASSERT_EQ(4, corners.size());

  ASSERT_NEAR(1.0, corners[0].x(), 1e-3);
  ASSERT_NEAR(0.0, corners[0].y(), 1e-3);

  ASSERT_NEAR(0.0, corners[1].x(), 1e-3);
  ASSERT_NEAR(-1.0, corners[1].y(), 1e-3);

  ASSERT_NEAR(-1.0, corners[2].x(), 1e-3);
  ASSERT_NEAR(0.0, corners[2].y(), 1e-3);

  ASSERT_NEAR(0.0, corners[3].x(), 1e-3);
  ASSERT_NEAR(1.0, corners[3].y(), 1e-3);
}

TEST(lidartag, rectangle_ransac_estimation)
{
  RectangleEstimator estimator;

  pcl::PointCloud<pcl::PointXYZ>::Ptr points1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points4(new pcl::PointCloud<pcl::PointXYZ>);

  int num_points_per_side = 6;

  points1->points.reserve(num_points_per_side);
  points2->points.reserve(num_points_per_side);
  points3->points.reserve(num_points_per_side);
  points4->points.reserve(num_points_per_side);

  double min_t = 0.1;
  double max_t = 0.9;

  for(int i = 0; i < num_points_per_side; i++) {

        double t = min_t + (max_t - min_t) * (1.0 * i) / num_points_per_side;

        double tx = t;
        double ty = 1.0 - t;

        points1->points.push_back(pcl::PointXYZ( tx, ty, 0.0));
        points2->points.push_back(pcl::PointXYZ( tx, -ty, 0.0));
        points3->points.push_back(pcl::PointXYZ( -tx, -ty, 0.0));
        points4->points.push_back(pcl::PointXYZ( -tx ,ty, 0.0));
  }

  points1->points.push_back(pcl::PointXYZ(10, 10, 0.0));
  points2->points.push_back(pcl::PointXYZ(-10, 10, 0.0));
  points3->points.push_back(pcl::PointXYZ(10, -10, 0.0));
  points4->points.push_back(pcl::PointXYZ(-10, -10, 0.0));

  ASSERT_EQ(7, points1->size());
  ASSERT_EQ(7, points2->size());
  ASSERT_EQ(7, points3->size());
  ASSERT_EQ(7, points4->size());

  estimator.setInlierError(0.05);
  estimator.setMaxIterations(100);
  estimator.setRANSAC(true);

  estimator.setInputPoints(points1, points2, points3, points4);

  estimator.estimate();

  Eigen::Vector4d c = estimator.getCCoefficients();
  Eigen::Vector2d n = estimator.getNCoefficients();

  double x = 0.5 * std::sqrt(2);

  Eigen::Vector2d n_test(x,x);
  Eigen::Vector4d c_test(-x, x, x, -x);

  int inliers = 0;
  std::vector<Eigen::Vector2d> inliers_vector;
  double error = estimator.getModelErrorAndInliers(n_test, c_test, inliers,
    inliers_vector, inliers_vector, inliers_vector, inliers_vector, false);

  ASSERT_NEAR(x, n(0), 1e-3);
  ASSERT_NEAR(x, n(1), 1e-3);

  ASSERT_NEAR(-x, c(0), 1e-3);
  ASSERT_NEAR(x, c(1), 1e-3);
  ASSERT_NEAR(x, c(2), 1e-3);
  ASSERT_NEAR(-x, c(3), 1e-3);

  std::vector<Eigen::Vector2d> corners = estimator.getCorners();
  ASSERT_EQ(4, corners.size());

  ASSERT_NEAR(1.0, corners[0].x(), 1e-3);
  ASSERT_NEAR(0.0, corners[0].y(), 1e-3);

  ASSERT_NEAR(0.0, corners[1].x(), 1e-3);
  ASSERT_NEAR(-1.0, corners[1].y(), 1e-3);

  ASSERT_NEAR(-1.0, corners[2].x(), 1e-3);
  ASSERT_NEAR(0.0, corners[2].y(), 1e-3);

  ASSERT_NEAR(0.0, corners[3].x(), 1e-3);
  ASSERT_NEAR(1.0, corners[3].y(), 1e-3);
}
