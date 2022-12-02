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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <thread>

#include <lidartag/decoding/naive_hamming_decoding.hpp>
#include <pcl/io/pcd_io.h>


TEST(hamming_decoding, decoding)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

  double min_white_border_bits = 5.0;
  double min_black_boder_bits = 5.0;
  double min_payload_bits = 10.0;
  double min_payload_margin = 2.0;
  double intensity_threshold = 0.5;
  double rbf_sigma = 0.25;
  double decoding_bit_threshold = 0.6;

  double tag_size = 0.8;
  int decoded_id = -1;
  int decoded_orientation = -1;

  // data
  int expected_id = 0;
  int expected_orientation = 2;
  std::string template_path = "templates";
  std::string pointcloud_path = "pointcloud.pcd";

  auto decoder = std::make_shared<NaiveHammingDecoding>("16",
    template_path, min_white_border_bits, min_black_boder_bits,
    min_payload_bits, min_payload_margin,
    intensity_threshold, rbf_sigma,
    decoding_bit_threshold);

  pcl::io::loadPCDFile(pointcloud_path, *input_pointcloud);

  Eigen::MatrixXd input_matrix(3, input_pointcloud->size());

  std::vector<float> pos_intensities;
  std::vector<float> neg_intensities;

  for (int i = 0; i < input_pointcloud->size(); ++i) {
    input_matrix(0, i) = input_pointcloud->points[i].x;
    input_matrix(1, i) = input_pointcloud->points[i].y;
    input_matrix(2, i) = input_pointcloud->points[i].z;

    if (input_pointcloud->points[i].x > 0.f)
    {
      pos_intensities.push_back(input_pointcloud->points[i].x);
    }
    else {
      neg_intensities.push_back(input_pointcloud->points[i].x);
    }
  }

  std::sort(pos_intensities.begin(), pos_intensities.end());
  std::sort(neg_intensities.begin(), neg_intensities.end());

  double pos_median = pos_intensities[pos_intensities.size() / 2];
  double neg_median = neg_intensities[neg_intensities.size() / 2];

  decoder->decode(input_matrix, tag_size, pos_median, neg_median, decoded_id, decoded_orientation);

  EXPECT_EQ(expected_id, decoded_id);
  EXPECT_EQ(expected_orientation, decoded_orientation);

}
