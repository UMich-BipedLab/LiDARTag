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

#ifndef LIDARTAG__DECODING__NAIVE_HAMMING_DECODING__HPP_
#define LIDARTAG__DECODING__NAIVE_HAMMING_DECODING__HPP_

#include <Eigen/Core>

#include <array>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

class NaiveHammingDecoding
{
public:
  NaiveHammingDecoding(
    const std::string & tag_family, const std::string & templates_path,
    double min_white_border_bits, double min_black_boder_bits, double min_payload_bits,
    double min_payload_margin, double intensity_threshold, double rbf_sigma,
    double decoding_bit_threshold);

  bool decode(
    const Eigen::MatrixXd & payload, double tag_size, double white_median, double black_median,
    int & decoded_id, int & decoded_orientation);

protected:
  void loadTemplates();
  Eigen::ArrayXXd computeScoreMatrix(
    const Eigen::MatrixXd & payload, double tag_size, double white_median, double black_median);
  void computeMatchScores(
    const Eigen::ArrayXXd & scores, std::vector<std::tuple<double, int, int>> & templates_bits,
    double & white_frame_bits, double & black_frame_bits);

  std::unordered_map<int, std::array<Eigen::ArrayXXd, 4>> tag_templates;
  Eigen::ArrayXXd white_frame_mask;
  Eigen::ArrayXXd black_frame_mask;
  Eigen::ArrayXXd payload_mask;

  std::string tag_family_;
  std::string templates_path_;
  bool initialized_;
  double min_white_border_bits_;
  double min_black_boder_bits_;
  double min_payload_bits_;
  double min_payload_margin_;
  double intensity_threshold_;
  double rbf_sigma_;
  double decoding_bit_threshold_;
};

#endif  // LIDARTAG__DECODING__NAIVE_HAMMING_DECODING__HPP_