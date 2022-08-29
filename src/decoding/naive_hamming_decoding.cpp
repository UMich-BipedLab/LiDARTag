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

#include <Eigen/Core>
#include <lidartag/decoding/naive_hamming_decoding.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <iostream>

NaiveHammingDecoding::NaiveHammingDecoding(
  const std::string & tag_family, const std::string & templates_path, double min_white_border_bits,
  double min_black_boder_bits, double min_payload_bits, double min_payload_margin,
  double intensity_threshold, double rbf_sigma, double decoding_bit_threshold)
: tag_family_(tag_family),
  templates_path_(templates_path),
  initialized_(false),
  min_white_border_bits_(min_white_border_bits),
  min_black_boder_bits_(min_black_boder_bits),
  min_payload_bits_(min_payload_bits),
  min_payload_margin_(min_payload_margin),
  intensity_threshold_(intensity_threshold),
  rbf_sigma_(rbf_sigma),
  decoding_bit_threshold_(decoding_bit_threshold)
{
  white_frame_mask.resize(8, 8);
  black_frame_mask.resize(8, 8);
  payload_mask.resize(8, 8);

  white_frame_mask.setConstant(1.0);
  black_frame_mask.setConstant(0.0);
  payload_mask.setConstant(0.0);

  white_frame_mask.block(1, 1, 6, 6).setConstant(0.0);
  black_frame_mask.block(1, 1, 6, 6).setConstant(1.0);
  black_frame_mask.block(2, 2, 4, 4).setConstant(0.0);
  payload_mask.block(2, 2, 4, 4).setConstant(1.0);
}

bool NaiveHammingDecoding::decode(
  const Eigen::MatrixXd & payload, double tag_size, double white_median, double black_median,
  int & decoded_id, int & decoded_orientation)
{
  if (!initialized_) {
    loadTemplates();
  }

  decoded_id = -1;
  decoded_orientation = -1;

  // Compute
  Eigen::ArrayXXd scores = computeScoreMatrix(payload, tag_size, white_median, black_median);

  std::vector<std::tuple<double, int, int>> templates_decoded_bits;
  double white_frame_bits;
  double black_frame_bits;

  computeMatchScores(scores, templates_decoded_bits, white_frame_bits, black_frame_bits);

  std::sort(
    templates_decoded_bits.begin(), templates_decoded_bits.end(),
    [](const std::tuple<double, int, int> & lhs, const std::tuple<double, int, int> & rhs) -> bool {
      return std::get<0>(lhs) > std::get<0>(rhs);
    });

  double best_score = std::get<0>(templates_decoded_bits[0]);
  double best_margin =
    std::get<0>(templates_decoded_bits[0]) - std::get<0>(templates_decoded_bits[1]);
  int best_id = std::get<1>(templates_decoded_bits[0]);
  int best_orientation = std::get<2>(templates_decoded_bits[0]);

  if (
    white_frame_bits < min_white_border_bits_ || black_frame_bits < min_black_boder_bits_ ||
    best_score < min_payload_bits_ || best_margin < min_payload_margin_) {
    return false;
  }

  decoded_id = best_id;
  decoded_orientation = best_orientation;

  return true;
}

void NaiveHammingDecoding::loadTemplates()
{
  for (int id = 0; id < 30; id++) {
    cv::Mat template_img =
      cv::imread(templates_path_ + "/" + std::to_string(id) + ".png", cv::IMREAD_GRAYSCALE);

    if (template_img.empty()) {
      continue;
    }

    // make into eigen. check dimensions
    Eigen::MatrixXd template_matrix;
    cv::cv2eigen(template_img, template_matrix);

    if (template_matrix.cols() != template_matrix.rows()) {
      continue;
    }

    Eigen::MatrixXd template_array = 2 * (template_matrix.array() > 127).cast<double>() - 1.0;

    tag_templates[id][0] = template_array;

    for (int orientation = 1; orientation < 4; orientation++) {

      Eigen::MatrixXd aux = template_array;

      for(int j = 0; j < aux.cols(); j++)
      {
        for(int i = 0; i < aux.cols(); i++)
        {
          aux(j, i) = template_array(7 - i, j);
        }
      }

      template_array = aux;
      tag_templates[id][4 - orientation] = template_array;
    }
  }

  if (tag_templates.size() > 0) {
    initialized_ = true;
  }
}

Eigen::ArrayXXd NaiveHammingDecoding::computeScoreMatrix(
  const Eigen::MatrixXd & payload, double tag_size, double white_median, double black_median)
{
  Eigen::ArrayXXd scores = Eigen::ArrayXXd::Zero(8, 8);
  Eigen::ArrayXXd scores_aux = Eigen::ArrayXXd::Zero(8, 8);
  scores_aux.setConstant(1e-5);

  double median = std::min(black_median, white_median);
  double cell_size = tag_size / 8.0;

  for (int col = 0; col < payload.cols(); col++) {
    Eigen::VectorXd p = payload.col(col);
    double x = p(1) / cell_size;
    double y = -p(2) / cell_size;
    double intensity = p(0);

    if ((std::abs(x) >= 4) || (std::abs(y) >= 4)) {
      continue;
    }

    int i = std::floor(x + 4);
    int j = std::floor(y + 4);

    assert(i >= 0 && i < 8);
    assert(j >= 0 && j < 8);

    double center_x = i + 0.5 - 4;
    double center_y = j + 0.5 - 4;

    double d = std::max(std::abs(center_x - x), std::abs(center_y - y));

    intensity = intensity * (std::abs(intensity) > intensity_threshold_ * median);

    double v = intensity * std::exp(-d / rbf_sigma_);
    scores(j, i) += v;
    scores_aux(j, i) += std::abs(v);
  }

  scores /= scores_aux;

  return scores;
}

void NaiveHammingDecoding::computeMatchScores(
  const Eigen::ArrayXXd & scores, std::vector<std::tuple<double, int, int>> & decoding_output,
  double & white_frame_bits, double & black_frame_bits)
{
  Eigen::ArrayXXd decoded_mask = (scores.abs() > decoding_bit_threshold_).cast<double>();
  const Eigen::ArrayXXd & decoded = scores;

  Eigen::ArrayXXd decoded_white_frame_array = (decoded * decoded_mask * white_frame_mask);
  Eigen::ArrayXXd decoded_black_frame_array = -(decoded * decoded_mask * black_frame_mask);

  auto extract_bits_from_array = [](const Eigen::ArrayXXd & array) -> double {
    double bits = 0.0;
    for (int j = 0; j < array.rows(); j++) {
      for (int i = 0; i < array.cols(); i++) {
        bits += array(j, i) > 0.0 ? array(j, i) : 0;
      }
    }

    return bits;
  };

  int max_white_bits = extract_bits_from_array(white_frame_mask);
  int max_back_bits = extract_bits_from_array(black_frame_mask);
  int max_bits = extract_bits_from_array(decoded_mask);

  white_frame_bits = extract_bits_from_array(decoded_white_frame_array);
  black_frame_bits = extract_bits_from_array(decoded_black_frame_array);

  for (auto & it : tag_templates) {
    int id = it.first;
    const std::array<Eigen::ArrayXXd, 4> id_templates = it.second;

    for (int direction = 0; direction < id_templates.size(); direction++) {
      Eigen::ArrayXXd aux = (decoded * payload_mask * decoded_mask * id_templates[direction]);
      double decoded_bits = extract_bits_from_array(aux);

      decoding_output.emplace_back(decoded_bits, id, direction);
    }
  }
}
