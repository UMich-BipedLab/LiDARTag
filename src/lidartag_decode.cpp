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
#include <lidartag/apriltag_utils.hpp>
#include <lidartag/tag16h5.hpp>
#include <lidartag/tag49h14.hpp>
#include <lidartag/utils.hpp>

#include <functional>
#include <numeric>

using namespace std;

namespace BipedLab
{
void LidarTag::getCodeNaive(string & code, pcl::PointCloud<LidarPoints_t *> payload)
{
  int topring = 0;
  int bottomring = beam_num_;
  PointXYZRI tl = payload[0]->point;
  PointXYZRI tr = payload[0]->point;
  PointXYZRI br = payload[0]->point;
  PointXYZRI bl = payload[0]->point;

  // Find the size of the payload
  for (int i = 0; i < payload.size(); ++i) {
    PointXYZRI point = payload[i]->point;
    if (point.y > tl.y && point.z > tl.z) tl = point;
    if (point.y > bl.y && point.z < bl.z) bl = point;
    if (point.y < tr.y && point.z > tr.z) tr = point;
    if (point.y < br.y && point.z < br.z) br = point;
  }

  vector<int> payload49(tag_family_, 0);
  int d = sqrt(tag_family_);
  float IntervalY = abs((tl.y + bl.y) / 2 - (tr.y + br.y) / 2) / (d + black_border_ - 1);
  float IntervalZ = abs((tl.z + tr.z) / 2 - (bl.z + br.z) / 2) / (d + black_border_ - 1);

  if (fake_tag_) {
    tl.y = 0;
    tl.z = 0;
    IntervalY = 1;
    IntervalZ = 1;
    payload.clear();
    payload.reserve((tag_family_ + 4 * d * black_border_ + 4 * (std::pow(black_border_, 2))));
    float j = 0;
    float k = 0;
    LidarPoints_t point;
    for (int i = 0; i < (tag_family_ + 4 * d * black_border_ + 4 * (std::pow(black_border_, 2)));
         ++i) {
      if (i % (d + 2 * black_border_) == 0 && i != 0) {
        k++;
        j = 0;
      }
      LidarPoints_t * point = new LidarPoints_t{{0, j, k, 0, 0}, 0, 0, 0, 0};
      payload.push_back(point);
      // cout << "j,k: " << j << ", " << k << endl;
      j++;
      // cout << "payload[i]" << payload[i]->point.y << ", " <<
      // payload[i]->point.z << endl;
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
  float avg_intensity = 0;
  for (int i = 0; i < payload.size(); ++i) {
    avg_intensity += payload[i]->point.intensity;
  }
  avg_intensity /= payload.size();
  // cout << "size: " << payload.size() << endl;
  // cout << "avg_intensity: " << avg_intensity << endl;

  // Split into grids
  for (int i = 0; i < payload.size(); ++i) {
    PointXYZRI * point_ptr = &(payload[i]->point);
    // cout << "i: " << i << endl;
    // cout << "point.y: " << point_ptr->y << endl;
    // cout << "point.z: " << point_ptr->z << endl;
    float delta_y = abs(point_ptr->y - tl.y);
    float delta_z = abs(point_ptr->z - tl.z);

    // if (delta_y==0 || (delta_y)) {
    //     if (point_ptr->intensity < avg_intensity) payload49[0] -= 1;
    //     else payload49[0] += 1;
    //     continue;
    // }
    int y = floor(delta_y / IntervalY);
    int z = floor(delta_z / IntervalZ);
    // cout << "Y: " << Y << endl;
    // cout << "Z: " << Z << endl;

    // remove black borders
    if (
      y >= black_border_ && z >= black_border_ && y <= d + black_border_ &&
      z <= d + black_border_) {
      int y_2 = (y - black_border_) % d;  // the yth column (remove the black border)
      int z_2 = (z - black_border_) % d;  // the zth row (remove the black border)

      int k = d * z_2 + y_2;  // index in a 1D vector
      // cout << "y: " << y << endl;
      // cout << "z: " << z << endl;
      // cout << "k: " << k << endl;
      // cout << "intensity: " << point_ptr->intensity << endl;
      if (point_ptr->intensity <= avg_intensity)
        payload49[k] -= 1;
      else
        payload49[k] += 1;
      // cout << "payload[k]: " << payload49[k] << endl;
      // cout << "--" << endl;
    }
  }

  // Threshold into black and white
  for (int i = 0; i < tag_family_; ++i) {
    if (payload49[i] < 0)
      code += to_string(0);
    else
      code += to_string(1);
  }

  for (int i = 0; i < tag_family_; ++i) {
    if (i % d == 0) {
      cout << "\n";
      cout << " " << code[i];
    } else {
      cout << " " << code[i];
    }
  }

  code += "UL";
  // cout << "\ncodeB: " << Code << endl;
  // 0101111111111111UL
  // cout << "Code:  " << "0010001100011011UL" << endl;
  cout << "\nCode 1:  \n"
       << " 0 0 1 0\n 1 1 1 0\n 1 0 1 0\n 0 1 1 1" << endl;
  // exit(-1);
  // code = "0010001100011011UL";
  // code = "0b0000011111100000011000011000110011100000111111111UL";
  // code = "11111100000011000011000110011100000111111111UL"; //1
  // code = "0001100111110000011001100011111000000110001100011UL"; // 2
  // cout << "codeA: " << Code << endl;
}

/* Decode using Weighted Gaussian weight
 * return  0: normal
 * return -1: not enough return
 * return -2: fail corner detection
 */
int LidarTag::getCodeWeightedGaussian(
  string & code, Homogeneous_t & pose, int & payload_points, const PointXYZRI & average,
  const pcl::PointCloud<LidarPoints_t *> & payload,
  const std::vector<LidarPoints_t *> & payload_boundary_ptr)
{
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
  visualization_msgs::msg::MarkerArray grid_marker_array;
  visualization_msgs::msg::Marker grid_marker;

  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = lidar_frame_;
  line_strip.header.stamp = current_scan_time_;
  line_strip.ns = "boundary";
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.scale.x = 0.002;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  PointXYZRI p11{0, 0, -1000, 0};
  PointXYZRI p21{0, -1000, 0, 0};
  PointXYZRI p31{0, 0, 1000, 0};
  PointXYZRI p41{0, 1000, 0, 0};

  PointXYZRI p12{0, 0, -1000, 0};
  PointXYZRI p22{0, -1000, 0, 0};
  PointXYZRI p32{0, 0, 1000, 0};
  PointXYZRI p42{0, 1000, 0, 0};

  // Find the largest
  for (int i = 0; i < payload_boundary_ptr.size(); ++i) {
    PointXYZRI point = payload_boundary_ptr[i]->point;

    if (point.z >= p11.z) {
      p11 = point;
    }

    if (point.y >= p21.y) {
      p21 = point;
    }

    if (point.z <= p31.z) {
      p31 = point;
    }

    if (point.y <= p41.y) {
      p41 = point;
    }
  }

  if (grid_viz_) {
    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("11"), 0, 0, 0, p11, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("21"), 0, 0, 0, p21, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("31"), 0, 0, 0, p31, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
    "Grid_" + string("41"), 0, 0, 0, p41, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);
  }

  // Find the second large
  for (int i = 0; i < payload_boundary_ptr.size(); ++i) {
    PointXYZRI point = payload_boundary_ptr[i]->point;

    if (point.z < p11.z && point.z >= p12.z) {
      p12 = point;
    }

    if (point.y < p21.y && point.y >= p22.y) {
      p22 = point;
    }

    if (point.z > p31.z && point.z <= p32.z) {
      p32 = point;
    }

    if (point.y > p41.y && point.y <= p42.y) {
      p42 = point;
    }
  }

  if (grid_viz_) {
    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("12"), 0, 0, 0, p12, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("22"), 0, 0, 0, p22, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("32"), 0, 0, 0, p32, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("42"), 0, 0, 0, p42, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);
  }

  PointXYZRI p1 = utils::pointsAddDivide(p11, p12, 2);
  PointXYZRI p2 = utils::pointsAddDivide(p21, p22, 2);
  PointXYZRI p3 = utils::pointsAddDivide(p31, p32, 2);
  PointXYZRI p4 = utils::pointsAddDivide(p41, p42, 2);

  // check condition of the detected corners
  // if corners are not between certain angle or distance,
  // consider the tag is up right =>
  // change way of detection
  int status = utils::checkCorners(payload_size_, p1, p2, p3, p4);
  if (status != 0) {
    // the tag is up right
    p1 = {0, 0, -1000, 0};
    p2 = {0, 0, 1000, 0};
    p3 = {0, 0, 1000, 0};
    p4 = {0, 0, -1000, 0};
    for (int i = 0; i < payload_boundary_ptr.size(); ++i) {
      PointXYZRI point = payload_boundary_ptr[i]->point;

      // left boundary
      if (point.z >= p1.z && point.y > average.y / 2) p1 = point;
      if (point.z <= p2.z && point.y > average.y / 2) p2 = point;

      // right boundary
      if (point.z <= p3.z && point.y < average.y / 2) p3 = point;
      if (point.z >= p4.z && point.y < average.y / 2) p4 = point;
    }
  }

  if (grid_viz_) {
    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("1"), 1, 1, 1, p1, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("2"), 1, 1, 1, p2, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("3"), 1, 1, 1, p3, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(grid_marker, visualization_msgs::msg::Marker::CUBE,
      "Grid_" + string("4"), 1, 1, 1, p4, 1, 0.01);
    grid_marker_array.markers.push_back(grid_marker);

    grid_marker_array.markers.push_back(grid_marker);
  }

  int d = std::sqrt(tag_family_);
  Eigen::MatrixXf vertices = Eigen::MatrixXf::Zero(3, 5);
  utils::formGrid(vertices, 0, 0, 0, payload_size_);
  Eigen::Matrix3f R;
  // Eigen::MatrixXf VerticesOffset = (vertices.colwise() -
  // utils::toEigen(average)); cout << "vertice: " << Vertices << endl; cout <<
  // "verticeOffset: " << VerticesOffset << endl; cout << "Average: " <<
  // utils::toEigen(average) << endl;
  utils::minus(p1, average);
  utils::minus(p2, average);
  utils::minus(p3, average);
  utils::minus(p4, average);

  utils::fitGrid(vertices, R, p1, p2, p3, p4);
  Eigen::Vector3f angle = utils::rotationMatrixToEulerAngles(R);

  if (grid_viz_) {
    LidarTag::assignMarker(
      grid_marker, visualization_msgs::msg::Marker::CUBE, "Model_" + string("1"), 0, 1, 0,
      utils::toVelodyne(vertices.col(1)), 1, 0.01);

    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(
      grid_marker, visualization_msgs::msg::Marker::CUBE, "Model_" + string("2"), 0, 1, 0,
      utils::toVelodyne(vertices.col(2)), 1, 0.01);

    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(
      grid_marker, visualization_msgs::msg::Marker::CUBE, "Model_" + string("3"), 0, 1, 0,
      utils::toVelodyne(vertices.col(3)), 1, 0.01);

    grid_marker_array.markers.push_back(grid_marker);

    LidarTag::assignMarker(
      grid_marker, visualization_msgs::msg::Marker::CUBE, "Model_" + string("4"), 0, 1, 0,
      utils::toVelodyne(vertices.col(4)), 1, 0.01);

    grid_marker_array.markers.push_back(grid_marker);

    geometry_msgs::msg::Point p;
    p.x = vertices(0, 1);
    p.y = vertices(1, 1);
    p.z = vertices(2, 1);
    line_strip.points.push_back(p);
    p.x = vertices(0, 2);
    p.y = vertices(1, 2);
    p.z = vertices(2, 2);
    line_strip.points.push_back(p);
    p.x = vertices(0, 3);
    p.y = vertices(1, 3);
    p.z = vertices(2, 3);
    line_strip.points.push_back(p);
    p.x = vertices(0, 4);
    p.y = vertices(1, 4);
    p.z = vertices(2, 4);
    line_strip.points.push_back(p);
    p.x = vertices(0, 1);
    p.y = vertices(1, 1);
    p.z = vertices(2, 1);
    line_strip.points.push_back(p);
  }

  // Calcutate Average intensity for thresholding
  float avg_intensity = 0;
  for (int i = 0; i < payload.size(); ++i) {
    avg_intensity += payload[i]->point.intensity;
  }

  avg_intensity /= payload.size();

  vector<PayloadVoting_t> Votes(payload.size());
  vector<float> v_r(std::pow((d + 2 * black_border_), 2));
  vector<float> v_g(std::pow((d + 2 * black_border_), 2));
  vector<float> v_b(std::pow((d + 2 * black_border_), 2));

  // pick a random color for each cell
  if (grid_viz_) {
    for (int i = 0; i < v_r.size(); ++i) {
      float r = (double)rand() / RAND_MAX;
      float g = (double)rand() / RAND_MAX;
      float b = (double)rand() / RAND_MAX;
      v_r[i] = r;
      v_g[i] = g;
      v_b[i] = b;
    }
  }

  // Split into grids
  for (int i = 0; i < payload.size(); ++i) {
    float t14, t12;
    Eigen::Vector2f v14, v12;
    PointXYZRI * point_ptr = &(payload[i]->point);
    utils::getProjection(p1, p4, *point_ptr, t14, v14);
    utils::getProjection(p1, p2, *point_ptr, t12, v12);
    Votes[i].p = point_ptr;
    PointXYZRI p;  // for visualization
    utils::assignCellIndex(payload_size_, R, p, average, d + 2 * black_border_, Votes[i]);
    if (grid_viz_) {
      LidarTag::assignMarker(
        grid_marker, visualization_msgs::msg::Marker::SPHERE, "TransPoints",
        v_r[Votes[i].cell], v_g[Votes[i].cell], v_b[Votes[i].cell], p, i, 0.005);
      grid_marker_array.markers.push_back(grid_marker);
    }

  }
  vector<vector<PayloadVoting_t *>> grid(std::pow((d + 2 * black_border_), 2));
  utils::sortPointsToGrid(grid, Votes);
  int too_less_return = 0;
  int payload_point_count = 0;
  for (int i = (d + 2 * black_border_) * black_border_ + black_border_;
       i < (d + 2 * black_border_) * (black_border_ + d) - black_border_; ++i) {
    if (
      (i % (d + 2 * black_border_) < black_border_) ||
      (i % (d + 2 * black_border_) > (d + black_border_ - 1)))
      continue;

    if (grid[i].size() < min_returns_per_grid_) too_less_return++;
    if (too_less_return > max_decode_hamming_) return -1;

    float weighted_prob = 0;
    float weight_sum = 0;
    float min_intensity = 10000.;
    float max_intensity = -1.;
    double r;
    double g;
    double b;

    // pick a random color for each cell
    if (grid_viz_) {
      r = (double)rand() / RAND_MAX;
      g = (double)rand() / RAND_MAX;
      b = (double)rand() / RAND_MAX;
    }

    for (int j = 0; j < grid[i].size(); ++j) {
      if (grid_viz_) {
        LidarTag::assignMarker(
          grid_marker, visualization_msgs::msg::Marker::SPHERE, "Point" + to_string(i), r, g, b,
          *(grid[i][j]->p), j, 0.005);
        grid_marker_array.markers.push_back(grid_marker);

        LidarTag::assignMarker(
          grid_marker, visualization_msgs::msg::Marker::SPHERE, "Center" + to_string(i), 1, 1, 1,
          grid[i][j]->centroid, j, 0.005);
        grid_marker_array.markers.push_back(grid_marker);

        LidarTag::assignMarker(
          grid_marker, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, "Prob" + to_string(i), 1, 1, 1,
          *(grid[i][j]->p), j, 0.003, to_string((grid[i][j]->p->intensity)));

        grid_marker_array.markers.push_back(grid_marker);
      }
      weighted_prob += grid[i][j]->weight * grid[i][j]->p->intensity;
      weight_sum += grid[i][j]->weight;
    }
    weighted_prob /= weight_sum;
    payload_point_count += grid[i].size();

    if (weighted_prob > 0.5)
      code += to_string(1);
    else
      code += to_string(0);
  }
  payload_points = payload_point_count;

  code += "UL";

  if (grid_viz_) {
    payload_grid_pub_->publish(grid_marker_array);
    payload_grid_line_pub_->publish(line_strip);
  }
  return 0;
}

Eigen::MatrixXf LidarTag::construct3DShapeMarker(
  RKHSDecoding_t & rkhs_decoding, const double & ell)
{
  Eigen::VectorXf offset_intensity =
    rkhs_decoding.template_points.row(3) -
    rkhs_decoding.ave_intensity * Eigen::MatrixXf::Ones(1, rkhs_decoding.num_points);
  Eigen::VectorXf pos_vec = Eigen::VectorXf::Zero(rkhs_decoding.num_points);
  Eigen::VectorXf neg_vec = Eigen::VectorXf::Zero(rkhs_decoding.num_points);
  int pos_indx = 0;
  int neg_indx = 0;
  for (int i = 0; i < rkhs_decoding.num_points; ++i) {
    if (offset_intensity[i] >= 0) {
      pos_vec[pos_indx] = offset_intensity[i];
      pos_indx++;
    } else {
      neg_vec[neg_indx] = offset_intensity[i];
      neg_indx++;
    }
  }

  pos_vec.conservativeResize(pos_indx);
  neg_vec.conservativeResize(neg_indx);

  // compute median
  float pos_median = std::abs(utils::computeMedian(pos_vec));
  float neg_median = std::abs(utils::computeMedian(neg_vec));
  // cout << "pos:" << pos_median << endl;
  // cout << "neg:" << neg_median << endl;

  pos_vec /= pos_median;
  neg_vec /= neg_median;

  pos_vec *= ell;
  neg_vec *= ell;

  Eigen::MatrixXf template_points_3d = rkhs_decoding.template_points;
  pos_indx = 0;
  neg_indx = 0;

  for (int i = 0; i < rkhs_decoding.num_points; ++i) {
    if (offset_intensity[i] > 0) {
      template_points_3d(0, i) = pos_vec[pos_indx];
      pos_indx++;
    } else {
      template_points_3d(0, i) = neg_vec[neg_indx];
      neg_indx++;
    }
  }

  return template_points_3d;
}

void LidarTag::singleTask(
  const Eigen::ArrayXf & x_ary, const Eigen::ArrayXf & y_ary, const Eigen::ArrayXf & z_ary,
  const Eigen::ArrayXf & i_ary, const Eigen::MatrixXf & pc1_j, const float & geo_sig,
  const float & feature_ell, const float & geo_ell, float & score)
{
  score = 0.0;
  const Eigen::VectorXf kernel_x = (x_ary - pc1_j(0)).square();
  const Eigen::VectorXf kernel_y = (y_ary - pc1_j(1)).square();
  const Eigen::VectorXf kernel_z = (z_ary - pc1_j(2)).square();
  const Eigen::VectorXf kernel_l = (i_ary - pc1_j(3)).square();

  Eigen::VectorXf geo_kernel = geo_sig * (-(kernel_x + kernel_y + kernel_z).cwiseSqrt()
    / (2 * std::pow(geo_ell, 2))).array().exp();  // 1Xnum1

  Eigen::VectorXf feat_kernel = 1.0 * (-kernel_l.cwiseSqrt()
    / (2 * std::pow(feature_ell, 2))).array().exp();  // 1Xnum1

  score += geo_kernel.dot(feat_kernel);
}

void LidarTag::singleTaskFixedSize(
  const Eigen::ArrayXf & x_ary, const Eigen::ArrayXf & y_ary, const Eigen::ArrayXf & z_ary,
  const Eigen::ArrayXf & i_ary, const Eigen::MatrixXf & pc1_j, const float & geo_sig,
  const float & feature_ell, const float & geo_ell, float & score)
{
  score = 0.0;
  int num_ary = x_ary.cols();
  Eigen::Matrix<float, 1, 3000> kernel_x = (x_ary - pc1_j(0)).square();
  Eigen::Matrix<float, 1, 3000> kernel_y = (y_ary - pc1_j(1)).square();
  Eigen::Matrix<float, 1, 3000> kernel_z = (z_ary - pc1_j(2)).square();
  Eigen::Matrix<float, 1, 3000> kernel_l = (i_ary - pc1_j(3)).square();
  Eigen::VectorXf geo_kernel = geo_sig * (-(kernel_x + kernel_y + kernel_z).cwiseSqrt()
    / (2 * std::pow(geo_ell, 2))).array().exp();  // 1Xnum1

  Eigen::VectorXf feat_kernel = 1.0 * (-kernel_l.cwiseSqrt()
    / (2 * std::pow(feature_ell, 2))).array().exp();  // 1Xnum1

  score += geo_kernel.dot(feat_kernel);
}

void LidarTag::multipleTasks(
  const Eigen::ArrayXf & x_ary, const Eigen::ArrayXf & y_ary, const Eigen::ArrayXf & z_ary,
  const Eigen::ArrayXf & i_ary, const Eigen::MatrixXf & pc1_j, const float & geo_sig,
  const float & feature_ell, const float & geo_ell, float & score)
{
  // cout << "cols: " << pc1_j.cols() << "," << pc1_j.rows() << endl;
  // cout << "x_ary.size: " << x_ary.rows() << "," << x_ary.cols() << endl;
  // cout << "y_ary.size: " << y_ary.rows() << "," << y_ary.cols() << endl;
  // cout << "z_ary.size: " << z_ary.rows() << "," << z_ary.cols() << endl;
  // cout << "i_ary.size: " << i_ary.rows() << "," << i_ary.cols() << endl;
  score = 0.0;
  float score_i = 0;

  for (int i = 0; i < pc1_j.cols(); ++i) {
    singleTask(x_ary, y_ary, z_ary, i_ary, pc1_j.col(i), geo_sig, feature_ell, geo_ell, score_i);
    score += score_i;
    // const Eigen::VectorXf kernel_x = (x_ary - pc1_j(0, i)).square();
    // const Eigen::VectorXf kernel_y = (y_ary - pc1_j(1, i)).square();
    // const Eigen::VectorXf kernel_z = (z_ary - pc1_j(2, i)).square();
    // const Eigen::VectorXf kernel_l = (i_ary - pc1_j(3, i)).square();
    // Eigen::VectorXf geo_kernel =
    //     geo_sig * (-(kernel_x + kernel_y + kernel_z).cwiseSqrt()
    //         / (2 * std::pow(geo_ell, 2))).array().exp(); // 1 x num1
    // Eigen::VectorXf feat_kernel =
    //     1.0 * (-kernel_l.cwiseSqrt()
    //         / (2 * std::pow(feature_ell, 2))).array().exp(); // 1 x num1

    // float dot_prod = geo_kernel.dot(feat_kernel);
    // cout << "dot prod" << dot_prod << endl;
    // score += dot_prod;
    // score += geo_kernel.dot(feat_kernel);
    // score += score;
  }
}

// The issue is when creating many large dynamic eigen array/vector/matrix in
// c++ 11 threads, it overwrites some existing memeory and further
// causes segfault This does not happen if create large fixed-size eigen
// objects. However, due to the fixed-size, it often needs to be large and
// the results are not correct.
void LidarTag::test(
  const Eigen::ArrayXf & x_ary, const Eigen::ArrayXf & y_ary, const Eigen::ArrayXf & z_ary,
  const Eigen::ArrayXf & i_ary, const Eigen::MatrixXf & pc1_j, const float & geo_sig,
  const float & feature_ell, const float & geo_ell, float & score)
{
  // cout << "cols: " << pc1_j.cols() << "," << pc1_j.rows() << endl;
  // cout << "x_ary.size: " << x_ary.rows() << "," << x_ary.cols() << endl;
  // cout << "y_ary.size: " << y_ary.rows() << "," << y_ary.cols() << endl;
  // cout << "z_ary.size: " << z_ary.rows() << "," << z_ary.cols() << endl;
  // cout << "i_ary.size: " << i_ary.rows() << "," << i_ary.cols() << endl;
  score = 0.0;
  float score_i = 0;
  float score_3000 = 0;

  constexpr int pre_set_size = 3000;
  int num_ary = x_ary.cols();
  Eigen::Matrix<float, 1, pre_set_size> zeros_out_mat =
    Eigen::Matrix<float, 1, pre_set_size>::Ones();

  zeros_out_mat.segment(num_ary, pre_set_size - 1).setZero();
  // Eigen::VectorXf kernel_x = Eigen::VectorXf::Zero(num_ary);
  // Eigen::VectorXf kernel_y = Eigen::VectorXf::Zero(num_ary);
  // Eigen::VectorXf kernel_z = Eigen::VectorXf::Zero(num_ary);
  // Eigen::VectorXf kernel_l = Eigen::VectorXf::Zero(num_ary);

  for (int i = 0; i < pc1_j.cols(); ++i) {
    // singleTask(x_ary, y_ary, z_ary, i_ary,
    //         pc1_j.col(i), geo_sig, feature_ell, geo_ell, score_i);
    // for (int j = 0; j < 4; ++j) {
    //    cout << pc1_j(j, i) << endl;
    //}
    int num_ary = x_ary.cols();
    Eigen::Matrix<float, 1, pre_set_size> kernel_x_fixed = (x_ary - pc1_j(0)).square();
    Eigen::Matrix<float, 1, pre_set_size> kernel_y_fixed = (y_ary - pc1_j(1)).square();
    Eigen::Matrix<float, 1, pre_set_size> kernel_z_fixed = (z_ary - pc1_j(2)).square();
    Eigen::Matrix<float, 1, pre_set_size> kernel_l_fixed = (i_ary - pc1_j(3)).square();
    Eigen::Matrix<float, 1, pre_set_size> kernel_x = kernel_x_fixed.cwiseProduct(zeros_out_mat);
    Eigen::Matrix<float, 1, pre_set_size> kernel_y = kernel_y_fixed.cwiseProduct(zeros_out_mat);
    Eigen::Matrix<float, 1, pre_set_size> kernel_z = kernel_z_fixed.cwiseProduct(zeros_out_mat);
    Eigen::Matrix<float, 1, pre_set_size> kernel_l = kernel_l_fixed.cwiseProduct(zeros_out_mat);

    if (kernel_x.hasNaN()) {
      cout << "k_x nan" << endl;
    }
    if (kernel_y.hasNaN()) {
      cout << "k_y nan" << endl;
    }
    if (kernel_z.hasNaN()) {
      cout << "k_z nan" << endl;
    }
    if (kernel_l.hasNaN()) {
      cout << "k_l nan" << endl;
    }
    Eigen::Matrix<float, 1, pre_set_size> geo_kernel_fixed = geo_sig
      * (-(kernel_x + kernel_y + kernel_z).cwiseSqrt() / (2 * std::pow(geo_ell, 2))).array().exp();  // 1Xnum1

    Eigen::Matrix<float, 1, pre_set_size> feat_kernel_fixed = 1.0 * (-kernel_l.cwiseSqrt()
      / (2 * std::pow(feature_ell, 2))).array().exp();  // 1Xnum1

    Eigen::Matrix<float, 1, pre_set_size> geo_kernel = geo_kernel_fixed.cwiseProduct(zeros_out_mat);
    Eigen::Matrix<float, 1, pre_set_size> feat_kernel = feat_kernel.cwiseProduct(zeros_out_mat);
    score_3000 = geo_kernel.dot(feat_kernel);
    // cout << "score_3000: " << score_3000 << endl;
    // score += score_3000;

    // Eigen::Matrix<float, 1, 2000> kernel_x = (x_ary - pc1_j(0, i));
    // Eigen::Matrix<float, 1, 2000> kernel_y = (y_ary - pc1_j(1, i));
    // Eigen::Matrix<float, 1, 2000> kernel_z = (z_ary - pc1_j(2, i));
    // Eigen::Matrix<float, 1, 2000> kernel_l = (i_ary - pc1_j(3, i));
    // score += score_i;
    // Eigen::VectorXf geo_kernel =
    //     geo_sig * (-(kernel_x + kernel_y + kernel_z).cwiseSqrt()
    //         / (2 * std::pow(geo_ell, 2))).array().exp(); // 1 x num1
    // Eigen::VectorXf feat_kernel =
    //     1.0 * (-kernel_l.cwiseSqrt()
    //         / (2 * std::pow(feature_ell, 2))).array().exp(); // 1 x num1

    // float dot_prod = geo_kernel.dot(feat_kernel);
    // cout << "dot prod" << dot_prod << endl;
    // score += dot_prod;
    // score += geo_kernel.dot(feat_kernel);
    // score += score;
  }
  // auto tmpx = x_ary.cols();
  // auto tmpy = y_ary.cols();
  // auto tmpz = z_ary.cols();
  // auto tmpi = i_ary.cols();
  // auto tmp1 = geo_sig;
  // auto tmp2 = feature_ell;
  // auto tmp3 = geo_ell;
  // score = 0;
  // for (int i = 0; i < pc1.cols(); ++i){
  //     auto tmp = pc1.col(i);
  //     score += 0.5;
  // }
  // //    singleTask(x_ary, y_ary, z_ary, i_ary,
  // //            , geo_sig, feature_ell, geo_ell, score_i);
  // // cout << "test" << endl;
}

void LidarTag::computeFunctionVectorInnerProductThreading(
  const Eigen::MatrixXf & pc1, const int & num_pc1, const Eigen::MatrixXf & pc2,
  const int & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  float inner_prod_sum = 0.0f;
  Eigen::Vector4f geo_dummy;
  Eigen::Vector4f feat_dummy;
  geo_dummy << 1, 1, 1, 0;
  feat_dummy << 0, 0, 0, 1;
  const Eigen::ArrayXf x_ary = pc2.row(0).array();
  const Eigen::ArrayXf y_ary = pc2.row(1).array();
  const Eigen::ArrayXf z_ary = pc2.row(2).array();
  const Eigen::ArrayXf i_ary = pc2.row(3).array();

  thread_vec_->reset_counter();
  const int quotient = num_pc1 / num_threads_;
  const int remainder = num_pc1 % num_threads_;
  Eigen::VectorXf score_vec = Eigen::VectorXf::Zero(num_threads_);
  int num_tasks = 0;
  int start_task = 0;
  int num_tasks_tot = 0;

  for (int i = 0; i < num_threads_; ++i) {
    start_task = quotient * i;
    if (i != num_threads_ - 1) {
      num_tasks = quotient;
    } else {
      num_tasks = num_pc1 - quotient * (num_threads_ - 1);
    }
    // cout << "i: " << i << endl;
    // cout << "size1: " << num_pc1 << endl;
    // cout << "start_task: " << start_task << endl;
    // cout << "num_tasks: " << num_tasks << endl;
    // cout << "start_task+num_tasks: " << start_task + num_tasks << endl;
    // Eigen::MatrixXf test_mat = pc1.block(0, start_task, 4, num_tasks);
    std::vector<int> indices(num_tasks);
    std::iota(indices.begin(), indices.end(), start_task);
    Eigen::MatrixXf test_mat = pc1(Eigen::all, indices);
    // thread_vec_->enqueueTask(std::bind(&LidarTag::test, this, pc1));

    thread_vec_->enqueueTask(std::bind(
      &LidarTag::multipleTasks, this, x_ary, y_ary, z_ary, i_ary, test_mat, geo_sig, feature_ell,
      geo_ell, std::ref(score_vec[i])));

    //thread_vec_->enqueueTask(std::bind(&LidarTag::multipleTasks, this,
    //            x_ary, y_ary, z_ary, i_ary,
    //            test_mat,
    //            geo_sig, feature_ell, geo_ell, std::ref(score_vec[i])));
  }
  thread_vec_->wait_until_finished(num_threads_);
  score = score_vec.sum() / num_pc1 / num_pc2;

  // XXX: works fine and results are correct
  // thread_vec_->reset_counter();
  // Eigen::VectorXf score_vec(num_pc1);
  // for (int i = 0; i < num_pc1; ++i) {
  //     thread_vec_->enqueueTask(
  //             std::bind(
  //                &LidarTag::singleTask, this,
  //                x_ary, y_ary, z_ary,
  //                i_ary, pc1.col(i),
  //                geo_sig, feature_ell, geo_ell,
  //                std::ref(score_vec[i])));
  // }
  // thread_vec_->wait_until_finished(num_pc1);
  // score = score_vec.sum()/num_pc1/num_pc2;

  // XXX: works fine and results are correct
  // thread_vec_->reset_counter();
  // Eigen::VectorXf score_vec = Eigen::VectorXf::Zero(num_pc1);
  // for (int i = 0; i < num_pc1; ++i) {
  //     thread_vec_->enqueueTask(std::bind(&LidarTag::multipleTasks, this,
  //                 x_ary, y_ary, z_ary, i_ary, pc1.block(0, i, 4, 1),
  //                 geo_sig, feature_ell, geo_ell, std::ref(score_vec[i])));
  // }
  // thread_vec_->wait_until_finished(num_pc1);
  // score = score_vec.sum()/num_pc1/num_pc2;

  // XXX: Running testing function
  // thread_vec_->reset_counter();
  // Eigen::VectorXf score_vec = Eigen::VectorXf::Zero(num_pc1);
  // for (int i = 0; i < num_pc1; ++i) {
  //     thread_vec_->enqueueTask(std::bind(&LidarTag::test, this));
  // }
  // thread_vec_->wait_until_finished(num_pc1);
  // score = score_vec.sum()/num_pc1/num_pc2;

  // XXX: Deosn't work
  // thread_vec_->reset_counter();
  // const int quotient = num_pc1 / num_threads_;
  // const int remainder = num_pc1 % num_threads_;
  // Eigen::VectorXf score_vec = Eigen::VectorXf::Zero(num_threads_);
  // int num_tasks = 0;
  // int start_task = 0;
  // int num_tasks_tot = 0;
  // for (int i = 0; i < num_threads_; ++i) {
  //     start_task = quotient * i;
  //     if (i != num_threads_ - 1) {
  //         num_tasks = quotient;
  //     }
  //     else {
  //         num_tasks = num_pc1 - quotient * (num_threads_ - 1);
  //     }
  //     // multipleTasks(x_ary, y_ary, z_ary, i_ary,
  //     //         pc1.block(0, start_task, 4, num_tasks),
  //     //         geo_sig, feature_ell, geo_ell, std::ref(score_vec[i]));
  //     // thread_vec_->enqueueTask(std::bind(&LidarTag::test, this));
  //     num_tasks_tot += num_tasks;
  //     // constexpr int tmp = num_tasks;
  //     Eigen::MatrixXf test_mat = pc1.block(0, start_task, 4, num_tasks);
  //
  //     //Eigen::MatrixXf test_mat = Eigen::MatrixXf::Zero(4, num_tasks);
  //     // for (int j = 0; j < num_tasks; ++j) {
  //     //     test_mat.col(j) = pc1.block<4, 1>(0, start_task);
  //     // }
  //
  //     // cout << i << "/" << num_threads_ << endl;
  //     // cout << "test_mat: " << test_mat.rows() << "," << test_mat.cols() <<
  //     endl;
  //     // cout << "pc1: " << pc1.rows() << "," << pc1.cols() << endl;
  //     // cout << "start_task, num tasks: " << start_task << "," << num_tasks
  //     << endl; thread_vec_->enqueueTask(std::bind(&LidarTag::multipleTasks,
  //     this,
  //                 x_ary, y_ary, z_ary, i_ary,
  //                 // std::cref(pc1.block(0, start_task, 4, num_tasks)),
  //                 test_mat,
  //                 // pc1.block(0, 0, 4, 10),
  //                 geo_sig, feature_ell, geo_ell, std::ref(score_vec[i])));
  // }
  // // cout << "wait..." << endl;
  // thread_vec_->wait_until_finished(num_threads_);
  // // std::cout << "score_vec: " << score_vec << endl;
  // score = score_vec.sum()/num_pc1/num_pc2;
  // // cout << "num_tasks_tot/total tasks: " << num_tasks_tot << "/" << num_pc1
  // << endl;
  // // cout << "score: " << score << endl;
  // // cout << "==============================" << endl;

  // XXX: sometimes work, sometimes do not
  // thread_vec_->reset_counter();
  // constexpr int points_per_task = 2;
  // int num_tasks = std::floor(num_pc1 / points_per_task);
  // Eigen::VectorXf score_vec = Eigen::VectorXf::Zero(num_tasks);
  // for (int task = 0; task < num_tasks; ++task) {
  //     int start_task = task * points_per_task;
  //     thread_vec_->enqueueTask(std::bind(&LidarTag::multipleTasks, this,
  //                 x_ary, y_ary, z_ary, i_ary,
  //                 pc1.block<4, points_per_task>(0, start_task),
  //                 geo_sig, feature_ell, geo_ell, std::ref(score_vec[task])));
  // }
  // thread_vec_->wait_until_finished(num_tasks);
  // score = score_vec.sum()/num_pc1/num_pc2;
}

void LidarTag::computeFunctionOriginalInnerProductTBB(
  const Eigen::MatrixXf & pc1, const float & num_pc1, const Eigen::MatrixXf & pc2,
  const float & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  Eigen::MatrixXf inner_prod = Eigen::MatrixXf::Zero(num_pc1, num_pc2);
  tbb::parallel_for(
    tbb::blocked_range2d<size_t>(0, num_pc1, num_threads_, 0, num_pc2, num_threads_),
    [&inner_prod, &pc1, &pc2, &feature_ell, &geo_sig,
     geo_ell](const tbb::blocked_range2d<size_t> & r) {
      for (auto i = r.rows().begin(); i < r.rows().end(); ++i) {
        const float feature1 = pc1(3, i);
        const Eigen::VectorXf p1 = pc1.block(0, i, 3, 1);
        for (auto j = r.cols().begin(); j < r.cols().end(); ++j) {
          const float feature2 = pc2(3, j);
          const Eigen::VectorXf p2 = pc2.block(0, j, 3, 1);
          const float feature_kernel =
            std::exp(-std::norm(feature1 - feature2) / (2 * std::pow(feature_ell, 2)));
          const float geometry_kernel =
            geo_sig * std::exp(-((p1 - p2).norm()) / (2 * std::pow(geo_ell, 2)));
          inner_prod(i, j) = feature_kernel * geometry_kernel;
        }
      }
    });

  score = inner_prod.sum() / num_pc1 / num_pc2;
}

void LidarTag::computeFunctionVectorInnerProductTBBThreadingManualScheduling(
  const Eigen::MatrixXf & pc1, const int & num_pc1, const Eigen::MatrixXf & pc2,
  const int & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  Eigen::Vector4f geo_dummy;
  Eigen::Vector4f feat_dummy;
  geo_dummy << 1, 1, 1, 0;
  feat_dummy << 0, 0, 0, 1;
  const Eigen::ArrayXf x_ary = pc2.row(0).array();
  const Eigen::ArrayXf y_ary = pc2.row(1).array();
  const Eigen::ArrayXf z_ary = pc2.row(2).array();
  const Eigen::ArrayXf i_ary = pc2.row(3).array();

  int num_total_tasks = num_threads_ * 1;
  const int quotient = num_pc1 / num_total_tasks;
  const int remainder = num_pc1 % num_total_tasks;
  Eigen::VectorXf score_vec = Eigen::VectorXf::Zero(num_total_tasks);
  int num_tasks = 0;
  int start_task = 0;
  int num_tasks_tot = 0;
  // for (int i = 0; i < num_threads_; ++i) {
  tbb::parallel_for(int(0), num_total_tasks, [&](int i) {
    start_task = quotient * i;
    if (i != num_threads_ - 1) {
      num_tasks = quotient;
    } else {
      num_tasks = num_pc1 - quotient * (num_threads_ - 1);
    }
    std::vector<int> indices(num_tasks);
    std::iota(indices.begin(), indices.end(), start_task);
    Eigen::MatrixXf test_mat = pc1(Eigen::all, indices);
    multipleTasks(
      x_ary, y_ary, z_ary, i_ary, test_mat, geo_sig, feature_ell, geo_ell, std::ref(score_vec[i]));
  });

  score = score_vec.sum() / num_pc1 / num_pc2;
}

void LidarTag::computeFunctionVectorInnerProductTBBThreadingNoScheduling(
  const Eigen::MatrixXf & pc1, const int & num_pc1, const Eigen::MatrixXf & pc2,
  const int & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  Eigen::Vector4f geo_dummy;
  Eigen::Vector4f feat_dummy;
  geo_dummy << 1, 1, 1, 0;
  feat_dummy << 0, 0, 0, 1;
  const Eigen::ArrayXf x_ary = pc2.row(0).array();
  const Eigen::ArrayXf y_ary = pc2.row(1).array();
  const Eigen::ArrayXf z_ary = pc2.row(2).array();
  const Eigen::ArrayXf i_ary = pc2.row(3).array();

  // tbb::atomic<float> score_thread = 0;
  Eigen::VectorXf score_vec(num_pc1);
  tbb::parallel_for(
    int(0), num_pc1,
    [&](int i) {
      singleTask(
        x_ary, y_ary, z_ary, i_ary, pc1.col(i), geo_sig, feature_ell, geo_ell, score_vec[i]);
      // score_thread = score_thread + score_vec[i];
    },
    tbb::auto_partitioner());

  // score = score_thread;
  score = score_vec.sum() / num_pc1 / num_pc2;
}

void LidarTag::computeFunctionVectorInnerProductTBBThreadingTBBScheduling(
  const Eigen::MatrixXf & pc1, const int & num_pc1, const Eigen::MatrixXf & pc2,
  const int & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  Eigen::Vector4f geo_dummy;
  Eigen::Vector4f feat_dummy;
  geo_dummy << 1, 1, 1, 0;
  feat_dummy << 0, 0, 0, 1;
  const Eigen::ArrayXf x_ary = pc2.row(0).array();
  const Eigen::ArrayXf y_ary = pc2.row(1).array();
  const Eigen::ArrayXf z_ary = pc2.row(2).array();
  const Eigen::ArrayXf i_ary = pc2.row(3).array();
  tbb::atomic<float> score_thread = 0;

  Eigen::VectorXf score_vec(num_pc1);
  // std::vector<float> score_vec(num_pc1);
  // tbb::parallel_for(tbb::blocked_range<size_t>(0, num_pc1),
  //         [&](const tbb::blocked_range<size_t>& r,
  //             const Eigen::ArrayXf &x_ary,
  //             const Eigen::ArrayXf &y_ary,
  //             const Eigen::ArrayXf &z_ary,
  //             const Eigen::ArrayXf &i_ary,
  //             const Eigen::MatrixXf &pc1,
  //             const float &geo_sig,
  //             const float &feature_ell,
  //             const float &geo_ell,
  //             Eigen::VectorXf &score_vec){
  //                 for (int i = r.begin(); i < r.end(); ++i) {
  //                     singleTask(x_ary, y_ary, z_ary, i_ary, pc1.col(i),
  //                             geo_sig, feature_ell, geo_ell, score_vec[i]);
  //                 }
  //             }
  // );

  score = score_vec.sum() / num_pc1 / num_pc2;
}

void LidarTag::computeFunctionVectorInnerProduct(
  const Eigen::MatrixXf & pc1, const float & num_pc1, const Eigen::MatrixXf & pc2,
  const float & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  float inner_prod_sum = 0.0f;
  Eigen::Vector4f geo_dummy;
  Eigen::Vector4f feat_dummy;
  geo_dummy << 1, 1, 1, 0;
  feat_dummy << 0, 0, 0, 1;
  const Eigen::ArrayXf x_ary = pc2.row(0).array();
  const Eigen::ArrayXf y_ary = pc2.row(1).array();
  const Eigen::ArrayXf z_ary = pc2.row(2).array();
  const Eigen::ArrayXf i_ary = pc2.row(3).array();
  Eigen::VectorXf score_vec((int)num_pc1);
  for (int j = 0; j < num_pc1; ++j) {
    const Eigen::VectorXf kernel_x = (x_ary - pc1(0, j)).square();
    const Eigen::VectorXf kernel_y = (y_ary - pc1(1, j)).square();
    const Eigen::VectorXf kernel_z = (z_ary - pc1(2, j)).square();
    const Eigen::VectorXf kernel_l = (i_ary - pc1(3, j)).square();

    Eigen::VectorXf geo_kernel = geo_sig * (-(kernel_x + kernel_y + kernel_z).cwiseSqrt()
      / (2 * std::pow(geo_ell, 2))).array().exp();  // 1Xnum1

    Eigen::VectorXf feat_kernel = 1.0 * (-kernel_l.cwiseSqrt()
      / (2 * std::pow(feature_ell, 2))).array().exp();  // 1Xnum1

    inner_prod_sum += geo_kernel.dot(feat_kernel);
  }

  score = inner_prod_sum / num_pc1 / num_pc2;
}

void LidarTag::computeFunctionMatrixInnerProduct(
  const Eigen::MatrixXf & pc1, const float & num_pc1, const Eigen::MatrixXf & pc2,
  const float & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(1, num_pc2);
  Eigen::Vector4f geo_dummy;
  Eigen::Vector4f feat_dummy;
  geo_dummy << 1, 1, 1, 0;
  feat_dummy << 0, 0, 0, 1;
  Eigen::MatrixXf inner_prod = Eigen::MatrixXf::Zero(num_pc1, num_pc2);

  for (int j = 0; j < num_pc1; ++j) {
    Eigen::MatrixXf repmat = pc1.col(j) * ones;
    Eigen::MatrixXf kernel = (pc2 - repmat).array().square().matrix();  // 4xnum1
    Eigen::MatrixXf geo_kernel = geo_sig * (-(geo_dummy.transpose() * kernel).array().sqrt()
      / (2 * std::pow(geo_ell, 2))).array().exp();  // 1Xnum1
    Eigen::MatrixXf feat_kernel = 1.0 * (-(feat_dummy.transpose() * kernel).array().sqrt()
      / (2 * std::pow(feature_ell, 2))).array().exp();  // 1Xnum1
    inner_prod.row(j) = (geo_kernel.array() * feat_kernel.array()).matrix();
  }

  score = inner_prod.sum() / num_pc1 / num_pc2;
}

void LidarTag::computeFunctionOriginalInnerProduct(
  const Eigen::MatrixXf & pc1, const float & num_pc1, const Eigen::MatrixXf & pc2,
  const float & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  // Original double sum
  // float score_tmp = 0;
  Eigen::MatrixXf inner_prod = Eigen::MatrixXf::Zero(num_pc1, num_pc2);
  for (int i = 0; i < num_pc1; ++i) {
    float feature1 = pc1(3, i);
    Eigen::VectorXf p1 = pc1.block(0, i, 3, 1);
    for (int j = 0; j < num_pc2; ++j) {
      float feature2 = pc2(3, j);
      Eigen::VectorXf p2 = pc2.block(0, j, 3, 1);
      float feature_kernel =
        std::exp(-std::norm(feature1 - feature2) / (2 * std::pow(feature_ell, 2)));
      float geometry_kernel = geo_sig * std::exp(-((p1 - p2).norm()) / (2 * std::pow(geo_ell, 2)));
      // score_tmp += feature_kernel * geometry_kernel;
      inner_prod(i, j) = feature_kernel * geometry_kernel;
    }
  }
  score = inner_prod.sum() / num_pc1 / num_pc2;
  // score = score_tmp;
}

void LidarTag::computeFunctionOriginalInnerProductKDTree(
  const Eigen::MatrixXf & pc1, const int & num_pc1, const Eigen::MatrixXf & pc2,
  const int & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  Eigen::MatrixXf pc2_points = pc2.topRows(3);
  Eigen::MatrixXf pc2_feat = pc2.bottomRows(1);

  constexpr float sp_thres = 1e-3;
  const float search_radius = geo_ell / 2;
  // constexpr float sp_thres = 1e-3;
  // const float search_radius = geo_ell;
  // std::abs(2.0 * geo_ell * geo_ell * std::log(sp_thres / geo_sig));
  // cout << "dis_thre: " << search_radius << endl;

  kd_tree_t mat_index(3 /*dim*/, std::cref(pc2_points), 10 /* max leaf */);
  mat_index.index->buildIndex();
  tbb::atomic<float> score_thread = 0;
  // float score_tmp = 0;
  // score = 0;
  tbb::concurrent_vector<float> score_vec;

  tbb::parallel_for(int(0), num_pc1, [&](int i) {
    // for(int i = 0; i < num_pc1; ++i){
    const float search_radius_tmp = search_radius;
    std::vector<std::pair<long int, float>> ret_matches;
    nanoflann::SearchParams params;
    Eigen::Vector3f query = pc2.block<3, 1>(0, i);
    vector<float> query_vec(query.data(), query.data() + query.size());
    const size_t n_matches =
      mat_index.index->radiusSearch(&query_vec[0], search_radius_tmp, ret_matches, params);
    // cout << "num_matches/total: " << n_matches << " / " << num_pc2 << endl;
    float feature1 = pc1(3, i);

    for (size_t j = 0; j < n_matches; ++j) {
      int idx = ret_matches[j].first;
      float dis_squared = ret_matches[j].second;
      float feature2 = pc2(3, idx);
      float feature_kernel =
        std::exp(-std::norm(feature1 - feature2) / (2 * std::pow(feature_ell, 2)));
      float geometry_kernel =
        geo_sig * std::exp(-(std::sqrt(dis_squared)) / (2 * std::pow(geo_ell, 2)));
      // tbb::atomic<float> inner_prod = feature_kernel * geometry_kernel;
      float inner_prod = feature_kernel * geometry_kernel;
      // cout << "inner_prod: " << inner_prod << endl;
      if (inner_prod > sp_thres)
        // score_tmp += inner_prod;
        // score += inner_prod;
        // score_vec.push_back(inner_prod);
        score_thread = score_thread + inner_prod;
    }
    // }
  });
  // return score_tmp;
  // score = score_tmp;
  score = score_thread / num_pc1 / num_pc2;

  // for (int i = 0; i < score_vec.size(); ++i) {
  //     score += score_vec[i];
  // }
}
void LidarTag::computeFunctionInnerProductModes(
  const int mode, const Eigen::MatrixXf & pc1, const float & num_pc1, const Eigen::MatrixXf & pc2,
  const float & num_pc2, const float & geo_sig, const float & feature_ell, const float & geo_ell,
  float & score)
{
  // std::clock_t c_start = std::clock();
  switch (mode) {
    case 0:
      // single thread: original double sum
      computeFunctionOriginalInnerProduct(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      // utils::printSpendHz(std::clock(), c_start, "Original: ");
      break;
    case 1:
      // single thread: convert to matrices
      // c_start = std::clock();
      computeFunctionMatrixInnerProduct(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      // utils::printSpendHz(std::clock(), c_start, "Matrix: ");
      break;

    case 2:
      // single thread: convert matrices to vectors
      // c_start = std::clock();
      computeFunctionVectorInnerProduct(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      // utils::printSpendHz(std::clock(), c_start, "Vectorization: ");
      break;
    case 3:
      // C++11 thread (works for each point for a thread
      // but not for blobs of points for a thread, reason are given above)
      // computeFunctionVectorInnerProductThreading(
      //         pc1, num_pc1, pc2, num_pc2,
      //         geo_sig, feature_ell, geo_ell, score);
      break;
    case 4:
      // c_start = std::clock();
      computeFunctionOriginalInnerProductTBB(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      // utils::printSpendHz(std::clock(), c_start, "TBB Original: ");
      break;
    case 5:
      // c_start = std::clock();
      computeFunctionVectorInnerProductTBBThreadingNoScheduling(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      // utils::printSpendHz(std::clock(), c_start, "No Scheduling TBB
      // Vectorization: ");
      break;
    case 6:
      // c_start = std::clock();
      // computeFunctionVectorInnerProductTBBThreadingManualScheduling(
      //         pc1, num_pc1, pc2, num_pc2,
      //         geo_sig, feature_ell, geo_ell, score);
      // utils::printSpendHz(std::clock(), c_start, "Manual Scheduling TBB
      // Vectorization: ");
      break;
    case 7:
      // c_start = std::clock();
      computeFunctionVectorInnerProductTBBThreadingTBBScheduling(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      // utils::printSpendHz(std::clock(), c_start, "TBB Scheduling TBB
      // Vectorization: ");
      break;
    case 8:
      // c_start = std::clock();
      computeFunctionOriginalInnerProductKDTree(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      // utils::printSpendHz(std::clock(), c_start, "TBB KDtree: ");
      // std::cout << "===================================" << endl;
      break;
  }
}

float LidarTag::computeFunctionInnerProduct(
  const Eigen::MatrixXf & pc1, const Eigen::MatrixXf & pc2, const float & geo_ell)
{
  float feature_ell = 10;
  float geo_sig = 1e5;
  int num_pc1 = pc1.cols();
  int num_pc2 = pc2.cols();
  float score = 0;

  std::chrono::duration<double> duration;

  if (num_pc1 < num_pc2) {
    if (!debug_decoding_time_) {
      computeFunctionInnerProductModes(
        decode_mode_, pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
    } else {
      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionOriginalInnerProduct(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.original +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "Original : " << 1.0 / duration.count() << " [Hz]" << endl;

      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionMatrixInnerProduct(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.matrix +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "Matrix : " << 1.0 / duration.count() << " [Hz]" << endl;

      // single thread
      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionVectorInnerProduct(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.vectorization +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "Vectorization : " << 1.0 / duration.count() << " [Hz]" <<
      // endl;

      // C++11 thread (works for each point for a thread
      // but not for blobs of points for a thread, reason are given above)
      // computeFunctionVectorInnerProductThreading(
      //         pc1, num_pc1, pc2, num_pc2,
      //         geo_sig, feature_ell, geo_ell, score);

      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionVectorInnerProductTBBThreadingNoScheduling(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.tbb_vectorization +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "No Scheduling TBB Vectorization : " << 1.0 / duration.count()
      // << " [Hz]" << endl;

      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionOriginalInnerProductTBB(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.tbb_original +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "TBB Original : " << 1.0 / duration.count() << " [Hz]" << endl;

      // c_start = std::clock();
      // clock_start = std::chrono::steady_clock::now();
      // computeFunctionVectorInnerProductTBBThreadingManualScheduling(
      //         pc1, num_pc1, pc2, num_pc2,
      //         geo_sig, feature_ell, geo_ell, score);
      // duration = std::chrono::steady_clock::now() - clock_start;
      // cout << "Manual Scheduling TBB Vectorization : "
      //      << 1.0 / duration.count() << " [Hz]" << endl;
      // utils::printSpendHz(std::clock(), c_start, "Manual Scheduling TBB
      // Vectorization: ");

      // c_start = std::clock();
      // clock_start = std::chrono::steady_clock::now();
      // computeFunctionVectorInnerProductTBBThreadingTBBScheduling(
      //         pc1, num_pc1, pc2, num_pc2,
      //         geo_sig, feature_ell, geo_ell, score);
      // duration = std::chrono::steady_clock::now() - clock_start;
      // cout << "Original : " << 1.0 / duration.count() << " [Hz]" << endl;
      // utils::printSpendHz(std::clock(), c_start, "TBB Scheduling TBB
      // Vectorization: ");

      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionOriginalInnerProductKDTree(
        pc1, num_pc1, pc2, num_pc2, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.tbb_kd_tree +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "TBB KDTree : " << 1.0 / duration.count() << " [Hz]" << endl;
      // std::cout << "===================================" << endl;
    }

  } else {
    if (!debug_decoding_time_) {
      computeFunctionInnerProductModes(
        decode_mode_, pc2, num_pc2, pc1, num_pc1, geo_sig, feature_ell, geo_ell, score);
    } else {
      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionOriginalInnerProduct(
        pc2, num_pc2, pc1, num_pc1, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.original +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "Original : " << 1.0 / duration.count() << " [Hz]" << endl;

      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionMatrixInnerProduct(
        pc2, num_pc2, pc1, num_pc1, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.matrix +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "Matrix : " << 1.0 / duration.count() << " [Hz]" << endl;

      // single thread
      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionVectorInnerProduct(
        pc2, num_pc2, pc1, num_pc1, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.vectorization +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "Vectorization : " << 1.0 / duration.count() << " [Hz]" <<
      // endl;

      // C++11 thread (works for each point for a thread
      // but not for blobs of points for a thread, reason are given above)
      // computeFunctionVectorInnerProductThreading(
      //         pc2, num_pc2, pc1, num_pc1,
      //         geo_sig, feature_ell, geo_ell, score);

      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionVectorInnerProductTBBThreadingNoScheduling(
        pc2, num_pc2, pc1, num_pc1, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.tbb_vectorization +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "No Scheduling TBB Vectorization : " << 1.0 / duration.count()
      // << " [Hz]" << endl;

      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionOriginalInnerProductTBB(
        pc2, num_pc2, pc1, num_pc1, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.tbb_original +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "TBB Original : " << 1.0 / duration.count() << " [Hz]" << endl;

      // clock_start = std::chrono::steady_clock::now();
      // computeFunctionVectorInnerProductTBBThreadingManualScheduling(
      //         pc2, num_pc2, pc1, num_pc1,
      //         geo_sig, feature_ell, geo_ell, score);
      // duration = std::chrono::steady_clock::now() - clock_start;
      // cout << "Manual Scheduling TBB Vectorization : "
      //     << 1.0 / duration.count() << " [Hz]" << endl;

      // clock_start = std::chrono::steady_clock::now();
      // computeFunctionVectorInnerProductTBBThreadingTBBScheduling(
      //         pc2, num_pc2, pc1, num_pc1,
      //         geo_sig, feature_ell, geo_ell, score);
      // duration = std::chrono::steady_clock::now() - clock_start;
      // cout << "Original : " << 1.0 / duration.count() << " [Hz]" << endl;

      time_decoding_.timing = std::chrono::steady_clock::now();
      computeFunctionOriginalInnerProductKDTree(
        pc2, num_pc2, pc1, num_pc1, geo_sig, feature_ell, geo_ell, score);
      time_decoding_.tbb_kd_tree +=
        utils::spendElapsedTimeMilli(std::chrono::steady_clock::now(), time_decoding_.timing);
      duration = std::chrono::steady_clock::now() - time_decoding_.timing;
      // cout << "TBB KDTree : " << 1.0 / duration.count() << " [Hz]" << endl;
      // std::cout << "===================================" << endl;
    }
  }
  // std::clock_t c_end = std::clock();
  // std::cout << std::fixed << std::setprecision(2) << "CPU time used: "
  //           << (double) 1e6 * (c_end-c_start) / CLOCKS_PER_SEC << " us\n";

  return score;

  // TESTING
  // int k = 2;
  // float feature1 = pc1(3, k);
  // float feature2 = pc2(3, k);

  // Eigen::VectorXf p1 = pc1.block(0, k, 3, 1);
  // Eigen::VectorXf p2 = pc2.block(0, k, 3, 1);

  // float feature_kernel = std::exp( -( std::norm(feature1 - feature2) ) /
  //         (2*std::pow(feature_ell, 2) ));
  // float geometry_kernel = geo_sig * std::exp( -( (p1 - p2).norm() ) /
  //         (2*std::pow(geo_ell, 2) ));
  // cout << "num_pc1: " << num_pc1 << endl;
  // cout << "num_pc2: " << num_pc2 << endl;
  // cout << "feature1: " << feature1 << endl;
  // cout << "feature2: " << feature2 << endl;
  // cout << "p1: " << p1 << endl;
  // cout << "p2: " << p2 << endl;
  // cout << "ell: " << geo_ell << endl;
  // cout << "2*ell^2: " << 2*std::pow(geo_ell, 2) << endl;
  // cout << "norm(p1-p2): " << (p1-p2).norm() << endl;
  // cout << "-norm(p1-p2)/(2ell^2): " << -((p1 -
  // p2).norm())/(2*std::pow(geo_ell, 2)) << endl; cout << "exp(norm())/2ell^2:
  // " << std::exp( -((p1 - p2).norm()) / (2*std::pow(geo_ell, 2))) << endl;
  // cout << "freature_kernel: " << feature_kernel << endl;
  // cout << "geometry_kernel: " << geometry_kernel << endl;
  // cout << "\n\n\n" << endl;

  // Vector format for num_pc1
  // float inner_prod_sum = 0.0f;
  // Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(1, num_pc1);
  // Eigen::Vector4f geo_dummy;
  // Eigen::Vector4f feat_dummy;
  // geo_dummy << 1,1,1,0;
  // feat_dummy << 0,0,0,1;
  // for (int j = 0; j < num_pc2; ++j) {
  //     /*
  //     Eigen::MatrixXf repmat = pc2.col(j) * ones;
  //     Eigen::MatrixXf kernel = (pc1 - repmat).array().square(); // 4xnum1
  //     Eigen::VectorXf geo_kernel =
  //         geo_sig * (-(geo_dummy.transpose() * kernel).cwiseSqrt()
  //                    / (2 * std::pow(geo_ell, 2))).array().exp(); // 1Xnum1
  //     Eigen::VectorXf feat_kernel =
  //         1.0 * (-(feat_dummy.transpose() * kernel).cwiseSqrt()
  //                / (2 * std::pow(feature_ell, 2))).array().exp(); // 1Xnum1
  //     */
  //     Eigen::VectorXf kernel_x = (pc1.row(0).array() - pc2(0, j)).square();
  //     Eigen::VectorXf kernel_y = (pc1.row(1).array() - pc2(1, j)).square();
  //     Eigen::VectorXf kernel_z = (pc1.row(2).array() - pc2(2, j)).square();
  //     Eigen::VectorXf kernel_l = (pc1.row(3).array() - pc2(3, j)).square();

  //     Eigen::VectorXf geo_kernel =
  //         geo_sig * (-(kernel_x + kernel_y + kernel_z).cwiseSqrt()
  //                    / (2 * std::pow(geo_ell, 2))).array().exp(); // 1Xnum1
  //     Eigen::VectorXf feat_kernel =
  //         1.0 * (-kernel_l.cwiseSqrt()
  //                / (2 * std::pow(feature_ell, 2))).array().exp(); // 1Xnum1

  //     inner_prod_sum += geo_kernel.dot(feat_kernel);
  // }
  // return inner_prod_sum / num_pc1 / num_pc2;

  // Vecfor format for num_pc2
  // Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(1, num_pc2);
  // Eigen::Vector4f geo_dummy;
  // Eigen::Vector4f feat_dummy;
  // geo_dummy << 1,1,1,0;
  // feat_dummy << 0,0,0,1;
  // for (int j = 0; j < num_pc1; ++j) {
  //     /*
  //     Eigen::MatrixXf repmat = pc2.col(j) * ones;
  //     Eigen::MatrixXf kernel = (pc1 - repmat).array().square(); // 4xnum1
  //     Eigen::VectorXf geo_kernel =
  //         geo_sig * (-(geo_dummy.transpose() * kernel).cwiseSqrt()
  //                    / (2 * std::pow(geo_ell, 2))).array().exp(); // 1Xnum1
  //     Eigen::VectorXf feat_kernel =
  //         1.0 * (-(feat_dummy.transpose() * kernel).cwiseSqrt()
  //                / (2 * std::pow(feature_ell, 2))).array().exp(); // 1Xnum1
  //     */
  //     Eigen::VectorXf kernel_x = (pc2.row(0).array() - pc1(0, j)).square();
  //     Eigen::VectorXf kernel_y = (pc2.row(1).array() - pc1(1, j)).square();
  //     Eigen::VectorXf kernel_z = (pc2.row(2).array() - pc1(2, j)).square();
  //     Eigen::VectorXf kernel_l = (pc2.row(3).array() - pc1(3, j)).square();

  //     Eigen::VectorXf geo_kernel =
  //         geo_sig * (-(kernel_x + kernel_y + kernel_z).cwiseSqrt()
  //                    / (2 * std::pow(geo_ell, 2))).array().exp(); // 1Xnum1
  //     Eigen::VectorXf feat_kernel =
  //         1.0 * (-kernel_l.cwiseSqrt()
  //                / (2 * std::pow(feature_ell, 2))).array().exp(); // 1Xnum1

  //     inner_prod_sum += geo_kernel.dot(feat_kernel);
  // }
  // return inner_prod_sum / num_pc1 / num_pc2;

  // Matrix format for num_pc2
  // Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(1, num_pc2);
  // Eigen::Vector4f geo_dummy;
  // Eigen::Vector4f feat_dummy;
  // geo_dummy << 1,1,1,0;
  // feat_dummy << 0,0,0,1;
  // for (int j = 0; j < num_pc1; ++j) {
  //     Eigen::MatrixXf repmat = pc1.col(j) * ones;
  //     // cout << "repmat.size(): " << repmat.rows() << "," << repmat.cols()
  //     << endl;

  //     Eigen::MatrixXf kernel = (pc2 - repmat).array().square().matrix(); //
  //     4xnum1
  //     // cout << "kernel.size(): " << kernel.rows() << "," << kernel.cols()
  //     << endl;

  //     Eigen::MatrixXf geo_kernel =
  //         geo_sig * (
  //                 -(geo_dummy.transpose() * kernel).array().sqrt()
  //                 / (2 * std::pow(geo_ell, 2))).array().exp(); // 1Xnum1
  //     // cout << "geokernel: " << geo_kernel.rows() << "," <<
  //     geo_kernel.cols() << endl;

  //     Eigen::MatrixXf feat_kernel =
  //         1.0 * (
  //                 -(feat_dummy.transpose() * kernel).array().sqrt()
  //                 / (2 * std::pow(feature_ell, 2))).array().exp(); // 1Xnum1
  //     // cout << "featkernel: " << feat_kernel.rows() << "," <<
  //     feat_kernel.cols() << endl;
  //
  //     // auto tmp = (geo_kernel.array() *
  //     feat_kernel.array()).matrix().transpose();
  //     // cout << "tmp: " << tmp.rows() << "," << tmp.cols() << endl;
  //     // cout << "tmp val: " << tmp(0,0) << endl;
  //     inner_prod.row(j) = (geo_kernel.array() *
  //     feat_kernel.array()).matrix();
  //     // cout << "j-th col: " << inner_prod(j,0) << endl;
  //     // inner_prod.col(j) = ? // num1 X 1
  // }

  // Original double sum
  // Eigen::MatrixXf inner_prod = Eigen::MatrixXf::Zero(num_pc1, num_pc2);
  // for (int i = 0; i < num_pc1; ++i) {
  //     float feature1 = pc1(3, i);
  //     Eigen::VectorXf p1 = pc1.block(0, i, 3, 1);
  //     for (int j = 0; j < num_pc2; ++j) {
  //         float feature2 = pc2(3, j);
  //         Eigen::VectorXf p2 = pc2.block(0, j, 3, 1);
  //         float feature_kernel = std::exp(
  //                 -std::norm(feature1 - feature2) / (2 *
  //                 std::pow(feature_ell, 2)));
  //         float geometry_kernel = geo_sig * std::exp(
  //                 -((p1 - p2).norm()) / (2 * std::pow(geo_ell, 2)));
  //         inner_prod(i, j) = feature_kernel * geometry_kernel;
  //     }
  // }
  // return inner_prod.sum()/num_pc1/num_pc2;
}

// return 0 if sucessfully or -1 if score is too low
int LidarTag::getCodeRKHS(RKHSDecoding_t & rkhs_decoding, const double & tag_size)
{
  int status;
  int size_num = rkhs_decoding.size_num;
  int num_codes = function_dic_[size_num].size();
  rkhs_decoding.ell = tag_size / (std::sqrt(tag_family_) + 4 * black_border_) / 2;
  rkhs_decoding.template_points_3d = construct3DShapeMarker(rkhs_decoding, rkhs_decoding.ell);
  rkhs_decoding.score = std::vector<float>(num_codes * 4);
  float area = tag_size * tag_size;
  float id_score = -1;
  // Eigen::initParallel();
  // std::clock_t c_start = std::clock();
  float cur_score = 0;
  for (int i = 0; i < num_codes; ++i) {
    cur_score = computeFunctionInnerProduct(
      rkhs_decoding.template_points_3d, function_dic_[size_num][i], rkhs_decoding.ell) / area;

    rkhs_decoding.score[i] = cur_score;
    // rkhs_decoding.score[i] = computeFunctionInnerProduct(
    //         rkhs_decoding.template_points_3d,
    //         _function_dic[size_num][i],
    //         rkhs_decoding.ell);
    // rkhs_decoding.score[i] = computeFunctionInnerProductKDTree(
    //         rkhs_decoding.template_points_3d,
    //         _function_dic[size_num][i],
    //         rkhs_decoding.ell);
    if (cur_score > id_score) {
      rkhs_decoding.id = i / 4;
      rkhs_decoding.rotation_angle = i % 4;
      // rkhs_decoding.id_score = rkhs_decoding.score[i];
      rkhs_decoding.id_score = cur_score;
      id_score = rkhs_decoding.id_score;
      rkhs_decoding.associated_pattern_3d = &function_dic_[size_num][i];
    }
  }
  // utils::printSpendHz(std::clock(), c_start, "A cluster decoding: ");

  // if (rkhs_decoding.template_points_3d.cols() > 1e3) {
  //     cout << "============" << endl;
  //     cout << "num_points: " << rkhs_decoding.template_points_3d.cols() <<
  //     endl;; cout << "id:" << rkhs_decoding.id << endl;
  //     // cout << "id_score:" << rkhs_decoding.id_score << endl;
  //     cout << "id_score:" << id_score << endl;
  //     cout << "rotation:" << rkhs_decoding.rotation_angle << endl;
  //     cout << "File:" << rkhs_function_name_list_[size_num][rkhs_decoding.id]
  //     << endl;
  // }

  // utils::printVector(utils::convertEigenToSTDVector(rkhs_decoding.score));
  if (id_score < params_.min_rkhs_score) {
    status = 0;
    if (debug_info_) {
      RCLCPP_WARN_STREAM(get_logger(), "==== getCodeRKHS ====");
      RCLCPP_WARN_STREAM(get_logger(), "Size number: " << size_num);
      RCLCPP_WARN_STREAM(get_logger(), "Score is too small: " << id_score);
      RCLCPP_WARN_STREAM(get_logger(), "Status: " << status);
      RCLCPP_WARN_STREAM(get_logger(), "========================");
    }
  } else {
    status = 1;
    if (debug_info_) {
      RCLCPP_DEBUG_STREAM(get_logger(), "==== getCodeRKHS ====");
      RCLCPP_DEBUG_STREAM(get_logger(), "Size number: " << size_num);
      RCLCPP_DEBUG_STREAM(get_logger(), "num_points: " << rkhs_decoding.template_points_3d.cols());
      RCLCPP_DEBUG_STREAM(get_logger(), "id: " << rkhs_decoding.id);
      RCLCPP_DEBUG_STREAM(get_logger(), "id_score: " << rkhs_decoding.id_score);
      RCLCPP_DEBUG_STREAM(get_logger(), "rotation: " << rkhs_decoding.rotation_angle);
      RCLCPP_DEBUG_STREAM(get_logger(), "file: " << rkhs_function_name_list_[size_num][rkhs_decoding.id]);
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << status);
      RCLCPP_DEBUG_STREAM(get_logger(), "========================");
    }
  }

  return status;
}

/* [Payload decoding]
 * A function to decode payload with different means
 * 0: Naive decoding
 * 1: Weighted Gaussian
 * 2: RKHS
 * 3: Deep learning
 * 4: ?!
 */
bool LidarTag::decodePayload(ClusterFamily_t & cluster)
{
  string code("");
  bool valid_tag = true;
  string msg;
  if (!params_.use_intensity_channel) {
    cluster.cluster_id = -1;
  }
  else if (decode_method_ == 0) {  // Naive decoder
    LidarTag::getCodeNaive(code, cluster.payload);
  } else if (decode_method_ == 1) {  // Weighted Gaussian
    int status = LidarTag::getCodeWeightedGaussian(
      code, cluster.pose_tag_to_lidar,
      cluster.payload_without_boundary,  // size of actual payload
      cluster.average, cluster.payload, cluster.payload_boundary_ptr);
    if (status == -1) {
      valid_tag = false;
      result_statistics_.cluster_removal.decoder_not_return++;
      cluster.valid = 0;
      msg = "Not enough return";
    } else if (status == -2) {
      valid_tag = false;
      result_statistics_.cluster_removal.decoder_fail_corner++;
      cluster.valid = 0;
      msg = "Fail corner detection";
    }
  } else if (decode_method_ == 2) {  // RKHS
    int status = LidarTag::getCodeRKHS(cluster.rkhs_decoding, cluster.tag_size);
    if (status == 1) {
      cluster.cluster_id = cluster.rkhs_decoding.id;
    } else {
      valid_tag = false;
      cluster.valid = 0;
      cluster.cluster_id = -1;
      result_statistics_.cluster_removal.decoding_failure++;
    }
  }

  if (params_.use_intensity_channel && (decode_method_ == 0 || decode_method_ == 1)) {
    if (valid_tag) {
      uint64_t r_code = stoull(code, nullptr, 2);
      BipedAprilLab::QuickDecodeCodeword(tf, r_code, &cluster.entry);
      cluster.cluster_id = cluster.entry.id;
      RCLCPP_INFO_STREAM(get_logger(), "id: " << cluster.entry.id);
      RCLCPP_DEBUG_STREAM(get_logger(), "hamming: " << cluster.entry.hamming);
      RCLCPP_DEBUG_STREAM(get_logger(), "rotation: " << cluster.entry.rotation);
    } else {
      // too big, return as an invalid tag
      code = "1111111111111111UL";
      RCLCPP_DEBUG_STREAM(get_logger(), "\nCODE: " << code);
      uint64_t r_code = stoull(code, nullptr, 2);
      BipedAprilLab::QuickDecodeCodeword(tf, r_code, &cluster.entry);
      cluster.cluster_id = 8888;
      RCLCPP_DEBUG_STREAM(get_logger(), "id: " << cluster.cluster_id);
      RCLCPP_DEBUG_STREAM(get_logger(), "hamming: " << cluster.entry.hamming);
      RCLCPP_DEBUG_STREAM(get_logger(), "rotation: " << cluster.entry.rotation);
    }
  }

  return valid_tag;
}

/* [Function Decoder]
 * A function to load continuous functions from a pre-computed point cloud
 */
void LidarTag::initFunctionDecoder()
{
  // prepare to rotate functions
  Eigen::Vector3f trans_v = Eigen::Vector3f::Zero(3);
  Eigen::Vector3f rot_v1;
  rot_v1 << 90, 0, 0;
  Eigen::Matrix4f H1 = utils::computeTransformation(rot_v1, trans_v);
  // cout << "H1: \n" << H1 << endl;

  Eigen::Vector3f rot_v2;
  rot_v2 << 180, 0, 0;
  Eigen::Matrix4f H2 = utils::computeTransformation(rot_v2, trans_v);
  // cout << "H2: \n" << H2 << endl;

  Eigen::Vector3f rot_v3;
  rot_v3 << 270, 0, 0;
  Eigen::Matrix4f H3 = utils::computeTransformation(rot_v3, trans_v);
  // cout << "H3: \n" << H3 << endl;
  function_dic_.resize(num_tag_sizes_);
  rkhs_function_name_list_.resize(num_tag_sizes_);
  std::vector<std::string> test;

  for (int tag_size = 0; tag_size < num_tag_sizes_; ++tag_size) {
    // cout << "path:" << library_path_ +  std::to_string(tag_size) + "/"
    //     << endl;
    std::string cur_path = library_path_ + std::to_string(tag_size) + "/";
    utils::readDirectory(cur_path, rkhs_function_name_list_[tag_size]);
    // utils::printVector(rkhs_function_name_list_[tag_size]);
    // utils::readDirectory(
    //         library_path_ +  std::to_string(tag_size) + "/", test);
    // utils::printVector(test);
    // consider four rotations
    int num_codes = std::min((int) rkhs_function_name_list_[tag_size].size(), num_codes_);
    std::vector<Eigen::MatrixXf> function_dic(num_codes * 4);
    // std::vector<Eigen::MatrixXf> function_dic_xyz(num_codes * 4);
    // std::vector<Eigen::MatrixXf> function_dic_feat(num_codes * 4);

    for (int i = 0, func_ind = 0; i < num_codes * 4; i += 4, ++func_ind) {
      // cout << "(i, ind) = (" << i << ", " << func_ind << ")" << endl;
      function_dic[i] =
        utils::loadCSV<Eigen::MatrixXf>(cur_path + rkhs_function_name_list_[tag_size][func_ind]);
      // rotate 90
      function_dic[i + 1] = H1 * utils::convertXYZIToHomogeneous(function_dic[i]);

      // rotate 180
      function_dic[i + 2] = H2 * utils::convertXYZIToHomogeneous(function_dic[i]);

      // rotate 270
      function_dic[i + 3] = H3 * utils::convertXYZIToHomogeneous(function_dic[i]);

      // split points and features for kdtree
      // _function_dic_xyz[i] = function_dic[i].topRows

      cout << "function loaded--" << cur_path + rkhs_function_name_list_[tag_size][func_ind]
           << endl;

      // if (i == 0) {
      //     cout << "tag0: \n" << function_dic[i] << endl;
      //     cout << "tag_h: \n" <<
      //     utils::convertXYZIToHomogeneous(function_dic[i]) << endl; cout <<
      //     "H1 * tag_h: \n" << H1 *
      //     utils::convertXYZIToHomogeneous(function_dic[i]) << endl;
      // }
    }
    function_dic_[tag_size] = function_dic;
    cout << "size: " << function_dic_[tag_size].size() << endl;
  }
  // _function_dic.resize(num_codes_); // consider four rotations
  // for (int i = 0, func_ind = 0; i < num_codes_ ; ++i) {
  //     // cout << "(i, ind) = (" << i << ", " << func_ind << ")" << endl;
  //     _function_dic[i] = utils::loadCSV<Eigen::MatrixXf>(
  //             library_path_ + rkhs_function_name_list_[i]);
  //     cout << "function loaded--"
  //          << library_path_ + rkhs_function_name_list_[i] << endl;
  //     // if (i == 0) {
  //     //     cout << "tag0: \n" << _function_dic[i] << endl;
  //     //     cout << "tag_h: \n" <<
  //     utils::convertXYZIToHomogeneous(_function_dic[i]) << endl;
  //     //     cout << "H1 * tag_h: \n" << H1 *
  //     utils::convertXYZIToHomogeneous(_function_dic[i]) << endl;
  //     // }
  // }
}

/* [Decoder]
 * Create hash table of chosen tag family
 */
void LidarTag::initDecoder()
{
  string famname = "tag" + to_string(tag_family_) + "h" + to_string(tag_hamming_distance_);
  if (famname == "tag49h14")
    tf = tag49h14_create();
  else if (famname == "tag16h5")
    tf = tag16h5_create();
  else {
    cout << "[ERROR]" << endl;
    cout << "Unrecognized tag family name: " << famname << ". Use e.g. \"tag16h5\". " << endl;
    cout << "This is line " << __LINE__ << " of file " << __FILE__ << " (function " << __func__
         << ")" << endl;
    exit(0);
  }
  tf->black_border = black_border_;
  cout << "Preparing for tags: " << famname << endl;
  if (decode_method_ == 0 || decode_method_ == 1) {
    BipedAprilLab::QuickDecodeInit(tf, max_decode_hamming_);
  } else if (decode_method_ == 2) {
    LidarTag::initFunctionDecoder();
  }
}

/*
 *
 */
void LidarTag::testInitDecoder()
{
  uint64_t rcode = 0x0001f019cf1cc653UL;
  QuickDecodeEntry_t entry;
  BipedAprilLab::QuickDecodeCodeword(tf, rcode, &entry);
  cout << "code: " << entry.rcode << endl;
  cout << "id: " << entry.id << endl;
  cout << "hamming: " << entry.hamming << endl;
  cout << "rotation: " << entry.rotation << endl;
  exit(0);
}
}  // namespace BipedLab
