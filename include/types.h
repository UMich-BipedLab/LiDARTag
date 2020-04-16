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

namespace BipedLab {
	typedef velodyne_pointcloud::PointXYZIR PointXYZRI;
	typedef struct QuickDecodeEntry {
		uint64_t rcode;   // the queried code
		uint16_t id;      // the tag id (a small integer)
		uint16_t hamming;  // how many errors corrected?
		uint16_t rotation; // number of rotations [0, 3]
	}QuickDecodeEntry_t;

	typedef struct QuickDecode {
		int nentries;
		QuickDecodeEntry_t *entries;
	}QuickDecode_t;

	typedef struct PayloadVoting{
		PointXYZRI *p;
		float weight;
		int cell;
		PointXYZRI centroid;
	}PayloadVoting_t;


	// velodyne_pointcloud::PointXYZIR operator+ (const PointXYZRI& p1, const PointXYZRI p2) {
	//         PointXYZRI tmp;
	//         tmp.x = p1.x + p2.x;
	//         tmp.y = p1.y + p2.y;
	//         tmp.z = p1.z + p2.z;
	//         tmp.intensity = p1.intensity + p2.intensity;
	//         return tmp;
	// };
	typedef struct MaxMin {
		int min;
		int average;
		int max;
	} MaxMin_t;

	// Structure for LiDAR system
	typedef struct LiDARSystem {
		std::vector<std::vector<int>> point_count_table; // point per ring  PointCountTable[Scan][ring]
		std::vector<MaxMin_t> max_min_table; // max min points in a scan
		std::vector<MaxMin_t> ring_average_table; // max, min, average points in a ring, examed through out a few seconds 
		double points_per_square_meter_at_one_meter; // TODO: only assume place the tag at dense-point area
		double beam_per_vertical_radian;
		double point_per_horizontal_radian;
	} LiDARSystem_t;

	// Struture for LiDAR PointCloud with index
	typedef struct LiDARPoints {
		PointXYZRI point;
		int index;
		double depth_gradient; // only take abs value due to uncertain direction 
		double intensity_gradient; // Also account for direction by knowing tag is white to black
		double threshold_intensity;
	} LiDARPoints_t;

	typedef struct TagLines{
		int upper_ring;
		int lower_ring;
		std::vector<LiDARPoints_t*> upper_line; // basically just a specific ring, just point to it should be fine
		std::vector<LiDARPoints_t*> lower_line; // same above
		std::vector<LiDARPoints_t*> left_line;   // same
		std::vector<LiDARPoints_t*> right_line;  // same above


		std::vector<LiDARPoints_t*> bottom_left; // basically just a specific ring, just point to it should be fine
		std::vector<LiDARPoints_t*> bottom_right; // same above
		std::vector<LiDARPoints_t*> top_left;   // same
		std::vector<LiDARPoints_t*> top_right;  // same above
	} TagLines_t;

	typedef struct TagBoundaries{
		int status; // 0 is up right, 1 is tilted
		std::vector<LiDARPoints_t*> line_one; // basically just a specific ring, just point to it should be fine
		std::vector<LiDARPoints_t*> line_two; // same above
		std::vector<LiDARPoints_t*> line_three;   // same
		std::vector<LiDARPoints_t*> line_four;  // same above
	} TagBoundaries_t;

	typedef struct Homogeneous{
		// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		float roll;
		float pitch;
		float yaw;
		Eigen::Matrix<float,3,1,Eigen::DontAlign> translation;
		Eigen::Matrix<float,3,3,Eigen::DontAlign> rotation;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> homogeneous;
	} Homogeneous_t;

	typedef struct Grid{
		float cx;
		float cz;
		float cy;
	} Grid_t;

	typedef struct ClusterFamily {
		// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		int cluster_id;
		int valid;
		PointXYZRI top_most_point;
		PointXYZRI bottom_most_point;

		PointXYZRI front_most_point;
		PointXYZRI back_most_point;

		PointXYZRI right_most_point;
		PointXYZRI left_most_point;

		PointXYZRI average; // Average point
		PointXYZRI max_intensity; // Maximux intensity point
		PointXYZRI min_intensity; // Minimum intensity point
		pcl::PointCloud<LiDARPoints_t> data; 

		std::vector<MaxMin_t> max_min_index_of_each_ring;  // to fill in points between end points in this cluster
		std::vector<std::vector<LiDARPoints_t*>> ordered_points_ptr;  // of the cluster (to find black margin of the tag)
		std::vector<double> accumulate_intensity_of_each_ring;  // to find the upper/lower lines of the tag
		TagLines_t tag_edges;  // store line segment from points
		TagBoundaries_t tag_boundaries;

		std::vector<LiDARPoints_t*> payload_right_boundary_ptr;  // of the cluster (to find black margin of the tag)
		std::vector<LiDARPoints_t*> payload_left_boundary_ptr;  // of the cluster (to find black margin of the tag)
		std::vector<LiDARPoints_t*> payload_boundary_ptr;  // of the cluster (to find black margin of the tag)

		pcl::PointCloud<LiDARPoints_t*> payload; // payload points with boundary
		int payload_without_boundary; // size of payload points without boundary
		// Eigen::Vector3f NormalVector; // Normal vectors of the payload
		Eigen::Matrix<float,3,1,Eigen::DontAlign> normal_vector;
		QuickDecodeEntry_t entry;
		Homogeneous_t pose;
		tf::Transform transform;



		/* VectorXf: 
		 *          point_on_line.x : the X coordinate of a point on the line
		 *          point_on_line.y : the Y coordinate of a point on the line
		 *          point_on_line.z : the Z coordinate of a point on the line
		 *          line_direction.x : the X coordinate of a line's direction
		 *          line_direction.y : the Y coordinate of a line's direction
		 *          line_direction.z : the Z coordinate of a line's direction
		 */
		std::vector<Eigen::VectorXf> line_coeff;  // Upper, left, bottom, right line (count-clockwise)
	} ClusterFamily_t;

	typedef struct GrizTagFamily {
		// How many codes are there in this tag family?
		uint32_t ncodes;

		// The codes in the family.
		uint64_t *codes;

		// how wide (in bit-sizes) is the black border? (usually 1)
		uint32_t black_border;

		// how many bits tall and wide is it? (e.g. 36bit tag ==> 6)
		uint32_t d;

		// minimum hamming distance between any two codes. (e.g. 36h11 => 11)
		uint32_t h;

		// a human-readable name, e.g., "tag36h11"
		char *name;

		// some detector implementations may preprocess codes in order to
		// accelerate decoding.  They put their data here. (Do not use the
		// same apriltag_family instance in more than one implementation)
		void *impl;
	}GrizTagFamily_t;

	typedef struct ClusterRemoval {
		int removed_by_point_check;
		int boundary_point_check;
		int no_edge_check;
		int minimum_return;
		int decode_fail;
		int decoder_not_return;
		int decoder_fail_corner;
	}ClusterRemoval_t;

	typedef struct Statistics {
		ClusterRemoval_t cluster_removal;
		int original_cluster_size;
		int remaining_cluster_size;
		int point_cloud_size;
		int edge_cloud_size;
	}Statistics_t;

	typedef struct Timing{
		// in us
		clock_t start_total_time; 
		clock_t start_computation_time;
		clock_t timing;

		double total_time;
		double computation_time;
		double edgingand_clustering_time;
		double to_pcl_vector_time;
		double point_check_time;
		double line_fitting_time;
		double payload_extraction_time;
		double normal_vector_time;
		double payload_decoding_time;
		double tag_to_robot_time;
	}Timing_t;

	typedef struct TestCluster {
		int flag;
		ClusterFamily_t new_cluster;
	}TestCluster_t;

	typedef struct Debug{
		std::vector<ClusterFamily_t*> point_check;
		std::vector<ClusterFamily_t*> boundary_point;
		std::vector<ClusterFamily_t*> no_edge;
		std::vector<ClusterFamily_t*> extract_payload;
	}Debug_t;
} // namespace
