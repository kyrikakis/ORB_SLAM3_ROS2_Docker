
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

#ifndef DISABLE_FLANN
#include <flann/flann.hpp>
typedef flann::Index<flann::L2<double> > FLANN;
typedef std::unique_ptr<FLANN> FLANN_;
typedef flann::Matrix<double> flannMatT;
typedef flann::Matrix<int> flannResultT;
typedef std::unique_ptr<flannMatT> flannMatT_;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// parameters
float scale_factor = 5;
float resize_factor = 5;
float cloud_max_x = 10;
float cloud_min_x = -10.0;
float cloud_max_z = 16;
float cloud_min_z = -5;
float free_thresh = 0.55;
float occupied_thresh = 0.50;
float thresh_diff = 0.01;
int visit_thresh = 0;
unsigned int use_local_counters = 0;
unsigned int use_gaussian_counters = 0;
bool use_boundary_detection = false;
bool use_height_thresholding = false;
int canny_thresh = 350;
bool show_camera_location = true;
unsigned int gaussian_kernel_size = 3;
int cam_radius = 2;
// no. of keyframes between successive goal messages that are published
unsigned int goal_gap = 20;
bool enable_goal_publishing = false;

#ifndef DISABLE_FLANN
double normal_thresh_deg = 0;
bool use_plane_normals = false;
double normal_thresh_y_rad=0.0;
std::vector<double> normal_angle_y;
FLANN_ flann_index;
#endif

float grid_max_x, grid_min_x,grid_max_z, grid_min_z;
cv::Mat global_occupied_counter, global_visit_counter;
cv::Mat local_occupied_counter, local_visit_counter;
cv::Mat local_map_pt_mask;
cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized;
cv::Mat grid_map_rgb;
cv::Mat gauss_kernel;
float norm_factor_x, norm_factor_z;
float norm_factor_x_us, norm_factor_z_us;
int h, w;
unsigned int n_kf_received;
bool loop_closure_being_processed = false;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_map;
nav_msgs::msg::OccupancyGrid grid_map_msg;
Eigen::Matrix4d transform_mat;

float kf_pos_x, kf_pos_z;
int kf_pos_grid_x, kf_pos_grid_z;
geometry_msgs::msg::Point kf_location;
geometry_msgs::msg::Quaternion kf_orientation;

std::shared_ptr<rclcpp::Node> node;

void ptCallback(const geometry_msgs::msg::PoseArray::ConstPtr& pts_and_pose);
void loopClosingCallback(const geometry_msgs::msg::PoseArray::ConstPtr& all_kf_and_pts);
void updateGridMap(const geometry_msgs::msg::PoseArray::ConstPtr& pts_and_pose);
void processMapPt(const geometry_msgs::msg::Point &curr_pt, cv::Mat &occupied, cv::Mat &visited, 
	cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z, unsigned int id);
void processMapPts(const std::vector<geometry_msgs::msg::Pose> &pts, unsigned int n_pts,
	unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);
void getGridMap();
void showGridMap(unsigned int id);
void resetGridMap(const geometry_msgs::msg::PoseArray::ConstPtr& all_kf_and_pts);
void printParams();

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("pcloud_to_map");

	node->declare_parameter("scale_factor", 5.0); 
    node->declare_parameter("resize_factor", 5.0);
    node->declare_parameter("cloud_max_x", 10.0);
    node->declare_parameter("cloud_min_x", -10.0);
    node->declare_parameter("cloud_max_z", 16.0);
    node->declare_parameter("cloud_min_z", -5.0);
    node->declare_parameter("free_thresh", 0.55);
    node->declare_parameter("occupied_thresh", 0.50);
    node->declare_parameter("thresh_diff", 0.01);
    node->declare_parameter("use_local_counters", 1);
    node->declare_parameter("visit_thresh", 5.0);
    node->declare_parameter("use_gaussian_counters", 0);
    node->declare_parameter("use_boundary_detection", false);
    node->declare_parameter("use_height_thresholding", false);
    scale_factor = node->get_parameter("scale_factor").as_double(); 
    resize_factor = node->get_parameter("resize_factor").as_double(); 
    cloud_max_x = node->get_parameter("cloud_max_x").as_double(); 
    cloud_min_x = node->get_parameter("cloud_min_x").as_double(); 
    cloud_max_z = node->get_parameter("cloud_max_z").as_double(); 
    cloud_min_z = node->get_parameter("cloud_min_z").as_double(); 
    free_thresh = node->get_parameter("free_thresh").as_double(); 
    occupied_thresh = node->get_parameter("occupied_thresh").as_double(); 
    thresh_diff = node->get_parameter("thresh_diff").as_double();
    use_local_counters = node->get_parameter("use_local_counters").as_int();
    visit_thresh = node->get_parameter("visit_thresh").as_double(); 
    use_gaussian_counters = node->get_parameter("use_gaussian_counters").as_int(); 
    use_boundary_detection = node->get_parameter("use_boundary_detection").as_bool(); 
    use_height_thresholding = node->get_parameter("use_height_thresholding").as_bool(); 
	printParams();

#ifndef DISABLE_FLANN
	if (normal_thresh_deg > 0 && normal_thresh_deg <= 90) {
		use_plane_normals = true;
		// threshold for angle with y axis = 90 - angle with xz plane
		normal_thresh_y_rad = (90 - normal_thresh_deg)*M_PI / 180.0;
		RCLCPP_INFO(node->get_logger(), "normal_thresh_y_rad: %f rad\n", normal_thresh_y_rad);
		flann_index.reset(new FLANN(flann::KDTreeIndexParams(6)));
	}
#endif

    grid_max_x = cloud_max_x*scale_factor;
	grid_min_x = cloud_min_x*scale_factor;
	grid_max_z = cloud_max_z*scale_factor;
	grid_min_z = cloud_min_z*scale_factor;
	RCLCPP_INFO(node->get_logger(), "grid_max: %f, %f\t grid_min: %f, %f\n", grid_max_x, grid_max_z, grid_min_x, grid_min_z);

	double grid_res_x = grid_max_x - grid_min_x, grid_res_z = grid_max_z - grid_min_z;

	h = grid_res_z;
	w = grid_res_x;
	RCLCPP_INFO(node->get_logger(), "grid_size: (%d, %d)\n", h, w);
	n_kf_received = 0;

	global_occupied_counter.create(h, w, CV_32FC1);
	global_visit_counter.create(h, w, CV_32FC1);
	global_occupied_counter.setTo(cv::Scalar(0));
	global_visit_counter.setTo(cv::Scalar(0));

	grid_map_msg.data.resize(h*w);
	grid_map_msg.info.width = w;
	grid_map_msg.info.height = h;
	grid_map_msg.info.resolution = 1.0/scale_factor;

	grid_map_int = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data()));

	grid_map.create(h, w, CV_32FC1);
	grid_map_thresh.create(h, w, CV_8UC1);
	grid_map_thresh_resized.create(h*resize_factor, w*resize_factor, CV_8UC1);
	grid_map_rgb.create(h*resize_factor, w*resize_factor, CV_8UC3);
	RCLCPP_INFO(node->get_logger(), "output_size: (%d, %d)\n", grid_map_thresh_resized.rows, grid_map_thresh_resized.cols);

	local_occupied_counter.create(h, w, CV_32FC1);
	local_visit_counter.create(h, w, CV_32FC1);
	local_map_pt_mask.create(h, w, CV_8UC1);

	gauss_kernel = cv::getGaussianKernel(gaussian_kernel_size, -1);

	norm_factor_x_us = float(cloud_max_x - cloud_min_x - 1) / float(cloud_max_x - cloud_min_x);
	norm_factor_z_us = float(cloud_max_z - cloud_min_z - 1) / float(cloud_max_z - cloud_min_z);

	norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
	norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);
	RCLCPP_INFO(node->get_logger(), "norm_factor_x: %f\n", norm_factor_x);
	RCLCPP_INFO(node->get_logger(), "norm_factor_z: %f\n", norm_factor_z);


    pub_grid_map = node->create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy_grid", 1000);

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr p_array_subscriber = 
	node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orbslam3/monocular/tracked_p_array",
        1000,
        &ptCallback);
	
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr map_reset_subscriber = 
	node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orbslam3/monocular/map_and_kf",
        1000,
        &loopClosingCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
}

void ptCallback(const geometry_msgs::msg::PoseArray::ConstPtr& pts_and_pose){

	if (loop_closure_being_processed){ return; }

	updateGridMap(pts_and_pose);
	grid_map_msg.info.map_load_time = pts_and_pose->header.stamp;
	grid_map_msg.header.stamp = pts_and_pose->header.stamp;
	// RCLCPP_INFO(node->get_logger(), "grid_map_msg.header.stamp: %i", grid_map_msg.header.stamp.sec);
	grid_map_msg.header.frame_id = "map";
	pub_grid_map->publish(grid_map_msg);
}

void loopClosingCallback(const geometry_msgs::msg::PoseArray::ConstPtr& all_kf_and_pts){
	loop_closure_being_processed = true;
	resetGridMap(all_kf_and_pts);
	loop_closure_being_processed = false;
}

void updateGridMap(const geometry_msgs::msg::PoseArray::ConstPtr& pts_and_pose){

	//geometry_msgs::Point min_pt, max_pt;
	//getMixMax(pts_and_pose, min_pt, max_pt);
	//RCLCPP_INFO(node->get_logger(), "max_pt: %f, %f\t min_pt: %f, %f\n", max_pt.x*scale_factor, max_pt.z*scale_factor, 
	//	min_pt.x*scale_factor, min_pt.z*scale_factor);

	//double grid_res_x = max_pt.x - min_pt.x, grid_res_z = max_pt.z - min_pt.z;

	//RCLCPP_INFO(node->get_logger(), "Received frame %u \n", pts_and_pose->header.seq);

	kf_location = pts_and_pose->poses[0].position;
	kf_orientation = pts_and_pose->poses[0].orientation;

	kf_pos_x = kf_location.x*scale_factor;
	kf_pos_z = kf_location.z*scale_factor;

	kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
	kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

	if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
		return;

	if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
		return;

	++n_kf_received;

	if (use_height_thresholding){
		Eigen::Vector4d kf_orientation_eig(kf_orientation.w, kf_orientation.x, kf_orientation.y, kf_orientation.z);
		kf_orientation_eig.array() /= kf_orientation_eig.norm();
		Eigen::Matrix3d keyframe_rotation = Eigen::Quaterniond(kf_orientation_eig).toRotationMatrix();
		Eigen::Vector3d keyframe_translation(kf_location.x*scale_factor, kf_location.y*scale_factor, kf_location.z*scale_factor);
		transform_mat.setIdentity();
		transform_mat.topLeftCorner<3, 3>() = keyframe_rotation.transpose();
		transform_mat.topRightCorner<3, 1>() = (-keyframe_rotation.transpose() * keyframe_translation);
	}
	unsigned int n_pts = pts_and_pose->poses.size() - 1;
	//RCLCPP_INFO(node->get_logger(), "Processing key frame %u and %u points\n",n_kf_received, n_pts);
	processMapPts(pts_and_pose->poses, n_pts, 1, kf_pos_grid_x, kf_pos_grid_z);

	getGridMap();
	showGridMap(1);
	//cout << endl << "Grid map saved!" << endl;
}

void processMapPt(const geometry_msgs::msg::Point &curr_pt, cv::Mat &occupied, cv::Mat &visited, 
	cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z, unsigned int pt_id) {
	float pt_pos_x = curr_pt.x*scale_factor;
	float pt_pos_z = curr_pt.z*scale_factor;

	int pt_pos_grid_x = int(floor((pt_pos_x - grid_min_x) * norm_factor_x));
	int pt_pos_grid_z = int(floor((pt_pos_z - grid_min_z) * norm_factor_z));

	if (pt_pos_grid_x < 0 || pt_pos_grid_x >= w)
		return;

	if (pt_pos_grid_z < 0 || pt_pos_grid_z >= h)
		return;
	bool is_ground_pt = false;
	bool is_in_horizontal_plane = false;
	if (use_height_thresholding){
		float pt_pos_y = curr_pt.y*scale_factor;
		Eigen::Vector4d transformed_point_location = transform_mat * Eigen::Vector4d(pt_pos_x, pt_pos_y, pt_pos_z, 1);
		double transformed_point_height = transformed_point_location[1] / transformed_point_location[3];
		is_ground_pt = transformed_point_height < 0;
	}
#ifndef DISABLE_FLANN
	if (use_plane_normals) {
		double normal_angle_y_rad = normal_angle_y[pt_id];
		is_in_horizontal_plane = normal_angle_y_rad < normal_thresh_y_rad;
	}
#endif
	if (is_ground_pt || is_in_horizontal_plane) {
		++visited.at<float>(pt_pos_grid_z, pt_pos_grid_x);
	}
	else {
		// Increment the occupency account of the grid cell where map point is located
		++occupied.at<float>(pt_pos_grid_z, pt_pos_grid_x);
		pt_mask.at<uchar>(pt_pos_grid_z, pt_pos_grid_x) = 255;
	}	

	//cout << "----------------------" << endl;
	//cout << okf_pos_grid_x << " " << okf_pos_grid_y << endl;

	// Get all grid cell that the line between keyframe and map point pass through
	int x0 = kf_pos_grid_x;
	int y0 = kf_pos_grid_z;
	int x1 = pt_pos_grid_x;
	int y1 = pt_pos_grid_z;
	bool steep = (abs(y1 - y0) > abs(x1 - x0));
	if (steep){
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1){
		swap(x0, x1);
		swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = abs(y1 - y0);
	double error = 0;
	double deltaerr = ((double)dy) / ((double)dx);
	int y = y0;
	int ystep = (y0 < y1) ? 1 : -1;
	for (int x = x0; x <= x1; ++x){
		if (steep) {
			++visited.at<float>(x, y);
		}
		else {
			++visited.at<float>(y, x);
		}
		error = error + deltaerr;
		if (error >= 0.5){
			y = y + ystep;
			error = error - 1.0;
		}
	}
}

void processMapPts(const std::vector<geometry_msgs::msg::Pose> &pts, unsigned int n_pts,
	unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z) {
	unsigned int end_id = start_id + n_pts;
#ifndef DISABLE_FLANN
	if (use_plane_normals) {
		cv::Mat cv_dataset(n_pts, 3, CV_64FC1);
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			cv_dataset.at<double>(pt_id - start_id, 0) = pts[pt_id].position.x;
			cv_dataset.at<double>(pt_id - start_id, 1) = pts[pt_id].position.y;
			cv_dataset.at<double>(pt_id - start_id, 2) = pts[pt_id].position.z;
		}
		//RCLCPP_INFO(node->get_logger(), "building FLANN index...\n");		
		flann_index->buildIndex(flannMatT((double *)(cv_dataset.data), n_pts, 3));
		normal_angle_y.resize(n_pts);
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			double pt[3], dists[3];
			pt[0] = pts[pt_id].position.x;
			pt[1] = pts[pt_id].position.y;
			pt[2] = pts[pt_id].position.z;

			int results[3];
			flannMatT flann_query(pt, 1, 3);
			flannMatT flann_dists(dists, 1, 3);
			flannResultT flann_result(results, 3, 1);
			flann_index->knnSearch(flann_query, flann_result, flann_dists, 3, flann::SearchParams());
			Eigen::Matrix3d nearest_pts;
			//RCLCPP_INFO(node->get_logger(), "Point %d: %f, %f, %f\n", pt_id - start_id, pt[0], pt[1], pt[2]);
			for (unsigned int i = 0; i < 3; ++i){
				nearest_pts(0, i) = cv_dataset.at<double>(results[i], 0);
				nearest_pts(1, i) = cv_dataset.at<double>(results[i], 1);
				nearest_pts(2, i) = cv_dataset.at<double>(results[i], 2);
				//RCLCPP_INFO(node->get_logger(), "Nearest Point %d: %f, %f, %f\n", results[i], nearest_pts(0, i), nearest_pts(1, i), nearest_pts(2, i));
			}
			Eigen::Vector3d centroid = nearest_pts.rowwise().mean();
			//RCLCPP_INFO(node->get_logger(), "centroid %f, %f, %f\n", centroid[0], centroid[1], centroid[2]);
			Eigen::Matrix3d centered_pts = nearest_pts.colwise() - centroid;
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(centered_pts, Eigen::ComputeThinU | Eigen::ComputeThinV);
			int n_cols = svd.matrixU().cols();
			// left singular vector corresponding to the smallest singular value
			Eigen::Vector3d normal_direction = svd.matrixU().col(n_cols - 1);
			//RCLCPP_INFO(node->get_logger(), "normal_direction %f, %f, %f\n", normal_direction[0], normal_direction[1], normal_direction[2]);
			// angle to y axis
			normal_angle_y[pt_id-start_id] = acos(normal_direction[1]);
			if (normal_angle_y[pt_id - start_id ]> (M_PI / 2.0)) {
				normal_angle_y[pt_id - start_id] = M_PI - normal_angle_y[pt_id - start_id];
			}
			//RCLCPP_INFO(node->get_logger(), "normal angle: %f rad or %f deg\n", normal_angle_y[pt_id - start_id], normal_angle_y[pt_id - start_id]*180.0/M_PI);
			//RCLCPP_INFO(node->get_logger(), "\n\n");
		}
	}
#endif
	if (use_local_counters) {
		local_map_pt_mask.setTo(0);
		local_occupied_counter.setTo(0);
		local_visit_counter.setTo(0);
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			processMapPt(pts[pt_id].position, local_occupied_counter, local_visit_counter,
				local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z, pt_id - start_id);
		}
		for (int row = 0; row < h; ++row){
			for (int col = 0; col < w; ++col){
				if (local_map_pt_mask.at<uchar>(row, col) == 0) {
					local_occupied_counter.at<float>(row, col) = 0;
				}
				else {
					local_occupied_counter.at<float>(row, col) = local_visit_counter.at<float>(row, col);
				}
			}
		}
		if (use_gaussian_counters) {
			cv::filter2D(local_occupied_counter, local_occupied_counter, CV_32F, gauss_kernel);
			cv::filter2D(local_visit_counter, local_visit_counter, CV_32F, gauss_kernel);
		}
		global_occupied_counter += local_occupied_counter;
		global_visit_counter += local_visit_counter;
		//cout << "local_occupied_counter: \n" << local_occupied_counter << "\n";
		//cout << "global_occupied_counter: \n" << global_occupied_counter << "\n";
		//cout << "local_visit_counter: \n" << local_visit_counter << "\n";
		//cout << "global_visit_counter: \n" << global_visit_counter << "\n";
	}
	else {
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			processMapPt(pts[pt_id].position, global_occupied_counter, global_visit_counter,
				local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z, pt_id - start_id);
		}
	}
}


void resetGridMap(const geometry_msgs::msg::PoseArray::ConstPtr& all_kf_and_pts){
	global_visit_counter.setTo(0);
	global_occupied_counter.setTo(0);

	unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
	if ((unsigned int) (all_kf_and_pts->poses[0].position.y) != n_kf ||
		(unsigned int) (all_kf_and_pts->poses[0].position.z) != n_kf) {
		RCLCPP_INFO(node->get_logger(), "resetGridMap :: Unexpected formatting in the keyframe count element\n");
		return;
	}
	RCLCPP_INFO(node->get_logger(), "Resetting grid map with %d key frames\n", n_kf);
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	unsigned int id = 0;
	for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id){
		const geometry_msgs::msg::Point &kf_location = all_kf_and_pts->poses[++id].position;
		//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		if ((unsigned int)(all_kf_and_pts->poses[id].position.y) != n_pts ||
			(unsigned int)(all_kf_and_pts->poses[id].position.z) != n_pts) {
			RCLCPP_INFO(node->get_logger(), "resetGridMap :: Unexpected formatting in the point count element for keyframe %d\n", kf_id);
			return;
		}
		float kf_pos_x = kf_location.x*scale_factor;
		float kf_pos_z = kf_location.z*scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
		int kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

		if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
			continue;

		if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
			continue;

		if (id + n_pts >= all_kf_and_pts->poses.size()) {
			RCLCPP_INFO(node->get_logger(), "resetGridMap :: Unexpected end of the input array while processing keyframe %u with %u points: only %u out of %u elements found\n",
				kf_id, n_pts, all_kf_and_pts->poses.size(), id + n_pts);
			return;
		}
		processMapPts(all_kf_and_pts->poses, n_pts, id + 1, kf_pos_grid_x, kf_pos_grid_z);
		id += n_pts;
	}	
	getGridMap();
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
	RCLCPP_INFO(node->get_logger(), "Done. Time taken: %f secs\n", ttrack);
	pub_grid_map->publish(grid_map_msg);
	showGridMap(1);
}

void getGridMap() {
	for (int row = 0; row < h; ++row){
		for (int col = 0; col < w; ++col){
			int visits = global_visit_counter.at<int>(row, col);
			int occupieds = global_occupied_counter.at<int>(row, col);

			if (visits <= visit_thresh){
				grid_map.at<float>(row, col) = 0.5;
			}
			else {
				grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
			}
			if (grid_map.at<float>(row, col) >= free_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 255;
			}
			else if (grid_map.at<float>(row, col) < free_thresh && grid_map.at<float>(row, col) >= occupied_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 128;
			}
			else {
				grid_map_thresh.at<uchar>(row, col) = 0;
			}
			grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
		}
	}
	cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
}

void showGridMap(unsigned int id) {
	cv::imshow("grid_map_msg", cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data())));
	cv::imshow("grid_map_thresh_resized", grid_map_thresh_resized);
	//cv::imshow("grid_map", grid_map);
	int key = cv::waitKey(1) % 256;
	if (key == 27) {
		cv::destroyAllWindows();
   		rclcpp::shutdown();
		exit(0);
	}
	else if (key == 'f') {
		free_thresh -= thresh_diff;
		if (free_thresh <= occupied_thresh){ free_thresh = occupied_thresh + thresh_diff; }

		RCLCPP_INFO(node->get_logger(), "Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'F') {
		free_thresh += thresh_diff;
		if (free_thresh > 1){ free_thresh = 1; }
		RCLCPP_INFO(node->get_logger(), "Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'o') {
		occupied_thresh -= thresh_diff;
		if (free_thresh < 0){ free_thresh = 0; }
		RCLCPP_INFO(node->get_logger(), "Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 'O') {
		occupied_thresh += thresh_diff;
		if (occupied_thresh >= free_thresh){ occupied_thresh = free_thresh - thresh_diff; }
		RCLCPP_INFO(node->get_logger(), "Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 's') {
		//saveMap(id);
	}
}

void printParams() {
	RCLCPP_INFO(node->get_logger(), "Using params:\n");
	RCLCPP_INFO(node->get_logger(), "scale_factor: %f\n", scale_factor);
	RCLCPP_INFO(node->get_logger(), "resize_factor: %f\n", resize_factor);
	RCLCPP_INFO(node->get_logger(), "cloud_max: %f, %f\t cloud_min: %f, %f\n", cloud_max_x, cloud_max_z, cloud_min_x, cloud_min_z);
	//RCLCPP_INFO(node->get_logger(), "cloud_min: %f, %f\n", cloud_min_x, cloud_min_z);
	RCLCPP_INFO(node->get_logger(), "free_thresh: %f\n", free_thresh);
	RCLCPP_INFO(node->get_logger(), "occupied_thresh: %f\n", occupied_thresh);
	RCLCPP_INFO(node->get_logger(), "use_local_counters: %d\n", use_local_counters);
	RCLCPP_INFO(node->get_logger(), "visit_thresh: %d\n", visit_thresh);
	RCLCPP_INFO(node->get_logger(), "use_gaussian_counters: %d\n", use_gaussian_counters);
	RCLCPP_INFO(node->get_logger(), "use_boundary_detection: %d\n", use_boundary_detection);
	RCLCPP_INFO(node->get_logger(), "use_height_thresholding: %d\n", use_height_thresholding);
#ifndef DISABLE_FLANN
	RCLCPP_INFO(node->get_logger(), "normal_thresh_deg: %f\n", normal_thresh_deg);
#endif
	RCLCPP_INFO(node->get_logger(), "canny_thresh: %d\n", canny_thresh);
	RCLCPP_INFO(node->get_logger(), "enable_goal_publishing: %d\n", enable_goal_publishing);
	RCLCPP_INFO(node->get_logger(), "show_camera_location: %d\n", show_camera_location);
	RCLCPP_INFO(node->get_logger(), "gaussian_kernel_size: %d\n", gaussian_kernel_size);
	RCLCPP_INFO(node->get_logger(), "cam_radius: %d\n", cam_radius);
}