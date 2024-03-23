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

// thirdparty
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

void SetGridParameter ();
void CreateCvMat (const unsigned int h, const unsigned int w);
void SetGridOrigin (nav_msgs::msg::OccupancyGrid &grid, float &grid_min_x, float &grid_min_y);
void SingleCallback (const geometry_msgs::msg::PoseArray::ConstPtr& kf_pts_array);
void UpdateGridMap (const geometry_msgs::msg::PoseArray::ConstPtr& kf_pts_array);
void ProcessPts (const std::vector<geometry_msgs::msg::Pose> &pts, unsigned int n_pts, unsigned int start_id);
void ProcessPt (const geometry_msgs::msg::Point &curr_pt, cv::Mat &occupied, cv::Mat &visited, cv::Mat &pt_mask);
void GetGridMap ();
void PublishTopic (const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &pub, nav_msgs::msg::OccupancyGrid &msg);

std::shared_ptr<rclcpp::Node> node;
bool loop_closure_being_processed_ = false;

float scale_factor_;
// x, y
float norm_fac_[2];
unsigned int h_, w_;

unsigned int n_kf_received_ = 0;

float cloud_lim_[4];
float free_thresh_;
float occupied_thresh_;
unsigned int visit_thresh_;
bool use_local_counter_ = false;
float thresh_diff_ = 0.01;

float grid_lim_[4];
cv::Mat global_occupied_counter_, global_visit_counter_;
cv::Mat local_occupied_counter_, local_visit_counter_;
cv::Mat local_map_pt_mask_;
cv::Mat grid_map_;                 // [0.0, 1.0], to compute free/occupied probability
cv::Mat grid_map_int_;             // [0, 100], free/occupied probability normalized to integer 100
cv::Mat grid_map_thresh_;          // for visualization, uint8 : 0, 128, 255 -> black, gray, white
float kf_pos_x_, kf_pos_y_;
int kf_pos_grid_x_, kf_pos_grid_y_;

nav_msgs::msg::OccupancyGrid grid_map_cost_msg_;
nav_msgs::msg::OccupancyGrid grid_map_visual_msg_;

std::string all_kfs_pts_topic_param_;
std::string single_kf_pts_topic_param_;
std::string frame_id_param_ = "map";
bool publish_grid_map_cost_param_ = true;
bool publish_grid_map_visual_param_ = true;

unsigned int map_count_ = 0;

rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_visual_publisher_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_cost_publisher_;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("project_map");

    node->declare_parameter("free_thresh", 0.6); 
    node->declare_parameter("occupied_thresh", 0.4);
    node->declare_parameter("visit_thresh", 0);
    node->declare_parameter("scale_factor", 100.0);
    node->declare_parameter("cloud_min_x", -100.0);
    node->declare_parameter("cloud_max_x", 100.0);
    node->declare_parameter("cloud_min_y", -100.0);
    node->declare_parameter("cloud_max_y", 100.0);
    free_thresh_ = node->get_parameter("free_thresh").as_double();
    occupied_thresh_= node->get_parameter("occupied_thresh").as_double();
    visit_thresh_ = node->get_parameter("visit_thresh").as_int();
    scale_factor_ = node->get_parameter("scale_factor").as_double();
    cloud_lim_[0] = node->get_parameter("cloud_min_x").as_double(); 
    cloud_lim_[1] = node->get_parameter("cloud_max_x").as_double(); 
    cloud_lim_[2] = node->get_parameter("cloud_min_y").as_double(); 
    cloud_lim_[3] = node->get_parameter("cloud_max_y").as_double();

    if (publish_grid_map_cost_param_)
    { grid_map_cost_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy_grid_cost", 1000); }
    if (publish_grid_map_visual_param_)
    { grid_map_visual_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy_grid_visual", 1000); }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr p_array_subscriber = 
    node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orbslam3/monocular/tracked_p_array",
        1000,
        &SingleCallback);
	
	// rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr map_reset_subscriber = 
	// node->create_subscription<geometry_msgs::msg::PoseArray>(
    //     "/orbslam3/monocular/map_and_kf",
    //     1000,updateWithPointCloud
    //     &AllCallback);

    SetGridParameter();
    RCLCPP_INFO(node->get_logger(), "project_map node initialized height: %i, width: %i", h_, w_);
    CreateCvMat(h_, w_);

    rclcpp::spin(node);
    rclcpp::shutdown();
}

void SetGridParameter ()
{
    for (auto i=0; i<4; i++)
        grid_lim_[i] = cloud_lim_[i] * scale_factor_;

    h_ = grid_lim_[1] - grid_lim_[0];
    w_ = grid_lim_[3] - grid_lim_[2];

    norm_fac_[0] = float(h_ - 1) / float(h_);
    norm_fac_[1] = float(w_ - 1) / float(w_);
}

void CreateCvMat (const unsigned int h, const unsigned int w)
{
    global_occupied_counter_.create(h, w, CV_32SC1);
    global_visit_counter_.create(h, w, CV_32SC1);
    global_occupied_counter_.setTo(cv::Scalar(0));
    global_visit_counter_.setTo(cv::Scalar(0));

    grid_map_.create(h, w, CV_32FC1);
    if (publish_grid_map_cost_param_)
    {
        grid_map_cost_msg_.data.resize(h*w);
        grid_map_cost_msg_.info.width = w;
        grid_map_cost_msg_.info.height = h;
        grid_map_cost_msg_.header.frame_id = "map";
        grid_map_cost_msg_.info.resolution = 1.0/scale_factor_;
        SetGridOrigin (grid_map_cost_msg_, grid_lim_ [0], grid_lim_[2]);
        grid_map_int_ = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_cost_msg_.data.data()));
    }
    if (publish_grid_map_visual_param_)
    {
        grid_map_visual_msg_.data.resize(h*w);
        grid_map_visual_msg_.info.width = w;
        grid_map_visual_msg_.info.height = h;
        grid_map_visual_msg_.header.frame_id = "map";
        grid_map_visual_msg_.info.resolution = 1.0/scale_factor_;
        SetGridOrigin (grid_map_visual_msg_, grid_lim_[0], grid_lim_[2]);
        grid_map_thresh_ = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_visual_msg_.data.data()));
    }

    //grid_map_thresh_resized_.create(h * resize_fac_, w * resize_fac_, CV_8UC1);

    local_occupied_counter_.create(h, w, CV_32SC1);
    local_visit_counter_.create(h, w, CV_32SC1);
    local_map_pt_mask_.create(h, w, CV_8UC1);
}

void SetGridOrigin (nav_msgs::msg::OccupancyGrid &grid, float &grid_min_x, float &grid_min_y)
{
    grid.info.origin.orientation.x = 1;
    grid.info.origin.orientation.y = 0;
    grid.info.origin.orientation.z = 0;
    grid.info.origin.orientation.w = 0;
    grid.info.origin.position.x = grid_min_x * grid.info.resolution ;
    grid.info.origin.position.y = grid_min_y * grid.info.resolution ;
    grid.info.origin.position.z = 0;
}

void SingleCallback(const geometry_msgs::msg::PoseArray::ConstPtr& kf_pts_array)
{
    // RCLCPP_INFO(node->get_logger(), "received poses: %i\n", kf_pts_array->poses.size());
    // loop closure takes time
    if (loop_closure_being_processed_) return;

    UpdateGridMap( kf_pts_array );
    if (publish_grid_map_visual_param_)
    { PublishTopic (grid_map_visual_publisher_, grid_map_visual_msg_); }
    if (publish_grid_map_cost_param_)
    { PublishTopic (grid_map_cost_publisher_, grid_map_cost_msg_); }
    map_count_++;
    // RCLCPP_INFO(node->get_logger(), "published in Single Callback: %i\n", map_count_);
}

void UpdateGridMap (const geometry_msgs::msg::PoseArray::ConstPtr& kf_pts_array)
{
    const geometry_msgs::msg::Point &kf_position = kf_pts_array->poses[0].position;
    kf_pos_x_ = kf_position.x * scale_factor_;
    kf_pos_y_ = kf_position.y * scale_factor_;
    kf_pos_grid_x_ = int( floor( (kf_pos_x_ - grid_lim_[0]) * norm_fac_[0] ) );
    kf_pos_grid_y_ = int( floor( (kf_pos_y_ - grid_lim_[2]) * norm_fac_[1] ) );

    if (kf_pos_grid_x_ < 0 || kf_pos_grid_x_ >= h_) return;
    if (kf_pos_grid_y_ < 0 || kf_pos_grid_y_ >= w_) return;

    ++n_kf_received_;

    unsigned int n_pts = kf_pts_array->poses.size() - 1;
    ProcessPts(kf_pts_array->poses, n_pts, 1);
    GetGridMap();
}

void ProcessPts (const std::vector<geometry_msgs::msg::Pose> &pts, unsigned int n_pts, unsigned int start_id)
{
    unsigned int end_id = start_id + n_pts;

    if (use_local_counter_)
    {
        local_map_pt_mask_.setTo(0);
        local_occupied_counter_.setTo(0);
        local_visit_counter_.setTo(0);

        for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
            ProcessPt(pts[pt_id].position, local_occupied_counter_, local_visit_counter_, local_map_pt_mask_);
        for (int row = 0; row < h_; ++row)
        {
            for (int col = 0; col < w_; ++col)
            {
                if (local_map_pt_mask_.at<uchar>(row, col) == 0)
                {
                    local_occupied_counter_.at<int>(row, col) = 0;
                }
                else
                {
                    local_occupied_counter_.at<int>(row, col) = local_visit_counter_.at<int>(row, col);
                }
            }
        }
        global_occupied_counter_ += local_occupied_counter_;
        global_visit_counter_ += local_visit_counter_;
    }
    else
    {
        for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
            ProcessPt(pts[pt_id].position, global_occupied_counter_, global_visit_counter_, local_map_pt_mask_);
    }
}

void ProcessPt (const geometry_msgs::msg::Point &curr_pt, cv::Mat &occupied, cv::Mat &visited, cv::Mat &pt_mask)
{
    float pt_pos_x = curr_pt.x * scale_factor_;
    float pt_pos_y = curr_pt.y * scale_factor_;

    int pt_pos_grid_x = int(floor((pt_pos_x - grid_lim_[0]) * norm_fac_[0]));
    int pt_pos_grid_y = int(floor((pt_pos_y - grid_lim_[2]) * norm_fac_[1]));

    if ( pt_pos_grid_x < 0 || pt_pos_grid_x >= h_ ) return;
    if ( pt_pos_grid_y < 0 || pt_pos_grid_y >= w_ ) return;

    ++occupied.at<int>(pt_pos_grid_x, pt_pos_grid_y);
    pt_mask.at<uchar>(pt_pos_grid_x, pt_pos_grid_y) = 255;

    int x0 = kf_pos_grid_x_;
    int y0 = kf_pos_grid_y_;
    int x1 = pt_pos_grid_x;
    int y1 = pt_pos_grid_y;

    bool steep = ( abs(y1-y0) > abs(x1-x0) );
    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dx = x1 - x0;
    int dy = abs(y1 - y0);
    double error = 0;
    double deltaerr = ((double)dy) / ((double)dx);
    int y = y0;
    int ystep = (y0 < y1) ? 1 : -1;
    for (int x = x0; x <= x1; ++x)
    {
        if (steep)
        {
            ++visited.at<int>(y, x);
        }
        else {
            ++visited.at<int>(x, y);
        }
        error = error + deltaerr;

        if (error >= 0.5){
            y += ystep;
            error -= - 1.0;
        }
    }
}

void GetGridMap ()
{
    for (int row = 0; row < h_; ++row)
    {
        for (int col = 0; col < w_; ++col)
        {
            int visits    = global_visit_counter_.at<int>(row, col);
            int occupieds = global_occupied_counter_.at<int>(row, col);

            grid_map_.at<float>(row, col) = (visits <= visit_thresh_) ? 0.5 :  (1.0 - float(occupieds / visits));
            if (publish_grid_map_visual_param_)
            {
                if (grid_map_.at<float>(row, col) >= free_thresh_)
                { grid_map_thresh_.at<uchar>(row, col) = 255; }
                else if (grid_map_.at<float>(row, col) < occupied_thresh_)
                { grid_map_thresh_.at<uchar>(row, col) = 0; }
                else
                { grid_map_thresh_.at<uchar>(row, col) = 128; }
            }
            if (publish_grid_map_cost_param_)
            { grid_map_int_.at<char>(row, col) = (1 - grid_map_.at<float>(row, col)) * 100; }

        }
    }
    RCLCPP_INFO(node->get_logger(), "found vistis: %i\n", global_visit_counter_.size());
    // cv::resize(grid_map_thresh_, grid_map_thresh_resized_, grid_map_thresh_resized_.size());
}

void PublishTopic (const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &pub, nav_msgs::msg::OccupancyGrid &msg)
{
    pub->publish( msg );
}