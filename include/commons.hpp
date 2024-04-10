#ifndef __COMMONS_SLAM_NODE_HPP__
#define __COMMONS_SLAM_NODE_HPP__

#include <tf2_eigen/tf2_eigen.hpp>
#include "rclcpp/rclcpp.hpp"

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/srv/empty.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"

#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Commons 
{
public:
    std::string map_frame_id;
    std::string pose_frame_id;
    std::string octomap_frame_id;
    tf2::Matrix3x3 tf_orb_to_ros;
    std::shared_ptr<rclcpp::Node> node;
    bool is_octomap_resetting = false;    
    int number_of_frames = 0;
    bool tracking_lost = true;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_points_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcloud_all;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client;

    Commons(std::shared_ptr<rclcpp::Node> p_node);

    void publish_tracking_img(const cv::Mat &image, const rclcpp::Time &current_frame_time);
    tf2::Transform from_orb_to_ros_tf_transform(cv::Mat transformation_mat);
    void publish_tf_transform(tf2::Transform tf_transform, std::string child_frame_id, rclcpp::Time current_frame_time);
    void publish_pose_stamped(tf2::Transform tf_transform, std::string child_frame_id, rclcpp::Time current_frame_time);
    void publish_keyframe_points(tf2::Transform tf,
                                    std::string child_frame_id,
                                    std::vector<ORB_SLAM3::MapPoint *> map_points,
                                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub, 
                                    const rclcpp::Time &current_frame_time);
    void publish_all_keyframes_points(vector<ORB_SLAM3::KeyFrame *> key_frames);
    void publish_all_map_points(const rclcpp::Time &current_frame_time, std::vector<ORB_SLAM3::MapPoint *> map_points);
    void process_tracking(ORB_SLAM3::System *m_SLAM, const Eigen::Matrix<float, 4, 4> matrix, const builtin_interfaces::msg::Time &timestamp);
};
#endif