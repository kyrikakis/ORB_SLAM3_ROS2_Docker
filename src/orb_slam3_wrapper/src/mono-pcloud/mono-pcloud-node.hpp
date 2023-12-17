#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

#include <ctime>

class MonoPcloudNode : public rclcpp::Node
{
public:
    MonoPcloudNode(ORB_SLAM3::System *pSLAM);

    ~MonoPcloudNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    std::string map_frame_id;
    std::string pose_frame_id;
    tf2::Matrix3x3 tf_orb_to_ros;

    tf2::Transform from_orb_to_ros_tf_transform(cv::Mat transformation_mat);
    void publish_ros_pose_tf(cv::Mat Tcw, rclcpp::Time current_frame_time);
    void publish_tf_transform(tf2::Transform tf_transform, rclcpp::Time current_frame_time);
    void publish_pose_stamped(tf2::Transform tf_transform, rclcpp::Time current_frame_time);
    sensor_msgs::msg::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points,
                                                                  rclcpp::Time current_frame_time);
    void publish_ros_tracking_img(const cv::Mat &image, const rclcpp::Time &current_frame_time);
    void publish_ros_tracking_mappoints(std::vector<ORB_SLAM3::MapPoint *> map_points, const rclcpp::Time &current_frame_time);
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System *m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub;
    image_transport::Publisher rendered_image_pub;
};

#endif
