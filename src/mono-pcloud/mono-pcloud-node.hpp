#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <std_srvs/srv/empty.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include "commons.hpp"

#include <ctime>

class MonoPcloudNode
{
public:
    MonoPcloudNode(ORB_SLAM3::System *pSLAM, std::shared_ptr<rclcpp::Node> node);

    ~MonoPcloudNode();

    using ImageMsg = sensor_msgs::msg::Image;
    void GrabImage(const ImageMsg::SharedPtr msg);

    ORB_SLAM3::System *m_SLAM;
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<Commons> commons;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client;

    int number_of_frames = 0;
    bool tracking_lost = true;
    bool is_octomap_resetting = false;
};

#endif
