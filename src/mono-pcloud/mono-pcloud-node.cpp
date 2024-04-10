#include "mono-pcloud-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

MonoPcloudNode::MonoPcloudNode(ORB_SLAM3::System *pSLAM, std::shared_ptr<rclcpp::Node> p_node)
{
    m_SLAM = pSLAM;
    node = p_node;
    commons = std::make_shared<Commons>(node);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(qos_profile));

    m_image_subscriber = node->create_subscription<ImageMsg>(
        "/orbslam3/image_stream/image_raw",
        qos,
        std::bind(&MonoPcloudNode::GrabImage, this, std::placeholders::_1));

    std::cout << "slam changed" << std::endl;
}

MonoPcloudNode::~MonoPcloudNode()
{
    RCLCPP_INFO(node->get_logger(), "Destructor called");
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoPcloudNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    rclcpp::Time current_frame_time = msg->header.stamp;
    commons->process_tracking(m_SLAM, m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp)).matrix(),
                              current_frame_time);
}
