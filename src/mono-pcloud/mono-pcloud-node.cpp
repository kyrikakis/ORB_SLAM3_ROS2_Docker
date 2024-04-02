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

    octomap_reset_client = node->create_client<std_srvs::srv::Empty>("/octomap_server_node/reset");

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
    try
    {
        number_of_frames++;
        cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp)).matrix());

        commons->publish_tracking_img(m_SLAM->GetCurrentFrame(), current_frame_time);
        
        if (m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::OK ||
            m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::RECENTLY_LOST)
        {
            if (!Tcw.empty())
            {
                tf2::Transform tf_transform =
                    commons->from_orb_to_ros_tf_transform(Tcw);

                // Rviz2
                commons->publish_tf_transform(tf_transform, commons->pose_frame_id, current_frame_time);
                commons->publish_pose_stamped(tf_transform, commons->pose_frame_id, current_frame_time);
                commons->publish_keyframe_points(tf_transform, commons->pose_frame_id, m_SLAM->GetTrackedMapPoints(),
                                        commons->map_points_pub, current_frame_time);
                if (number_of_frames == 10)
                {
                    commons->publish_all_map_points(current_frame_time, m_SLAM->getMap()->GetCurrentMap()->GetAllMapPoints());
                }

                // Octomap
                if (!commons->is_octomap_resetting)
                {
                    commons->publish_tf_transform(tf_transform, commons->octomap_frame_id, current_frame_time);
                    commons->publish_keyframe_points(tf_transform, commons->octomap_frame_id, m_SLAM->GetTrackedMapPoints(),
                                            commons->octomap_points_pub, current_frame_time);
                }

                if (m_SLAM->getLoopClosing()->isMapReady() || tracking_lost)
                {
                    // Octomap reset
                    RCLCPP_INFO(node->get_logger(), "Resetting Octomap");
                    commons->is_octomap_resetting = true;
                    auto request = std::make_shared<std_srvs::srv::Empty_Request>();

                    using ServiceResponseFuture =
                        rclcpp::Client<std_srvs::srv::Empty>::SharedFutureWithRequest;

                    auto response_received_callback =
                        [logger = node->get_logger(), this](ServiceResponseFuture future)
                    {
                        // std::shared_ptr<MonoPcloudNode> node = std::make_shared<MonoPcloudNode>(this);
                        std::thread *m_thread = new std::thread(&Commons::publish_all_keyframes_points, commons, m_SLAM->getMap()->GetAllKeyFrames());
                        m_thread->detach();
                    };

                    tracking_lost = false;
                    octomap_reset_client->async_send_request(request, std::move(response_received_callback));
                }
            }
        }
        else
        {
            tracking_lost = true;
        }

        if (number_of_frames > 10)
        {
            number_of_frames = 0;
        }
    }
    catch (const runtime_error &e)
    {
        RCLCPP_ERROR(node->get_logger(), "m_SLAM exception: %s", e.what());
        return;
    }
}
