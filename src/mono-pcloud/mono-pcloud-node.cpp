#include "mono-pcloud-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

MonoPcloudNode::MonoPcloudNode(ORB_SLAM3::System *pSLAM, std::shared_ptr<rclcpp::Node> p_node)
{
    m_SLAM = pSLAM;
    node = p_node;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(qos_profile));

    rmw_qos_profile_t qos_profile_reliable = rmw_qos_profile_parameters;
    qos_profile_reliable.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto qos_reliable = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(qos_profile_reliable));

    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = node->create_subscription<ImageMsg>(
        "/orbslam3/image_stream/image_raw",
        qos,
        std::bind(&MonoPcloudNode::GrabImage, this, std::placeholders::_1));
    map_frame_id = "map";
    pose_frame_id = "base_link";
    octomap_frame_id = "octomap_link";
    tf_orb_to_ros.setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);

    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/camera_pose", qos);
    map_points_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/keyframe_points", qos);
    octomap_points_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in", qos_reliable);
    pcloud_all = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/pcloud_all", qos);
    pub_image = node->create_publisher<sensor_msgs::msg::Image>("~/tracking_image", qos);

    // std::shared_ptr<rclcpp::Node> image_transport_node = rclcpp::Node::make_shared("image_publisher");
    // image_transport::ImageTransport image_transport(image_transport_node);
    // rendered_image_pub = image_transport.advertise("~/tracking_image", 5);

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

void MonoPcloudNode::publish_tracking_img(const cv::Mat &image, const rclcpp::Time &current_frame_time)
{
    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    const std::shared_ptr<sensor_msgs::msg::Image> rendered_image_msg =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();

    pub_image->publish(*rendered_image_msg.get());
}

tf2::Transform MonoPcloudNode::from_orb_to_ros_tf_transform(cv::Mat transformation_mat)
{
    cv::Mat orb_rotation(3, 3, CV_32F);
    cv::Mat orb_translation(3, 1, CV_32F);

    orb_rotation = transformation_mat.rowRange(0, 3).colRange(0, 3);
    orb_translation = transformation_mat.rowRange(0, 3).col(3);

    tf2::Matrix3x3 tf_camera_rotation(
        orb_rotation.at<float>(0, 0), orb_rotation.at<float>(0, 1),
        orb_rotation.at<float>(0, 2), orb_rotation.at<float>(1, 0),
        orb_rotation.at<float>(1, 1), orb_rotation.at<float>(1, 2),
        orb_rotation.at<float>(2, 0), orb_rotation.at<float>(2, 1),
        orb_rotation.at<float>(2, 2));

    tf2::Vector3 tf_camera_translation(orb_translation.at<float>(0),
                                       orb_translation.at<float>(1),
                                       orb_translation.at<float>(2));

    // Transform from orb coordinate system to ros coordinate system on camera
    // coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    // Transform from orb coordinate system to ros coordinate system on map
    // coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}

void MonoPcloudNode::publish_tf_transform(tf2::Transform tf_transform, std::string child_frame_id, rclcpp::Time current_frame_time)
{
    static tf2_ros::TransformBroadcaster tf_broadcaster(node);

    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = header;
    tf_msg.child_frame_id = child_frame_id;
    tf_msg.transform = tf2::toMsg(tf_transform);

    tf_broadcaster.sendTransform(tf_msg);
}

void MonoPcloudNode::publish_pose_stamped(tf2::Transform tf_transform, std::string child_frame_id, rclcpp::Time current_frame_time)
{
    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = child_frame_id;

    geometry_msgs::msg::Pose pose;
    tf2::toMsg(tf_transform, pose);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = header;
    pose_msg.pose = pose;

    pose_pub->publish(pose_msg);
}

void MonoPcloudNode::publish_all_map_points(const rclcpp::Time &current_frame_time)
{
    std::vector<ORB_SLAM3::MapPoint *> map_points = m_SLAM->getMap()->GetCurrentMap()->GetAllMapPoints();
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = current_frame_time;
    cloud.header.frame_id = map_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {

            tf2::Vector3 point_translation(map_points[i]->GetWorldPos()(0),
                                            map_points[i]->GetWorldPos()(1),
                                            map_points[i]->GetWorldPos()(2));

            point_translation = tf_orb_to_ros * point_translation;

            float data_array[num_channels] = {
                point_translation.x(), point_translation.y(), point_translation.z()};

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
                    num_channels * sizeof(float));
        }
    }
    pcloud_all->publish(cloud);
}

void MonoPcloudNode::publish_keyframe_points(
    tf2::Transform tf,
    std::string child_frame_id,
    std::vector<ORB_SLAM3::MapPoint *> map_points,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub,
    const rclcpp::Time &current_frame_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    int j = 0;
    cloud.header.stamp = current_frame_time;
    cloud.header.frame_id = child_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = header;
    tf_msg.child_frame_id = child_frame_id;
    tf_msg.transform = tf2::toMsg(tf.inverse());

    Eigen::Matrix4d transform_matrix = tf2::transformToEigen(tf_msg).matrix();

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            tf2::Vector3 point_translation(map_points[i]->GetWorldPos()(0),
                                           map_points[i]->GetWorldPos()(1),
                                           map_points[i]->GetWorldPos()(2));

            point_translation = tf_orb_to_ros * point_translation;
            Eigen::Vector4d point_homo(point_translation.x(), point_translation.y(), point_translation.z(), 1.0);

            // Apply transform using matrix multiplication
            Eigen::Vector4d transformed_point = transform_matrix * point_homo;
            float data_array[num_channels] = {
                transformed_point[0], transformed_point[1], transformed_point[2]};

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
                   num_channels * sizeof(float));
        }
    }
    j++;
    points_pub->publish(cloud);
}

void MonoPcloudNode::publish_all_keyframes_points()
{
    RCLCPP_INFO(node->get_logger(), "Publish keyframes to octomap");

    vector<ORB_SLAM3::KeyFrame *> key_frames = m_SLAM->getMap()->GetAllKeyFrames();
    sort(key_frames.begin(), key_frames.end(), ORB_SLAM3::KeyFrame::lId);
    unsigned int n_kf = 0;
    for (auto key_frame : key_frames)
    {
        auto keyframe_time = node->get_clock()->now();
        if (key_frame->isBad())
            continue;

        cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(key_frame->GetPose().matrix());
        tf2::Transform tf =
            from_orb_to_ros_tf_transform(Tcw);

        publish_tf_transform(tf, octomap_frame_id, keyframe_time);

        std::this_thread::sleep_for(40ms);
        std::set<ORB_SLAM3::MapPoint *> map_points = key_frame->GetMapPoints();
        std::vector map_points_vector(map_points.begin(), map_points.end());
        publish_keyframe_points(tf, octomap_frame_id, map_points_vector, octomap_points_pub, keyframe_time);
        ++n_kf;
        RCLCPP_INFO(node->get_logger(), "Publishing %u cloudpoints for %u keyframe", map_points_vector.size(), n_kf);
    }
    is_octomap_resetting = false;
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

        publish_tracking_img(m_SLAM->GetCurrentFrame(), current_frame_time);
        
        if (m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::OK ||
            m_SLAM->GetTrackingState() == ORB_SLAM3::Tracking::eTrackingState::RECENTLY_LOST)
        {
            if (!Tcw.empty())
            {
                tf2::Transform tf_transform =
                    from_orb_to_ros_tf_transform(Tcw);

                // Rviz2
                publish_tf_transform(tf_transform, pose_frame_id, current_frame_time);
                publish_pose_stamped(tf_transform, pose_frame_id, current_frame_time);
                publish_keyframe_points(tf_transform, pose_frame_id, m_SLAM->GetTrackedMapPoints(),
                                        map_points_pub, current_frame_time);
                if (number_of_frames == 10)
                {
                    publish_all_map_points(current_frame_time);
                }

                // Octomap
                if (!is_octomap_resetting)
                {
                    publish_tf_transform(tf_transform, octomap_frame_id, current_frame_time);
                    publish_keyframe_points(tf_transform, octomap_frame_id, m_SLAM->GetTrackedMapPoints(),
                                            octomap_points_pub, current_frame_time);
                }

                if (m_SLAM->getLoopClosing()->isMapReady() || tracking_lost)
                {
                    // Octomap reset
                    RCLCPP_INFO(node->get_logger(), "Resetting Octomap");
                    is_octomap_resetting = true;
                    auto request = std::make_shared<std_srvs::srv::Empty_Request>();

                    using ServiceResponseFuture =
                        rclcpp::Client<std_srvs::srv::Empty>::SharedFutureWithRequest;

                    auto response_received_callback =
                        [logger = node->get_logger(), this](ServiceResponseFuture future)
                    {
                        // std::shared_ptr<MonoPcloudNode> node = std::make_shared<MonoPcloudNode>(this);
                        std::thread *m_thread = new std::thread(&MonoPcloudNode::publish_all_keyframes_points, this);
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
