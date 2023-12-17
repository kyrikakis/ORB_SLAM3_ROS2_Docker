#include "mono-pcloud-node.hpp"

#include <opencv2/core/core.hpp>

#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

MonoPcloudNode::MonoPcloudNode(ORB_SLAM3::System *pSLAM)
    : Node("orb_slam3")
{
    size_t depth = 10;
    rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
    qos_profile.depth = depth;

    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos_profile.reliability = reliability_policy;

    // The history policy determines how messages are saved until the message is taken by the reader.
    // KEEP_ALL saves all messages until they are taken.
    // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
    // parameter.
    qos_profile.history = history_policy;

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(qos_profile));

    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/camera/image_raw",
        qos,
        std::bind(&MonoPcloudNode::GrabImage, this, std::placeholders::_1));
    map_frame_id = "map";
    pose_frame_id = "odom";
    tf_orb_to_ros.setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);

    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/camera_pose", qos);
    map_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in", qos);

    std::shared_ptr<rclcpp::Node> image_transport_node = rclcpp::Node::make_shared("image_publisher");
    image_transport::ImageTransport image_transport(image_transport_node);
    rendered_image_pub = image_transport.advertise("~/tracking_image", 5);
    std::cout << "slam changed" << std::endl;
}

MonoPcloudNode::~MonoPcloudNode()
{
    RCLCPP_INFO(this->get_logger(), "Destructor called");
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
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

    // cout << setprecision(9) << "Rotation: " << endl << orb_rotation << endl;
    // cout << setprecision(9) << "Translation xyz: " << orb_translation.at<float>
    // (0) << " " << orb_translation.at<float> (1) << " " <<
    // orb_translation.at<float> (2) << endl;

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

void MonoPcloudNode::publish_ros_pose_tf(cv::Mat Tcw, rclcpp::Time current_frame_time)
{
    if (!Tcw.empty())
    {
        tf2::Transform tf_transform =
            from_orb_to_ros_tf_transform(Tcw);

        publish_tf_transform(tf_transform, current_frame_time);

        publish_pose_stamped(tf_transform, current_frame_time);
    }
}

void MonoPcloudNode::publish_tf_transform(tf2::Transform tf_transform, rclcpp::Time current_frame_time)
{
    static tf2_ros::TransformBroadcaster tf_broadcaster(this);

    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = header;
    tf_msg.child_frame_id = pose_frame_id;
    tf_msg.transform = tf2::toMsg(tf_transform);

    tf_broadcaster.sendTransform(tf_msg);
}

void MonoPcloudNode::publish_pose_stamped(tf2::Transform tf_transform, rclcpp::Time current_frame_time)
{
    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = pose_frame_id;

    geometry_msgs::msg::Pose pose;
    tf2::toMsg(tf_transform, pose);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = header;
    pose_msg.pose = pose;

    pose_pub->publish(pose_msg);
}

sensor_msgs::msg::PointCloud2 MonoPcloudNode::tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points,
                                                                              rclcpp::Time current_frame_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    int j = 0;
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
    j++;
    return cloud;
}

void MonoPcloudNode::publish_ros_tracking_mappoints(
    std::vector<ORB_SLAM3::MapPoint *> map_points,
    const rclcpp::Time &current_frame_time)
{
    sensor_msgs::msg::PointCloud2 cloud =
        tracked_mappoints_to_pointcloud(map_points, current_frame_time);

    map_points_pub->publish(cloud);
}

void MonoPcloudNode::publish_ros_tracking_img(const cv::Mat &image, const rclcpp::Time &current_frame_time)
{
    std_msgs::msg::Header header;
    int j = 0;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    const std::shared_ptr<sensor_msgs::msg::Image> rendered_image_msg =
        cv_bridge::CvImage(header, "rgb8", image).toImageMsg();

    rendered_image_pub.publish(rendered_image_msg);
    j++;
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
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    rclcpp::Time current_frame_time = msg->header.stamp;

    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp)).matrix());

    publish_ros_pose_tf(Tcw, current_frame_time);
    publish_ros_tracking_mappoints(m_SLAM->GetTrackedMapPoints(), current_frame_time);
    publish_ros_tracking_img(Tcw, current_frame_time);
}
