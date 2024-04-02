#include "commons.hpp"

Commons::Commons(std::shared_ptr<rclcpp::Node> p_node) 
{
    node = p_node;
    map_frame_id = "map";
    pose_frame_id = "base_link";
    octomap_frame_id = "octomap_link";
    tf_orb_to_ros.setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(qos_profile));

    rmw_qos_profile_t qos_profile_reliable = rmw_qos_profile_parameters;
    qos_profile_reliable.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto qos_reliable = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(qos_profile_reliable));

    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/camera_pose", qos);
    map_points_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/keyframe_points", qos);
    octomap_points_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in", qos_reliable);
    pcloud_all = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/pcloud_all", qos);
    pub_image = node->create_publisher<sensor_msgs::msg::Image>("~/tracking_image", qos);
}

void Commons::publish_tracking_img(const cv::Mat &image, const rclcpp::Time &current_frame_time)
{
    std_msgs::msg::Header header;
    header.stamp = current_frame_time;
    header.frame_id = map_frame_id;

    const std::shared_ptr<sensor_msgs::msg::Image> rendered_image_msg =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();

    pub_image->publish(*rendered_image_msg.get());
}

tf2::Transform Commons::from_orb_to_ros_tf_transform(cv::Mat transformation_mat)
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

void Commons::publish_tf_transform(tf2::Transform tf_transform, std::string child_frame_id, rclcpp::Time current_frame_time)
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

void Commons::publish_pose_stamped(tf2::Transform tf_transform, std::string child_frame_id, rclcpp::Time current_frame_time)
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

void Commons::publish_keyframe_points(tf2::Transform tf,
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

void Commons::publish_all_keyframes_points(vector<ORB_SLAM3::KeyFrame *> key_frames)
{
    RCLCPP_INFO(node->get_logger(), "Publish keyframes to octomap");

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
    is_octomap_resetting = true;
}

void Commons::publish_all_map_points(const rclcpp::Time &current_frame_time, std::vector<ORB_SLAM3::MapPoint *> map_points)
{
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