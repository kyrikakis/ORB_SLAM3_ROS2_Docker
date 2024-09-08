#include "stereo-image-stream-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

StereoImageStreamNode::StereoImageStreamNode()
:   Node("stereo_camera"), left_cim(this), right_cim(this)
{
    size_t depth = 5;
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
    
    this->declare_parameter("camera_stream", "tcp://192.168.1.172:8889");
    camera_stream = this->get_parameter("camera_stream").as_string();
    this->declare_parameter("left_config_file", "/workspaces/ORB_SLAM3_ROS2_Docker/calibrations/left.yaml");
    left_config_file = this->get_parameter("left_config_file").as_string();
    this->declare_parameter("right_config_file", "/workspaces/ORB_SLAM3_ROS2_Docker/calibrations/right.yaml");
    right_config_file = this->get_parameter("right_config_file").as_string();
    this->declare_parameter("display_image", false);
    display_image = this->get_parameter("display_image").as_bool(); 
    
    left_pub_image = this->create_publisher<sensor_msgs::msg::Image>("/orbslam3/left/image_raw", qos);
    left_pub_ci = this->create_publisher<sensor_msgs::msg::CameraInfo>("/orbslam3/left/camera_info", qos);
    right_pub_image = this->create_publisher<sensor_msgs::msg::Image>("/orbslam3/right/image_raw", qos);
    right_pub_ci = this->create_publisher<sensor_msgs::msg::CameraInfo>("/orbslam3/right/camera_info", qos);

    left_cim.loadCameraInfo(left_config_file);
    right_cim.loadCameraInfo(right_config_file);
    StereoImageStreamNode::StreamImage();
}

void StereoImageStreamNode::StreamImage() 
{
    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.open(camera_stream);

    RCLCPP_INFO(this->get_logger(), "left buffer size: %i", cap.get(cv::CAP_PROP_BUFFERSIZE));
    cv::Mat frame;
    cap >> frame;
    RCLCPP_INFO(this->get_logger(), "left frame received size: %i", frame.size());

    try {
        while (rclcpp::ok()) {
            cap >> frame;
            // Check if the frame is empty (end of video stream)
            if(frame.empty()) {
                RCLCPP_ERROR(this->get_logger(),  "Left empty frame, exiting" );
                break;
            }
            int img_height = frame.size().height;
            int img_width = frame.size().width;

            cv::Mat left_image = frame(cv::Range(0,img_height), cv::Range(0,img_width/2));
            cv::Mat right_image = frame(cv::Range(0,img_height), cv::Range(img_width/2,img_width));

            if(display_image) {
                cv::imshow("left", left_image);
                cv::imshow("right", right_image);
                cv::waitKey(1);
                continue;
            }

            // // send image data
            std_msgs::msg::Header header;
            header.frame_id = "camera";
            header.stamp = this->get_clock()->now();

            cv_bridge::CvImage left_img_bridge;
            cv_bridge::CvImage right_img_bridge;

            left_img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, left_image);
            ImageMsg::SharedPtr msg_img_left = left_img_bridge.toImageMsg();
            left_pub_image->publish(*msg_img_left);

            right_img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, right_image);
            ImageMsg::SharedPtr msg_img_right = right_img_bridge.toImageMsg();
            right_pub_image->publish(*msg_img_right); 

            sensor_msgs::msg::CameraInfo left_ci = left_cim.getCameraInfo();
            left_ci.header = header;
            left_ci.height = left_image.size().height;
            left_ci.width = left_image.size().width;
            left_pub_ci->publish(left_ci);

            sensor_msgs::msg::CameraInfo right_ci = right_cim.getCameraInfo();
            right_ci.header = header;
            right_ci.height = right_image.size().height;
            right_ci.width = right_image.size().width;
            right_pub_ci->publish(right_ci);
        }
        // Clean up
        cv::destroyAllWindows();
        cap.release();
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(),  "OpenCV exception: %s", e.what());
        return;
    } catch (std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),  "Error: %s", e.what());
    }
}

StereoImageStreamNode::~StereoImageStreamNode()
{
    RCLCPP_INFO(this->get_logger(), "Destructor called");
}