#include "image-stream-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

ImageStreamNode::ImageStreamNode()
:   Node("camera"), cim(this)
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

    
    this->declare_parameter("video_capture_stream", "tcp://192.168.1.17:8888");
    video_capture_stream = this->get_parameter("video_capture_stream").as_string();
    this->declare_parameter("config_file", "/workspaces/ORB_SLAM3_ROS2_Docker/calibrations/left.yaml");
    config_file = this->get_parameter("config_file").as_string();
    this->declare_parameter("display_image", false);
    display_image = this->get_parameter("display_image").as_bool();
    
    pub_image = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
    pub_ci = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", qos);

    cim.loadCameraInfo(config_file);
    ImageStreamNode::StreamImage();
}

void ImageStreamNode::StreamImage() 
{

    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.open(video_capture_stream);

    RCLCPP_INFO(this->get_logger(), "buffer size: %i", cap.get(cv::CAP_PROP_BUFFERSIZE));
    cv::Mat frame;
    cap >> frame;
    RCLCPP_INFO(this->get_logger(), "frame received size: %i", frame.size());

    try {
        while (rclcpp::ok()) {
            cap >> frame;
            // Check if the frame is empty (end of video stream)
            if(frame.empty()) {
                RCLCPP_ERROR(this->get_logger(),  "Empty frame, exiting" );
                break;
            }

            if(frame.size().width < 0) {
                RCLCPP_ERROR(this->get_logger(),  "0 size, exiting" );
                break;
            }

            if(display_image) {
                RCLCPP_INFO(this->get_logger(), "frame received size width: %i", frame.size().width);
                RCLCPP_INFO(this->get_logger(), "frame received size height: %i", frame.size().height);
                cv::imshow(this->get_name(), frame);
                cv::waitKey(1);
            } else {

                // send image data
                std_msgs::msg::Header header;
                header.frame_id = "camera";
                header.stamp = this->get_clock()->now();

                cv_bridge::CvImage img_bridge;

                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
                ImageMsg::SharedPtr msg_img = img_bridge.toImageMsg();
                pub_image->publish(*msg_img); 

                sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
                ci.header = header;
                ci.height = frame.size().height;
                ci.width = frame.size().width;
                pub_ci->publish(ci);
            }
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

ImageStreamNode::~ImageStreamNode()
{
    RCLCPP_INFO(this->get_logger(), "Destructor called");
}