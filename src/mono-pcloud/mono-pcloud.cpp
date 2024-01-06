#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "mono-pcloud-node.hpp"

#include "System.h"


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("orb_slam3");

    node->declare_parameter("vocabulary_file", ""); 
    node->declare_parameter("slam_config_file", "");
    std::string vocabulary_file = node->get_parameter("vocabulary_file").as_string(); 
    std::string slam_config_file = node->get_parameter("slam_config_file").as_string(); 

    RCLCPP_INFO(node->get_logger(), "Using vocabulary file: %s", vocabulary_file.c_str());
    RCLCPP_INFO(node->get_logger(), "Using slam config file: %s", slam_config_file.c_str());

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;
    ORB_SLAM3::System m_SLAM(vocabulary_file.c_str(), slam_config_file.c_str(), ORB_SLAM3::System::MONOCULAR, visualization);
    MonoPcloudNode mono_orb(&m_SLAM, node.get());
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
