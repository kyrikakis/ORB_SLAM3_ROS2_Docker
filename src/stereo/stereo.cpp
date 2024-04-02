#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("orb_slam3");

    node->declare_parameter("vocabulary_file", ""); 
    node->declare_parameter("slam_config_file", "");
    std::string vocabulary_file = node->get_parameter("vocabulary_file").as_string(); 
    std::string slam_config_file = node->get_parameter("slam_config_file").as_string(); 

    bool visualization = false;
    ORB_SLAM3::System pSLAM(vocabulary_file.c_str(), slam_config_file.c_str(), ORB_SLAM3::System::STEREO, visualization);

    StereoSlamNode stereo(&pSLAM, node, slam_config_file, true);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
