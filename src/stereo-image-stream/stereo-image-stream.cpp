#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-image-stream-node.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StereoImageStreamNode>();
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout << "shutdown image_stream" << std::endl;

    return 0;
}