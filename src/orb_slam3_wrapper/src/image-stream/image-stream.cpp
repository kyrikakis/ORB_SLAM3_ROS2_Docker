#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "image-stream-node.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImageStreamNode>();
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}