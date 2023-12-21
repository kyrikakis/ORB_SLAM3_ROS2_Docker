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

    auto node = std::make_shared<MonoPcloudNode>();
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
