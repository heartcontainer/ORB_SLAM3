#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "mono-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono-inertial path_to_vocabulary path_to_settings imu_topic color_topic" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    auto node = std::make_shared<MonoInertialNode>(&pSLAM, argv[3], argv[4]);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
