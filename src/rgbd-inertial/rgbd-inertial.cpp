#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if (argc < 6)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono-inertial path_to_vocabulary path_to_settings color_topic depth_topic imu_topic" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, visualization);

    auto node = std::make_shared<RgbdInertialNode>(&pSLAM, argv[3], argv[4], argv[5]);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
