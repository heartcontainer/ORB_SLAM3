#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, const string &color_topic)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        color_topic,
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    m_node = rclcpp::Node::make_shared("ORB_SLAM3_MONO");
    m_pointcloud2_publisher = m_node->create_publisher<sensor_msgs::msg::PointCloud2>("orbslam3/cloud", 10);
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    sensor_msgs::msg::PointCloud2 cloud = Utility::MapPointsToPointCloud(m_SLAM->GetAllMapPoints(), "camera_link", m_node->get_clock()->now());
    m_pointcloud2_publisher->publish(cloud);
}
