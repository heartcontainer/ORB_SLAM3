#include "rgbd-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdInertialNode::RgbdInertialNode(ORB_SLAM3::System *SLAM, const string &color_topic, const string &depth_topic, const string &imu_topic) : Node("ORB_SLAM3_ROS2"),
                                                                                                                                             SLAM_(SLAM)
{
    subImu_ = this->create_subscription<ImuMsg>(imu_topic, 1000, std::bind(&RgbdInertialNode::GrabImu, this, _1));

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, color_topic);
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, depth_topic);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdInertialNode::GrabRGBD, this);

    syncThread_ = new std::thread(&RgbdInertialNode::SyncWithImu, this);
}

RgbdInertialNode::~RgbdInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    imuMutex_.lock();
    imuBuf_.push(msg);
    imuMutex_.unlock();
}

void RgbdInertialNode::GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD)
{
    imgMutex_.lock();

    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push({msgRGB, msgD});

    imgMutex_.unlock();
}

cv::Mat RgbdInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void RgbdInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imColor, imDepth;
        double tIm;
        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tIm = Utility::StampToSec(imgBuf_.front().rgb->header.stamp);
            if (tIm > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            imgMutex_.lock();
            imColor = GetImage(imgBuf_.front().rgb);
            imDepth = GetImage(imgBuf_.front().d);
            imgBuf_.pop();
            imgMutex_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            imuMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            imuMutex_.unlock();
            // if (mbClahe)
            //     mClahe->apply(im, im);

            SLAM_->TrackRGBD(imColor, imDepth, tIm, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}