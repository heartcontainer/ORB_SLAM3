#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = stamp.sec + (stamp.nanosec * pow(10, -9));
    return seconds;
  }

  static sensor_msgs::msg::PointCloud2 MapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint *> map_points, const std::string &frame_id,
                                                             rclcpp::Time node_time)
  {
    if (map_points.size() == 0)
    {
      std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 cloud;

    const int num_channels = 3; // x y z

    cloud.header.stamp = node_time;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++)
    {
      cloud.fields[i].name = channel_id[i];
      cloud.fields[i].offset = i * sizeof(float);
      cloud.fields[i].count = 1;
      cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[num_channels];
    for (unsigned int i = 0; i < cloud.width; i++)
    {
      if (map_points.at(i)->nObs >= 2)
      {

        data_array[0] = (float)map_points.at(i)->GetWorldPos()(2);          // x. Do the transformation by just reading at the position of z instead of x
        data_array[1] = (float)(-1.0 * map_points.at(i)->GetWorldPos()(0)); // y. Do the transformation by just reading at the position of x instead of y
        data_array[2] = (float)(-1.0 * map_points.at(i)->GetWorldPos()(1)); // z. Do the transformation by just reading at the position of y instead of z
        // TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

        memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
      }
    }
    return cloud;
  }
};

#endif
