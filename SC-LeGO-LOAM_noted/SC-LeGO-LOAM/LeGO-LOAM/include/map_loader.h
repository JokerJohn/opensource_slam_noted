#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
// pcl
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>

class MapLoader {
public:
    ros::Publisher pc_map_pub_;
    std::vector<std::string> file_list_;

    MapLoader(ros::NodeHandle &nh);

private:
    sensor_msgs::PointCloud2 CreatePcd();
}; //MapLoader

#endif