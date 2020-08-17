#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

// 工具
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// 自定义msg
#include "cloud_msgs/cloud_info.h"
// opencv
#include <opencv/cv.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
// 系统头文件
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;
// rosbag filter "HK-Data20190316-2 20190331_NJ_LL.bag" "lidaronly_HK-Data20190316-2 20190331_NJ_LL.bag" "topic == '/velodyne_points'"
// rosbag filter "HK-Data20190117.bag" "lidaronly_HK-Data20190117.bag" "topic == '/velodyne_points'"

typedef pcl::PointXYZI PointType;

//  get params
extern const std::string package_path = ros::package::getPath("lego_loam");
extern const std::string config_file_path = package_path + "/params/lego.yaml";

YAML::Node config_node = YAML::LoadFile(config_file_path);
extern const int N_SCAN = config_node["N_SCAN"].as<int>();
extern const int Horizon_SCAN = config_node["Horizon_SCAN"].as<int>();

extern const double ang_res_x = config_node["ang_res_x"].as<double>();
extern const double ang_res_y = config_node["ang_res_y"].as<double>();
extern const double ang_bottom = config_node["ang_bottom"].as<double>();
extern const int groundScanInd = config_node["groundScanInd"].as<int>();

extern const std::string pointCloudTopic = config_node["pointCloudTopic"].as<std::string>();
extern const std::string imuTopic = config_node["imuTopic"].as<std::string>();
extern const bool useCloudRing = config_node["useCloudRing"].as<bool>();

//  odom轨迹
// extern const std::string odom_trajectory_path = package_path + "/map/stamped_legoloam_map_estimate.txt";
//extern const std::string pcd_file_path = package_path + "/map/";

// disabled, it would be better to save the pcd in $(find lego_loam)/map folder
// extern const string fileDirectory = "/tmp/";


/*
//extern const string pointCloudTopic = "/velodyne_points";
//const string pointCloudTopic = "/fused/velodyne_points";
// extern const string pointCloudTopic = "/kitti_scan";
extern const string pointCloudTopic = "/os1_points";
extern const string imuTopic = "/imu/data111";


// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
// extern const bool useCloudRing = false; // if true, ang_res_y and ang_bottom are not used
extern const bool useCloudRing = false; // if true, ang_res_y and ang_bottom are not used

// VLP-16
//extern const int N_SCAN = 16;
//extern const int Horizon_SCAN = 1800;
//extern const float ang_res_x = 0.2;
//extern const float ang_res_y = 2.0;
//extern const float ang_bottom = 15.0 + 0.1;
//extern const int groundScanInd = 7;

// VLP-32C
//extern const int N_SCAN = 32;
//extern const int Horizon_SCAN = 1800;
//extern const float ang_res_x = 360.0 / float(Horizon_SCAN);
//extern const float ang_res_y = 41.33 / float(N_SCAN - 1);
//extern const float ang_bottom = 30.67;
//extern const int groundScanInd = 20;

// // HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// VLS-128
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.3;
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 10;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF IMU), please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1024;
extern const float ang_res_x = 360.0 / float(Horizon_SCAN);
extern const float ang_res_y = 33.2 / float(N_SCAN - 1);
extern const float ang_bottom = 16.6 + 0.1;
extern const int groundScanInd = 15;
*/

extern const bool loopClosureEnableFlag = true;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;

extern const float sensorMinimumRange = 3.0;
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 60.0 / 180.0 * M_PI; // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;


// Mapping Params
extern const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)

// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius = 20.0; // NOT used in Scan Context-based loop detector / default 7.0; key frame that is within n meters from current pose will be considerd for loop closure
extern const int historyKeyframeSearchNum = 25; // 2n+1 number of history key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 1.6; // default 0.3; the smaller the better alignment

extern const float globalMapVisualizationSearchRadius = 1500.0; // key frames with in n meters will be visualized

struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has "ring" channel
    */
struct PointXYZIR {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)
)
/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
                                           (double, time, time)
)
typedef PointXYZIRPYT PointTypePose;


#endif
