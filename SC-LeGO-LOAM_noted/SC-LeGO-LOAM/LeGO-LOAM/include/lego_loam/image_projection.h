//
// Created by xchu on 2020/6/30.
//

#ifndef SRC_IMAGE_PROJECTION_H
#define SRC_IMAGE_PROJECTION_H


#include "lego_loam/utility.h"

namespace lego_loam {
    class ImageProjection {
    public:
        ImageProjection();

        ~ImageProjection() {}

        void InitParams();

        void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

        void allocateMemory();

        void resetParameters();

        void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

        void findStartEndAngle();

        void projectPointCloud();

        void groundRemoval();

        void cloudSegmentation();

        void labelComponents(int row, int col);

        void publishCloud();

    private:
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        ros::Subscriber subLaserCloud;

        ros::Publisher pubFullCloud;
        ros::Publisher pubFullInfoCloud;

        ros::Publisher pubGroundCloud;
        ros::Publisher pubSegmentedCloud;
        ros::Publisher pubSegmentedCloudPure;
        ros::Publisher pubSegmentedCloudInfo;
        ros::Publisher pubOutlierCloud;

        pcl::PointCloud<PointType>::Ptr laserCloudIn;  //  进来的原始pcl转sensors_msg点云
        pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing; //  每个激光点计算了ring的点云

        pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
        pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

        pcl::PointCloud<PointType>::Ptr groundCloud;
        pcl::PointCloud<PointType>::Ptr segmentedCloud;
        pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
        pcl::PointCloud<PointType>::Ptr outlierCloud;

        PointType nanPoint; // fill in fullCloud at each iteration

        cv::Mat rangeMat; // range matrix for range image
        cv::Mat labelMat; // label matrix for segmentaiton marking
        cv::Mat groundMat; // ground matrix for ground cloud marking
        int labelCount;

        float startOrientation;
        float endOrientation;

        cloud_msgs::cloud_info segMsg; // info of segmented cloud
        std_msgs::Header cloudHeader;

        std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

        uint16_t *allPushedIndX; // array for tracking points of a segmented object
        uint16_t *allPushedIndY;

        uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
        uint16_t *queueIndY;

        // params
//        std::string pointCloudTopic;
//        std::string imuTopic;
//        bool useCloudRing = false;
//
//        int N_SCAN;
//        int Horizon_SCAN;
//        double ang_res_x;
//        double ang_res_y;
//        double ang_bottom;
//        int groundScanInd;
//
//        float segmentAlphaX;
//        float segmentAlphaY;
    };
}


#endif //SRC_IMAGE_PROJECTION_H
