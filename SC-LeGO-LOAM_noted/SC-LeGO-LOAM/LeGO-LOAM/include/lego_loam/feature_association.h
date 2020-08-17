//
// Created by xchu on 2020/6/30.
//

#ifndef SRC_FEATURE_ASSOCIATION_H
#define SRC_FEATURE_ASSOCIATION_H

#include "lego_loam/utility.h"

namespace lego_loam{
    class FeatureAssociation {

    public:
        inline double rad2deg(double radians) {
            return radians * 180.0 / M_PI;
        }

        inline double deg2rad(double degrees) {
            return degrees * M_PI / 180.0;
        }

        FeatureAssociation();
        void InitParams();

        //  主程序入口
        void runFeatureAssociation();

        void initializationValue();

        void updateImuRollPitchYawStartSinCos();

        void ShiftToStartIMU(float pointTime);

        void VeloToStartIMU();

        void TransformToStartIMU(PointType *p);

        void AccumulateIMUShiftAndRotation();

        void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn);

        void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

        void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msgIn);

        void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr &msgIn);

        void adjustDistortion();

        void calculateSmoothness();

        void markOccludedPoints();

        void extractFeatures();

        void publishCloud();

        void TransformToStart(PointType const *const pi, PointType *const po);

        void TransformToEnd(PointType const *const pi, PointType *const po);

        void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz,
                               float alx, float aly, float alz, float &acx, float &acy, float &acz);

        void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                                float &ox, float &oy, float &oz);

        void findCorrespondingCornerFeatures(int iterCount);

        void findCorrespondingSurfFeatures(int iterCount);

        bool calculateTransformationSurf(int iterCount);

        bool calculateTransformationCorner(int iterCount);

        bool calculateTransformation(int iterCount);

        void checkSystemInitialization();

        void updateInitialGuess();

        void updateTransformation();

        void integrateTransformation();

        void publishOdometry();

        void adjustOutlierCloud();

        void publishCloudsLast();

    private:

        ros::NodeHandle nh;

        ros::Subscriber subLaserCloud;
        ros::Subscriber subLaserCloudInfo;
        ros::Subscriber subOutlierCloud;
        ros::Subscriber subImu;

        ros::Publisher pubCornerPointsSharp;
        ros::Publisher pubCornerPointsLessSharp;
        ros::Publisher pubSurfPointsFlat;
        ros::Publisher pubSurfPointsLessFlat;

        pcl::PointCloud<PointType>::Ptr segmentedCloud;
        pcl::PointCloud<PointType>::Ptr outlierCloud;

        pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
        pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
        pcl::PointCloud<PointType>::Ptr surfPointsFlat;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

        pcl::VoxelGrid<PointType> downSizeFilter;

        double timeScanCur;
        double timeNewSegmentedCloud;
        double timeNewSegmentedCloudInfo;
        double timeNewOutlierCloud;

        bool newSegmentedCloud;
        bool newSegmentedCloudInfo;
        bool newOutlierCloud;

        cloud_msgs::cloud_info segInfo;
        std_msgs::Header cloudHeader;

        int systemInitCount;
        bool systemInited;

        std::vector<smoothness_t> cloudSmoothness;
        float *cloudCurvature;
        int *cloudNeighborPicked;
        int *cloudLabel;

        int imuPointerFront;
        int imuPointerLast;
        int imuPointerLastIteration;

        float imuRollStart, imuPitchStart, imuYawStart;
        float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;
        float imuRollCur, imuPitchCur, imuYawCur;

        float imuVeloXStart, imuVeloYStart, imuVeloZStart;
        float imuShiftXStart, imuShiftYStart, imuShiftZStart;

        float imuVeloXCur, imuVeloYCur, imuVeloZCur;
        float imuShiftXCur, imuShiftYCur, imuShiftZCur;

        float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
        float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

        float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;
        float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast;
        float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;

        double imuTime[imuQueLength];
        float imuRoll[imuQueLength];
        float imuPitch[imuQueLength];
        float imuYaw[imuQueLength];

        float imuAccX[imuQueLength];
        float imuAccY[imuQueLength];
        float imuAccZ[imuQueLength];

        float imuVeloX[imuQueLength];
        float imuVeloY[imuQueLength];
        float imuVeloZ[imuQueLength];

        float imuShiftX[imuQueLength];
        float imuShiftY[imuQueLength];
        float imuShiftZ[imuQueLength];

        float imuAngularVeloX[imuQueLength];
        float imuAngularVeloY[imuQueLength];
        float imuAngularVeloZ[imuQueLength];

        float imuAngularRotationX[imuQueLength];
        float imuAngularRotationY[imuQueLength];
        float imuAngularRotationZ[imuQueLength];


        ros::Publisher pubLaserCloudCornerLast;
        ros::Publisher pubLaserCloudSurfLast;
        ros::Publisher pubLaserOdometry;
        ros::Publisher pubOutlierCloudLast;

        int skipFrameNum;
        bool systemInitedLM;

        int laserCloudCornerLastNum;
        int laserCloudSurfLastNum;

        int *pointSelCornerInd;
        float *pointSearchCornerInd1;
        float *pointSearchCornerInd2;

        int *pointSelSurfInd;
        float *pointSearchSurfInd1;
        float *pointSearchSurfInd2;
        float *pointSearchSurfInd3;

        float transformCur[6];
        float transformSum[6];

        float imuRollLast, imuPitchLast, imuYawLast;
        float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
        float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
        pcl::PointCloud<PointType>::Ptr laserCloudOri;
        pcl::PointCloud<PointType>::Ptr coeffSel;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

        nav_msgs::Odometry laserOdometry;

        tf::TransformBroadcaster tfBroadcaster;
        tf::StampedTransform laserOdometryTrans;

        bool isDegenerate;
        cv::Mat matP;

        int frameCount;

//        // params
//        std::string imuTopic;
//
//        int N_SCAN;
//        int Horizon_SCAN;
    };
}




#endif //SRC_FEATURE_ASSOCIATION_H
