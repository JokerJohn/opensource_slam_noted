//
// Created by xchu on 2020/6/30.
//

#ifndef SRC_MAP_OPTMIZATION_H
#define SRC_MAP_OPTMIZATION_H


//  引入外部头文件
#include "scan_context/Scancontext.h"
#include "lego_loam/utility.h"

//  gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace lego_loam {
    using namespace gtsam;

    class mapOptimization {

    public:
        mapOptimization();

        void InitParams();

        void run();

        void allocateMemory();

        void transformAssociateToMap();

        void transformUpdate();

        void updatePointAssociateToMapSinCos();

        void pointAssociateToMap(PointType const *const pi, PointType *const po);

        void updateTransformPointCloudSinCos(PointTypePose *tIn);

        pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn);

        pcl::PointCloud<PointType>::Ptr
        transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

        void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg);

        void laserCloudRawHandler(const sensor_msgs::PointCloud2ConstPtr &msg);

        void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg);

        void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg);

        void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);

        void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn);

        PointTypePose trans2PointTypePose(float transformIn[]);

        void publishKeyPosesAndFrames();

        void TransformAndSaveMap();

        void visualizeGlobalMapThread();

        void publishGlobalMap();

        void loopClosureThread();

        bool detectLoopClosure();

        void performLoopClosure(void);// performLoopClosure

        void publishTF();

        Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

        Eigen::Affine3f pclPointToAffine3fCameraToLidar(PointTypePose thisPoint);

        void extractSurroundingKeyFrames();

        void downsampleCurrentScan();

        void cornerOptimization(int iterCount);

        void surfOptimization(int iterCount);

        bool LMOptimization(int iterCount);

        void scan2MapOptimization();

        void saveKeyFramesAndFactor(); // saveKeyFramesAndFactor

        void correctPoses();

        void clearCloud();

    private:
        NonlinearFactorGraph gtSAMgraph;
        Values initialEstimate;
        Values optimizedEstimate;
        ISAM2 *isam;
        Values isamCurrentEstimate;

        noiseModel::Diagonal::shared_ptr priorNoise;
        noiseModel::Diagonal::shared_ptr odometryNoise;
        noiseModel::Diagonal::shared_ptr constraintNoise;
        noiseModel::Base::shared_ptr robustNoiseModel;

        ros::NodeHandle nh;

        ros::Publisher pubLaserCloudSurround;
        ros::Publisher pubOdomAftMapped;
        ros::Publisher pubKeyPoses;

        ros::Publisher pubHistoryKeyFrames;
        ros::Publisher pubIcpKeyFrames;
        ros::Publisher pubRecentKeyFrames;
        ros::Publisher pubRegisteredCloud;

        ros::Subscriber subLaserCloudRaw;
        ros::Subscriber subLaserCloudCornerLast;
        ros::Subscriber subLaserCloudSurfLast;
        ros::Subscriber subOutlierCloudLast;
        ros::Subscriber subLaserOdometry;
        ros::Subscriber subImu;

        nav_msgs::Odometry odomAftMapped;
        tf::StampedTransform aftMappedTrans;
        tf::TransformBroadcaster tfBroadcaster;

        vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
        vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
        vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;

        deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
        deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
        deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
        int latestFrameID;

        vector<int> surroundingExistingKeyPosesID;
        deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
        deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
        deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;

        PointType previousRobotPosPoint;
        PointType currentRobotPosPoint;

        pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
        pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;


        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

        pcl::PointCloud<PointType>::Ptr laserCloudRaw;
        pcl::PointCloud<PointType>::Ptr laserCloudRawDS;
        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
        pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

        pcl::PointCloud<PointType>::Ptr laserCloudOutlierLast; // corner feature set from odoOptimization
        pcl::PointCloud<PointType>::Ptr laserCloudOutlierLastDS; // corner feature set from odoOptimization

        pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLast; // surf feature set from odoOptimization
        pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLastDS; // downsampled corner featuer set from odoOptimization

        pcl::PointCloud<PointType>::Ptr laserCloudOri;
        pcl::PointCloud<PointType>::Ptr coeffSel;

        pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
        pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

        pcl::PointCloud<PointType>::Ptr RSlatestSurfKeyFrameCloud; // giseop, RS: radius search
        pcl::PointCloud<PointType>::Ptr RSnearHistorySurfKeyFrameCloud;
        pcl::PointCloud<PointType>::Ptr RSnearHistorySurfKeyFrameCloudDS;

        pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
        pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
        pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloud;
        pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloudDS;

        pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
        pcl::PointCloud<PointType>::Ptr SClatestSurfKeyFrameCloud; // giseop, SC: scan context
        pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pcl::VoxelGrid<PointType> downSizeFilterScancontext;
        pcl::VoxelGrid<PointType> downSizeFilterCorner;
        pcl::VoxelGrid<PointType> downSizeFilterSurf;
        pcl::VoxelGrid<PointType> downSizeFilterOutlier;
        pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames; // for histor key frames of loop closure
        pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization

        double timeLaserCloudCornerLast;
        double timeLaserCloudSurfLast;
        double timeLaserOdometry;
        double timeLaserCloudOutlierLast;
        double timeLastGloalMapPublish;

        bool newLaserCloudCornerLast;
        bool newLaserCloudSurfLast;
        bool newLaserOdometry;
        bool newLaserCloudOutlierLast;


        float transformLast[6];
        float transformSum[6];
        float transformIncre[6];
        float transformTobeMapped[6];
        float transformBefMapped[6];
        float transformAftMapped[6];


        int imuPointerFront;
        int imuPointerLast;

        double imuTime[imuQueLength];
        float imuRoll[imuQueLength];
        float imuPitch[imuQueLength];

        std::mutex mtx;

        double timeLastProcessing;

        PointType pointOri, pointSel, pointProj, coeff;

        cv::Mat matA0;
        cv::Mat matB0;
        cv::Mat matX0;

        cv::Mat matA1;
        cv::Mat matD1;
        cv::Mat matV1;

        bool isDegenerate;
        cv::Mat matP;

        int laserCloudCornerFromMapDSNum;
        int laserCloudSurfFromMapDSNum;
        int laserCloudCornerLastDSNum;
        int laserCloudSurfLastDSNum;
        int laserCloudOutlierLastDSNum;
        int laserCloudSurfTotalLastDSNum;

        bool potentialLoopFlag;
        double timeSaveFirstCurrentScanForLoopClosure;
        int RSclosestHistoryFrameID;
        int SCclosestHistoryFrameID; // giseop
        int latestFrameIDLoopCloure;
        float yawDiffRad;

        bool aLoopIsClosed;

        float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
        float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

        std::string pcd_file_path;
        float init_tf_x, init_tf_y, init_tf_z, init_tf_roll, init_tf_pitch, init_tf_yaw;

        // // loop detector
        SC2::SCManager scManager;
    };
}


#endif //SRC_MAP_OPTMIZATION_H
