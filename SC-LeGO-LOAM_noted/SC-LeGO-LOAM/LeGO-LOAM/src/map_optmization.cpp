#include "lego_loam/map_optmization.h"

namespace lego_loam {
    mapOptimization::mapOptimization() : nh("~") {
        //  用于闭环图优化的参数设置，使用gtsam库
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        InitParams();
        //  subscriber
        subLaserCloudRaw = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 2,
                                                                  &mapOptimization::laserCloudRawHandler, this);
        subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2,
                                                                         &mapOptimization::laserCloudCornerLastHandler,
                                                                         this);
        subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2,
                                                                       &mapOptimization::laserCloudSurfLastHandler,
                                                                       this);
        subOutlierCloudLast = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2,
                                                                     &mapOptimization::laserCloudOutlierLastHandler,
                                                                     this);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5,
                                                            &mapOptimization::laserOdometryHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &mapOptimization::imuHandler, this);

        //  publisher
        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
        pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);

        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/history_cloud", 2);
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);
        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);
        pubRegisteredCloud = nh.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 2);

        float filter_size;
        downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
        filter_size = 0.5;
        downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
        filter_size = 0.3;
        downSizeFilterSurf.setLeafSize(filter_size, filter_size, filter_size); // default 0.4;
        downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

        filter_size = 0.3;
        downSizeFilterHistoryKeyFrames.setLeafSize(filter_size, filter_size,
                                                   filter_size); // default 0.4; for histor key frames of loop closure
        filter_size = 1.0;
        downSizeFilterSurroundingKeyPoses.setLeafSize(filter_size, filter_size,
                                                      filter_size); // default 1; for surrounding key poses of scan-to-map optimization

        downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for global map visualization

        odomAftMapped.header.frame_id = "/camera_init";
        odomAftMapped.child_frame_id = "/aft_mapped";

        aftMappedTrans.frame_id_ = "/camera_init";
        aftMappedTrans.child_frame_id_ = "/aft_mapped";

        //  nh.param<std::string>("file_dir", pcd_file_path, "/tmp/");

        pcd_file_path = package_path + "/map/";

        init_tf_x = config_node["x"].as<float>();
        init_tf_y = config_node["y"].as<float>();
        init_tf_z = config_node["z"].as<float>();
        init_tf_roll = config_node["roll"].as<float>();
        init_tf_pitch = config_node["pitch"].as<float>();
        init_tf_yaw = config_node["yaw"].as<float>();

        allocateMemory();
    }

    void mapOptimization::InitParams() {
        std::cout << "***************** mapOptimization *************************" << std::endl;
        std::cout << "pointCloudTopic : " << pointCloudTopic << std::endl;
        std::cout << "imuTopic : " << imuTopic << std::endl;
        std::cout << "pcd_file_path : " << pcd_file_path << std::endl;
        std::cout << "imuTopic : " << imuTopic << std::endl;
        std::cout << "******************** mapOptimization **********************" << std::endl;
    }

    void mapOptimization::run() {
        //  有新数据进来，才执行后续
        if (newLaserCloudCornerLast && std::abs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
            newLaserCloudSurfLast && std::abs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
            newLaserCloudOutlierLast && std::abs(timeLaserCloudOutlierLast - timeLaserOdometry) < 0.005 &&
            newLaserOdometry) {

            newLaserCloudCornerLast = false;
            newLaserCloudSurfLast = false;
            newLaserCloudOutlierLast = false;
            newLaserOdometry = false;

            std::lock_guard<std::mutex> lock(mtx);
            if (timeLaserOdometry - timeLastProcessing >= mappingProcessInterval) {
                timeLastProcessing = timeLaserOdometry;

                transformAssociateToMap();  //  坐标系->map

                extractSurroundingKeyFrames();  //  暂时不懂

                downsampleCurrentScan();

                scan2MapOptimization();

                saveKeyFramesAndFactor();

                correctPoses();

                publishTF();

                publishKeyPosesAndFrames();

                clearCloud();
            }
        }
    }

    void mapOptimization::allocateMemory() {

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
        surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

        laserCloudRaw.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudRawDS.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(
                new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(
                new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization
        laserCloudOutlierLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudOutlierLastDS.reset(
                new pcl::PointCloud<PointType>()); // downsampled corner feature set from odoOptimization
        laserCloudSurfTotalLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudSurfTotalLastDS.reset(
                new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());


        nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
        SCnearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        SCnearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        SClatestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        RSlatestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>()); // giseop
        RSnearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        RSnearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
        globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
        globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
        globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
        globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

        timeLaserCloudCornerLast = 0;
        timeLaserCloudSurfLast = 0;
        timeLaserOdometry = 0;
        timeLaserCloudOutlierLast = 0;
        timeLastGloalMapPublish = 0;

        timeLastProcessing = -1;

        newLaserCloudCornerLast = false;
        newLaserCloudSurfLast = false;

        newLaserOdometry = false;
        newLaserCloudOutlierLast = false;

        for (int i = 0; i < 6; ++i) {
            transformLast[i] = 0;
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformTobeMapped[i] = 0;
            transformBefMapped[i] = 0;
            transformAftMapped[i] = 0;
        }

        imuPointerFront = 0;
        imuPointerLast = -1;

        for (int i = 0; i < imuQueLength; ++i) {
            imuTime[i] = 0;
            imuRoll[i] = 0;
            imuPitch[i] = 0;
        }

        gtsam::Vector Vector6(6);
        Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
        priorNoise = noiseModel::Diagonal::Variances(Vector6);
        odometryNoise = noiseModel::Diagonal::Variances(Vector6);

        matA0 = cv::Mat(5, 3, CV_32F, cv::Scalar::all(0));
        matB0 = cv::Mat(5, 1, CV_32F, cv::Scalar::all(-1));
        matX0 = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));

        matA1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
        matD1 = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
        matV1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

        isDegenerate = false;
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        laserCloudCornerFromMapDSNum = 0;
        laserCloudSurfFromMapDSNum = 0;
        laserCloudCornerLastDSNum = 0;
        laserCloudSurfLastDSNum = 0;
        laserCloudOutlierLastDSNum = 0;
        laserCloudSurfTotalLastDSNum = 0;

        potentialLoopFlag = false;
        aLoopIsClosed = false;

        latestFrameID = 0;
    }

    void mapOptimization::transformAssociateToMap() {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
                   - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
                   + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz)
                    - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly)
                                     - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly)
                    - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz)
                                     - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
        transformTobeMapped[0] = -asin(srx);

        float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly)
                               - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx)
                       - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz)
                                        + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) -
                                        calx * cblx * cbly * saly)
                       + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz)
                                        + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) +
                                        calx * cblx * saly * sbly);
        float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz)
                               - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx)
                       + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx)
                                        + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) +
                                        calx * caly * cblx * cbly)
                       - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly)
                                        + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) -
                                        calx * caly * cblx * sbly);
        transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                       crycrx / cos(transformTobeMapped[0]));

        float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz)
                                                             - calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                                                             cblx * cbly * salx)
                       - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly)
                                                               - calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                                                               cblx * salx * sbly)
                       + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
        float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly)
                                                             - calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                                                             cblx * salx * sbly)
                       - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz)
                                                               - calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                                                               cblx * cbly * salx)
                       + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
        transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                       crzcrx / cos(transformTobeMapped[0]));

        x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
        y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
        z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

        transformTobeMapped[3] = transformAftMapped[3]
                                 - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
        transformTobeMapped[4] = transformAftMapped[4] - y2;
        transformTobeMapped[5] = transformAftMapped[5]
                                 - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
    }

    void mapOptimization::transformUpdate() {
        if (imuPointerLast >= 0) {
            float imuRollLast = 0, imuPitchLast = 0;
            while (imuPointerFront != imuPointerLast) {
                if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
                    break;
                }
                imuPointerFront = (imuPointerFront + 1) % imuQueLength;
            }

            if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
                imuRollLast = imuRoll[imuPointerFront];
                imuPitchLast = imuPitch[imuPointerFront];
            } else {
                int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack])
                                   / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod)
                                  / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

                imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
                imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
            }

            transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
            transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
        }

        for (int i = 0; i < 6; i++) {
            transformBefMapped[i] = transformSum[i];
            transformAftMapped[i] = transformTobeMapped[i];
        }
    }

    void mapOptimization::updatePointAssociateToMapSinCos() {
        cRoll = cos(transformTobeMapped[0]);
        sRoll = sin(transformTobeMapped[0]);

        cPitch = cos(transformTobeMapped[1]);
        sPitch = sin(transformTobeMapped[1]);

        cYaw = cos(transformTobeMapped[2]);
        sYaw = sin(transformTobeMapped[2]);

        tX = transformTobeMapped[3];
        tY = transformTobeMapped[4];
        tZ = transformTobeMapped[5];
    }

    void mapOptimization::pointAssociateToMap(PointType const *const pi, PointType *const po) {
        float x1 = cYaw * pi->x - sYaw * pi->y;
        float y1 = sYaw * pi->x + cYaw * pi->y;
        float z1 = pi->z;

        float x2 = x1;
        float y2 = cRoll * y1 - sRoll * z1;
        float z2 = sRoll * y1 + cRoll * z1;

        po->x = cPitch * x2 + sPitch * z2 + tX;
        po->y = y2 + tY;
        po->z = -sPitch * x2 + cPitch * z2 + tZ;
        po->intensity = pi->intensity;
    }

    void mapOptimization::updateTransformPointCloudSinCos(PointTypePose *tIn) {

        ctRoll = cos(tIn->roll);
        stRoll = sin(tIn->roll);

        ctPitch = cos(tIn->pitch);
        stPitch = sin(tIn->pitch);

        ctYaw = cos(tIn->yaw);
        stYaw = sin(tIn->yaw);

        tInX = tIn->x;
        tInY = tIn->y;
        tInZ = tIn->z;
    }

    pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn) {
        // !!! DO NOT use pcl for point cloud transformation, results are not accurate
        // Reason: unkown
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i) {

            pointFrom = &cloudIn->points[i];
            float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
            float y1 = stYaw * pointFrom->x + ctYaw * pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = ctRoll * y1 - stRoll * z1;
            float z2 = stRoll * y1 + ctRoll * z1;

            pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
            pointTo.y = y2 + tInY;
            pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr
    mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn) {

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i) {

            pointFrom = &cloudIn->points[i];
            float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
            float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw) * pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
            float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

            pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
            pointTo.y = y2 + transformIn->y;
            pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    void mapOptimization::laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
        timeLaserCloudOutlierLast = msg->header.stamp.toSec();
        laserCloudOutlierLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudOutlierLast);
        newLaserCloudOutlierLast = true;
    }

    void mapOptimization::laserCloudRawHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
        laserCloudRaw->clear();
        pcl::fromROSMsg(*msg, *laserCloudRaw);
    }

    void mapOptimization::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
        timeLaserCloudCornerLast = msg->header.stamp.toSec();
        laserCloudCornerLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudCornerLast);
        newLaserCloudCornerLast = true;
    }

    void mapOptimization::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg) {
        timeLaserCloudSurfLast = msg->header.stamp.toSec();
        laserCloudSurfLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudSurfLast);
        newLaserCloudSurfLast = true;
    }

    void mapOptimization::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry) {
        timeLaserOdometry = laserOdometry->header.stamp.toSec();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;
        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;
        newLaserOdometry = true;
    }

    void mapOptimization::imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn) {
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        imuPointerLast = (imuPointerLast + 1) % imuQueLength;
        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
    }

    void mapOptimization::publishTF() {

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
        odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
        odomAftMapped.pose.pose.orientation.z = geoQuat.x;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.pose.pose.position.x = transformAftMapped[3];
        odomAftMapped.pose.pose.position.y = transformAftMapped[4];
        odomAftMapped.pose.pose.position.z = transformAftMapped[5];
        odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
        odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
        odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
        odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
        odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
        odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
        pubOdomAftMapped.publish(odomAftMapped);

        aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
        tfBroadcaster.sendTransform(aftMappedTrans);
    }

    PointTypePose mapOptimization::trans2PointTypePose(float transformIn[]) {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }

    void mapOptimization::publishKeyPosesAndFrames() {

        if (pubKeyPoses.getNumSubscribers() != 0) {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubKeyPoses.publish(cloudMsgTemp);
        }

        if (pubRecentKeyFrames.getNumSubscribers() != 0) {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubRecentKeyFrames.publish(cloudMsgTemp);
        }

        if (pubRegisteredCloud.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfTotalLast, &thisPose6D);

            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudOut, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubRegisteredCloud.publish(cloudMsgTemp);
        }
    }

    void mapOptimization::TransformAndSaveMap() {

        Eigen::Translation3f tf_t(init_tf_x, init_tf_y, init_tf_z);         // tl: translation
        Eigen::AngleAxisf rot_x(init_tf_roll, Eigen::Vector3f::UnitX());    // rot: rotation
        Eigen::AngleAxisf rot_y(init_tf_pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rot_z(init_tf_yaw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f map_to_init_trans_matrix = (tf_t * rot_z * rot_y * rot_x).matrix();

        pcl::PointCloud<PointType>::Ptr transformed_pc_ptr(new pcl::PointCloud<PointType>());

        pcl::transformPointCloud(*globalMapKeyFramesDS, *transformed_pc_ptr, map_to_init_trans_matrix);

        pcl::io::savePCDFileASCII(pcd_file_path + "finalCloud.pcd", *transformed_pc_ptr);

        // 后面的各种特征点地图可以注释掉
        pcl::PointCloud<PointType>::Ptr cornerMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cornerMapCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceMapCloudDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < cornerCloudKeyFrames.size(); i++) {
            *cornerMapCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *surfaceMapCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *surfaceMapCloud += *transformPointCloud(outlierCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        }

        downSizeFilterCorner.setInputCloud(cornerMapCloud);
        downSizeFilterCorner.filter(*cornerMapCloudDS);
        downSizeFilterSurf.setInputCloud(surfaceMapCloud);
        downSizeFilterSurf.filter(*surfaceMapCloudDS);

        pcl::transformPointCloud(*globalMapKeyFramesDS, *transformed_pc_ptr, map_to_init_trans_matrix);

        pcl::io::savePCDFileASCII(pcd_file_path + "cornerMap.pcd", *cornerMapCloudDS);
        pcl::io::savePCDFileASCII(pcd_file_path + "surfaceMap.pcd", *surfaceMapCloudDS);
        pcl::io::savePCDFileASCII(pcd_file_path + "trajectory.pcd", *cloudKeyPoses3D);

    }

    void mapOptimization::visualizeGlobalMapThread() {
        ros::Rate rate(0.2);
        while (ros::ok()) {
            rate.sleep();
            publishGlobalMap();
        }
        // save final point cloud
        TransformAndSaveMap();
    }

    void mapOptimization::publishGlobalMap() {

        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;
        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, globalMapVisualizationSearchRadius, pointSearchIndGlobalMap,
                                      pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        // extract visualized and downsampled key frames
        for (int i = 0; i < globalMapKeyPosesDS->points.size(); ++i) {
            int thisKeyInd = (int) globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
                                                        &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],
                                                        &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(outlierCloudKeyFrames[thisKeyInd],
                                                        &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = "/camera_init";
        pubLaserCloudSurround.publish(cloudMsgTemp);

        globalMapKeyPoses->clear();
        globalMapKeyPosesDS->clear();
        globalMapKeyFrames->clear();
        // globalMapKeyFramesDS->clear();
    }


    void mapOptimization::loopClosureThread() {
        //  回环检测标志位
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(1);
        while (ros::ok()) {
            rate.sleep();
            performLoopClosure();
        }
    } // loopClosureThread


    bool mapOptimization::detectLoopClosure() {

        // 资源分配时初始化
        // 在互斥量被析构前不解锁
        std::lock_guard<std::mutex> lock(mtx);

        /*
         * 邻域搜索闭环
         * 1. xyz distance-based radius search (contained in the original LeGO LOAM code)
         * - for fine-stichting trajectories (for not-recognized nodes within scan context search)
         */
        RSlatestSurfKeyFrameCloud->clear();
        RSnearHistorySurfKeyFrameCloud->clear();
        RSnearHistorySurfKeyFrameCloudDS->clear();

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;  //  搜索完的邻域点对应的索引
        std::vector<float> pointSearchSqDisLoop;  //  搜索完的邻域点与当前点的欧氏距离
        kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
        //  0：返回的邻域个数，为0表示返回全部的邻域点
        kdtreeHistoryKeyPoses->radiusSearch(currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop,
                                            pointSearchSqDisLoop, 0);

        RSclosestHistoryFrameID = -1;
        int curMinID = 1000000;
        // policy: take Oldest one (to fix error of the whole trajectory)
        for (int i = 0; i < pointSearchIndLoop.size(); ++i) {
            int id = pointSearchIndLoop[i];
            //  时间差值大于30s, 认为是闭环
            if (abs(cloudKeyPoses6D->points[id].time - timeLaserOdometry) > 30.0) {
                // RSclosestHistoryFrameID = id;
                // break;
                if (id < curMinID) {
                    curMinID = id;
                    RSclosestHistoryFrameID = curMinID;
                }
            }
        }
        if (RSclosestHistoryFrameID == -1) {
            // Do nothing here
            // then, do the next check: Scan context-based search
            // not return false here;
        } else {
            //  检测到回环了会保存四种点云
            // save latest key frames
            latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
            // 点云的xyz坐标进行坐标系变换(分别绕xyz轴旋转)
            *RSlatestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure],
                                                               &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
            *RSlatestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure],
                                                               &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
            // latestSurfKeyFrameCloud中存储的是下面公式计算后的index(intensity):
            // thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
            // 滤掉latestSurfKeyFrameCloud中index<0的点??? index值会小于0?
            pcl::PointCloud<PointType>::Ptr RShahaCloud(new pcl::PointCloud<PointType>());
            int cloudSize = RSlatestSurfKeyFrameCloud->points.size();
            for (int i = 0; i < cloudSize; ++i) {
                if ((int) RSlatestSurfKeyFrameCloud->points[i].intensity >= 0) {
                    RShahaCloud->push_back(RSlatestSurfKeyFrameCloud->points[i]);
                }
            }
            RSlatestSurfKeyFrameCloud->clear();
            *RSlatestSurfKeyFrameCloud = *RShahaCloud;

            // save history near key frames
            // historyKeyframeSearchNum在utility.h中定义为25，前后25个点进行变换
            for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j) {
                if (RSclosestHistoryFrameID + j < 0 || RSclosestHistoryFrameID + j > latestFrameIDLoopCloure)
                    continue;
                // 要求closestHistoryFrameID + j在0到cloudKeyPoses3D->points.size()-1之间,不能超过索引
                *RSnearHistorySurfKeyFrameCloud += *transformPointCloud(
                        cornerCloudKeyFrames[RSclosestHistoryFrameID + j],
                        &cloudKeyPoses6D->points[RSclosestHistoryFrameID + j]);
                *RSnearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[RSclosestHistoryFrameID + j],
                                                                        &cloudKeyPoses6D->points[
                                                                                RSclosestHistoryFrameID + j]);
            }
            //  下采样
            downSizeFilterHistoryKeyFrames.setInputCloud(RSnearHistorySurfKeyFrameCloud);
            downSizeFilterHistoryKeyFrames.filter(*RSnearHistorySurfKeyFrameCloudDS);
        }

        /*
         * 2. Scan context-based global localization
         */
        SClatestSurfKeyFrameCloud->clear();
        SCnearHistorySurfKeyFrameCloud->clear();
        SCnearHistorySurfKeyFrameCloudDS->clear();

        // std::lock_guard<std::mutex> lock(mtx);
        latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
        SCclosestHistoryFrameID = -1; // init with -1

        //  这里检测回环
        auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
        SCclosestHistoryFrameID = detectResult.first;
        yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)

        // if all close, reject
        if (SCclosestHistoryFrameID == -1) {
            return false;
        }

        // SC检测到了回环
        // save latest key frames: query ptcloud (corner points + surface points)
        // NOTE: using "closestHistoryFrameID" to make same root of submap points to get a direct relative between
        // the query point cloud (latestSurfKeyFrameCloud) and the map (nearHistorySurfKeyFrameCloud). by giseop
        // i.e., set the query point cloud within mapside's local coordinate
        *SClatestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure],
                                                           &cloudKeyPoses6D->points[SCclosestHistoryFrameID]);
        *SClatestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure],
                                                           &cloudKeyPoses6D->points[SCclosestHistoryFrameID]);

        pcl::PointCloud<PointType>::Ptr SChahaCloud(new pcl::PointCloud<PointType>());
        int cloudSize = SClatestSurfKeyFrameCloud->points.size();
        for (int i = 0; i < cloudSize; ++i) {
            if ((int) SClatestSurfKeyFrameCloud->points[i].intensity >= 0) {
                SChahaCloud->push_back(SClatestSurfKeyFrameCloud->points[i]);
            }
        }
        SClatestSurfKeyFrameCloud->clear();
        *SClatestSurfKeyFrameCloud = *SChahaCloud;

        // save history near key frames: map ptcloud (icp to query ptcloud)
        for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j) {
            if (SCclosestHistoryFrameID + j < 0 || SCclosestHistoryFrameID + j > latestFrameIDLoopCloure)
                continue;
            *SCnearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[SCclosestHistoryFrameID + j],
                                                                    &cloudKeyPoses6D->points[SCclosestHistoryFrameID +
                                                                                             j]);
            *SCnearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[SCclosestHistoryFrameID + j],
                                                                    &cloudKeyPoses6D->points[SCclosestHistoryFrameID +
                                                                                             j]);
        }
        downSizeFilterHistoryKeyFrames.setInputCloud(SCnearHistorySurfKeyFrameCloud);
        downSizeFilterHistoryKeyFrames.filter(*SCnearHistorySurfKeyFrameCloudDS);

        // // optional: publish history near key frames
        // if (pubHistoryKeyFrames.getNumSubscribers() != 0){
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
        //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        //     cloudMsgTemp.header.frame_id = "/camera_init";
        //     pubHistoryKeyFrames.publish(cloudMsgTemp);
        // }

        return true;
    } // detectLoopClosure


    void mapOptimization::performLoopClosure(void) {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // try to find close key frame if there are any
        if (potentialLoopFlag == false) {
            if (detectLoopClosure() == true) {
                std::cout << std::endl;
                potentialLoopFlag = true; // find some key frames that is old enough or close enough for loop closure
                timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
            }
            if (potentialLoopFlag == false) {
                return;
            }
        }

        // reset the flag first no matter icp successes or not
        potentialLoopFlag = false;

        // *****
        // Main
        // *****
        // make common variables at forward
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionCameraFrame;
        float noiseScore = 0.5; // constant is ok...
        gtsam::Vector Vector6(6);
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        constraintNoise = noiseModel::Diagonal::Variances(Vector6);
        robustNoiseModel = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
                gtsam::noiseModel::Diagonal::Variances(Vector6)
        ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

        bool isValidRSloopFactor = false;
        bool isValidSCloopFactor = false;

        /*
         * 1. RS loop factor (radius search)
         * 将RS检测到的历史帧和当前帧匹配，求transform, 作为约束边
         */
        if (RSclosestHistoryFrameID != -1) {
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(100);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Align clouds
            icp.setInputSource(RSlatestSurfKeyFrameCloud);
            icp.setInputTarget(RSnearHistorySurfKeyFrameCloudDS);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);

            //  通过score阈值判定icp是否匹配成功
            std::cout << "[RS] ICP fit score: " << icp.getFitnessScore() << std::endl;
            if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
                std::cout << "[RS] Reject this loop (bad icp fit score, > " << historyKeyframeFitnessScore << ")"
                          << std::endl;
                isValidRSloopFactor = false;
            } else {
                std::cout << "[RS] The detected loop factor is added between Current [ " << latestFrameIDLoopCloure
                          << " ] and RS nearest [ " << RSclosestHistoryFrameID << " ]" << std::endl;
                isValidRSloopFactor = true;
            }

            //  这里RS检测成功，加入约束边
            if (isValidRSloopFactor == true) {
                correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
                pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
                Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);
                //  transform from world origin to wrong pose
                Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(
                        cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
                //  transform from world origin to corrected pose
                Eigen::Affine3f tCorrect =
                        correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
                pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
                gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[RSclosestHistoryFrameID]);
                gtsam::Vector Vector6(6);

                std::lock_guard<std::mutex> lock(mtx);
                gtSAMgraph.add(
                        BetweenFactor<Pose3>(latestFrameIDLoopCloure, RSclosestHistoryFrameID, poseFrom.between(poseTo),
                                             robustNoiseModel));
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
        }

        /*
         * 2. SC loop factor (scan context)
         * SC检测成功，进行icp匹配
         */
        if (SCclosestHistoryFrameID != -1) {
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(100);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Align clouds
            // Eigen::Affine3f icpInitialMatFoo = pcl::getTransformation(0, 0, 0, yawDiffRad, 0, 0); // because within cam coord: (z, x, y, yaw, roll, pitch)
            // Eigen::Matrix4f icpInitialMat = icpInitialMatFoo.matrix();
            icp.setInputSource(SClatestSurfKeyFrameCloud);
            icp.setInputTarget(SCnearHistorySurfKeyFrameCloudDS);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);
            // icp.align(*unused_result, icpInitialMat); // PCL icp non-eye initial is bad ... don't use (LeGO LOAM author also said pcl transform is weird.)

            std::cout << "[SC] ICP fit score: " << icp.getFitnessScore() << std::endl;
            if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
                std::cout << "[SC] Reject this loop (bad icp fit score, > " << historyKeyframeFitnessScore << ")"
                          << std::endl;
                isValidSCloopFactor = false;
            } else {
                std::cout << "[SC] The detected loop factor is added between Current [ " << latestFrameIDLoopCloure
                          << " ] and SC nearest [ " << SCclosestHistoryFrameID << " ]" << std::endl;
                isValidSCloopFactor = true;
            }

            // icp匹配成功也加入约束边
            if (isValidSCloopFactor == true) {
                correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
                pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
                gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

                std::lock_guard<std::mutex> lock(mtx);
                // gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom.between(poseTo), constraintNoise)); // original
                gtSAMgraph.add(
                        BetweenFactor<Pose3>(latestFrameIDLoopCloure, SCclosestHistoryFrameID, poseFrom.between(poseTo),
                                             robustNoiseModel)); // giseop
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
        }

        // just for visualization
        // // publish corrected cloud
//         if (pubIcpKeyFrames.getNumSubscribers() != 0){
//             pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
//             pcl::transformPointCloud (*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
//             sensor_msgs::PointCloud2 cloudMsgTemp;
//             pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
//             cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
//             cloudMsgTemp.header.frame_id = "/camera_init";
//             pubIcpKeyFrames.publish(cloudMsgTemp);
//         }

        // flagging
        aLoopIsClosed = true;

    } // performLoopClosure


    Pose3 mapOptimization::pclPointTogtsamPose3(PointTypePose thisPoint) { // camera frame to lidar frame
        return Pose3(Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll), double(thisPoint.pitch)),
                     Point3(double(thisPoint.z), double(thisPoint.x), double(thisPoint.y)));
    }

    Eigen::Affine3f
    mapOptimization::pclPointToAffine3fCameraToLidar(PointTypePose thisPoint) { // camera frame to lidar frame
        return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y, thisPoint.yaw, thisPoint.roll,
                                      thisPoint.pitch);
    }

    void mapOptimization::extractSurroundingKeyFrames() {

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        if (loopClosureEnableFlag == true) {
            // only use recent key poses for graph building
            if (recentCornerCloudKeyFrames.size() <
                surroundingKeyframeSearchNum) { // queue is not full (the beginning of mapping or a loop is just closed)
                // clear recent key frames queue
                recentCornerCloudKeyFrames.clear();
                recentSurfCloudKeyFrames.clear();
                recentOutlierCloudKeyFrames.clear();
                int numPoses = cloudKeyPoses3D->points.size();
                for (int i = numPoses - 1; i >= 0; --i) {
                    int thisKeyInd = (int) cloudKeyPoses3D->points[i].intensity;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    // extract surrounding map
                    recentCornerCloudKeyFrames.push_front(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                    recentSurfCloudKeyFrames.push_front(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                    recentOutlierCloudKeyFrames.push_front(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
                    if (recentCornerCloudKeyFrames.size() >= surroundingKeyframeSearchNum)
                        break;
                }
            } else {  // queue is full, pop the oldest key frame and push the latest key frame
                if (latestFrameID != cloudKeyPoses3D->points.size() -
                                     1) {  // if the robot is not moving, no need to update recent frames

                    recentCornerCloudKeyFrames.pop_front();
                    recentSurfCloudKeyFrames.pop_front();
                    recentOutlierCloudKeyFrames.pop_front();
                    // push latest scan to the end of queue
                    latestFrameID = cloudKeyPoses3D->points.size() - 1;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[latestFrameID];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    recentCornerCloudKeyFrames.push_back(transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
                    recentSurfCloudKeyFrames.push_back(transformPointCloud(surfCloudKeyFrames[latestFrameID]));
                    recentOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
                }
            }

            for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i) {
                *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
                *laserCloudSurfFromMap += *recentSurfCloudKeyFrames[i];
                *laserCloudSurfFromMap += *recentOutlierCloudKeyFrames[i];
            }
        } else {
            surroundingKeyPoses->clear();
            surroundingKeyPosesDS->clear();
            // extract all the nearby key poses and downsample them
            kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
            kdtreeSurroundingKeyPoses->radiusSearch(currentRobotPosPoint, (double) surroundingKeyframeSearchRadius,
                                                    pointSearchInd, pointSearchSqDis, 0);
            for (int i = 0; i < pointSearchInd.size(); ++i)
                surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
            downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
            downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
            // delete key frames that are not in surrounding region
            int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
            for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
                bool existingFlag = false;
                for (int j = 0; j < numSurroundingPosesDS; ++j) {
                    if (surroundingExistingKeyPosesID[i] == (int) surroundingKeyPosesDS->points[j].intensity) {
                        existingFlag = true;
                        break;
                    }
                }
                if (existingFlag == false) {
                    surroundingExistingKeyPosesID.erase(surroundingExistingKeyPosesID.begin() + i);
                    surroundingCornerCloudKeyFrames.erase(surroundingCornerCloudKeyFrames.begin() + i);
                    surroundingSurfCloudKeyFrames.erase(surroundingSurfCloudKeyFrames.begin() + i);
                    surroundingOutlierCloudKeyFrames.erase(surroundingOutlierCloudKeyFrames.begin() + i);
                    --i;
                }
            }
            // add new key frames that are not in calculated existing key frames
            for (int i = 0; i < numSurroundingPosesDS; ++i) {
                bool existingFlag = false;
                for (auto iter = surroundingExistingKeyPosesID.begin();
                     iter != surroundingExistingKeyPosesID.end(); ++iter) {
                    if ((*iter) == (int) surroundingKeyPosesDS->points[i].intensity) {
                        existingFlag = true;
                        break;
                    }
                }
                if (existingFlag == true) {
                    continue;
                } else {
                    int thisKeyInd = (int) surroundingKeyPosesDS->points[i].intensity;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    surroundingExistingKeyPosesID.push_back(thisKeyInd);
                    surroundingCornerCloudKeyFrames.push_back(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                    surroundingSurfCloudKeyFrames.push_back(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                    surroundingOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
                }
            }

            for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
                *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
                *laserCloudSurfFromMap += *surroundingSurfCloudKeyFrames[i];
                *laserCloudSurfFromMap += *surroundingOutlierCloudKeyFrames[i];
            }
        }
        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
    }

    void mapOptimization::downsampleCurrentScan() {

        laserCloudRawDS->clear();
        downSizeFilterScancontext.setInputCloud(laserCloudRaw);
        downSizeFilterScancontext.filter(*laserCloudRawDS);

        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();
        // std::cout << "laserCloudCornerLastDSNum: " << laserCloudCornerLastDSNum << std::endl;

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();
        // std::cout << "laserCloudSurfLastDSNum: " << laserCloudSurfLastDSNum << std::endl;

        laserCloudOutlierLastDS->clear();
        downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
        downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
        laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

        laserCloudSurfTotalLast->clear();
        laserCloudSurfTotalLastDS->clear();
        *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
        *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
        downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
        downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
        laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
    }

    void mapOptimization::cornerOptimization(int iterCount) {

        updatePointAssociateToMapSinCos();
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
                                      * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
                                      + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
                                        * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
                                      + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))
                                        * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                    float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
                                + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
                                 - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
                                 + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOri->push_back(pointOri);
                        coeffSel->push_back(coeff);
                    }
                }
            }
        }
    }

    void mapOptimization::surfOptimization(int iterCount) {
        updatePointAssociateToMapSinCos();
        for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++) {
            pointOri = laserCloudSurfTotalLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0.at<float>(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0.at<float>(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0.at<float>(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                float pa = matX0.at<float>(0, 0);
                float pb = matX0.at<float>(1, 0);
                float pc = matX0.at<float>(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                                                              + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOri->push_back(pointOri);
                        coeffSel->push_back(coeff);
                    }
                }
            }
        }
    }

    bool mapOptimization::LMOptimization(int iterCount) {
        float srx = sin(transformTobeMapped[0]);
        float crx = cos(transformTobeMapped[0]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[2]);
        float crz = cos(transformTobeMapped[2]);

        int laserCloudSelNum = laserCloudOri->points.size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        for (int i = 0; i < laserCloudSelNum; i++) {
            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x
                        + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y
                        + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) *
                          coeff.z;

            float ary = ((cry * srx * srz - crz * sry) * pointOri.x
                         + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x
                        + ((-cry * crz - srx * sry * srz) * pointOri.x
                           + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

            float arz =
                    ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x
                    + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y
                    +
                    ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                pow(matX.at<float>(3, 0) * 100, 2) +
                pow(matX.at<float>(4, 0) * 100, 2) +
                pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true;
        }
        return false;
    }

    void mapOptimization::scan2MapOptimization() {

        // laserCloudCornerFromMapDSNum是extractSurroundingKeyFrames()函数最后降采样得到的coner点云数
        // laserCloudSurfFromMapDSNum是extractSurroundingKeyFrames()函数降采样得到的surface点云数
        if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {

            // laserCloudCornerFromMapDS和laserCloudSurfFromMapDS的来源有2个：
            // 当有闭环时，来源是：recentCornerCloudKeyFrames，没有闭环时，来源surroundingCornerCloudKeyFrames
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 10; iterCount++) {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization(iterCount);
                surfOptimization(iterCount);

                if (LMOptimization(iterCount) == true)
                    break;
            }
            // 迭代结束更新相关的转移矩阵
            transformUpdate();
        }
    }

    void mapOptimization::saveKeyFramesAndFactor() {

        //  此函数保存关键帧和factor
        currentRobotPosPoint.x = transformAftMapped[3];
        currentRobotPosPoint.y = transformAftMapped[4];
        currentRobotPosPoint.z = transformAftMapped[5];

        bool saveThisKeyFrame = true;
        if (sqrt((previousRobotPosPoint.x - currentRobotPosPoint.x) * (previousRobotPosPoint.x - currentRobotPosPoint.x)
                 +
                 (previousRobotPosPoint.y - currentRobotPosPoint.y) * (previousRobotPosPoint.y - currentRobotPosPoint.y)
                 + (previousRobotPosPoint.z - currentRobotPosPoint.z) *
                   (previousRobotPosPoint.z - currentRobotPosPoint.z)) < 0.3) { // save keyframe every 0.3 meter
            saveThisKeyFrame = false;
        }
        if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty())
            return;

        previousRobotPosPoint = currentRobotPosPoint;
        /**
         * update grsam graph
         */
        if (cloudKeyPoses3D->points.empty()) {
            gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                                                                    transformTobeMapped[1]),
                                                       Point3(transformTobeMapped[5], transformTobeMapped[3],
                                                              transformTobeMapped[4])), priorNoise));
            initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                                                         transformTobeMapped[1]),
                                            Point3(transformTobeMapped[5], transformTobeMapped[3],
                                                   transformTobeMapped[4])));
            for (int i = 0; i < 6; ++i)
                transformLast[i] = transformTobeMapped[i];
        } else {
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
                                          Point3(transformLast[5], transformLast[3], transformLast[4]));
            gtsam::Pose3 poseTo = Pose3(
                    Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                    Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(),
                                                poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->points.size(),
                                   Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                                                      transformAftMapped[1]),
                                         Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
        }
        /**
         * update iSAM
         */
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        /**
         * save key poses
         */
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

        //  cloudKeyPoses3D指当前的点云，cloudKeyPoses6D指带pose的
        thisPose3D.x = latestEstimate.translation().y();
        thisPose3D.y = latestEstimate.translation().z();
        thisPose3D.z = latestEstimate.translation().x();
        thisPose3D.intensity = cloudKeyPoses3D->points.size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().pitch();
        thisPose6D.pitch = latestEstimate.rotation().yaw();
        thisPose6D.yaw = latestEstimate.rotation().roll(); // in camera frame
        thisPose6D.time = timeLaserOdometry;
        cloudKeyPoses6D->push_back(thisPose6D);

        /**
         * save updated transform
         */
        if (cloudKeyPoses3D->points.size() > 1) {
            transformAftMapped[0] = latestEstimate.rotation().pitch();
            transformAftMapped[1] = latestEstimate.rotation().yaw();
            transformAftMapped[2] = latestEstimate.rotation().roll();
            transformAftMapped[3] = latestEstimate.translation().y();
            transformAftMapped[4] = latestEstimate.translation().z();
            transformAftMapped[5] = latestEstimate.translation().x();

            for (int i = 0; i < 6; ++i) {
                transformLast[i] = transformAftMapped[i];
                transformTobeMapped[i] = transformAftMapped[i];
            }
        }

        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(new pcl::PointCloud<PointType>());

        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
        pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);

        /*
            Scan Context loop detector
            - ver 1: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
            - ver 2: using downsampled original point cloud (/full_cloud_projected + downsampling)
            */
        bool usingRawCloud = true;
        if (usingRawCloud) { // v2 uses downsampled raw point cloud, more fruitful height information than using feature points (v1)
            //  这里对点云提取scan context特征
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*laserCloudRawDS, *thisRawCloudKeyFrame);
            scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        } else { // v1 uses thisSurfKeyFrame, it also works. (empirically checked at Mulran dataset sequences)
            scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame);
        }

        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
    } // saveKeyFramesAndFactor

    void mapOptimization::correctPoses() {
        if (aLoopIsClosed == true) {
            recentCornerCloudKeyFrames.clear();
            recentSurfCloudKeyFrames.clear();
            recentOutlierCloudKeyFrames.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i) {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().z();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().x();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            }

            aLoopIsClosed = false;
        }
    }

    void mapOptimization::clearCloud() {
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        laserCloudCornerFromMapDS->clear();
        laserCloudSurfFromMapDS->clear();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");

    lego_loam::mapOptimization MO;

    //  两个线程分别直行回环检测和可视化
    std::thread loopthread(&lego_loam::mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&lego_loam::mapOptimization::visualizeGlobalMapThread, &MO);

    ros::Rate rate(200);
    while (ros::ok())
        // while ( 1 )
    {
        ros::spinOnce();

        MO.run();

        rate.sleep();
    }

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}