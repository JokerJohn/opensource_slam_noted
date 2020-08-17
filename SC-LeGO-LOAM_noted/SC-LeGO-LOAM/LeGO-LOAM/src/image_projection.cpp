#include "lego_loam/image_projection.h"

namespace lego_loam {
    ImageProjection::ImageProjection() : nh("~") {

        // init params

        InitParams();
        // subscriber
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic.c_str(), 1,
                                                               &ImageProjection::cloudHandler, this);
        // publisher
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);  // 离群点或异常点

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    void ImageProjection::InitParams() {
        std::cout << "***************** ImageProjection *************************" << std::endl;

//        N_SCAN = config_node["N_SCAN"].as<int>();
//        Horizon_SCAN = config_node["Horizon_SCAN"].as<int>();
//
//        ang_res_x = config_node["ang_res_x"].as<double>();
//        ang_res_y = config_node["ang_res_y"].as<double>();
//        ang_bottom = config_node["ang_bottom"].as<double>();
//        groundScanInd = config_node["groundScanInd"].as<int>();
//
//        pointCloudTopic = config_node["pointCloudTopic"].as<std::string>();
//        imuTopic = config_node["imuTopic"].as<std::string>();
//        useCloudRing = config_node["useCloudRing"].as<bool>();
//
//
//        segmentAlphaX = ang_res_x / 180.0 * M_PI;
//        segmentAlphaY = ang_res_y / 180.0 * M_PI;

        std::cout << "pointCloudTopic : " << pointCloudTopic << std::endl;
        std::cout << "imuTopic : " << imuTopic << std::endl;
        std::cout << "useCloudRing : " << useCloudRing << std::endl;
        std::cout << "N_SCAN : " << N_SCAN << std::endl;
        std::cout << "ang_res_x : " << ang_res_x << std::endl;
        std::cout << "ang_res_y : " << ang_res_y << std::endl;
        std::cout << "ang_bottom : " << ang_bottom << std::endl;
        std::cout << "groundScanInd : " << groundScanInd << std::endl;
        std::cout << "******************** ImageProjection **********************" << std::endl;
    }

    void ImageProjection::cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan
        findStartEndAngle();
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();   //  细节需要再看一下
        // 5. Point cloud segmentation
        cloudSegmentation();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
    }

    void ImageProjection::allocateMemory() {

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN * Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN * Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN * Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN * Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1;
        neighbor.second = 0;
        neighborIterator.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = 1;
        neighborIterator.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = -1;
        neighborIterator.push_back(neighbor);
        neighbor.first = 1;
        neighbor.second = 0;
        neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];
    }

    void ImageProjection::resetParameters() {
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    void ImageProjection::copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        cloudHeader = laserCloudMsg->header;
        cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // have "ring" channel in the cloud
        if (useCloudRing == true) {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {  //  是否有NAN
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
    }

    void ImageProjection::findStartEndAngle() {
        // 这个水平上看激光雷达启动时的初始角度，一圈下来应该和原始角度一致，实际运动中会有畸变
        // start and end orientation of this cloud   计算角度时以x轴负轴为基准
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                       laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    void ImageProjection::projectPointCloud() {
        // 激光点云投影成二维图像，行表示激光线束数量，列表示每一个线上扫描到的点（0.1s扫描一圈，一个圆圈摊平就是360度）
        // 计算点云深度，保存到深度图中
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        for (size_t i = 0; i < cloudSize; ++i) {

            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // find the row and column index in the iamge for this point
            // 计算竖直方向上的点的角度以及在整个雷达点云中的哪一条水平线上
            if (useCloudRing == true) {
                // 用vlp的时候有ring属性
                rowIdn = laserCloudInRing->points[i].ring;
            } else {
                // 其他lidar需要根据计算垂直方向的俯仰角？
                verticalAngle =
                        atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // 水平方向上的角度，一行1800个像素点
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            // round是四舍五入
            columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // 每个点的深度
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;
            // 在rangeMat矩阵中保存该点的深度，保存单通道像素值
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float) rowIdn + (float) columnIdn / 10000.0;

            index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }
    }

    void ImageProjection::groundRemoval() {
        // 利用不同的扫描圈来表示地面，进而检测地面是否水平。
        // 取七个扫描圈，每两个圈之间进行一次比较，角度相差10°以内的我们可以看做是平地。
        // 并且将扫描圈中的点加入到groundCloud点云
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j) {   // 每行
            for (size_t i = 0; i < groundScanInd; ++i) {  // 每列
                //  只用7个光束检测地面
                lowerInd = j + (i) * Horizon_SCAN;
                upperInd = j + (i + 1) * Horizon_SCAN;

                // intensity在投影的时候已经归一化
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1) {
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i, j) = -1;
                    continue;
                }

                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

                // 相邻圈小于10度
                if (abs(angle - sensorMountAngle) <= 10) {
                    groundMat.at<int8_t>(i, j) = 1;
                    groundMat.at<int8_t>(i + 1, j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i) {
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i, j) == FLT_MAX) {
                    labelMat.at<int>(i, j) = -1;
                }
            }
        }

        if (pubGroundCloud.getNumSubscribers() != 0) {
            for (size_t i = 0; i <= groundScanInd; ++i) {
                for (size_t j = 0; j < Horizon_SCAN; ++j) {
                    if (groundMat.at<int8_t>(i, j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                }
            }
        }
    }

    void ImageProjection::cloudSegmentation() {
        // segmentation process
        // /在排除地面点与异常点之后，逐一检测邻点特征并生成局部特征
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i, j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {
            segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                // 如果是特征点或者是地面点，就可以纳入被分割点云
                if (labelMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1) {
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i, j) == 999999) {
                        if (i > groundScanInd && j % 5 == 0) {
                            outlierCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                            continue;
                        } else {
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    // 地面点云每隔5个点纳入被分割点云
                    if (groundMat.at<int8_t>(i, j) == 1) {
                        if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i, j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud] = rangeMat.at<float>(i, j);
                    // save seg cloud 把当前点纳入分割点云中
                    segmentedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
        }

        // extract segmented cloud for visualization
        // 在当前有节点订阅便将分割点云的几何信息也发布出去
        if (pubSegmentedCloudPure.getNumSubscribers() != 0) {
            for (size_t i = 0; i < N_SCAN; ++i) {
                for (size_t j = 0; j < Horizon_SCAN; ++j) {
                    if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999) {
                        segmentedCloudPure->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i, j);
                    }
                }
            }
        }
    }

    void ImageProjection::labelComponents(int row, int col) {
        // use std::queue std::vector std::deque will slow the program down greatly
        // 特征检测，检测点与其临近点的特征
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        while (queueSize > 0) {
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;

            //检查上下左右四个邻点
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter) {
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;
                // d1与d2分别是该点与某邻点的深度
                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                // angle其实是该特点与某邻点的连线与XOZ平面的夹角，这个角代表了局部特征的敏感性
                angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

                // 如果夹角大于60°，则将这个邻点纳入到局部特征中，该邻点可以用来配准使用
                if (angle > segmentTheta) {

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        // 当邻点数目达到30后，则该帧雷达点云的几何特征配置成功
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum) {
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;
        }
        // segment is valid, mark these points
        if (feasibleSegment == true) {
            ++labelCount;
        } else { // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i) {
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    void ImageProjection::publishCloud() {
        // 1. Publish Seg Cloud Info
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0) {
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0) {
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "lego_loam");

    lego_loam::ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");
    ros::spin();
    return 0;
}