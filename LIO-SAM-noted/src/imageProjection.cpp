#include "utility.h"
#include "lio_sam/cloud_info.h"

struct PointXYZIRT {
  PCL_ADD_POINT4D

  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16;   // 强制SSE填充以正确对齐内存

// 定义新类型里元素包括XYZI+ring+time, time的话主要方便去畸变,ring的话主要是对点云进行归类
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
                                   (float, x, x)(float, y, y)
                                       (float, z, z)(float, intensity, intensity)
                                       (uint16_t, ring, ring)(float, time, time)
)

const int queueLength = 500;

/**
 * 点云投影成深度图,类似lego loam中的做法
 * 列表示线束数量，行表示横向解析度，比如16*1800,横向解析度则为0.2
 */
class ImageProjection : public ParamServer {
 public:
  ImageProjection() :
      deskewFlag(0) {
    // subscriber, 订阅imu和odometry以及原始点云，这里的odom应该是gps+imu通过robot_localization包融合得到的.
    // imu和odom callback中主要是用来装数据
    // 点云处理的逻辑全部在cloudHandler中
    subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this,
                                            ros::TransportHints().tcpNoDelay()); // 允许指定hints到roscpp的传输层
    subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic, 2000, &ImageProjection::odometryHandler, this,
                                               ros::TransportHints().tcpNoDelay());
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this,
                                                           ros::TransportHints().tcpNoDelay());

    // publisher, 发布自定义的cloud_info和用于odometry的cloud
    pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);
    pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1);

    // 初始化参数
    allocateMemory();
    resetParameters();

    // 设置控制台信息输出
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  void allocateMemory() {
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    fullCloud.reset(new pcl::PointCloud<PointType>());
    extractedCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(N_SCAN * Horizon_SCAN);

    cloudInfo.startRingIndex.assign(N_SCAN, 0);
    cloudInfo.endRingIndex.assign(N_SCAN, 0);

    cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
    cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

    resetParameters();
  }

  void resetParameters() {
    laserCloudIn->clear();
    extractedCloud->clear();
    // reset range matrix for range image projection
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

    imuPointerCur = 0;
    firstPointFlag = true;
    odomDeskewFlag = false;

    for (int i = 0; i < queueLength; ++i) {
      imuTime[i] = 0;
      imuRotX[i] = 0;
      imuRotY[i] = 0;
      imuRotZ[i] = 0;
    }
  }

  ~ImageProjection() {}

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg) {
    // 将imu数据转到lidar坐标系下, 这里在params中配置过imu到lidar的外参
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

    // 这里用双端队列来装数据
    std::lock_guard<std::mutex> lock1(imuLock);
    imuQueue.push_back(thisImu);

    // debug IMU data
    // cout << std::setprecision(6);
    // cout << "IMU acc: " << endl;
    // cout << "x: " << thisImu.linear_acceleration.x <<
    //       ", y: " << thisImu.linear_acceleration.y <<
    //       ", z: " << thisImu.linear_acceleration.z << endl;
    // cout << "IMU gyro: " << endl;
    // cout << "x: " << thisImu.angular_velocity.x <<
    //       ", y: " << thisImu.angular_velocity.y <<
    //       ", z: " << thisImu.angular_velocity.z << endl;
    // double imuRoll, imuPitch, imuYaw;
    // tf::Quaternion orientation;
    // tf::quaternionMsgToTF(thisImu.orientation, orientation);
    // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    // cout << "IMU roll pitch yaw: " << endl;
    // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
  }

  void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg) {
    // 锁存后装数据
    std::lock_guard<std::mutex> lock2(odoLock);
    odomQueue.push_back(*odometryMsg);
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    // 清除临时点云，并检查当前点云帧里面是否有ring和time通道
    if (!cachePointCloud(laserCloudMsg))
      return;

    // 配置好用于去畸变的imu和odom参数
    // 找到点云时间戳前后的GPS odom和imu 数据，并分别计算在这两帧时间内的位姿增量和旋转增量
    // 用于后续去畸变
    if (!deskewInfo())
      return;

    // 每一帧点云进来都这么处理

    // 点云投影成深度图->去畸变
    projectPointCloud();

    // 从深度图中提取点云, 给lidar odometry用
    cloudExtraction();

    // 发布点云
    publishClouds();

    // 重置参数
    resetParameters();
  }

  bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    // 清除内存中的临时点云
    cloudQueue.push_back(*laserCloudMsg);

    if (cloudQueue.size() <= 2)
      return false;
    else {
      // 点云队列先进先出,z最新的点云 存到currentCloudMsg
      currentCloudMsg = cloudQueue.front();
      cloudQueue.pop_front(); // 老数据弹出去

      // 这里得到的已经是下一帧的数据了(最新)
      cloudHeader = currentCloudMsg.header;
      timeScanCur = cloudHeader.stamp.toSec();
      timeScanNext = cloudQueue.front().header.stamp.toSec();
    }

    // convert cloud
    pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

    // check dense flag
    if (laserCloudIn->is_dense == false) {
      ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
      ros::shutdown();
    }

    // check ring channel, veloodyne和ouster都有中才有ring通道,不然需要自行计算
    // 其他雷达需要注释掉这一部分,自行计算
    static int ringFlag = 0;
    if (ringFlag == 0) {
      ringFlag = -1;
      for (int i = 0; i < currentCloudMsg.fields.size(); ++i) {
        if (currentCloudMsg.fields[i].name == "ring") {
          ringFlag = 1;
          break;
        }
      }
      if (ringFlag == -1) {
        ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
        ros::shutdown();
      }
    }

    // check point time
    if (deskewFlag == 0) {
      deskewFlag = -1;
      for (int i = 0; i < currentCloudMsg.fields.size(); ++i) {
        if (currentCloudMsg.fields[i].name == "time") {
          deskewFlag = 1;
          break;
        }
      }
      if (deskewFlag == -1)
        ROS_WARN(
            "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }

    return true;
  }

  bool deskewInfo() {
    std::lock_guard<std::mutex> lock1(imuLock);
    std::lock_guard<std::mutex> lock2(odoLock);

    // 保证imu和odom数据，并且当前帧点云数据, 其时间戳在队列中的imu数据之间
    if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur ||
        imuQueue.back().header.stamp.toSec() < timeScanNext) {
      ROS_DEBUG("Waiting for IMU data ...");
      return false;
    }

    // 遍历imu队列, 计算点云帧对应的imu数据, 包括积分计算其转过的角度
    // 用于后续去畸变
    imuDeskewInfo();

    // odom去畸变参数配置
    odomDeskewInfo();

    return true;
  }

  void imuDeskewInfo() {
    //这个参数在地图优化程序中用到  首先为false 完成相关操作后置true
    cloudInfo.imuAvailable = false;

    // imu去畸变参数
    // timeScanCur指当前点云帧的时间戳
    while (!imuQueue.empty()) {
      // 以0.01为阈值 舍弃较旧的imu数据
      if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
        imuQueue.pop_front();
      else
        break;
    }

    if (imuQueue.empty())
      return;

    imuPointerCur = 0;

    for (int i = 0; i < imuQueue.size(); ++i) {
      sensor_msgs::Imu thisImuMsg = imuQueue[i];
      double currentImuTime = thisImuMsg.header.stamp.toSec();

      // get roll, pitch, and yaw estimation for this scan
      if (currentImuTime <= timeScanCur) // 点云时间戳在前
        // 用imu的欧拉角做扫描的位姿估计,直接把值给了cloudInfo在地图优化程序中使用
        imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
      //  如果当前Imu时间比下一帧时间大于0.01退出, imu频率大于100hz才比较有用
      if (currentImuTime > timeScanNext + 0.01)
        break;

      // 第一次初始化时以下值都是0
      if (imuPointerCur == 0) {
        imuRotX[0] = 0;
        imuRotY[0] = 0;
        imuRotZ[0] = 0;
        imuTime[0] = currentImuTime;
        ++imuPointerCur;
        continue;
      }

      // get angular velocity
      double angular_x, angular_y, angular_z;
      imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

      // integrate rotation
      // 把角速度和时间间隔积分出转角,用于后续的去畸变
      double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
      imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
      imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
      imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
      imuTime[imuPointerCur] = currentImuTime;
      ++imuPointerCur;
    }

    --imuPointerCur;

    if (imuPointerCur <= 0)
      return;

    // 这一帧点云的imu数据可用
    cloudInfo.imuAvailable = true;
  }

  void odomDeskewInfo() {
    // 类似imu数据,用于标志当前点云帧的odom处理的信息是否有效
    cloudInfo.odomAvailable = false;

    while (!odomQueue.empty()) {
      // 保证点云帧的时间戳在odom队列中间
      if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
        odomQueue.pop_front();
      else
        break;
    }

    if (odomQueue.empty())
      return;

    // 点云数据在前
    if (odomQueue.front().header.stamp.toSec() > timeScanCur)
      return;

    // get start odometry at the beinning of the scan
    // 遍历odom队列,将odom的位姿作为点云信息中预测位姿
    nav_msgs::Odometry startOdomMsg;

    for (int i = 0; i < odomQueue.size(); ++i) {
      startOdomMsg = odomQueue[i];

      // 之前已经将小于timeScanCur超过0.01的数据弹出
      // 所以startOdomMsg已经可代表起始激光扫描的起始时刻的里程计消息
      if (ROS_TIME(&startOdomMsg) < timeScanCur)
        continue;
      else
        break;
    }

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // Initial guess used in mapOptimization
    // 用当前odom队列的起始位姿作为当前点云的初始位姿
    cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
    cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
    cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
    cloudInfo.initialGuessRoll = roll;
    cloudInfo.initialGuessPitch = pitch;
    cloudInfo.initialGuessYaw = yaw;
    cloudInfo.imuPreintegrationResetId = round(startOdomMsg.pose.covariance[0]);

    cloudInfo.odomAvailable = true;

    // get end odometry at the end of the scan
    // 获得一帧扫描末尾的里程计消息,和初始位姿无关, 主要用于去畸变、运动补偿
    odomDeskewFlag = false;

    // 计算激光扫描阵=帧结尾时刻的里程计消息
    if (odomQueue.back().header.stamp.toSec() < timeScanNext)
      return;

    // 扫描结束时的odom
    nav_msgs::Odometry endOdomMsg;
    for (int i = 0; i < odomQueue.size(); ++i) {
      endOdomMsg = odomQueue[i];

      if (ROS_TIME(&endOdomMsg) < timeScanNext)
        continue;
      else
        break;
    }

    // 位姿协方差矩阵判断,位姿协方差不一致的话退出
    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
      return;

    // 获得起始变换
    Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x,
                                                        startOdomMsg.pose.pose.position.y,
                                                        startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    // 获得结尾变换
    tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x,
                                                      endOdomMsg.pose.pose.position.y,
                                                      endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    // 获得一帧扫描起始与结束时刻间的变换, 参考loam
    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

    // 计算这个首尾odom的增量, 用于后续去畸变
    float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

    // 标志位
    odomDeskewFlag = true;
  }

  void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur) {
    *rotXCur = 0;
    *rotYCur = 0;
    *rotZCur = 0;

    // 当前point_time在imu_time前面，舍弃
    // 最终要保证点云时间在两帧imu数据中间。
    int imuPointerFront = 0;
    while (imuPointerFront < imuPointerCur) {
      if (pointTime < imuTime[imuPointerFront])
        break;
      ++imuPointerFront;
    }

    // pointTime在imu队列的起点，直接获取其旋转增量(在imuDeskewInfo函数中已经计算了)
    // pointTime在imu队列中间的话，按比率获取
    if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
      *rotXCur = imuRotX[imuPointerFront];
      *rotYCur = imuRotY[imuPointerFront];
      *rotZCur = imuRotZ[imuPointerFront];
    } else {
      // 根据点的时间信息,获得每个点的时刻的旋转变化量
      int imuPointerBack = imuPointerFront - 1;
      //  back  point_time front
      //  计算point_time在imu队列的前后占比
      double ratioFront =
          (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
      *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
      *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
    }
  }

  void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur) {

    // 这里注释掉了,如果高速移动可能有用,低速车辆提升不大
    // 用到了里程计增量值
    *posXCur = 0;
    *posYCur = 0;
    *posZCur = 0;

    // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

    // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
    //     return;

    // float ratio = relTime / (timeScanNext - timeScanCur);

    // *posXCur = ratio * odomIncreX;
    // *posYCur = ratio * odomIncreY;
    // *posZCur = ratio * odomIncreZ;
  }

  PointType deskewPoint(PointType *point, double relTime) {
    // 根据时间戳,对每个点去畸变
    if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
      return *point;

    // relTime是点在当前帧内的实际时间
    double pointTime = timeScanCur + relTime;

    // 用于补偿旋转和平移，根据点云的时间戳去计算其旋转
    float rotXCur, rotYCur, rotZCur;
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

    float posXCur, posYCur, posZCur;
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

    // 如果是第一帧数据
    if (firstPointFlag == true) {
      // 起始矩阵赋值再取逆
      transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur,
                                                  rotZCur)).inverse();
      firstPointFlag = false;
    }

    // transform points to start
    // 把点投影到每一帧扫描的起始时刻,参考Loam
    Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = transStartInverse * transFinal;

    // 去完畸变的点
    PointType newPoint;
    newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
    newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
    newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
    newPoint.intensity = point->intensity;

    return newPoint;
  }

  void projectPointCloud() {
    // 将点云投影成深度图
    int cloudSize = laserCloudIn->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i) {
      // 遍历每个点, 按线束进行分类
      PointType thisPoint;
      thisPoint.x = laserCloudIn->points[i].x;
      thisPoint.y = laserCloudIn->points[i].y;
      thisPoint.z = laserCloudIn->points[i].z;
      thisPoint.intensity = laserCloudIn->points[i].intensity;

      // 没有ring 的话需要依据垂直角度计算
      int rowIdn = laserCloudIn->points[i].ring;
      if (rowIdn < 0 || rowIdn >= N_SCAN)
        continue;
      // 激光点的水平角度, 计算一帧点云转过多少度，进而计算每个点投影到深度图中的列id和时间戳
      float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
      // 角分辨率 360/1800
      float ang_res_x = 360.0 / float(Horizon_SCAN); // 深度图的列数
      int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2; // 列
      if (columnIdn >= Horizon_SCAN)
        columnIdn -= Horizon_SCAN;

      if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
        continue;

      // 计算点到雷达的距离, 过近过远均舍弃
      float range = pointDistance(thisPoint);

      if (range < 1.0)
        continue;

      if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
        continue;

      // for the amsterdam dataset
      // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
      //     continue;
      // if (thisPoint.z < -2.0)
      //     continue;

      //  深度图中像素值存深度
      rangeMat.at<float>(rowIdn, columnIdn) = range;

      //  点云去畸变,运动补偿,这里需要用到雷达信息中的time这个field
      thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
      // 索引值,类似于图像中像素索引的概念应该比较好理解
      int index = columnIdn + rowIdn * Horizon_SCAN;
      // 去完畸变的点云存储到fullCloud中
      fullCloud->points[index] = thisPoint;
    }
  }

  void cloudExtraction() {
    // 提取点云给odometry
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCAN; ++i) {
      cloudInfo.startRingIndex[i] = count - 1 + 5;

      for (int j = 0; j < Horizon_SCAN; ++j) {
        // 图像上不一定每个位置都有点，只取有深度的点
        if (rangeMat.at<float>(i, j) != FLT_MAX) {
          // mark the points' column index for marking occlusion later
          cloudInfo.pointColInd[count] = j;
          // save range info
          cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
          // save extracted cloud
          extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
          // size of extracted cloud
          ++count;
        }
      }
      cloudInfo.endRingIndex[i] = count - 1 - 5;
    }
  }

  void publishClouds() {
    cloudInfo.header = cloudHeader;
    cloudInfo.cloud_deskewed = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");
    pubLaserCloudInfo.publish(cloudInfo);
  }

 private:

  std::mutex imuLock;
  std::mutex odoLock;

  ros::Subscriber subLaserCloud;
  ros::Publisher pubLaserCloud;

  ros::Publisher pubExtractedCloud;
  ros::Publisher pubLaserCloudInfo;

  ros::Subscriber subImu;
  std::deque<sensor_msgs::Imu> imuQueue;

  ros::Subscriber subOdom;
  std::deque<nav_msgs::Odometry> odomQueue;

  std::deque<sensor_msgs::PointCloud2> cloudQueue;
  sensor_msgs::PointCloud2 currentCloudMsg;

  double *imuTime = new double[queueLength];
  double *imuRotX = new double[queueLength];
  double *imuRotY = new double[queueLength];
  double *imuRotZ = new double[queueLength];

  int imuPointerCur;
  bool firstPointFlag;
  Eigen::Affine3f transStartInverse;

  pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
  pcl::PointCloud<PointType>::Ptr fullCloud;
  pcl::PointCloud<PointType>::Ptr extractedCloud;

  int deskewFlag;
  cv::Mat rangeMat;

  bool odomDeskewFlag;
  float odomIncreX;
  float odomIncreY;
  float odomIncreZ;

  lio_sam::cloud_info cloudInfo;
  double timeScanCur;
  double timeScanNext;
  std_msgs::Header cloudHeader;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_sam");

  ImageProjection IP;

  ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

  // 这里原来是std::thread, 现在改成了ros MultiThreadedSpinner
  // 阻塞微调, 类似于ros::spin(), 你可以在它的构造函数中指定线程数量,
  // 但如果不指定或者设为0, 它会根据你的CPU内核数创建线程.
  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();

  return 0;
}