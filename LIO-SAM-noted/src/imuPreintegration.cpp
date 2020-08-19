#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class IMUPreintegration : public ParamServer {
 public:
  IMUPreintegration() {

    // subscriber 订阅imu数据和激光Odom
    // 业务逻辑都在callback里面写, 两个数据是耦合关系, imu通过激光odom给出优化后的预积分预测
    // odom根据预测的位姿优化、融合出新的odom
    subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &IMUPreintegration::imuHandler, this,
                                            ros::TransportHints().tcpNoDelay());
    subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5,
                                                   &IMUPreintegration::odometryHandler, this,
                                                   ros::TransportHints().tcpNoDelay());

    // publisher 发布融合后的imu path和预积分完成优化后预测的odom
    pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
    pubImuPath = nh.advertise<nav_msgs::Path>("lio_sam/imu/path", 1);

    map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));

    // 下面是预积分使用到的gtsam的一些参数配置
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance =
        gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias
        prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

    priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)
        << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e2); // m/s
    priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correctionNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2); // meter
    noiseModelBetweenBias =
        (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

    // 优化前后的imu
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
    imuIntegratorOpt_ =new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization
  }

  void resetOptimization() {
    // gtsam相关优化参数重置
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newGraphFactors;
    graphFactors = newGraphFactors;

    gtsam::Values NewGraphValues;
    graphValues = NewGraphValues;
  }

  void resetParams() {
    lastImuT_imu = -1;
    doneFirstOpt = false;
    systemInitialized = false;
  }

  void odometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    double currentCorrectionTime = ROS_TIME(odomMsg);

    // make sure we have imu data to integrate
    // 保证有imu数据,两个回调函数是互有联系的,
    // 在imu的回调里就强调要完成一次优化才往下执行
    if (imuQueOpt.empty())
      return;

    // 从雷达odom中取出位姿数据
    float p_x = odomMsg->pose.pose.position.x;
    float p_y = odomMsg->pose.pose.position.y;
    float p_z = odomMsg->pose.pose.position.z;
    float r_x = odomMsg->pose.pose.orientation.x;
    float r_y = odomMsg->pose.pose.orientation.y;
    float r_z = odomMsg->pose.pose.orientation.z;
    float r_w = odomMsg->pose.pose.orientation.w;
    int currentResetId = round(odomMsg->pose.covariance[0]);
    gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z),
                                          gtsam::Point3(p_x, p_y, p_z));

    // correction pose jumped, reset imu pre-integration
    if (currentResetId != imuPreintegrationResetId) {
      resetParams();
      imuPreintegrationResetId = currentResetId;
    }


    // 0. initialize system
    // 第一帧进来初始化系统
    if (systemInitialized == false) {
      resetOptimization(); // 重置优化参数

      // pop old IMU message
      // 去掉一些比较旧的imu数据, 只需要保证雷达odom时间戳在imu队列中间
      // 因为imu是高频数据, 这里是有效的
      while (!imuQueOpt.empty()) {
        if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
          lastImuT_opt = ROS_TIME(&imuQueOpt.front());
          imuQueOpt.pop_front();
        } else
          break;
      }
      // initial pose
      prevPose_ = lidarPose.compose(lidar2Imu); // 雷达odom转到imu系下
      //PriorFactor,包括了位姿、速度和bias
      //加入PriorFactor在图优化中基本都是必需的前提
      gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
      graphFactors.add(priorPose);
      // initial velocity
      prevVel_ = gtsam::Vector3(0, 0, 0);
      gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
      graphFactors.add(priorVel);
      // initial bias
      prevBias_ = gtsam::imuBias::ConstantBias();
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
      graphFactors.add(priorBias);
      // add values、
      // 除了因子外, 还要有节点value
      graphValues.insert(X(0), prevPose_);
      graphValues.insert(V(0), prevVel_);
      graphValues.insert(B(0), prevBias_);
      // optimize once
      // 进行一次优化
      optimizer.update(graphFactors, graphValues);
      graphFactors.resize(0);
      graphValues.clear();     //图和节点均清零, 为什么要清零?

      // 重置积分器
      imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
      imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

      key = 1; // 计数
      systemInitialized = true;
      return;
    }


    // reset graph for speed
    // key超过设定的100则重置整个图
    // 减小计算压力,保存最后的噪声值
    if (key == 100) {
      // get updated noise before reset
      gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(
          optimizer.marginalCovariance(X(key - 1)));
      gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(
          optimizer.marginalCovariance(V(key - 1)));
      gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(
          optimizer.marginalCovariance(B(key - 1)));
      // reset graph 重置参数
      resetOptimization();

      // 重置之后还有类似与初始化的过程 区别在于噪声值不同
      // add pose
      gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
      graphFactors.add(priorPose);
      // add velocity
      gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
      graphFactors.add(priorVel);
      // add bias
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
      graphFactors.add(priorBias);
      // add values
      graphValues.insert(X(0), prevPose_);
      graphValues.insert(V(0), prevVel_);
      graphValues.insert(B(0), prevBias_);
      // optimize once
      // 优化一次, 相当于初始化
      optimizer.update(graphFactors, graphValues);
      graphFactors.resize(0);
      graphValues.clear();

      key = 1;
    }


    // 1. integrate imu data and optimize
    // 这里才开始主要的优化流程
    while (!imuQueOpt.empty()) {
      // pop and integrate imu data that is between two optimizations
      // 对两次优化的之间的imu数据进行优化
      sensor_msgs::Imu *thisImu = &imuQueOpt.front(); // 最新的imu数据帧
      double imuTime = ROS_TIME(thisImu);
      if (imuTime < currentCorrectionTime - delta_t) {
        // 求dt,初始是1/500,后续是两帧imu数据的时间差
        double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);

        // 进行预积分得到新的状态值,注意用到的是imu数据的加速度和角速度
        // 作者要求的9轴imu数据中欧拉角在本程序中没有任何用到, 全在地图优化里用到的
        imuIntegratorOpt_->integrateMeasurement(
            gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y,
                           thisImu->linear_acceleration.z),
            gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y,
                           thisImu->angular_velocity.z), dt);

        //在pop出一次数据前保存上一个数据的时间戳
        lastImuT_opt = imuTime;
        imuQueOpt.pop_front();
      } else
        break;
    }

    // add imu factor to graph
    // 利用两帧之间的IMU数据完成了预积分后增加imu因子到因子图中
    // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements
        &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
    gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    graphFactors.add(imu_factor);
    // add imu bias between factor
    graphFactors.add(
        gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                           gtsam::noiseModel::Diagonal::Sigmas(
                                                               sqrt(imuIntegratorOpt_->deltaTij()) *
                                                                   noiseModelBetweenBias)));
    // add pose factor
    //  还加入了pose factor,其实对应于作者论文中的因子图结构
    //  就是与imu因子一起的 Lidar odometry factor
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, correctionNoise);
    graphFactors.add(pose_factor);
    // insert predicted values
    // 插入预测的值 即因子图中x0 x1 x2……节点
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(V(key), propState_.v());
    graphValues.insert(B(key), prevBias_);

    // optimize 优化后重置
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step.
    // 用这次的优化结果重写或者说是覆盖相关初始值, 为下一次优化准备
    gtsam::Values result = optimizer.calculateEstimate();
    prevPose_ = result.at<gtsam::Pose3>(X(key));
    prevVel_ = result.at<gtsam::Vector3>(V(key));
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
    // Reset the optimization preintegration object.
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    // check optimization
    // 检查是否有失败情况,如有则重置参数
    if (failureDetection(prevVel_, prevBias_)) {
      resetParams();
      return;
    }


    // 2. after optiization, re-propagate imu odometry preintegration
    // 为了维持实时性imuIntegrateImu就得在每次odom触发优化后立刻获取最新的bias,
    // 同时对imu测量值imuQueImu执行bias改变的状态重传播处理, 这样可以最大限度的保证实时性和准确性?
    prevStateOdom = prevState_;
    prevBiasOdom = prevBias_;
    // first pop imu message older than current correction data
    // 去除旧的imu数据
    double lastImuQT = -1;
    while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t) {
      lastImuQT = ROS_TIME(&imuQueImu.front());
      imuQueImu.pop_front();
    }
    // repropogate
    // 重传播？
    if (!imuQueImu.empty()) {
      // reset bias use the newly optimized bias
      // 使用最新的优化后的bias更新bias值
      imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
      // integrate imu message from the beginning of this optimization
      // 利用imuQueImu中的数据进行预积分,主要区别旧在于上一行的更新了bias
      for (int i = 0; i < (int) imuQueImu.size(); ++i) {
        sensor_msgs::Imu *thisImu = &imuQueImu[i];
        double imuTime = ROS_TIME(thisImu); // 时间戳
        double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);

        // 进行预计分
        imuIntegratorImu_->integrateMeasurement(
            gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y,
                           thisImu->linear_acceleration.z),
            gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y,
                           thisImu->angular_velocity.z), dt);
        lastImuQT = imuTime;
      }
    }

    ++key;
    doneFirstOpt = true;
  }

  bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur) {
    // 检测预计分失败的函数, 即时爆出错误,重置积分器
    Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 10) {
      ROS_WARN("Large velocity, reset IMU-preintegration!");
      return true;
    }

    Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
    Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
    if (ba.norm() > 0.1 || bg.norm() > 0.1) {
      ROS_WARN("Large bias, reset IMU-preintegration!");
      return true;
    }

    return false;
  }

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imu_raw) {
    // imu数据转换到雷达坐标系下
    sensor_msgs::Imu thisImu = imuConverter(*imu_raw);
    // publish static tf map->odom
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, thisImu.header.stamp, "map", "odom"));

    // 两个双端队列分别装着优化前后的imu数据
    imuQueOpt.push_back(thisImu);
    imuQueImu.push_back(thisImu);

    // 检查有没有执行过一次优化,这里需要先在odomhandler中优化一次后再进行该函数后续的工作
    if (doneFirstOpt == false)
      return;

    // 获得时间间隔, 第一次为1/500,之后是两次imuTime间的差
    double imuTime = ROS_TIME(&thisImu);
    double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
    lastImuT_imu = imuTime;

    // integrate this single imu message
    // 进行预积分
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y,
                                                           thisImu.linear_acceleration.z),
                                            gtsam::Vector3(thisImu.angular_velocity.x,
                                                           thisImu.angular_velocity.y,
                                                           thisImu.angular_velocity.z), dt);

    // predict odometry
    // 根据预计分结果, 预测odom
    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

    // publish odometry 发布新的odom
    nav_msgs::Odometry odometry;
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "odom_imu";

    // transform imu pose to ldiar imu位姿转到雷达系
    // 预测值currentState获得imu位姿, 再由imu到雷达变换, 获得雷达位姿
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
    gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    odometry.pose.covariance[0] = double(imuPreintegrationResetId);
    pubImuOdometry.publish(odometry);

    // publish imu path
    // 预测的imu path, 只保留3s内的轨迹
    static nav_msgs::Path imuPath;
    static double last_path_time = -1;
    if (imuTime - last_path_time > 0.1) {
      last_path_time = imuTime;
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = thisImu.header.stamp;
      pose_stamped.header.frame_id = "odom";
      pose_stamped.pose = odometry.pose.pose;
      imuPath.poses.push_back(pose_stamped);
      while (!imuPath.poses.empty() &&
          abs(imuPath.poses.front().header.stamp.toSec() - imuPath.poses.back().header.stamp.toSec()) > 3.0)
        imuPath.poses.erase(imuPath.poses.begin());
      if (pubImuPath.getNumSubscribers() != 0) {
        imuPath.header.stamp = thisImu.header.stamp;
        imuPath.header.frame_id = "odom";
        pubImuPath.publish(imuPath);
      }
    }

    // publish transformation
    // 发布odom->base_link的变换
    tf::Transform tCur;
    tf::poseMsgToTF(odometry.pose.pose, tCur);
    tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, thisImu.header.stamp, "odom", "base_link");
    tfOdom2BaseLink.sendTransform(odom_2_baselink);
  }

 public:
  ros::Subscriber subImu;
  ros::Subscriber subOdometry;
  ros::Publisher pubImuOdometry;
  ros::Publisher pubImuPath;

  // map -> odom
  tf::Transform map_to_odom;
  tf::TransformBroadcaster tfMap2Odom;
  // odom -> base_link
  tf::TransformBroadcaster tfOdom2BaseLink;

  bool systemInitialized = false;

  gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
  gtsam::Vector noiseModelBetweenBias;

  gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
  gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

  std::deque<sensor_msgs::Imu> imuQueOpt;
  std::deque<sensor_msgs::Imu> imuQueImu;

  gtsam::Pose3 prevPose_;
  gtsam::Vector3 prevVel_;
  gtsam::NavState prevState_;
  gtsam::imuBias::ConstantBias prevBias_;

  gtsam::NavState prevStateOdom;
  gtsam::imuBias::ConstantBias prevBiasOdom;

  bool doneFirstOpt = false;
  double lastImuT_imu = -1;
  double lastImuT_opt = -1;

  gtsam::ISAM2 optimizer;
  gtsam::NonlinearFactorGraph graphFactors;
  gtsam::Values graphValues;

  const double delta_t = 0;

  int key = 1;
  int imuPreintegrationResetId = 0;

  // 雷达->imu外餐
  gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                                        gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
  gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                                        gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "roboat_loam");

  IMUPreintegration ImuP;

  ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

  ros::spin();

  return 0;
}