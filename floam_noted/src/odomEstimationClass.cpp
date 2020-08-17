// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"

void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution) {
  //init local map
  laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  //downsampling size
  downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
  downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

  //kd-tree
  kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  odom = Eigen::Isometry3d::Identity();
  last_odom = Eigen::Isometry3d::Identity();
  optimization_count = 2;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in) {
  *laserCloudCornerMap += *edge_in;
  *laserCloudSurfMap += *surf_in;
  optimization_count = 12;
}

void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in,
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in) {
  if (optimization_count > 2)
    optimization_count--;

  // 根据上一帧的odom计算当前预测，并更新last_odom
  Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
  last_odom = odom;
  odom = odom_prediction;

  // q_w_curr t_w_curr，优化后再更新
  q_w_curr = Eigen::Quaterniond(odom.rotation());
  t_w_curr = odom.translation();

  // 下采样
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
  downSamplingToMap(edge_in, downsampledEdgeCloud, surf_in, downsampledSurfCloud);
  //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
  if (laserCloudCornerMap->points.size() > 10 && laserCloudSurfMap->points.size() > 50) {
    kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
    kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

    // ceres优化，附近的多帧点云构建优化问题
    for (int iterCount = 0; iterCount < optimization_count; iterCount++) {
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); // 鲁棒核
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options); // 优化问题

      // 残差块
      problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

      // 对两种特征点云分别构建点到平面、点到边缘的残差项
      addEdgeCostFactor(downsampledEdgeCloud, laserCloudCornerMap, problem, loss_function);
      addSurfCostFactor(downsampledSurfCloud, laserCloudSurfMap, problem, loss_function);

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;

      // 求解问题, 优化完成后的结果更新在 q_w_curr、t_w_curr
      ceres::Solve(options, &problem, &summary);
    }
  } else {
    printf("not enough points in map to associate, map error");
  }

  // odom是全局变量
  odom = Eigen::Isometry3d::Identity();
  odom.linear() = q_w_curr.toRotationMatrix();
  odom.translation() = t_w_curr;
  addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud); //  加到地图中
}

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po) {
  // 点转换到地图坐标系
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
  //po->intensity = 1.0;
}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_in,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_out,
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out) {
  downSizeFilterEdge.setInputCloud(edge_pc_in);
  downSizeFilterEdge.filter(*edge_pc_out);
  downSizeFilterSurf.setInputCloud(surf_pc_in);
  downSizeFilterSurf.filter(*surf_pc_out);
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
                                            ceres::Problem &problem,
                                            ceres::LossFunction *loss_function) {
  int corner_num = 0;
  for (int i = 0; i < (int) pc_in->points.size(); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&(pc_in->points[i]), &point_temp); // 将当前点转换到地图坐标系

    // 寻找最近邻的5个点
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
    if (pointSearchSqDis[4] < 1.0) {
      std::vector<Eigen::Vector3d> nearCorners;
      // 计算均值
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                            map_in->points[pointSearchInd[j]].y,
                            map_in->points[pointSearchInd[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;
      // 计算方差
      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      }

      //  主成分分析，获取直线的参数
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) { // 如果最大的特征向量，明显比第二大的大，则认为是直线
        Eigen::Vector3d point_on_line = center;
        Eigen::Vector3d point_a, point_b;
        // 取直线的两点，中点+-0.1×(直线方向单位向量)
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        // 用点O，A，B构造点到线的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差 = 点O到直线AB的距离
        ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
        // 添加边缘点关联构建的残差项
        problem.AddResidualBlock(cost_function, loss_function, parameters);
        corner_num++;
      }
    }
  }
  if (corner_num < 20) {
    printf("not enough correct points");
  }

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                                            const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in,
                                            ceres::Problem &problem,
                                            ceres::LossFunction *loss_function) {
  int surf_num = 0;
  for (int i = 0; i < (int) pc_in->points.size(); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&(pc_in->points[i]), &point_temp);
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

    // 与上面的建立corner特征点之间的关联类似，寻找平面特征点O的最近邻点ABC，
    // 即基于最近邻原理建立surf特征点之间的关联，find correspondence for plane features
    // 寻找五个紧邻点
    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (pointSearchSqDis[4] < 1.0) {

       // 5*3的矩阵
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
        matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
        matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
      }
      // 计算平面的法向量
      // 平面方程Ax+By+Cz+D = 0 =>  (x,y,z)(A/D,B/D,C/D)^T = -1  => 求解  Ax = b ,x即为法向量
      Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      // 判断平面是否有效
      // Here n(pa, pb, pc) is unit norm of plane   X^T*n = -1 =>  X^T*n/|n| + 1/|n| = 0  如果结果>0.2则认为有误
      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
            norm(1) * map_in->points[pointSearchInd[j]].y +
            norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2) {
          planeValid = false;
          break;
        }
      }
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
      if (planeValid) {
        // 有效的话t添加点到平面的残差项
        // 用点O，A，B，C构造点到面的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差 = 点O到平面ABC的距离
        ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
        problem.AddResidualBlock(cost_function, loss_function, parameters);

        surf_num++;
      }
    }

  }
  if (surf_num < 20) {
    printf("not enough correct points");
  }

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledEdgeCloud,
                                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud) {

  for (int i = 0; i < (int) downsampledEdgeCloud->points.size(); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp); // 将点加到地图中，而不是直接点云×T
    laserCloudCornerMap->push_back(point_temp);
  }

  for (int i = 0; i < (int) downsampledSurfCloud->points.size(); i++) {
    pcl::PointXYZI point_temp;
    pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
    laserCloudSurfMap->push_back(point_temp);
  }

  double x_min = +odom.translation().x() - 100;
  double y_min = +odom.translation().y() - 100;
  double z_min = +odom.translation().z() - 100;
  double x_max = +odom.translation().x() + 100;
  double y_max = +odom.translation().y() + 100;
  double z_max = +odom.translation().z() + 100;

  // 点云截取和下采样
  //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
  cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  cropBoxFilter.setNegative(false);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
  cropBoxFilter.setInputCloud(laserCloudSurfMap);
  cropBoxFilter.filter(*tmpSurf);
  cropBoxFilter.setInputCloud(laserCloudCornerMap);
  cropBoxFilter.filter(*tmpCorner);

  downSizeFilterEdge.setInputCloud(tmpSurf);
  downSizeFilterEdge.filter(*laserCloudSurfMap);
  downSizeFilterSurf.setInputCloud(tmpCorner);
  downSizeFilterSurf.filter(*laserCloudCornerMap);
}

void OdomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloudMap) {

  *laserCloudMap += *laserCloudSurfMap;
  *laserCloudMap += *laserCloudCornerMap;
}

OdomEstimationClass::OdomEstimationClass() {

}