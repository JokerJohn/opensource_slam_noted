#include "utility.h"
#include "lio_sam/cloud_info.h"

struct smoothness_t {
  float value;
  size_t ind;
};

struct by_value {
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};

class FeatureExtraction : public ParamServer {

 public:

  ros::Subscriber subLaserCloudInfo;

  ros::Publisher pubLaserCloudInfo;
  ros::Publisher pubCornerPoints;
  ros::Publisher pubSurfacePoints;

  pcl::PointCloud<PointType>::Ptr extractedCloud;
  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  lio_sam::cloud_info cloudInfo;
  std_msgs::Header cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  float *cloudCurvature;
  int *cloudNeighborPicked;
  int *cloudLabel;

  FeatureExtraction() {
    // 这里只订阅去完畸变的点云, 业务逻辑依然在点云callback里面写
    subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1,
                                                          &FeatureExtraction::laserCloudInfoHandler, this,
                                                          ros::TransportHints().tcpNoDelay());

    //  提取2种特征, surface和corner
    pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1);
    pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);

    initializationValue();
  }

  void initializationValue() {
    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    cloudCurvature = new float[N_SCAN * Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
    cloudLabel = new int[N_SCAN * Horizon_SCAN];
  }

  void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr &msgIn) {
    cloudInfo = *msgIn; // new cloud info
    cloudHeader = msgIn->header; // new cloud header
    pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

    // 计算每个点的曲率
    calculateSmoothness();

    // 标记遮挡和平行点
    markOccludedPoints();

    // 提取surface和corner特征
    extractFeatures();

    // 发布特征点云
    publishFeatureCloud();
  }

  void calculateSmoothness() {
    int cloudSize = extractedCloud->points.size();
    // 计算曲率, 年后五个点深度的平方和, 存到cloudCurvature中
    // 前五个点的距离属性之和加后五个点的距离之和-10倍该点的距离,算出差值
    // 确定一种连续点之间的起伏趋势
    for (int i = 5; i < cloudSize - 5; i++) {
      float diffRange = cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4]
          + cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2]
          + cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10
          + cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2]
          + cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4]
          + cloudInfo.pointRange[i + 5];

      cloudCurvature[i] = diffRange * diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;

      cloudNeighborPicked[i] = 0;
      cloudLabel[i] = 0;
      // cloudSmoothness for sorting
      // 记录曲率以及索引, h后续需要根据曲率进行排序，打乱点的顺序
      cloudSmoothness[i].value = cloudCurvature[i];
      cloudSmoothness[i].ind = i;
    }
  }

  void markOccludedPoints() {
    // cloudNeighborPicked中有了点是否选择为特征点的标记
    int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i) {
      // occluded points
      float depth1 = cloudInfo.pointRange[i];
      float depth2 = cloudInfo.pointRange[i + 1];
      // 深度图中列的差值
      int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

      // 平行线和遮挡的判断参考LOAM
      if (columnDiff < 10) {
        // 10 pixel diff in range image
        // 根据深度差异进行区分,并设定标志变量为1
        if (depth1 - depth2 > 0.3) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        } else if (depth2 - depth1 > 0.3) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
      // parallel beam
      // 平行线的情况,根据左右两点与该点的深度差,确定该点是否会被选择为特征点
      float diff1 = std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
      float diff2 = std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

      if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
        cloudNeighborPicked[i] = 1;
    }
  }

  void extractFeatures() {
    cornerCloud->clear();
    surfaceCloud->clear();

    // 提取corner和surface特征
    // 为了保证各方向均匀提取, 将深度图分为6个子图
    // 每个子图中对点的曲率进行排序，sp和ep分别是这段点云的起始点与终止点
    // 从而判断出角点与平面点
    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < N_SCAN; i++) {
      surfaceCloudScan->clear();

      // 只用下方的7根线作为平面点的提取
      for (int j = 0; j < 6; j++) {

        int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
        int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

        if (sp >= ep)
          continue;

        // 每个子图中的点按曲率排序
        std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

        int largestPickedNum = 0;
        for (int k = ep; k >= sp; k--) {
          int ind = cloudSmoothness[k].ind;
          // 曲率比较大的是边缘点
          if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold) {
            largestPickedNum++;
            if (largestPickedNum <= 20) {  // 选取20个边缘点
              cloudLabel[ind] = 1;
              cornerCloud->push_back(extractedCloud->points[ind]);
            } else {
              break;
            }
            // 防止特征点聚集
            // 从ind+l开始后面5个点，每个点index之间的差值，
            // 确保columnDiff<=10,然后标记为我们需要的点
            cloudNeighborPicked[ind] = 1;
            for (int l = 1; l <= 5; l++) {
              int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--) {
              int columnDiff = std::abs(
                  int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        // 标记平面点
        int smallestPickedNum = 0;
        for (int k = sp; k <= ep; k++) {
          int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold) {
            // 在还没有标记并且曲率较小的点里面选
            cloudLabel[ind] = -1;
            cloudNeighborPicked[ind] = 1;

            // 防止平面点聚集
            for (int l = 1; l <= 5; l++) {
              int columnDiff = std::abs(
                  int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--) {
              int columnDiff = std::abs(
                  int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        // label小于0的点是平面点
        for (int k = sp; k <= ep; k++) {
          if (cloudLabel[k] <= 0) {
            surfaceCloudScan->push_back(extractedCloud->points[k]);
          }
        }
      }

      // 下采样
      surfaceCloudScanDS->clear();
      downSizeFilter.setInputCloud(surfaceCloudScan);
      downSizeFilter.filter(*surfaceCloudScanDS);

      *surfaceCloud += *surfaceCloudScanDS;
    }
  }

  void freeCloudInfoMemory() {
    cloudInfo.startRingIndex.clear();
    cloudInfo.endRingIndex.clear();
    cloudInfo.pointColInd.clear();
    cloudInfo.pointRange.clear();
  }

  void publishFeatureCloud() {
    // free cloud info memory
    freeCloudInfoMemory();
    // save newly extracted features
    cloudInfo.cloud_corner = publishCloud(&pubCornerPoints, cornerCloud, cloudHeader.stamp, "base_link");
    cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, "base_link");
    // publish to mapOptimization
    pubLaserCloudInfo.publish(cloudInfo);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_sam");

  FeatureExtraction FE;

  ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

  ros::spin();

  return 0;
}