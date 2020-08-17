//
// Created by xchu on 2020/6/30.
//

#ifndef SRC_TRANSFORM_FUSION_H
#define SRC_TRANSFORM_FUSION_H

#include "lego_loam/utility.h"

namespace lego_loam{
    /**
 * scan-scan 和 scan -map 的俩odom融合
 */
    class TransformFusion {

    public:
        TransformFusion();

        void transformAssociateToMap();

        void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);

        void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr &odomAftMapped);

    private:
        ros::NodeHandle nh;

        ros::Publisher pubLaserOdometry2;
        ros::Subscriber subLaserOdometry;
        ros::Subscriber subOdomAftMapped;


        nav_msgs::Odometry laserOdometry2;
        tf::StampedTransform laserOdometryTrans2;
        tf::TransformBroadcaster tfBroadcaster2;

        tf::StampedTransform map_2_camera_init_Trans;
        tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

        tf::StampedTransform camera_2_base_link_Trans;
        tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

        float transformSum[6];
        float transformIncre[6];
        float transformMapped[6];
        float transformBefMapped[6];
        float transformAftMapped[6];

        std_msgs::Header currentHeader;
        nav_msgs::Path laser_gt_path;

        ros::Publisher pubLaserAfterMappedPath;
        nav_msgs::Path laserAfterMappedPath;
        // std::vector<geometry_msgs::PoseStamped> laserAfterMappedPath;
        std::string odom_trajectory_path;
    };


}

#endif //SRC_TRANSFORM_FUSION_H
