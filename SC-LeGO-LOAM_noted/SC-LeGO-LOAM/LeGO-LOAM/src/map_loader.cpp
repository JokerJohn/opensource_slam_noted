#include "map_loader.h"

MapLoader::MapLoader(ros::NodeHandle &nh) {
    std::string pcd_file_path, map_topic;
    nh.param<std::string>("pcd_path", pcd_file_path, "");
    nh.param<std::string>("map_topic", map_topic, "point_map");

    pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 10, true);

    file_list_.push_back(pcd_file_path);

    auto pc_msg = CreatePcd();

    if (pc_msg.width != 0) {
        pc_msg.header.frame_id = "map";
        pc_map_pub_.publish(pc_msg);
    }

}

sensor_msgs::PointCloud2 MapLoader::CreatePcd() {
    sensor_msgs::PointCloud2 pcd, part;
    for (const std::string &path : file_list_) {
        // Following outputs are used for progress bar of Runtime Manager.
        if (pcd.width == 0) {
            if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
                std::cerr << "load failed " << path << std::endl;
            }
        } else {
            if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
                std::cerr << "load failed " << path << std::endl;
            }
            pcd.width += part.width;
            pcd.row_step += part.row_step;
            pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
        }
        std::cerr << "load " << path << std::endl;
        if (!ros::ok()) break;
    }

    return pcd;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "map_loader");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

    ros::NodeHandle nh("~");

    MapLoader map_loader(nh);

    ros::spin();

    return 0;
}
