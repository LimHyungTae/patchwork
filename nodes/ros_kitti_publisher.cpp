//
// Created by Hyungtae Lim on 6/23/21.
//

// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE
#include <patchwork/node.h>
#include "patchwork/patchwork.hpp"
#include <visualization_msgs/Marker.h>
#include "tools/kitti_loader.hpp"
#include <signal.h>


using PointType = PointXYZILID;
using namespace std;

ros::Publisher CloudPublisher;

boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename;
std::string pcd_savepath;
std::string data_path;
string      algorithm;
string      seq;
bool        save_flag;

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id ) {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "Ros-Kitti-Publisher");

    ros::NodeHandle nh;
    condParam<string>(&nh, "/algorithm", algorithm, "patchwork", "");
    condParam<string>(&nh, "/seq", seq, "00", "");
    condParam<string>(&nh, "/data_path", data_path, "/", "");

    ros::Rate r(10);
    ros::Publisher CloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/cloud", 100, true);

    signal(SIGINT, signal_callback_handler);

    KittiLoader loader(data_path);
    int      N = loader.size();

    std::cerr << "\033[1;32m[Kitti Publisher] Total " << N << " clouds are loaded\033[0m" << std::endl;
    for (int n = 0; n < N; ++n) {
        cout << n << "th node is published!" << endl;
        pcl::PointCloud<PointType> pc_curr;
        loader.get_cloud(n, pc_curr);
        CloudPublisher.publish(cloud2msg(pc_curr, PatchworkGroundSeg->frame_patchwork));
        r.sleep();
    }
    return 0;
}