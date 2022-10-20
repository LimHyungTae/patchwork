//
// Created by Hyungtae Lim on 6/23/21.
//

// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE
#include "patchwork/patchwork.hpp"
#include <cstdlib>


using PointType = pcl::PointXYZ;
using namespace std;

ros::Publisher CloudPublisher;
ros::Publisher PositivePublisher;
ros::Publisher NegativePublisher;

boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      extension;
string      file_dir;
string      mode;
string      seq;
bool        save_flag;

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

int count_num_files(const string folder_dir, const string extension) {
    int num_frames = 0;
    for (num_frames = 0;; num_frames++) {
        std::string filename = (boost::format("%s/%06d.%s") % folder_dir % num_frames % extension).str();
        if (!boost::filesystem::exists(filename)) {
          break;
        }
    }
    if (!num_frames) throw invalid_argument("Something is wrong. The # of files is zero.");

    return num_frames;
}

int main(int argc, char **argv) {
    bool stop_each_frame;
    ros::init(argc, argv, "Benchmark");
    ros::NodeHandle nh;
    nh.param<string>("/algorithm", algorithm, "patchwork");
    nh.param<string>("/extension", extension, "pcd");
    nh.param<string>("/file_dir", file_dir, "/catkin_ws/src/patchwork/materials");
    nh.param<bool>("/stop_each_frame", stop_each_frame, false);
    ros::Rate loop_rate(10);

    PatchworkGroundSeg.reset(new PatchWork<PointType>(&nh));

    CloudPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100, true);
    PositivePublisher     = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/ground", 100, true);
    NegativePublisher     = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/non_ground", 100, true);

    string abs_file_dir = std::getenv("HOME") + file_dir;

    int num_pcds = count_num_files(abs_file_dir, extension);

    for (int i = 0; i < num_pcds; ++i) {
        // An example for Loading own data
        pcl::PointCloud<PointType> pc_curr;
        std::string filename = (boost::format("%s/%06d.%s") % abs_file_dir % i % extension).str();
        if (pcl::io::loadPCDFile<PointType>(filename, pc_curr) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }

        pcl::PointCloud<PointType> pc_ground;
        pcl::PointCloud<PointType> pc_non_ground;

        static double time_taken;
        cout << i << "th operation..." << endl;

        PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
        CloudPublisher.publish(cloud2msg(pc_curr));
        PositivePublisher.publish(cloud2msg(pc_ground));
        NegativePublisher.publish(cloud2msg(pc_non_ground));

        if (stop_each_frame) {
            cout<< "STOP!" <<endl;
            cin.ignore();
        }
    }
    ros::spin();

    return 0;
}
