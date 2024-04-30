// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE
#include <patchwork/node.h>
#include <patchwork/ground_estimate.h>
#include "patchwork/patchwork.hpp"
#include <visualization_msgs/Marker.h>

#include "tools/kitti_loader.hpp"
#include "tools/pcd_loader.hpp"

using namespace std;


ros::Publisher CloudPublisher;
ros::Publisher PositivePublisher;
ros::Publisher NegativePublisher;
ros::Publisher EstimatePublisher;

//using PointType = PointXYZILID;
using PointType = pcl::PointXYZ;
boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      mode;
string      seq;
bool        save_flag;


template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(const sensor_msgs::PointCloud2::ConstPtr& cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(*cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud)
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    return cloud_ROS;
}

void callbackNode(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cout << msg->header.seq << "th node come" << endl;
    pcl::PointCloud<PointType> pc_curr = cloudmsg2cloud<PointType>(msg);
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    pc_ground.header = pc_curr.header;
    pc_non_ground.header = pc_curr.header;

    static double time_taken;

    cout << "Operating patchwork..." << endl;
    PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);


    auto msg_curr = cloud2msg(pc_curr, PatchworkGroundSeg->frame_patchwork);
    auto msg_ground = cloud2msg(pc_ground, PatchworkGroundSeg->frame_patchwork);

    patchwork::ground_estimate cloud_estimate;
    cloud_estimate.header = msg->header;
    cloud_estimate.curr = msg_curr;
    cloud_estimate.ground = msg_ground;
    EstimatePublisher.publish(cloud_estimate);

    /*
    CloudPublisher.publish(cloud2msg(pc_curr, PatchworkGroundSeg->frame_patchwork));
    PositivePublisher.publish(cloud2msg(pc_ground, PatchworkGroundSeg->frame_patchwork));
    NegativePublisher.publish(cloud2msg(pc_non_ground, PatchworkGroundSeg->frame_patchwork));
    */
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Benchmark");
    ros::NodeHandle nh;
    condParam<string>(&nh, "/algorithm", algorithm, "patchwork", "");
    condParam<string>(&nh, "/seq", seq, "00", "");

    PatchworkGroundSeg.reset(new PatchWork<PointType>(&nh));

    /* Publisher for source cloud, ground, non-ground */
    CloudPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100, true);
    PositivePublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/P", 100, true);
    NegativePublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/N", 100, true);

    /* Publisher for combined msg of source cloud, ground cloud */
    EstimatePublisher = nh.advertise<patchwork::ground_estimate>("/benchmark/ground_estimate", 100, true);

    ros::Subscriber NodeSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/node", 5000, callbackNode);

    ros::spin();

    return 0;
}
