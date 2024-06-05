// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE
#include <patchwork/node.h>
#include "patchwork/patchwork.hpp"
#include <visualization_msgs/Marker.h>

#include "tools/kitti_loader.hpp"
#include "tools/pcd_loader.hpp"

using namespace std;

ros::Publisher CloudPublisher;
ros::Publisher GroundPublisher;
ros::Publisher NonGroundPublisher;
ros::Publisher TPPublisher;
ros::Publisher FPPublisher;
ros::Publisher FNPublisher;
ros::Publisher PrecisionPublisher;
ros::Publisher RecallPublisher;

using PointType = PointXYZILID;
boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      mode;
string      seq;
bool        save_flag;
bool        is_kitti;

void pub_score(std::string mode, double measure) {
    static int                 SCALE = 5;
    visualization_msgs::Marker marker;
    marker.header.frame_id                  = PatchworkGroundSeg->frame_patchwork;
    marker.header.stamp                     = ros::Time();
    marker.ns                               = "my_namespace";
    marker.id                               = 0;
    marker.type                             = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action                           = visualization_msgs::Marker::ADD;
    if (mode == "p") marker.pose.position.x = 28.5;
    if (mode == "r") marker.pose.position.x = 25;
    marker.pose.position.y                  = 30;

    marker.pose.position.z    = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = SCALE;
    marker.scale.y            = SCALE;
    marker.scale.z            = SCALE;
    marker.color.a            = 1.0; // Don't forget to set the alpha!
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.text               = mode + ": " + std::to_string(measure);
    if (mode == "p") PrecisionPublisher.publish(marker);
    if (mode == "r") RecallPublisher.publish(marker);

}

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud)
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    return cloud_ROS;
}

void callbackNode(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cout << msg->header.seq << "th node come" << endl;
    pcl::PointCloud<PointType> pc_curr = cloudmsg2cloud<PointType>(*msg);
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    pc_ground.header = pc_curr.header;
    pc_non_ground.header = pc_curr.header;

    static double time_taken;

    cout << "Operating patchwork..." << endl;
    PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

    // Estimation
    double precision, recall, precision_naive, recall_naive;
    calculate_precision_recall(pc_curr, pc_ground, precision, recall);
    calculate_precision_recall(pc_curr, pc_ground, precision_naive, recall_naive, false);

    cout << "\033[1;32m" << msg->header.seq << "th, " << " takes : " << time_taken << " | " << pc_curr.size() << " -> " << pc_ground.size()
         << "\033[0m" << endl;

    cout << "\033[1;32m P: " << precision << " | R: " << recall << "\033[0m" << endl;

//    output_filename = "/home/shapelim/patchwork_debug.txt";
//    ofstream sc_output(output_filename, ios::app);
//    sc_output << msg->header.seq << "," << time_taken << "," << precision << "," << recall << "," << precision_naive << "," << recall_naive;
//
//    sc_output << std::endl;
//    sc_output.close();

    // Publish msg
    pcl::PointCloud<PointType> TP;
    pcl::PointCloud<PointType> FP;
    pcl::PointCloud<PointType> FN;
    pcl::PointCloud<PointType> TN;
    TP.header = pc_curr.header;
    FP.header = pc_curr.header;
    FN.header = pc_curr.header;
    TN.header = pc_curr.header;

    if (is_kitti) {
        discern_ground(pc_ground, TP, FP);
        discern_ground(pc_non_ground, FN, TN);

        if (save_flag) {
            std::map<int, int> pc_curr_gt_counts, g_est_gt_counts;
            double accuracy;
            save_all_accuracy(pc_curr, pc_ground, acc_filename, accuracy, pc_curr_gt_counts, g_est_gt_counts);

            std::string count_str = std::to_string(msg->header.seq);
            std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
            std::string pcd_filename = pcd_savepath + "/" + count_str_padded + ".pcd";

//    pc2pcdfile(TP, FP, FN, TN, pcd_filename);
        }
    }

    CloudPublisher.publish(cloud2msg(pc_curr));
    if (is_kitti) {
        TPPublisher.publish(cloud2msg(TP));
        FPPublisher.publish(cloud2msg(FP));
        FNPublisher.publish(cloud2msg(FN));
    } else {
        // Since other dataset has no labels,
        // so only estimated ground points / non-ground points are available
        if (GroundPublisher.getNumSubscribers())
            GroundPublisher.publish(cloud2msg(pc_ground));
        if (NonGroundPublisher.getNumSubscribers())
            NonGroundPublisher.publish(cloud2msg(pc_non_ground));
    }
    pub_score("p", precision);
    pub_score("r", recall);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Benchmark");
    ros::NodeHandle nh;
    condParam<string>(&nh, "/algorithm", algorithm, "patchwork","");
    condParam<string>(&nh, "/seq", seq, "00","");
    condParam<bool>(&nh, "/is_kitti", is_kitti, true,"");

    PatchworkGroundSeg.reset(new PatchWork<PointType>(&nh));

    CloudPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100, true);
    TPPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/TP", 100, true);
    FPPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FP", 100, true);
    FNPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FN", 100, true);
    GroundPublisher = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/ground", 100, false);
    NonGroundPublisher = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/non_ground", 100, false);

    PrecisionPublisher = nh.advertise<visualization_msgs::Marker>("/precision", 1, true);
    RecallPublisher    = nh.advertise<visualization_msgs::Marker>("/recall", 1, true);

    ros::Subscriber NodeSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/patchwork/cloud", 5000, callbackNode);

    if (is_kitti) {
        std::cout << "\033[1;33mKITTI mode is activated. If not, please check `is_kitti` parameter\033[0m" << std::endl;
    }
    ros::spin();

    return 0;
}
