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
#include "label_generator/label_generator.hpp"


using PointType = PointXYZILID;
using namespace std;

ros::Publisher CloudPublisher;
ros::Publisher TPPublisher;
ros::Publisher FPPublisher;
ros::Publisher FNPublisher;
ros::Publisher TNPublisher;
ros::Publisher PrecisionPublisher;
ros::Publisher RecallPublisher;
ros::Publisher EstGroundPublisher;
ros::Publisher EstGroundFilteredPublisher;

boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string abs_save_dir;
std::string output_filename;
std::string acc_filename;
std::string pcd_savepath;
std::string data_path;
string      algorithm;
string      seq;
bool        save_flag;
bool        use_sor_before_save;

pcl::PointCloud<PointType>::Ptr filtered;

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}

void pub_score(std::string mode, double measure) {
    static int                 SCALE = 5;
    visualization_msgs::Marker marker;
    marker.header.frame_id                  = "map";
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
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg) {
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg, cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}


int main(int argc, char**argv) {
    ros::init(argc, argv, "Offline KITTI");

    ros::NodeHandle nh;
    int start_frame, end_frame;
    nh.param<int>("/start_frame", start_frame, 0);
    nh.param<int>("/end_frame", end_frame, 10000);
    nh.param<bool>("/save_flag", save_flag, false);
    nh.param<bool>("/use_sor_before_save", use_sor_before_save, false);
    nh.param<string>("/algorithm", algorithm, "patchwork");
    nh.param<string>("/seq", seq, "00");
    nh.param<string>("/data_path", data_path, "/");

    CloudPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100, true);
    TPPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/TP", 100, true);
    FPPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FP", 100, true);
    FNPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FN", 100, true);
    TNPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/TN", 100, true);
    PrecisionPublisher = nh.advertise<visualization_msgs::Marker>("/precision", 1, true);
    RecallPublisher    = nh.advertise<visualization_msgs::Marker>("/recall", 1, true);

    EstGroundPublisher         = nh.advertise<sensor_msgs::PointCloud2>("/estimate/ground", 100, true);
    EstGroundFilteredPublisher = nh.advertise<sensor_msgs::PointCloud2>("/estimate/ground_filtered", 100, true);

    signal(SIGINT, signal_callback_handler);

    if (save_flag) {
        abs_save_dir = data_path + "/patchwork";
        std::cout << "\033[1;34m" << abs_save_dir << "\033[0m" << std::endl;
        std::experimental::filesystem::create_directory(abs_save_dir);
    }

    PatchworkGroundSeg.reset(new PatchWork<PointType>(&nh));
    cout << "Target data: " << data_path << endl;
    KittiLoader loader(data_path);

    int      N = loader.size();
    for (int n = max(0, start_frame); n < min(N, end_frame); ++n) {
        cout << n << "th node come" << endl;
        pcl::PointCloud<PointType> pc_curr;
        loader.get_cloud(n, pc_curr);
        pcl::PointCloud<PointType> pc_ground;
        pcl::PointCloud<PointType> pc_non_ground;

        static double time_taken;
        cout << "Operating patchwork..." << endl;
        PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

        // Estimation
        double precision, recall, precision_naive, recall_naive;
        calculate_precision_recall(pc_curr, pc_ground, precision, recall);
        calculate_precision_recall(pc_curr, pc_ground, precision_naive, recall_naive, false);

        cout << "\033[1;32m" << n << "th, " << " takes : " << time_taken << " | " << pc_curr.size() << " -> "
             << pc_ground.size()
             << "\033[0m" << endl;

        cout << "\033[1;32m P: " << precision << " | R: " << recall << "\033[0m" << endl;


// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save precision/recall in a text file, revise this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        output_filename = "/home/shapelim/patchwork.txt";
        ofstream ground_output(output_filename, ios::app);
        ground_output << n << "," << time_taken << "," << precision << "," << recall << "," << precision_naive << "," << recall_naive;
        ground_output << std::endl;
        ground_output.close();
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        // Publish msg
        pcl::PointCloud<PointType> TP;
        pcl::PointCloud<PointType> FP;
        pcl::PointCloud<PointType> FN;
        pcl::PointCloud<PointType> TN;
        discern_ground(pc_ground, TP, FP);
        discern_ground(pc_non_ground, FN, TN);

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save the direct output of pcd, revise this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        if (save_flag) {
//            std::map<int, int> pc_curr_gt_counts, g_est_gt_counts;
//            double             accuracy;
//            save_all_accuracy(pc_curr, pc_ground, acc_filename, accuracy, pc_curr_gt_counts, g_est_gt_counts);
//
//            std::string count_str        = std::to_string(n);
//            std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
//            std::string pcd_filename     = pcd_savepath + "/" + count_str_padded + ".pcd";
//            pc2pcdfile(TP, FP, FN, TN, pcd_filename);
//        }
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save the estimate as label file, please this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        if (save_flag) { // To make `.label` file
            if (use_sor_before_save) {
                filtered.reset(new pcl::PointCloud<PointType>());
                filter_by_sor(pc_ground, *filtered);
                save_ground_label(abs_save_dir, n, pc_curr, *filtered);
            } else {
                save_ground_label(abs_save_dir, n, pc_curr, pc_ground);
            }
        }
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        CloudPublisher.publish(cloud2msg(pc_curr));
        TPPublisher.publish(cloud2msg(TP));
        FPPublisher.publish(cloud2msg(FP));
        FNPublisher.publish(cloud2msg(FN));
        TNPublisher.publish(cloud2msg(TN));
        EstGroundPublisher.publish(cloud2msg(pc_ground));
        if (use_sor_before_save) {
            EstGroundFilteredPublisher.publish(cloud2msg(*filtered));
        }
        pub_score("p", precision);
        pub_score("r", recall);
    }

    return 0;
}
