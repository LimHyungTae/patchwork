#ifndef TRAVEL_UTILS_H
#define TRAVEL_UTILS_H

#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <signal.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include "label_generator/nanoflann.hpp"
#include "label_generator/nanoflann_utils.hpp"

#define EST_NON_GROUND 0
using namespace std;

using num_t = float; // for NanoFlann

template <typename PointType>
    void filter_by_sor(const pcl::PointCloud<PointType> &src, pcl::PointCloud<PointType> &filtered,
                      int num_neighbor_pts=20, float std_multiplier=5.0) {

    typename pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>);
    *tmp = src;

    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud (tmp);
    sor.setMeanK (num_neighbor_pts);
    sor.setStddevMulThresh (std_multiplier);
    sor.filter(filtered);
}

template <typename PointType>
void save_ground_label(const std::string abs_dir, const int frame_num,
                      const pcl::PointCloud<PointType> &cloud_raw,
                      const pcl::PointCloud<PointType> &cloud_est_ground,
                      uint32_t GROUND_LABEL=1) {
    // Save the estimate ground points into a `.label` file
    // It is relevant to 3DUIS benchmark and Stachniss lab's format
    // https://codalab.lisn.upsaclay.fr/competitions/2183?secret_key=4763e3d2-1f22-45e6-803a-a862528426d2
    const float SQR_EPSILON = 0.00001;

    int num_cloud_raw = cloud_raw.points.size();
    std::vector<uint32_t> labels(num_cloud_raw, EST_NON_GROUND); // 0: Non ground points

    size_t N = cloud_est_ground.points.size();
    PointCloud<num_t> cloud;
    cloud.pts.resize(N);
    for (size_t i = 0; i < N; i++) {
        cloud.pts[i].x = cloud_est_ground.points[i].x;
        cloud.pts[i].y = cloud_est_ground.points[i].y;
        cloud.pts[i].z = cloud_est_ground.points[i].z;
    }

    // construct a kd-tree index:
    using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t>>,
            PointCloud<num_t>, 3 /* dim */
    >;

    my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});

    int num_valid = 0;
    for (size_t j = 0; j < cloud_raw.points.size(); ++j) {
        const auto query_pcl = cloud_raw.points[j];
        const num_t query_pt[3] = {query_pcl.x, query_pcl.y, query_pcl.z};

        size_t num_results = 1;
        std::vector<uint32_t> ret_index(num_results);
        std::vector<num_t> out_dist_sqr(num_results);

        num_results = index.knnSearch(
                &query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

        ret_index.resize(num_results);
        out_dist_sqr.resize(num_results);
        if(out_dist_sqr[0] < SQR_EPSILON) { // it is the same point!
            // Est. ground - 1, Est. nonground - 0
            // But BE CAREFUL! In some cases, ground are labeled as `9`,
            // which is Rhiney (Xieyuanli Chen)'s previous researches
            // Please refer to the link:
            labels[j] = GROUND_LABEL;
            ++num_valid;
        }
    }

    // Must be equal to the # of above-ground points
    std::cout << "# of valid points: " << num_valid << std::endl;

    //  To follow the KITTI format, # of zeros are set to 6
    const int NUM_ZEROS = 6;

    std::string count_str = std::to_string(frame_num);
    std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
    std::string abs_label_path = abs_dir + "/" + count_str_padded + ".label";

    std::cout << "\033[1;32m" << abs_label_path << "\033[0m" << std::endl;
    std::ofstream output_file(abs_label_path, std::ios::out | std::ios::binary);
    output_file.write(reinterpret_cast<char*>(&labels[0]), num_cloud_raw * sizeof(uint32_t));
}

#endif
