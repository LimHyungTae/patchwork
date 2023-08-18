#ifndef PATCHWORK_H
#define PATCHWORK_H

#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

#include <tbb/parallel_for.h>

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000
#define MARKER_Z_VALUE -2.2

// Below colors are for visualization purpose
#define COLOR_CYAN 0.55  // cyan
#define COLOR_GREEN 0.2  // green
#define COLOR_BLUE 0.0   // blue
#define COLOR_RED 1.0    // red
#define COLOR_GLOBALLY_TOO_HIGH_ELEVATION 0.8 // I'm not sure...haha

int NOT_ASSIGNED = -2;
int FEW_POINTS = -1;
int UPRIGHT_ENOUGH = 0; // cyan
int FLAT_ENOUGH = 1; // green
int TOO_HIGH_ELEVATION = 2; // blue
int TOO_TILTED = 3; // red
int GLOBALLY_TOO_HIGH_ELEVATION = 4;

std::vector<float> COLOR_MAP = {COLOR_CYAN, COLOR_GREEN, COLOR_BLUE, COLOR_RED, COLOR_GLOBALLY_TOO_HIGH_ELEVATION};

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

using namespace std;

/*
    @brief PathWork ROS Node.
*/
template<typename PointT>
bool point_z_cmp(PointT a, PointT b) {
    return a.z < b.z;
}
struct PatchIdx
{
    int zone_idx_;
    int ring_idx_;
    int sector_idx_;
    int concentric_idx_;
};

struct PCAFeature {
    Eigen::Vector3f principal_;
    Eigen::Vector3f normal_;
    Eigen::Vector3f singular_values_;
    Eigen::Vector3f mean_;
    float    d_;
    float    th_dist_d_;
    float    linearity_;
    float    planarity_;
};

template<typename PointT>
class PatchWork {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::vector<pcl::PointCloud<PointT> > Ring;
    typedef std::vector<Ring>                     Zone;

    PatchWork() {};

    PatchWork(ros::NodeHandle *nh) : node_handle_(*nh) {
        // Init ROS related
        ROS_INFO("Inititalizing PatchWork...");

        node_handle_.param<double>("/sensor_height", sensor_height_, 1.723);
        node_handle_.param<bool>("/patchwork/verbose", verbose_, false);

        node_handle_.param<bool>("/patchwork/ATAT/ATAT_ON", ATAT_ON_, false);
        node_handle_.param("/patchwork/ATAT/max_r_for_ATAT", max_r_for_ATAT_, 5.0);
        node_handle_.param("/patchwork/ATAT/num_sectors_for_ATAT", num_sectors_for_ATAT_, 20);
        node_handle_.param("/patchwork/ATAT/noise_bound", noise_bound_, 0.2);

        node_handle_.param("/patchwork/num_iter", num_iter_, 3);
        node_handle_.param("/patchwork/num_lpr", num_lpr_, 20);
        node_handle_.param("/patchwork/num_min_pts", num_min_pts_, 10);
        node_handle_.param("/patchwork/th_seeds", th_seeds_, 0.4);
        node_handle_.param("/patchwork/th_dist", th_dist_, 0.3);
        node_handle_.param("/patchwork/max_r", max_range_, 80.0);
        node_handle_.param("/patchwork/min_r", min_range_, 2.7); // It indicates bodysize of the car.
        node_handle_.param("/patchwork/uniform/num_rings", num_rings_, 30);
        node_handle_.param("/patchwork/uniform/num_sectors", num_sectors_, 108);
        node_handle_.param("/patchwork/uprightness_thr", uprightness_thr_, 0.5); // The larger, the more strict
        node_handle_.param("/patchwork/adaptive_seed_selection_margin", adaptive_seed_selection_margin_,
                           -1.1); // The more larger, the more soft

        // It is not in the paper
        // It is also not matched our philosophy, but it is employed to reject some FPs easily & intuitively.
        // For patchwork, it is only applied on Z3 and Z4
        node_handle_.param<bool>("/patchwork/using_global_elevation", using_global_thr_, true);
        node_handle_.param("/patchwork/global_elevation_threshold", global_elevation_thr_, 0.0);

        if (using_global_thr_) {
            cout << "\033[1;33m[Warning] Global elevation threshold is turned on :" << global_elevation_thr_ << "\033[0m" << endl;
        } else { cout << "Global thr. is not in use" << endl; }

        ROS_INFO("Sensor Height: %f", sensor_height_);
        ROS_INFO("Num of Iteration: %d", num_iter_);
        ROS_INFO("Num of LPR: %d", num_lpr_);
        ROS_INFO("Num of min. points: %d", num_min_pts_);
        ROS_INFO("Seeds Threshold: %f", th_seeds_);
        ROS_INFO("Distance Threshold: %f", th_dist_);
        ROS_INFO("Max. range:: %f", max_range_);
        ROS_INFO("Min. range:: %f", min_range_);
        ROS_INFO("Num. rings: %d", num_rings_);
        ROS_INFO("Num. sectors: %d", num_sectors_);
        ROS_INFO("adaptive_seed_selection_margin: %f", adaptive_seed_selection_margin_);

        // CZM denotes 'Concentric Zone Model'. Please refer to our paper
        node_handle_.getParam("/patchwork/czm/num_zones", num_zones_);
        node_handle_.getParam("/patchwork/czm/num_sectors_each_zone", num_sectors_each_zone_);
        node_handle_.getParam("/patchwork/czm/num_rings_each_zone", num_rings_each_zone_);
        node_handle_.getParam("/patchwork/czm/min_ranges_each_zone", min_ranges_);
        node_handle_.getParam("/patchwork/czm/elevation_thresholds", elevation_thr_);
        node_handle_.getParam("/patchwork/czm/flatness_thresholds", flatness_thr_);

        ROS_INFO("\033[1;32mUprightness\33[0m threshold: %f", uprightness_thr_);
        ROS_INFO("\033[1;32mElevation\33[0m thresholds: %f %f %f %f", elevation_thr_[0],elevation_thr_[1], elevation_thr_[2], elevation_thr_[3]);
        ROS_INFO("\033[1;32mFlatness\033[0m thresholds: %f %f %f %f", flatness_thr_[0], flatness_thr_[1], flatness_thr_[2], flatness_thr_[3]);

        ROS_INFO("Num. zones: %d", num_zones_);

        check_input_parameters_are_correct();
//        cout_params();

        // It equals to elevation_thr_.size()/flatness_thr_.size();
        num_rings_of_interest_ = elevation_thr_.size();

        node_handle_.param("/patchwork/visualize", visualize_, true);
        poly_list_.header.frame_id = "/map";
        poly_list_.polygons.reserve(130000);

        reverted_points_by_flatness_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

        PlanePub      = node_handle_.advertise<jsk_recognition_msgs::PolygonArray>("/gpf/plane", 100);
        RevertedCloudPub = node_handle_.advertise<sensor_msgs::PointCloud2>("/revert_pc", 100);
        RejectedCloudPub = node_handle_.advertise<sensor_msgs::PointCloud2>("/reject_pc", 100);

        min_range_z2_ = min_ranges_[1];
        min_range_z3_ = min_ranges_[2];
        min_range_z4_ = min_ranges_[3];

        min_ranges_   = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_};
        ring_sizes_   = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                         (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                         (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                         (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
        sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0), 2 * M_PI / num_sectors_each_zone_.at(1),
                         2 * M_PI / num_sectors_each_zone_.at(2),
                         2 * M_PI / num_sectors_each_zone_.at(3)};
        cout << "INITIALIZATION COMPLETE" << endl;

        for (int iter = 0; iter < num_zones_; ++iter) {
            Zone z;
            initialize_zone(z, num_sectors_each_zone_.at(iter), num_rings_each_zone_.at(iter));
            ConcentricZoneModel_.push_back(z);
        }
        PatchIdx patch_idx;
        int concentric_idx_tmp = 0;
        for (int zone_idx = 0; zone_idx < num_zones_; ++zone_idx) {
            for (uint16_t ring_idx = 0; ring_idx < num_rings_each_zone_[zone_idx]; ++ring_idx) {
                for (uint16_t sector_idx = 0; sector_idx < num_sectors_each_zone_[zone_idx]; ++sector_idx) {
                    patch_idx.zone_idx_ = zone_idx;
                    patch_idx.ring_idx_ = ring_idx;
                    patch_idx.sector_idx_ = sector_idx;
                    patch_idx.concentric_idx_ = concentric_idx_tmp;
                    patch_indices_.emplace_back(patch_idx);
                }
                concentric_idx_tmp++;
            }
        }

        int num_patches = patch_indices_.size();
        indices_.resize(num_patches);
        std::iota(indices_.begin(), indices_.end(), 0);
        statuses_.assign(num_patches, NOT_ASSIGNED);
        features_.resize(num_patches);
        regionwise_grounds_.resize(num_patches);
        regionwise_nongrounds_.resize(num_patches);
    }

    void estimate_ground(
            const pcl::PointCloud<PointT> &cloud_in,
            pcl::PointCloud<PointT> &ground,
            pcl::PointCloud<PointT> &nonground,
            double &time_taken);

    geometry_msgs::PolygonStamped set_plane_polygon(const MatrixXf &normal_v, const float &d);

private:
    ros::NodeHandle node_handle_;

    // For ATAT (All-Terrain Automatic heighT estimator)
    bool ATAT_ON_;
    double noise_bound_;
    double max_r_for_ATAT_;
    int num_sectors_for_ATAT_;

    int num_iter_;
    int num_lpr_;
    int num_min_pts_;
    int num_rings_;
    int num_sectors_;
    int num_zones_;
    int num_rings_of_interest_;

    double sensor_height_;
    double th_seeds_;
    double th_dist_;
    double max_range_;
    double min_range_;
    double uprightness_thr_;
    double adaptive_seed_selection_margin_;
    double min_range_z2_; // 12.3625
    double min_range_z3_; // 22.025
    double min_range_z4_; // 41.35

    bool verbose_;
    bool initialized_ = true;

    // For global threshold
    bool   using_global_thr_;
    double global_elevation_thr_;

    double          ring_size;
    double          sector_size;
    // For visualization
    bool            visualize_;

    vector<int>      indices_;
    vector<int>      statuses_;
    vector<PatchIdx> patch_indices_;
    vector<PCAFeature> features_;
    vector<pcl::PointCloud<PointT>> regionwise_grounds_;
    vector<pcl::PointCloud<PointT>> regionwise_nongrounds_;

    vector<int> num_sectors_each_zone_;
    vector<int> num_rings_each_zone_;

    vector<double> sector_sizes_;
    vector<double> ring_sizes_;
    vector<double> min_ranges_;
    vector<double> elevation_thr_;
    vector<double> flatness_thr_;

    vector<Zone> ConcentricZoneModel_;

    jsk_recognition_msgs::PolygonArray poly_list_;

    ros::Publisher          PlanePub, RevertedCloudPub, RejectedCloudPub;
    pcl::PointCloud<PointT> reverted_points_by_flatness_, rejected_points_by_elevation_;


    void initialize_zone(Zone &z, int num_sectors, int num_rings);

    void flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings);

    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    void pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm);

    void estimate_plane_(const pcl::PointCloud<PointT> &ground, PCAFeature& feat);

    void extract_piecewiseground(
            const int zone_idx, const pcl::PointCloud<PointT> &src,
            PCAFeature &feat,
            pcl::PointCloud<PointT> &regionwise_ground,
            pcl::PointCloud<PointT> &regionwise_nonground,
            bool is_h_available=true);

    double consensus_set_based_height_estimation(const Eigen::RowVectorXd& X,
                                                 const Eigen::RowVectorXd& ranges,
                                                 const Eigen::RowVectorXd& weights);

    void estimate_sensor_height(pcl::PointCloud<PointT> cloud_in);

    void extract_initial_seeds_(
            const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
            pcl::PointCloud<PointT> &init_seeds, bool is_h_available=true);

    void check_input_parameters_are_correct();

    void cout_params();

    /***
     * For visulization of Ground Likelihood Estimation
     */
    geometry_msgs::PolygonStamped set_polygons(int zone_idx, int r_idx, int theta_idx, int num_split);

    int determine_ground_likelihood_estimation_status(
            const int concentric_idx,
            const double z_vec,
            const double z_elevation,
            const double surface_variable);
};


template<typename PointT>
inline
void PatchWork<PointT>::initialize_zone(Zone &z, int num_sectors, int num_rings) {
    z.clear();
    pcl::PointCloud<PointT> cloud;
    cloud.reserve(1000);
    Ring     ring;
    for (int i = 0; i < num_sectors; i++) {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++) {
        z.emplace_back(ring);
    }
}

template<typename PointT>
inline
void PatchWork<PointT>::flush_patches_in_zone(Zone &patches, int num_sectors, int num_rings) {
    for (int i = 0; i < num_sectors; i++) {
        for (int j = 0; j < num_rings; j++) {
            if (!patches[j][i].points.empty()) patches[j][i].points.clear();
        }
    }
}

template<typename PointT>
inline
void PatchWork<PointT>::estimate_plane_(const pcl::PointCloud<PointT> &ground, PCAFeature& feat) {
    Eigen::Vector4f pc_mean;
    Eigen::Matrix3f cov;
    pcl::computeMeanAndCovarianceMatrix(ground, cov, pc_mean);

    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    feat.singular_values_ = svd.singularValues();

    feat.linearity_ = ( feat.singular_values_(0) - feat.singular_values_(1) ) / feat.singular_values_(0);
    feat.planarity_ = ( feat.singular_values_(1) - feat.singular_values_(2) ) / feat.singular_values_(0);

    // use the least singular vector as normal
    feat.normal_ = (svd.matrixU().col(2));
    if (feat.normal_(2, 0) < 0) { // z 방향 vector는 위를 무조건 향하게 해야 함
        feat.normal_ = -feat.normal_;
    }
    // mean ground seeds value
    feat.mean_ = pc_mean.head<3>();
    // according to normal.T*[x,y,z] = -d
    feat.d_ = -(feat.normal_.transpose() * feat.mean_)(0, 0);
    feat.th_dist_d_ = th_dist_ - feat.d_;
}

template<typename PointT>
inline
void PatchWork<PointT>::extract_initial_seeds_(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds, bool is_h_available) {
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int    cnt = 0;

    int init_idx = 0;
    // Empirically, adaptive seed selection applying to Z1 is fine
    if (is_h_available) {
        static double lowest_h_margin_in_close_zone = (sensor_height_ == 0.0) ? -0.1 : adaptive_seed_selection_margin_ *
                                                                                       sensor_height_;
        if (zone_idx == 0) {
            for (int i = 0; i < p_sorted.points.size(); i++) {
                if (p_sorted.points[i].z < lowest_h_margin_in_close_zone) {
                    ++init_idx;
                } else {
                    break;
                }
            }
        }
    }

    // Calculate the mean height value.
    for (int i          = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double   lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}
template<typename PointT>
inline
double PatchWork<PointT>::consensus_set_based_height_estimation(const Eigen::RowVectorXd& X,
                                                              const Eigen::RowVectorXd& ranges,
                                                              const Eigen::RowVectorXd& weights) {
    // check input parameters
    bool dimension_inconsistent = (X.rows() != ranges.rows()) || (X.cols() != ranges.cols());

    bool only_one_element = (X.rows() == 1) && (X.cols() == 1);
    assert(!dimension_inconsistent);
    assert(!only_one_element); // TODO: admit a trivial solution

    int N = X.cols();
    std::vector<std::pair<double, int>> h;
    for (size_t i= 0 ;i < N ;++i){
        h.push_back(std::make_pair(X(i) - ranges(i), i+1));
        h.push_back(std::make_pair(X(i) + ranges(i), -i-1));
    }

    // ascending order
    std::sort(h.begin(), h.end(), [](std::pair<double, int> a, std::pair<double, int> b) { return a.first < b.first; });

    int nr_centers = 2 * N;
    Eigen::RowVectorXd x_hat = Eigen::MatrixXd::Zero(1, nr_centers);
    Eigen::RowVectorXd x_cost = Eigen::MatrixXd::Zero(1, nr_centers);

    double ranges_inverse_sum = ranges.sum();
    double dot_X_weights = 0;
    double dot_weights_consensus = 0;
    int consensus_set_cardinal = 0;
    double sum_xi = 0;
    double sum_xi_square = 0;

    for (size_t i = 0 ; i < nr_centers ; ++i){

        int idx = int(std::abs(h.at(i).second)) - 1; // Indices starting at 1
        int epsilon = (h.at(i).second > 0) ? 1 : -1;

        consensus_set_cardinal += epsilon;
        dot_weights_consensus += epsilon * weights(idx);
        dot_X_weights += epsilon * weights(idx) * X(idx);
        ranges_inverse_sum -= epsilon * ranges(idx);
        sum_xi += epsilon * X(idx);
        sum_xi_square += epsilon * X(idx) * X(idx);

        x_hat(i) = dot_X_weights / dot_weights_consensus;

        double residual = consensus_set_cardinal * x_hat(i) * x_hat(i) + sum_xi_square  - 2 * sum_xi * x_hat(i);
        x_cost(i) = residual + ranges_inverse_sum;

    }

    size_t min_idx;
    x_cost.minCoeff(&min_idx);
    double estimate_temp = x_hat(min_idx);
    return estimate_temp;
}

template<typename PointT>
inline
void PatchWork<PointT>::estimate_sensor_height(pcl::PointCloud<PointT> cloud_in) {
    // ATAT: All-Terrain Automatic HeighT estimator
    Ring ring_for_ATAT(num_sectors_for_ATAT_);
    for (auto const &pt : cloud_in.points) {
        int    ring_idx, sector_idx;
        double r = xy2radius(pt.x, pt.y);

        float sector_size_for_ATAT = 2 * M_PI / num_sectors_for_ATAT_;

        if ((r <= max_r_for_ATAT_) && (r > min_range_)) {
            double theta = xy2theta(pt.x, pt.y);

            sector_idx = min(static_cast<int>((theta / sector_size_for_ATAT)), num_sectors_for_ATAT_);
            ring_for_ATAT[sector_idx].points.emplace_back(pt);
        }
    }

    // Assign valid measurements and corresponding linearities/planarities
    vector<double> ground_elevations_wrt_the_origin;
    vector<double> linearities;
    vector<double> planarities;
    for (int i = 0; i < num_sectors_for_ATAT_; ++i) {

        if(ring_for_ATAT[i].size() < num_min_pts_ ){ continue; }

        pcl::PointCloud<PointT> dummy_est_ground;
        pcl::PointCloud<PointT> dummy_est_non_ground;
        PCAFeature feat;
        extract_piecewiseground(0, ring_for_ATAT[i], feat, dummy_est_ground, dummy_est_non_ground, false);

        const double ground_z_vec       = abs(feat.normal_(2));
        const double ground_z_elevation = feat.mean_(2);

        // Check whether the vector is sufficiently upright and flat
        if (ground_z_vec > uprightness_thr_ && feat.linearity_ < 0.9) {
            ground_elevations_wrt_the_origin.push_back(ground_z_elevation);
            linearities.push_back(feat.linearity_);
            planarities.push_back(feat.planarity_);
        }
    }

    // Setting for consensus set-based height estimation
    int N = ground_elevations_wrt_the_origin.size();
    Eigen::Matrix<double, 1, Eigen::Dynamic> values = Eigen::MatrixXd::Ones(1, N);
    Eigen::Matrix<double, 1, Eigen::Dynamic> ranges = noise_bound_ * Eigen::MatrixXd::Ones(1, N);
    Eigen::Matrix<double, 1, Eigen::Dynamic> weights = 1.0 / (noise_bound_ * noise_bound_) * Eigen::MatrixXd::Ones(1, N);
    for (int i = 0; i < N; ++i) {
        values(0, i) = ground_elevations_wrt_the_origin[i];
        ranges(0, i) = ranges(0, i) * linearities[i];
        weights(0, i) = weights(0, i) * planarities[i] * planarities[i];
    }

    double estimated_h = consensus_set_based_height_estimation(values, ranges, weights);
    cout << "\033[1;33m[ATAT] The sensor height is auto-calibrated via the ground points in the vicinity of the vehicle\033[0m" << endl;
    cout << "\033[1;33m[ATAT] Elevation of the ground w.r.t. the origin is " << estimated_h << " m\033[0m" << endl;

    // Note that these are opposites
    sensor_height_ = -estimated_h;
}

template<typename PointT>
inline
void PatchWork<PointT>::estimate_ground(
        const pcl::PointCloud<PointT> &cloud_in,
        pcl::PointCloud<PointT> &ground,
        pcl::PointCloud<PointT> &nonground,
        double &time_taken) {

    // Just for visualization
    poly_list_.header.stamp = ros::Time::now();
    if (!poly_list_.polygons.empty()) poly_list_.polygons.clear();
    if (!poly_list_.likelihood.empty()) poly_list_.likelihood.clear();

    if (initialized_ && ATAT_ON_) {
        estimate_sensor_height(cloud_in);
        initialized_ = false;
    }

    static double start, t0, t1, t2, end;

    double                  t_total_ground   = 0.0;
    double                  t_total_estimate = 0.0;
    // 1.Msg to pointcloud
    pcl::PointCloud<PointT> cloud_in_tmp = cloud_in;

    start = ros::Time::now().toSec();

    // Error point removal
    // As there are some error mirror reflection under the ground,
    // Sort point according to height, here uses z-axis in default
    // -2.0 is a rough criteria
    int i = 0;
    while (i < cloud_in_tmp.points.size()) {
        if (cloud_in_tmp.points[i].z < -sensor_height_ - 2.0) {
            std::iter_swap(cloud_in_tmp.points.begin() + i, cloud_in_tmp.points.end() - 1);
            cloud_in_tmp.points.pop_back();
        } else {
            ++i;
        }
    }

    t1 = ros::Time::now().toSec();

    for (int k = 0; k < num_zones_; ++k) {
        flush_patches_in_zone(ConcentricZoneModel_[k], num_sectors_each_zone_[k], num_rings_each_zone_[k]);
    }

    pc2czm(cloud_in_tmp, ConcentricZoneModel_);

    ground.clear();
    nonground.clear();
    reverted_points_by_flatness_.clear();
    rejected_points_by_elevation_.clear();

    int num_patches = patch_indices_.size();

    tbb::parallel_for(0, num_patches, [&](int i) {
        const auto &patch_idx = patch_indices_[i];
        const int zone_idx = patch_idx.zone_idx_;
        const int ring_idx = patch_idx.ring_idx_;
        const int sector_idx = patch_idx.sector_idx_;
        const int concentric_idx = patch_idx.concentric_idx_;

        auto &patch = ConcentricZoneModel_[zone_idx][ring_idx][sector_idx];

        auto &feat = features_[i];
        auto &regionwise_ground = regionwise_grounds_[i];
        auto &regionwise_nonground = regionwise_nongrounds_[i];
        auto &status = statuses_[i];

        if (patch.points.size() > num_min_pts_) {
            double t_tmp0 = ros::Time::now().toSec();
            // 22.05.02 update
            // Region-wise sorting is adopted, which is much faster than global sorting!
            sort(patch.points.begin(), patch.points.end(), point_z_cmp<PointT>);
            extract_piecewiseground(zone_idx, patch, feat, regionwise_ground, regionwise_nonground);
            double t_tmp1 = ros::Time::now().toSec();

            const double ground_z_vec       = abs(feat.normal_(2));
            const double ground_z_elevation = feat.mean_(2);
            const double surface_variable   =
                                 feat.singular_values_.minCoeff() /
                                 (feat.singular_values_(0) + feat.singular_values_(1) + feat.singular_values_(2));

            status = determine_ground_likelihood_estimation_status(concentric_idx, ground_z_vec,
                                                                         ground_z_elevation, surface_variable);
        } else {
            // Why? Because it is better to reject noise points
            // That is, these noise points sometimes lead to mis-recognition or wrong clustering
            // Thus, in practice, just rejecting them is better than using them
            // But note that this may degrade quantitative ground segmentation performance
            regionwise_ground = patch;
            regionwise_nonground.clear();
            status = FEW_POINTS;
        }
    });

    std::for_each(indices_.begin(), indices_.end(), [&](const int &i) {
        const auto &patch_idx     = patch_indices_[i];
        const int  zone_idx       = patch_idx.zone_idx_;
        const int  ring_idx       = patch_idx.ring_idx_;
        const int  sector_idx     = patch_idx.sector_idx_;
        const int  concentric_idx = patch_idx.concentric_idx_;

        const auto &patch                = ConcentricZoneModel_[zone_idx][ring_idx][sector_idx];
        const auto &feat                 = features_[i];
        const auto &regionwise_ground    = regionwise_grounds_[i];
        const auto &regionwise_nonground = regionwise_nongrounds_[i];
        const auto status                = statuses_[i];

        if (visualize_ && (status != FEW_POINTS && status != NOT_ASSIGNED)) {
            auto polygons = set_polygons(zone_idx, ring_idx, sector_idx, 3);
            polygons.header = poly_list_.header;
            poly_list_.polygons.emplace_back(polygons);
            poly_list_.likelihood.emplace_back(COLOR_MAP[status]);
        }

        double t_tmp2 = ros::Time::now().toSec();
        if (status == FEW_POINTS) {
            ground += regionwise_ground;
            nonground += regionwise_nonground;
        } else if (status == TOO_TILTED) {
            // All points are rejected
            nonground += regionwise_ground;
            nonground += regionwise_nonground;
        } else if (status == GLOBALLY_TOO_HIGH_ELEVATION) {
            cout << "\033[1;33m[Global elevation] " << feat.mean_(2) << " > " << global_elevation_thr_
            << "\033[0m\n";
            nonground += regionwise_ground;
            nonground += regionwise_nonground;
        } else if (status == TOO_HIGH_ELEVATION) {
            if (verbose_) {
                std::cout << "\033[1;34m[Elevation] Rejection operated. Check "
                << concentric_idx
                << "th param. of elevation_thr_: " << -sensor_height_ + elevation_thr_[concentric_idx]
                << " < " << feat.mean_(2) << "\033[0m\n";
                rejected_points_by_elevation_ += regionwise_ground;
            }
            nonground += regionwise_ground;
            nonground += regionwise_nonground;
        } else if (status == FLAT_ENOUGH) {
            if (verbose_) {
                std::cout << "\033[1;36m[Flatness] Recovery operated. Check "
                          << concentric_idx
                          << "th param. flatness_thr_: " << flatness_thr_[concentric_idx]
                          << " > "
                          << feat.singular_values_.minCoeff() /
                                 (feat.singular_values_(0) + feat.singular_values_(1) + feat.singular_values_(2)) << "\033[0m\n";
                reverted_points_by_flatness_ += regionwise_ground;
            }
            ground += regionwise_ground;
            nonground += regionwise_nonground;
        } else if (status == UPRIGHT_ENOUGH) {
            ground += regionwise_ground;
            nonground += regionwise_nonground;
        } else {
            std::invalid_argument("Something wrong in `determine_ground_likelihood_estimation_status()` fn!");
        }
        double t_tmp3 = ros::Time::now().toSec();
        t_total_estimate += t_tmp3 - t_tmp2;
    });

    end         = ros::Time::now().toSec();
    time_taken  = end - start;
//    ofstream time_txt("/home/shapelim/patchwork_time_anal.txt", std::ios::app);
//    time_txt<<t0 - start<<" "<<t1 - t0 <<" "<<t2-t1<<" "<<t_total_ground<< " "<<t_total_estimate<<"\n";
//    time_txt.close();

    if (verbose_) {
        sensor_msgs::PointCloud2 cloud_ROS;
        pcl::toROSMsg(reverted_points_by_flatness_, cloud_ROS);
        cloud_ROS.header.stamp    = ros::Time::now();
        cloud_ROS.header.frame_id = "/map";
        RevertedCloudPub.publish(cloud_ROS);
        pcl::toROSMsg(rejected_points_by_elevation_, cloud_ROS);
        cloud_ROS.header.stamp    = ros::Time::now();
        cloud_ROS.header.frame_id = "/map";
        RejectedCloudPub.publish(cloud_ROS);
    }
    PlanePub.publish(poly_list_);
}

template<typename PointT>
inline
double PatchWork<PointT>::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    /*
      if (y >= 0) {
          return atan2(y, x); // 1, 2 quadrant
      } else {
          return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
      }
    */
    auto atan_value = atan2(y,x); // EDITED!
    return atan_value > 0 ? atan_value : atan_value + 2*M_PI; // EDITED!
}

template<typename PointT>
inline
double PatchWork<PointT>::xy2radius(const double &x, const double &y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

template<typename PointT>
inline
void PatchWork<PointT>::pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm) {

    std::for_each(src.points.begin(), src.points.end(), [&](const auto &pt) {
        int    ring_idx, sector_idx;
        double r = xy2radius(pt.x, pt.y);
        if ((r <= max_range_) && (r > min_range_)) {
            double theta = xy2theta(pt.x, pt.y);

            if (r < min_range_z2_) { // In First rings
                ring_idx =
                    min(static_cast<int>(((r - min_range_) / ring_sizes_[0])), num_rings_each_zone_[0] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[0])), num_sectors_each_zone_[0] - 1);
                czm[0][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z3_) {
                ring_idx =
                    min(static_cast<int>(((r - min_range_z2_) / ring_sizes_[1])), num_rings_each_zone_[1] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[1])), num_sectors_each_zone_[1] - 1);
                czm[1][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z4_) {
                ring_idx =
                    min(static_cast<int>(((r - min_range_z3_) / ring_sizes_[2])), num_rings_each_zone_[2] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[2])), num_sectors_each_zone_[2] - 1);
                czm[2][ring_idx][sector_idx].points.emplace_back(pt);
            } else { // Far!
                ring_idx =
                    min(static_cast<int>(((r - min_range_z4_) / ring_sizes_[3])), num_rings_each_zone_[3] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[3])), num_sectors_each_zone_[3] - 1);
                czm[3][ring_idx][sector_idx].points.emplace_back(pt);
            }
        }
//    }
    });
}

// For adaptive
template<typename PointT>
inline
void PatchWork<PointT>::extract_piecewiseground(
        const int zone_idx, const pcl::PointCloud<PointT> &src,
        PCAFeature &feat,
        pcl::PointCloud<PointT> &regionwise_ground,
        pcl::PointCloud<PointT> &regionwise_nonground,
        bool is_h_available) {
    // 0. Initialization
    int N = src.points.size();
    pcl::PointCloud<PointT> ground_tmp;
    ground_tmp.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
    if (!regionwise_ground.empty()) regionwise_ground.clear();
    if (!regionwise_nonground.empty()) regionwise_nonground.clear();

    // 1. set seeds!
    extract_initial_seeds_(zone_idx, src, ground_tmp, is_h_available);

    // 2. Extract ground
    for (int i = 0; i < num_iter_; i++) {
        estimate_plane_(ground_tmp, feat);
        ground_tmp.clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(N, 3);
        int             j      = 0;
        for (auto       &p:src.points) {
            points.row(j++) = p.getVector3fMap();
        }
        // ground plane model
        Eigen::VectorXf result = points * feat.normal_;
        // threshold filter
        for (int        r      = 0; r < N; r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < feat.th_dist_d_) {
                    ground_tmp.points.emplace_back(src[r]);
                }
            } else { // Final stage
                if (result[r] < feat.th_dist_d_) {
                    regionwise_ground.points.emplace_back(src[r]);
                } else {
                    regionwise_nonground.points.emplace_back(src[r]);
                }
            }
        }
    }
}

template<typename PointT>
inline
geometry_msgs::PolygonStamped PatchWork<PointT>::set_polygons(int zone_idx, int r_idx, int theta_idx, int num_split) {
    geometry_msgs::PolygonStamped polygons;
    // Set point of polygon. Start from RL and ccw
    geometry_msgs::Point32        point;

    // RL
    double zone_min_range = min_ranges_[zone_idx];
    double r_len          = r_idx * ring_sizes_[zone_idx] + zone_min_range;
    double angle          = theta_idx * sector_sizes_[zone_idx];

    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);
    // RU
    r_len = r_len + ring_sizes_[zone_idx];
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    // RU -> LU
    for (int idx = 1; idx <= num_split; ++idx) {
        angle = angle + sector_sizes_[zone_idx] / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }

    r_len = r_len - ring_sizes_[zone_idx];
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    point.z = MARKER_Z_VALUE;
    polygons.polygon.points.push_back(point);

    for (int idx = 1; idx < num_split; ++idx) {
        angle = angle - sector_sizes_[zone_idx] / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        point.z = MARKER_Z_VALUE;
        polygons.polygon.points.push_back(point);
    }
    return polygons;
}

template<typename PointT>
inline
int PatchWork<PointT>::determine_ground_likelihood_estimation_status(
        const int concentric_idx,
        const double z_vec,
        const double z_elevation,
        const double surface_variable) {
    if (z_vec < uprightness_thr_) {
        return TOO_TILTED;
    } else { //orthogonal
        if (concentric_idx < num_rings_of_interest_) {
            if (z_elevation > -sensor_height_ + elevation_thr_[concentric_idx]) {
                if (flatness_thr_[concentric_idx] > surface_variable) {
                    return FLAT_ENOUGH;
                } else {
                    return TOO_HIGH_ELEVATION;
                }
            } else {
                return UPRIGHT_ENOUGH;
            }
        } else {
            if (using_global_thr_ && (z_elevation > global_elevation_thr_)) {
                return GLOBALLY_TOO_HIGH_ELEVATION;
            } else {
                return UPRIGHT_ENOUGH;
            }
        }
    }
}

template<typename PointT>
inline
void PatchWork<PointT>::check_input_parameters_are_correct() {
    string SET_SAME_SIZES_OF_PARAMETERS = "Some parameters are wrong! the size of parameters should be same";

    int n_z = num_zones_;
    int n_r = num_rings_each_zone_.size();
    int n_s = num_sectors_each_zone_.size();
    int n_m = min_ranges_.size();

    if ((n_z != n_r) || (n_z != n_s) || (n_z != n_m)) {
        throw invalid_argument(SET_SAME_SIZES_OF_PARAMETERS);
    }

    if ((n_r != n_s) || (n_r != n_m) || (n_s != n_m)) {
        throw invalid_argument(SET_SAME_SIZES_OF_PARAMETERS);
    }

    if (min_range_ != min_ranges_[0]) {
        throw invalid_argument("Setting min. ranges are weired! The first term should be eqaul to min_range_");
    }

    if (elevation_thr_.size() != flatness_thr_.size()) {
        throw invalid_argument("Some parameters are wrong! Check the elevation/flatness_thresholds");
    }
}

template<typename PointT>
inline
void PatchWork<PointT>::cout_params() {
    cout << (boost::format("Num. sectors: %d, %d, %d, %d") % num_sectors_each_zone_[0] % num_sectors_each_zone_[1] %
             num_sectors_each_zone_[2] %
             num_sectors_each_zone_[3]).str() << endl;
    cout << (boost::format("Num. rings: %01d, %01d, %01d, %01d") % num_rings_each_zone_[0] %
             num_rings_each_zone_[1] %
             num_rings_each_zone_[2] %
             num_rings_each_zone_[3]).str() << endl;
    cout << (boost::format("elevation_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % elevation_thr_[0] % elevation_thr_[1] %
             elevation_thr_[2] %
             elevation_thr_[3]).str() << endl;
    cout << (boost::format("flatness_thr_: %0.4f, %0.4f, %0.4f, %0.4f ") % flatness_thr_[0] % flatness_thr_[1] %
             flatness_thr_[2] %
             flatness_thr_[3]).str() << endl;
}

#endif
