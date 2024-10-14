//
// Created by shapelim on 22. 10. 23.
//

#ifndef PATCHWORK_ZONE_MODEL_HPP
#define PATCHWORK_ZONE_MODEL_HPP

#include <iostream>
#include <vector>

#include "patchwork/sensor_configs.hpp"
#include "sensor_configs.hpp"

#define INVALID_RING_IDX -1
#define OVERFLOWED_IDX -2

class ZoneModel {
 public:
  ZoneModel() {}

  ~ZoneModel() {}

  SensorConfig sensor_config_;

  /*
   * Common functions
   */
  inline size_t size() const;
  // Important! 'virtual' is necessary to be declared in the base class
  // + the functions must be declared, i.e. {} is needed
};

class ConcentricZoneModel : public ZoneModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ConcentricZoneModel() {}

  ConcentricZoneModel(const std::string &sensor_model,
                      const double sensor_height,
                      const float min_range,
                      const float max_range) {
    sensor_config_ = SensorConfig(sensor_model);
    num_zones_ = sensor_config_.num_laser_channels_per_zone_.size();
    max_ring_index_in_first_zone = sensor_config_.num_laser_channels_per_zone_[0].size();

    sensor_height_ = sensor_height;
    min_range_ = min_range;
    max_range_ = max_range;

    sqr_min_range_ = min_range * min_range;
    sqr_max_range_ = max_range * max_range;

    set_concentric_zone_model();
  }
  bool is_range_boundary_set_;
  size_t max_ring_index_in_first_zone;
  size_t num_zones_;
  size_t num_total_rings_;
  double sensor_height_;
  float min_range_;
  float max_range_;
  float sqr_min_range_;
  float sqr_max_range_;

  vector<int> num_sectors_per_ring_;
  // sqr: For reducing computational complexity
  vector<float> sqr_boundary_ranges_;
  // For visualization
  vector<float> boundary_ranges_;
  vector<float> boundary_ratios_;

  ~ConcentricZoneModel() {}

  inline void set_concentric_zone_model() {
    set_num_sectors_for_each_ring();
    // Seting ring boundaries to consider # of laser rings
    float smallest_incidence_angle = 90.0 + sensor_config_.lower_fov_boundary_;
    // For defensive programming
    if (tan(DEG2RAD(smallest_incidence_angle)) * sensor_height_ < min_range_) {
      cout << tan(DEG2RAD(smallest_incidence_angle)) * sensor_height_ << " vs " << min_range_
           << endl;
      throw invalid_argument(
          "\033[1;31m[CZM] The parameter `min_r` is wrong. "
          "Check your sensor height or min. range\033[0m");
    }
    sanity_check();
    set_sqr_boundary_ranges(sensor_height_, smallest_incidence_angle);
  }

  inline void sanity_check() {
    string SET_SAME_SIZES_OF_PARAMETERS =
        "Some parameters are wrong! the size of parameters should be same";

    int n_z = num_zones_;
    int n_r = sensor_config_.num_laser_channels_per_zone_.size();
    int n_s = sensor_config_.num_sectors_for_each_zone_.size();

    if ((n_z != n_r) || (n_z != n_s) || (n_r != n_s)) {
      throw invalid_argument(SET_SAME_SIZES_OF_PARAMETERS);
    }
  }

  bool is_boundary_set() { return is_range_boundary_set_; }
  inline void set_num_sectors_for_each_ring() {
    num_sectors_per_ring_.clear();
    int count = 0;
    for (const auto &channel_set : sensor_config_.num_laser_channels_per_zone_) {
      for (int j = 0; j < channel_set.size(); ++j) {
        num_sectors_per_ring_.push_back(sensor_config_.num_sectors_for_each_zone_[count]);
      }
      count++;
    }
    num_total_rings_ = num_sectors_per_ring_.size();
  }

  void set_sqr_boundary_ranges(const double sensor_height, const float smallest_incidence_angle) {
    /***
     * Why squared value?
     * Because `sqrt` operation requires more computational cost
     */
    // sqr (square): For speed-up
    // original : For viz polygon
    is_range_boundary_set_ = true;

    boundary_ranges_.clear();
    boundary_ranges_.push_back(min_range_);
    sqr_boundary_ranges_.clear();
    sqr_boundary_ranges_.push_back(pow(min_range_, 2));
    float incidence_angle = smallest_incidence_angle;
    std::cout << "min_range: " << min_range_ << std::endl;

    float incidence_angle_prev = incidence_angle;
    int count = 0;
    for (int i = 0; i < sensor_config_.num_laser_channels_per_zone_.size(); ++i) {
      vector<int> num_channels_per_ring = sensor_config_.num_laser_channels_per_zone_[i];
      for (int j = 0; j < num_channels_per_ring.size(); ++j) {
        incidence_angle += static_cast<float>(num_channels_per_ring[j]) *
                           sensor_config_.vertical_angular_resolution_;
        float incidence_angle_w_margin =
            incidence_angle +
            0.5 * sensor_config_.vertical_angular_resolution_;  // For safety margin
        cout << "\033[1;32m" << incidence_angle_w_margin << "\033[0m" << endl;

        // Check whether the angle is over the 90 degrees
        float boundary_range;
        if (incidence_angle_w_margin >= 90) {
          cout << "\033[1;33mIncidence angle is over the 90 deg!!\033[0m" << endl;
          cout << "\033[1;33mBins are evenly divided!\033[0m" << endl;
          static float denominator = static_cast<float>(num_total_rings_ - count + 1);
          static float left_b = boundary_ranges_.back();
          static float right_b = max_range_;
          float k = num_total_rings_ - count;
          boundary_range = left_b * (denominator - k) / denominator + max_range_ * k / denominator;
        } else {
          boundary_range = tan(DEG2RAD(incidence_angle_w_margin)) * sensor_height;
          incidence_angle_prev = incidence_angle_w_margin;
        }
        boundary_ranges_.push_back(boundary_range);
        sqr_boundary_ranges_.push_back(pow(boundary_range, 2));
        cout << boundary_range << " m " << endl;
        ++count;
      }
    }
    float total_diff = boundary_ranges_.back() - min_range_;
    for (const auto boundary_range : boundary_ranges_) {
      float ratio = (boundary_range - min_range_) / total_diff;
      boundary_ratios_.push_back(ratio);
    }

    // This part is important! Without this line, a segmentation fault occurs.
    // If you want to enlarge the max range, please modify
    // `num_laser_channels_per_zone_` in `sensor_configs.hpp`.
    //        if (boundary_ranges_.back() < max_range_) {
    //            boundary_ranges_.push_back(max_range_);
    //            sqr_boundary_ranges_.push_back(max_range_ * max_range_);
    //            // Just copy the last value
    //            num_sectors_per_ring_.push_back(num_sectors_per_ring_.back());
    //        }

    if (boundary_ranges_.back() < max_range_) {
      std::cout << "\033[1;33m"
                << "Max range is shrinked: ";
      std::cout << max_range_ << " -> " << boundary_ranges_.back() << "\033[0m" << std::endl;
      sqr_max_range_ = boundary_ranges_.back() * boundary_ranges_.back();
      max_range_ = boundary_ranges_.back();
    }
  }

  inline void cout_params() {
    const auto &num_sectors_each_zone_ = sensor_config_.num_sectors_for_each_zone_;
    const auto &num_rings_each_zone_ = sensor_config_.num_rings_for_each_zone_;

    std::cout << "Boundary range: ";
    for (const auto &range : boundary_ranges_) {
      std::cout << range << " ";
    }
    std::cout << std::endl;
  }

  inline float xy2sqr_r(const float &x, const float &y) { return x * x + y * y; }

  inline float xy2theta(const float &x, const float &y) {  // 0 ~ 2 * PI
    /*
      if (y >= 0) {
          return atan2(y, x); // 1, 2 quadrant
      } else {
          return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
      }
    */
    auto atan_value = atan2(y, x);                               // EDITED!
    return atan_value > 0 ? atan_value : atan_value + 2 * M_PI;  // EDITED!
  }

  inline int get_ring_idx(const float &x, const float &y) {
    static auto &b = sqr_boundary_ranges_;
    float sqr_r = xy2sqr_r(x, y);
    // Exception for UAVs such as NTU VIRAL dataset
    if (sqr_r < b[0]) {
      //            std::cout << "\033[1;33mInvalid ring idx has come:";
      //            std::cout << "(" << sqrt(sqr_r) << " < " << sqrt(b[0]) <<
      //            ")\033[0m" << std::endl;
      return INVALID_RING_IDX;
    }
    if (sqr_r > sqr_max_range_) {
      return OVERFLOWED_IDX;
    }

    int bound_idx = lower_bound(b.begin(), b.end(), sqr_r) - b.begin();
    // bound_idx: idx whose value is larger than sqr_r
    // Thus, b[bound_idx - 1] < sqr_r < b[bound_idx]
    // And note that num_rings + 1 = b.size(); thus, minus 1 is needed
    return bound_idx - 1;
  }

  inline int get_sector_idx(const float &x, const float &y, const int ring_idx) {
    float theta = xy2theta(x, y);
    int num_sectors = num_sectors_per_ring_[ring_idx];
    float sector_size = 2.0 * M_PI / static_cast<float>(num_sectors);

    // min: for defensive programming
    return min(static_cast<int>(theta / sector_size), num_sectors - 1);
  }

  inline std::pair<int, int> get_ring_sector_idx(const float &x, const float &y) {
    int ring_idx = get_ring_idx(x, y);
    if (ring_idx == INVALID_RING_IDX) {
      return std::make_pair(INVALID_RING_IDX, INVALID_RING_IDX);
    }
    if (ring_idx == OVERFLOWED_IDX) {
      return std::make_pair(OVERFLOWED_IDX, OVERFLOWED_IDX);
    }

    int sector_idx = get_sector_idx(x, y, ring_idx);
    return std::make_pair(ring_idx, sector_idx);
  }
};

#endif  // PATCHWORK_ZONE_MODEL_HPP
