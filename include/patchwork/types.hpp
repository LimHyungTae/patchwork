// Shared data types and visualization constants for Patchwork.
//
// Extracted from `patchwork.hpp` so the algorithm class and the POD
// types it operates on can be read in isolation.
//
// These declarations were previously inlined at the top of
// `patchwork.hpp`; consumers should keep including `patchwork.hpp` and
// will pick this up transitively.
#ifndef INCLUDE_PATCHWORK_TYPES_HPP_
#define INCLUDE_PATCHWORK_TYPES_HPP_

#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000
#define MARKER_Z_VALUE -2.2

// Visualization colors used for RViz patch-likelihood markers.
inline constexpr float COLOR_CYAN                        = 0.55f;
inline constexpr float COLOR_GREEN                       = 0.2f;
inline constexpr float COLOR_BLUE                        = 0.0f;
inline constexpr float COLOR_RED                         = 1.0f;
inline constexpr float COLOR_GLOBALLY_TOO_HIGH_ELEVATION = 0.8f;

// Patch ground-likelihood-estimation status codes.
inline constexpr int NOT_ASSIGNED                = -2;
inline constexpr int FEW_POINTS                  = -1;
inline constexpr int UPRIGHT_ENOUGH              = 0;  // cyan
inline constexpr int FLAT_ENOUGH                 = 1;  // green
inline constexpr int TOO_HIGH_ELEVATION          = 2;  // blue
inline constexpr int TOO_TILTED                  = 3;  // red
inline constexpr int GLOBALLY_TOO_HIGH_ELEVATION = 4;

inline const std::vector<float> COLOR_MAP = {
    COLOR_CYAN,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_RED,
    COLOR_GLOBALLY_TOO_HIGH_ELEVATION,
};

/// PCA result for a fitted plane plus a couple of derived scalars used by
/// the GLE classifier.
struct PCAFeature {
  Eigen::Vector3f principal_;
  Eigen::Vector3f normal_;
  Eigen::Vector3f singular_values_;
  Eigen::Vector3f mean_;
  float d_;
  float th_dist_d_;
  float linearity_;
  float planarity_;
};

/// One concentric-zone patch (one ring × one sector of one zone).
template <typename PointT>
struct Patch {
  bool is_close_to_origin_ = false;  // closer rings can use stricter thresholds
  int ring_idx_            = NOT_ASSIGNED;
  int sector_idx_          = NOT_ASSIGNED;

  int status_ = NOT_ASSIGNED;

  PCAFeature feature_;

  pcl::PointCloud<PointT> cloud_;
  pcl::PointCloud<PointT> ground_;
  pcl::PointCloud<PointT> non_ground_;

  void clear() {
    if (!cloud_.empty()) cloud_.clear();
    if (!ground_.empty()) ground_.clear();
    if (!non_ground_.empty()) non_ground_.clear();
  }
};

/// Ascending-z comparator (works for any `PointT` with a `.z` field).
template <typename PointT>
inline bool point_z_cmp(PointT a, PointT b) {
  return a.z < b.z;
}

#endif  // INCLUDE_PATCHWORK_TYPES_HPP_
