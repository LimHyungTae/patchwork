//
// Created by shapelim on 6/23/21.
//
#include <filesystem>
#include <string>
#include <vector>

#include "patchwork/utils.hpp"

#ifndef PATCHWORK_PCD_LOADER_HPP
#define PATCHWORK_PCD_LOADER_HPP

namespace fs = std::filesystem;

class KittiLoader {
 public:
  explicit KittiLoader(const std::string &abs_path) {
    pc_path_ = abs_path + "/velodyne";
    label_path_ = abs_path + "/labels";

    for (num_frames_ = 0;; num_frames_++) {
      std::stringstream ss;
      ss << pc_path_ << "/" << std::setfill('0') << std::setw(6) << num_frames_ << ".bin";
      std::string filename = ss.str();
      if (!fs::exists(filename)) {
        break;
      }
    }
    int num_labels;
    for (num_labels = 0;; num_labels++) {
      std::stringstream ss;
      ss << label_path_ << "/" << std::setfill('0') << std::setw(6) << num_labels << ".label";
      std::string filename = ss.str();
      if (!fs::exists(filename)) {
        break;
      }
    }

    if (num_frames_ == 0) {
      std::cerr << "\033[1;31mError: No files in " << pc_path_ << "\033[0m" << std::endl;
    }
    if (num_frames_ != num_labels) {
      std::cerr << "\033[1;31mError: The # of point clouds and # of labels are "
                   "not same\033[0m"
                << std::endl;
    }
    std::cout << "Total " << num_frames_ << " files are loaded" << std::endl;
  }

  ~KittiLoader() {}

  size_t size() const { return num_frames_; }

  template <typename T>
  void get_cloud(size_t idx, pcl::PointCloud<T> &cloud) const {
    std::stringstream ss;
    ss << pc_path_ << "/" << std::setfill('0') << std::setw(6) << idx << ".bin";
    std::string filename = ss.str();
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
      throw invalid_argument("Could not open the .bin file!");
    }
    std::vector<float> buffer(3000000);
    size_t num_points =
        fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    cloud.points.resize(num_points);
    if (std::is_same<T, pcl::PointXYZ>::value) {
      for (int i = 0; i < num_points; i++) {
        auto &pt = cloud.at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
      }
    } else if (std::is_same<T, pcl::PointXYZI>::value) {
      for (int i = 0; i < num_points; i++) {
        auto &pt = cloud.at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        pt.intensity = buffer[i * 4 + 3];
      }
    } else if (std::is_same<T, PointXYZILID>::value) {
      std::stringstream ss;
      ss << label_path_ << "/" << std::setfill('0') << std::setw(6) << idx << ".label";
      std::string label_name = ss.str();
      //
      //            std::cout << label_name << std::endl;
      std::ifstream label_input(label_name, std::ios::binary);
      if (!label_input.is_open()) {
        throw invalid_argument("Could not open the label!");
      }
      label_input.seekg(0, std::ios::beg);
      std::vector<uint32_t> labels(num_points);
      label_input.read(reinterpret_cast<char *>(&labels[0]), num_points * sizeof(uint32_t));

      for (size_t i = 0; i < num_points; i++) {
        auto &pt = cloud.at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        pt.intensity = buffer[i * 4 + 3];
        pt.label = labels[i] & 0xFFFF;
        pt.id = labels[i] >> 16;
      }
    }
  }

 private:
  int num_frames_;
  std::string label_path_;
  std::string pc_path_;
};

#endif  // PATCHWORK_PCD_LOADER_HPP
