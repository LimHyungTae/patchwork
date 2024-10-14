//
// Created by shapelim on 22. 10. 23.
//

#ifndef PATCHWORK_SENSOR_CONFIG_HPP
#define PATCHWORK_SENSOR_CONFIG_HPP

#include <iostream>
#include <string>
#include <vector>

using namespace std;

struct SensorConfig {
  vector<vector<int>> num_laser_channels_per_zone_;
  vector<int> num_rings_for_each_zone_;
  vector<int> num_sectors_for_each_zone_;
  // Please refer to setPriorIdxesOfPatches

  int num_channels_;
  float lower_fov_boundary_;
  float vertical_angular_resolution_;
  int horizontal_resolution_;

  /**
   * Note: Only use-defined parameters are `num_laser_channels_per_zone_` and
   * `num_sectors_for_each_zone_`
   */
  explicit SensorConfig(const string &sensor_name) {
    cout << "\033[1;32mTarget Sensor: " << sensor_name << "\033[0m" << endl;
    if (sensor_name == "VLP-16") {  // For Kimera-Multi dataset
      // https://www.amtechs.co.jp/product/VLP-16-Puck.pdf
      num_laser_channels_per_zone_ = {{2, 1}, {1, 1}, {1, 1}, {1}};
      num_sectors_for_each_zone_ = {16, 32, 56, 32};
      lower_fov_boundary_ = -15;
      vertical_angular_resolution_ = 2.0;
      // https://github.com/TixiaoShan/LIO-SAM/blob/master/config/params.yaml
      horizontal_resolution_ = 1800;
      num_channels_ = 16;
    } else if (sensor_name == "HDL-32E") {
      // https://velodynelidar.com/wp-content/uploads/2019/12/97-0038-Rev-N-97-0038-DATASHEETWEBHDL32E_Web.pdf
      num_laser_channels_per_zone_ = {{10, 5}, {3, 2, 1, 1}, {1, 1, 1}, {1, 1, 1}};
      num_sectors_for_each_zone_ = {16, 32, 56, 32};
      lower_fov_boundary_ = -30.67;
      vertical_angular_resolution_ = 1.33;
      horizontal_resolution_ = 1080;
      num_channels_ = 32;
    } else if (sensor_name == "HDL-64E") {
      num_laser_channels_per_zone_ = {{24, 12}, {4, 3, 2, 2}, {2, 2, 2, 1}, {1, 1, 1, 1, 1}};
      num_sectors_for_each_zone_ = {16, 32, 56, 32};
      lower_fov_boundary_ = -24.8;
      vertical_angular_resolution_ = 0.4;
      // https://github.com/TixiaoShan/LIO-SAM/blob/master/config/params.yaml
      horizontal_resolution_ = 1800;
      num_channels_ = 64;
    } else if (sensor_name == "OS1-16") {  // For NTU_VIRAL dataset
      // https://www.dataspeedinc.com/app/uploads/2019/10/Ouster-OS1-Datasheet.pdf
      // But not work in NTU-VIRAL dataset, haha
      num_laser_channels_per_zone_ = {{2, 1}, {1, 1}, {1}, {1}};
      num_sectors_for_each_zone_ = {16, 32, 56, 32};
      lower_fov_boundary_ = -16.6;
      vertical_angular_resolution_ = 2.075;
      // There are three main options: 512, 1024, 2048
      // https://github.com/TixiaoShan/LIO-SAM/blob/master/config/params.yaml
      horizontal_resolution_ = 1024;
      num_channels_ = 16;
      // It's not thorough, yet once the cloud points are sufficiently dense,
      // then it's fine
    } else if (sensor_name == "OS1-64" || sensor_name == "OS1-128") {
      num_laser_channels_per_zone_ = {{12, 6}, {3, 2, 1, 1}, {1, 1, 1}, {1, 1, 1}};
      num_sectors_for_each_zone_ = {16, 32, 56, 32};
      lower_fov_boundary_ = -22.5;
      vertical_angular_resolution_ = 0.7;
      horizontal_resolution_ = 1024;
      num_channels_ = 64;
    } else {
      throw invalid_argument("Sensor name is wrong! Please check the parameter");
    }
  }
  SensorConfig() = default;
};

#endif  // PATCHWORK_SENSOR_CONFIG_HPP
