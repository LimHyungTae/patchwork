<div align="center">
    <h1>Patchwork</h1>
    <a href="https://github.com/LimHyungTae/patchwork"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/LimHyungTae/patchwork/releases/tag/v0.2"><img src="https://img.shields.io/badge/ROS-Noetic (Click here)-blue" /></a>
    <a href="https://github.com/LimHyungTae/patchwork"><img src="https://img.shields.io/badge/ROS2-Jazy-orange" /></a>
    <a href="https://github.com/LimHyungTae/patchwork"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://ieeexplore.ieee.org/document/9466396"><img src="https://img.shields.io/badge/DOI-10.1109/LRA.2021.3093009-004088.svg"/>
    <br />
    <br />
    <a href=https://youtu.be/rclqeDi4gow>Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/LimHyungTae/patchwork?tab=readme-ov-file#requirements">Install by ROS</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://arxiv.org/abs/2108.05560>Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://github.com/LimHyungTae/patchwork/wiki>Project Wiki (for beginners)</a>
  <br />
  <br />
  <div style="display: flex; justify-content: space-between; width: 100%;">
      <img src="img/demo_kitti00_v2.gif" alt="animated" style="width: 90%;" />
      <img src="img/demo_terrain_v3.gif" alt="animated" style="width: 90%;" />
  </div>
  <br />
</div>

---

**IMPORTANT**: (Aug. 18th, 2024) I employ TBB, so its FPS is increased from **50 Hz** to **100 Hz**!
If you want to use the paper version of Patchwork for SOTA comparison purpose, Please use this [ground seg. benchmark code](https://github.com/url-kaist/Ground-Segmentation-Benchmark).


Patchwork                  |  Concept of our method (CZM & GLE)
:-------------------------:|:-------------------------:
![](img/patchwork_concept_resized.jpg) |  ![](img/patchwork.gif)

It's an overall updated version of **R-GPF of ERASOR** [**[Code](https://github.com/LimHyungTae/ERASOR)**] [**[Paper](https://arxiv.org/abs/2103.04316)**].

---


## :open_file_folder: Contents
0. [Test Env.](#Test-Env.)
0. [Requirements](#requirements)
0. [How to Run Patchwork](#How-to-Run-Patchwork)
0. [Citation](#citation)

### Test Env.

The code is tested successfully at
* Linux 24.04 LTS
* ROS2 Jazzy

ROS Noetic version can be found [here](https://github.com/LimHyungTae/patchwork/releases/tag/v0.2)

## :package: Prerequisite Installation

```bash
mkdir -p ~/colcon/src
cd ~/colcon/src
git clone https://github.com/LimHyungTae/patchwork.git
cd ..
colcon build --packages-up-to patchwork --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## :gear: How to Run Patchwork

#### :chart_with_upwards_trend: Offline KITTI dataset

1. Download [SemanticKITTI](http://www.semantic-kitti.org/dataset.html#download) Odometry dataset (We also need labels since we also open the evaluation code! :)

2. The `dataset_path` should consist of `velodyne` folder and `labels` folder as follows:

```
data_path (e.g. 00, 01, ..., or 10)
_____velodyne
     |___000000.bin
     |___000001.bin
     |___000002.bin
     |...
_____labels
     |___000000.label
     |___000001.label
     |___000002.label
     |...
_____...

```

3. Run launch file

```
ros2 launch patchwork evaluate.launch.yaml evaluate_semantickitti:=true dataset_path:=<YOUR_TARGET_SEQUENCE_DIR>"
e.g.,
ros2 launch patchwork evaluate.launch.yaml evaluate_semantickitti:=true dataset_path:="/home/hyungtae_lim/semkitti/dataset/sequences/04"
```


#### :runner: Online Ground Segmentation

```
ros2 launch patchwork run_patchwork.launch.yaml scan_topic:=<YOUR_TOPIC_NAME> sensor_type:=<YOUR_SENSOR_TYPE>
e.g.,
ros2 launch patchwork run_patchwork.launch.yaml scan_topic:="/acl_jackal2/lidar_points" sensor_type:="velodyne16"
```


For better understanding of the parameters of Patchwork, please read [our wiki, 4. IMPORTANT: Setting Parameters of Patchwork in Your Own Env.](https://github.com/LimHyungTae/patchwork/wiki/4.-IMPORTANT:-Setting-Parameters-of-Patchwork-in-Your-Own-Env.).

---------

## Citation

If you use our code or method in your work, please consider citing the following:

	@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2021}
    }


---

## Updates

#### NEWS (22.12.24)
- Merry christmas eve XD! `include/label_generator` is added to make the `.label` file, following the SemanticKITTI format.
- The `.label` files can be directly used in [3DUIS benchmark](https://github.com/PRBonn/3DUIS)
- Thank [Lucas Nunes](https://scholar.google.com/citations?user=PCxhsf4AAAAJ&hl=en&oi=ao) and [Xieyuanli Chen](https://scholar.google.com/citations?user=DvrngV4AAAAJ&hl=en&oi=sra) for providing code snippets to save a `.label` file.

#### NEWS (22.07.25)
- Pybinding + more advanced version is now available on [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) as a preprocessing step for deep learning users (i.e., python users can also use our robust ground segmentation)!

#### NEWS (22.07.13)
- For increasing convenience of use, the examples and codes are extensively revised by reflecting [issue #12](https://github.com/LimHyungTae/patchwork/issues/12).

#### NEWS (22.05.22)
- The meaning of `elevation_thresholds` is changed to increase the usability. The meaning is explained in [wiki](https://github.com/LimHyungTae/patchwork/wiki/4.-IMPORTANT:-Setting-Parameters-of-Patchwork-in-Your-Own-Env.).
- A novel height estimator, called *All-Terrain Automatic heighT estimator (ATAT)* is added within the patchwork code, which auto-calibrates the sensor height using the ground points in the vicinity of the vehicle/mobile robot.
  - Please refer to the function `consensus_set_based_height_estimation()`.
