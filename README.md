# Patchwork

Official page of *"Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor"*, which is accepted by RA-L with IROS'21 option 

#### [[Demo Video](https://youtu.be/rclqeDi4gow)] [[Preprint Paper](https://urserver.kaist.ac.kr/publicdata/patchwork/RA_L_21_patchwork_final_submission.pdf)] [[Project Wiki](https://github.com/LimHyungTae/patchwork/wiki)]

Patchwork                  |  Concept of our method (CZM & GLE)
:-------------------------:|:-------------------------:
![](img/patchwork_concept_resized.jpg) |  ![](img/patchwork.gif)

It's an overall updated version of **R-GPF of ERASOR** [[Code]](https://github.com/LimHyungTae/ERASOR) [[Paper]](https://arxiv.org/abs/2103.04316). 

----

# Demo

### KITTI 00 

![](img/demo_kitti00_v2.gif)

### Rough Terrain

![](img/demo_terrain_v3.gif)

----


### Characteristics

* Single hpp file (include/patchwork/patchwork.hpp)

* Robust ground consistency

As shown in the below gif, our method shows most promising robust performance compared with other state-of-the-art methods. (Since ground is uneven, so the algorithms are not sure about the optimality of fitting of the ground.

Please kindly note that the concept of *traversable area* and the *ground* in thie repo. is quite different! Please refer to our paper.


## Contents
0. [Test Env.](#Test-Env.)
0. [Requirements](#requirements)
0. [How to Run Patchwork](#How-to-Run-Patchwork)
0. [Benchmark](#benchmark)
0. [Citation](#citation)

### Test Env.

The code is tested successfully at
* Linux 18.04 LTS
* ROS Melodic

## Requirements

### ROS Setting
- Install [ROS](http://torch.ch/docs/getting-started.html) on a machine. 
- Also, [jsk-visualization](https://github.com/jsk-ros-pkg/jsk_visualization) is required to visualize Ground Likelihood Estimation status.

```bash
sudo apt-get install ros-melodic-jsk-recognition
sudo apt-get install ros-melodic-jsk-common-msgs
sudo apt-get install ros-melodic-jsk-rviz-plugins
```

[여기](https://limhyungtae.github.io/2020-09-05-ROS-jsk_visualization-%EC%84%A4%EC%B9%98%ED%95%98%EB%8A%94-%EB%B2%95/)를 보고 따라 설치하면 됨

## How to Run Patchwork

We provide three examples
* Offline KITTI dataset
* Onine (ROS Callback) KITTI dataset
* Own dataset using pcd files

```
$ roslaunch nonplanar_gpf gpf.launch
```

현재 baseline인 gpf만 포팅해둔 상태임.

## Descriptions

## How to run
```
roslaunch patchwork ground_semgentation.launch target_alg:="patchwork" target_seq:="00"
```


```
Base Folder
_____00
     |___velodyne [raw data *.bin]
     |___pcd [*.pcd] (can be generated from *.bin by run_kittibin2pcd.sh)
     |___labels [raw semantic label *.label] (optional for semantic aided lidar odometry) 
     |___label_pcd [*.pcd] (optional for semantic aided lidar odometry, can be generated from *.label and *.bin by run_semantic_kitti_labelbin2pcd.sh) 
     |___00.txt [ground truth (gnssins) pose] (optional for evaluation)
     |___calib.txt [extrinsic transformation matrix (from body to lidar coordinate system)] (optional for evaluation)
     |___result [output of MULLS: generated map, pose and evaluation] (would be generated automatically after the transaction) 
_____01
     |___velodyne
     |___pcd
     |___labels
     |...
_____...
   
```

#### Point label 관련
* point의 member 변수들은 `utils/common.hpp`에 나와있음: `x, y, z, intensity, label, id`로 구분됨. 여기서 id는 각 object의 아이디임 (본 레포에서는 안 쓰일듯)
* label은 int로 돼있는데, 각 int가 나타내는 건 [SemanticKITTI API](https://github.com/PRBonn/semantic-kitti-api/blob/master/config/semantic-kitti.yaml)에 나와있음
* 아마 learning_map에서 보듯이, (40, 48, 49, 60)는 바닥인거 같은데, 44와 72도 확인 요망

#### 알고리즘 개발 관련

* **msg의 node.msg는 절대 변경하지 마셈!** 변경하면 bag 파일의 node를 callback 못 받음...bag 파일을 재생성해야 callback을 받을 수 있음
* 새로운 알고리즘을 만들 때는 `launch/gpf.launch`의 `algorithm`을 해당 이름에 맞게 바꾸고, `nodes/main.cpp`에 연동시켜 결과를 뽑으면 됨
* 해당 알고리즘에 대한 rviz 파일을 따로 생성하길 권장
* c++에서는 결과를 저장하는 txt나 csv파일 형식으로 저장하고 분석을 파이썬을 이용해서 `scripts/analysis` 내에 만들길 권장
* 현재 `scripts/analysis/output` 내를 보면 kitti 내에서 z의 변화에 대해서 visualization해둠!

## Citation

If you use our code or method in your work, please consider citing the following:

	@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2021}
    }

---------

### Contact

If you have any questions, please let me know:

- Hyungtae Lim {[shapelim@kaist.ac.kr]()}


### TODO List

- [x] Add ROS support
- [ ] Add preprint paper
- [ ] Add demo videos
- [ ] Add demo examples
- [ ] Update camera-ready paper

-----


