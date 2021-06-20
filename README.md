# Patchwork

Official page of [*"ERASOR: Egocentric Ratio of Pseudo Occupancy-based Dynamic Object Removal for Static 3D Point Cloud Map Building"*](https://arxiv.org/abs/2103.04316), which is accepted by RA-L with ICRA'21 option [[Demo Video](https://www.youtube.com/watch?v=Nx27ZO8afm0)].

![Image text](img/patchwork.gif)

## Requirements

* unavlib
* jsk_visualization
* ceres: CMakeLists.txt에서 target_link_libraries에 ceres로 써야 함!

jsk_visualization는 향후 plane marker를 그릴 때 유용함.

[여기](https://limhyungtae.github.io/2020-09-05-ROS-jsk_visualization-%EC%84%A4%EC%B9%98%ED%95%98%EB%8A%94-%EB%B2%95/)를 보고 따라 설치하면 됨

## How to use

```
$ roslaunch nonplanar_gpf gpf.launch
```

현재 baseline인 gpf만 포팅해둔 상태임.

## Descriptions

## How to run
```
roslaunch non_planar_gpf ground_semgentation.launch target_alg:="patchwork" target_seq:="00"
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

예시:
![sequence00](scripts/analysis/output/00_z.png)

## 현재 진행사항

