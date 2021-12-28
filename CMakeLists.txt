cmake_minimum_required(VERSION 3.0.2)
project(patchwork)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        cv_bridge
        pcl_conversions
        pcl_ros
        geometry_msgs
        laser_geometry
        sensor_msgs
        message_generation
        )

add_message_files(
        FILES
        node.msg
        ground_estimate.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)

add_executable(ros_kitti nodes/ros_kitti.cpp)
target_link_libraries(ros_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ros_kitti patchwork_generate_messages_cpp)

add_executable(pub_for_legoloam nodes/pub_for_legoloam.cpp)
target_link_libraries(pub_for_legoloam ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pub_for_legoloam patchwork_generate_messages_cpp)

add_executable(offline_kitti nodes/offline_kitti.cpp)
target_link_libraries(offline_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(offline_kitti patchwork_generate_messages_cpp)

add_executable(offline_own_data nodes/offline_own_data.cpp)
target_link_libraries(offline_own_data ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(offline_own_data patchwork_generate_messages_cpp)
