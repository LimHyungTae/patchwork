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
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        src
)

#add_executable(classviz utils/viz_each_class.cpp utils/common.hpp)
#target_link_libraries(classviz ${catkin_LIBRARIES})
#add_dependencies(classviz nonplanar_gpf_generate_messages_cpp)
#
#add_executable(estimateviz utils/viz_estimates.cpp utils/common.hpp)
#target_link_libraries(estimateviz ${catkin_LIBRARIES})
#add_dependencies(estimateviz nonplanar_gpf_generate_messages_cpp)
#
#add_executable(filecheck utils/filesystem_check.cpp)
#target_link_libraries(filecheck ${catkin_LIBRARIES})
#add_dependencies(filecheck nonplanar_gpf_generate_messages_cpp)
#
#add_executable(ceres_check utils/speed_check_ceres.cpp utils/optim.hpp)
#target_link_libraries(ceres_check ${catkin_LIBRARIES} ${PCL_LIBRARIES} ceres)
#add_dependencies(ceres_check nonplanar_gpf_generate_messages_cpp)


set(alg_source
    nodes/patchwork/utils.hpp
    nodes/patchwork/patchwork.hpp
    )

add_executable(benchmark nodes/main.cpp ${alg_source})
target_link_libraries(benchmark ${catkin_LIBRARIES} ${PCL_LIBRARIES} ceres)
add_dependencies(benchmark patchwork_generate_messages_cpp)
