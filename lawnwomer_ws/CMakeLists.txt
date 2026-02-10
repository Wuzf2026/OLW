cmake_minimum_required(VERSION 2.8.3)
project(lawnwomer_ws)
# 设置CATKIN工作空间
set(CATKIN_DEVEL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/devel)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)
# 查找catkin依赖
find_package(catkin REQUIRED)
# 生成工作空间配置
catkin_make_workspace()
# 定义环境变量
set(ENV{ROS_PACKAGE_PATH} "$ENV{ROS_PACKAGE_PATH}:${CMAKE_CURRENT_SOURCE_DIR}")
set(ENV{ROS_ROOT} "$ENV{ROS_ROOT}:${CMAKE_CURRENT_SOURCE_DIR}/devel/share/ros")
# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# 包含第三方库路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hesai_jt128_sdk/HesaiLidar_SDK_2.0-master/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gemini335_sdk/OrbbecSDK_ROS1-2-main/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/rtk_um982_sdk/handsfree_rtk/include
)
# 添加子目录
add_subdirectory(src/lawnwomer)
add_subdirectory(src/hesai_lidar)
add_subdirectory(src/orbbec_camera)
add_subdirectory(src/rtk_gps)
add_subdirectory(src/ultrasonic)
add_subdirectory(src/motor_control)
