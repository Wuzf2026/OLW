#include "lawnmower/sensor/DepthCamera.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
namespace lawnmower {
DepthCamera::DepthCamera(ros::NodeHandle& nh) : it_(nh) {
    is_initialized_ = false;
}
DepthCamera::~DepthCamera() {
    stop();
}
bool DepthCamera::initialize() {
    try {
        // 查找设备
        std::vector<ObDeviceInfo> devices = ObSensor::getDeviceList();
        if (devices.empty()) {
            ROS_ERROR("No Orbbec device found!");
            return false;
        }
        
        // 打开第一个设备
        ob_sensor_.reset(new ObSensor(devices[0].uri));
        if (!ob_sensor_->isOpened()) {
            ROS_ERROR("Failed to open Orbbec device!");
            return false;
        }
        
        // 设置相机参数
        width = 1280;
        height = 800;
        fps = 30.0;
        
        ob_sensor_->setVideoFormat(OB_FORMAT_RGB888);
        ob_sensor_->setVideoResolution(width, height);
        ob_sensor_->setVideoFps(fps);
        ob_sensor_->setDepthResolution(width, height);
        ob_sensor_->setDepthFps(fps);
        
        // 启动数据流
        ob_sensor_->start();
        
        // 创建ROS发布者
        color_pub_ = it_.advertise("color/image_raw", 10);
        depth_pub_ = it_.advertise("depth/image_raw", 10);
        camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 10);
        
        is_initialized_ = true;
        ROS_INFO("Orbbec Gemini335 initialized successfully!");
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Orbbec initialization failed: %s", e.what());
        return false;
    }
}
void DepthCamera::start() {
    if (is_initialized_) {
        ob_sensor_->start();
        ROS_INFO("Orbbec camera started.");
    }
}
void DepthCamera::stop() {
    if (is_initialized_) {
        ob_sensor_->stop();
        ob_sensor_->close();
        ROS_INFO("Orbbec camera stopped.");
    }
}
void DepthCamera::update() {
    if (!is_initialized_) return;
    
    try {
        // 等待新的帧数据
        ObFrame* color_frame = ob_sensor_->getColorFrame();
        ObFrame* depth_frame = ob_sensor_->getDepthFrame();
        
        if (color_frame && depth_frame) {
            // 发布彩色图像
            cv::Mat color_image(height, width, CV_8UC3, color_frame->getData());
            sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
            color_pub_.publish(color_msg);
            
            // 发布深度图像
            cv::Mat depth_image(height, width, CV_16UC1, depth_frame->getData());
            sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_image).toImageMsg();
            depth_pub_.publish(depth_msg);
            
            // 发布相机信息
            sensor_msgs::CameraInfo camera_info;
            camera_info.header.stamp = ros::Time::now();
            camera_info.header.frame_id = "camera_link";
            
            // 设置相机内参（示例参数）
            camera_info.K = {525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1};
            camera_info.D = {0, 0, 0, 0, 0};
            camera_info.R = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            camera_info.P = {525.0, 0, 319.5, 0, 0, 525.0, 239.5, 0, 0, 0, 1, 0};
            
            camera_info_pub_.publish(camera_info);
            
            // 释放帧内存
            color_frame->release();
            depth_frame->release();
        }
    } catch (const std::exception& e) {
        ROS_WARN("Orbbec update failed: %s", e.what());
    }
}
} // namespace lawnmower
