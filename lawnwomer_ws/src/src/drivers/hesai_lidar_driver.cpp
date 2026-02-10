#include "lawnmower/sensor/LaserScanner.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace lawnmower {
LaserScanner::LaserScanner() {
    is_initialized_ = false;
    udp_port = 2368;
    ptc_port = 9347;
    device_ip = "192.168.1.201";
}
LaserScanner::~LaserScanner() {
    stop();
}
bool LaserScanner::initialize(const std::string& config_file) {
    try {
        // 读取配置文件
        YAML::Node config = YAML::LoadFile(config_file);
        
        device_ip = config["device_ip"].as<std::string>();
        udp_port = config["udp_port"].as<int>();
        ptc_port = config["ptc_port"].as<int>();
        correction_file = config["correction_file"].as<std::string>();
        firetimes_file = config["firetimes_file"].as<std::string>();
        
        // 创建激光雷达对象
        hesai_lidar_.reset(new HesaiLidar());
        
        // 配置激光雷达
        hesai_lidar_->setDeviceIP(device_ip);
        hesai_lidar_->setUDPPort(udp_port);
        hesai_lidar_->setPTCPort(ptc_port);
        hesai_lidar_->setCorrectionFile(correction_file);
        hesai_lidar_->setFiretimesFile(firetimes_file);
        
        // 初始化
        if (!hesai_lidar_->init()) {
            ROS_ERROR("Hesai lidar initialization failed!");
            return false;
        }
        
        // 创建ROS发布者
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_raw", 10);
        
        is_initialized_ = true;
        ROS_INFO("Hesai JT128 lidar initialized successfully!");
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Hesai lidar initialization failed: %s", e.what());
        return false;
    }
}
void LaserScanner::start() {
    if (is_initialized_) {
        hesai_lidar_->start();
        ROS_INFO("Hesai lidar started.");
    }
}
void LaserScanner::stop() {
    if (is_initialized_) {
        hesai_lidar_->stop();
        ROS_INFO("Hesai lidar stopped.");
    }
}
void LaserScanner::update() {
    if (!is_initialized_) return;
    
    try {
        // 获取点云数据
        HesaiLidar::PointCloudData cloud_data = hesai_lidar_->getPointCloud();
        
        // 转换为ROS消息
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(cloud_data.cloud, ros_cloud);
        
        ros_cloud.header.stamp = ros::Time::now();
        ros_cloud.header.frame_id = "laser_link";
        
        pointcloud_pub_.publish(ros_cloud);
        
        // 发布IMU数据（如果有）
        if (!cloud_data.imu.empty()) {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "laser_link";
            
            imu_msg.orientation.x = cloud_data.imu[0];
            imu_msg.orientation.y = cloud_data.imu[1];
            imu_msg.orientation.z = cloud_data.imu[2];
            imu_msg.orientation.w = cloud_data.imu[3];
            
            imu_msg.angular_velocity.x = cloud_data.imu[4];
            imu_msg.angular_velocity.y = cloud_data.imu[5];
            imu_msg.angular_velocity.z = cloud_data.imu[6];
            
            imu_msg.linear_acceleration.x = cloud_data.imu[7];
            imu_msg.linear_acceleration.y = cloud_data.imu[8];
            imu_msg.linear_acceleration.z = cloud_data.imu[9];
            
            // 发布IMU消息
            // imu_pub_.publish(imu_msg);
        }
    } catch (const std::exception& e) {
        ROS_WARN("Hesai lidar update failed: %s", e.what());
    }
}
} // namespace lawnmower
