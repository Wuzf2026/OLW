#ifndef LAWNMOWER_SENSOR_LASERSCANNER_H
#define LAWNMOWER_SENSOR_LASERSCANNER_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "HesaiLidar.h"
namespace lawnmower {
class LaserScanner {
private:
    ros::Publisher pointcloud_pub_;
    std::unique_ptr<HesaiLidar> hesai_lidar_;
    
    bool is_initialized_;
    std::mutex mtx_;
    
public:
    LaserScanner();
    ~LaserScanner();
    
    bool initialize(const std::string& config_file);
    void start();
    void stop();
    void update();
    
    bool isInitialized() const { return is_initialized_; }
    const HesaiLidar& getLidar() const { return *hesai_lidar_; }
    
    // 配置参数
    std::string device_ip;
    int udp_port;
    int ptc_port;
    std::string correction_file;
    std::string firetimes_file;
};
} // namespace lawnmower
#endif // LAWNMOWER_SENSOR_LASERSCANNER_H
