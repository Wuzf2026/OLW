#ifndef LAWNMOWER_SENSOR_RTKGPS_H
#define LAWNMOWER_SENSOR_RTKGPS_H
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "handsfree/rtk/rtk_gps.h"
namespace lawnmower {
class RTKGPS {
private:
    ros::Publisher gps_pub_;
    ros::Publisher imu_pub_;
    
    std::unique_ptr<RTK_GPS> rtk_gps_;
    
    bool is_initialized_;
    std::mutex mtx_;
    
public:
    RTKGPS();
    ~RTKGPS();
    
    bool initialize(const std::string& port, int baudrate);
    void start();
    void stop();
    void update();
    
    bool isInitialized() const { return is_initialized_; }
    
    // 定位状态
    bool is_fixed;
    double latitude;
    double longitude;
    double altitude;
    double hdop;
    double vdop;
    double speed;
    double heading;
};
} // namespace lawnmower
#endif // LAWNMOWER_SENSOR_RTKGPS_H
