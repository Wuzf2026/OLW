#ifndef LAWNMOWER_NODE_H
#define LAWNMOWER_NODE_H
#include <ros/ros.h>
#include <memory>
#include <atomic>
#include <mutex>
#include "lawnmower/sensor/LaserScanner.h"
#include "lawnmower/sensor/DepthCamera.h"
#include "lawnmower/sensor/RTKGPS.h"
#include "lawnmower/sensor/UltrasonicArray.h"
#include "lawnmower/motor/MotorController.h"
#include "lawnmower/utils/StateMachine.h"
#include "lawnmower/utils/Singleton.h"
namespace lawnmower {
class LawnmowerNode : public Singleton<LawnmowerNode> {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    std::unique_ptr<LaserScanner> laser_scanner_;
    std::unique_ptr<DepthCamera> depth_camera_;
    std::unique_ptr<RTKGPS> rtk_gps_;
    std::unique_ptr<UltrasonicArray> ultrasonic_array_;
    std::unique_ptr<MotorController> motor_controller_;
    
    std::unique_ptr<StateMachine> state_machine_;
    
    std::atomic<bool> is_running_;
    std::atomic<bool> is_emergency_stop_;
    std::mutex mtx_;
    
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);
    
public:
    LawnmowerNode();
    ~LawnmowerNode();
    
    void init();
    void run();
    void stop();
    
    bool isRunning() const { return is_running_; }
    bool isEmergencyStop() const { return is_emergency_stop_; }
    
    // 传感器接口
    const LaserScanner& getLaserScanner() const { return *laser_scanner_; }
    const DepthCamera& getDepthCamera() const { return *depth_camera_; }
    const RTKGPS& getRTKGPS() const { return *rtk_gps_; }
    const UltrasonicArray& getUltrasonicArray() const { return *ultrasonic_array_; }
    const MotorController& getMotorController() const { return *motor_controller_; }
};
} // namespace lawnmower
#endif // LAWNMOWER_NODE_H
