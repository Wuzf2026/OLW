#ifndef LAWNMOWER_MOTOR_MOTORCONTROLLER_H
#define LAWNMOWER_MOTOR_MOTORCONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/Frame.h>
#include <vector>
#include "motor/MotorStatus.h"
namespace lawnmower {
class MotorController {
private:
    ros::Publisher motor_status_pub_;
    ros::Publisher can_frame_pub_;
    ros::Subscriber twist_sub_;
    
    std::vector<MotorStatus> motor_statuses_;
    
    bool is_initialized_;
    std::mutex mtx_;
    
    void twistCallback(const geometry_msgs/Twist::ConstPtr& msg);
    void sendMotorCommand(const MotorCommand& cmd);
    
public:
    MotorController();
    ~MotorController();
    
    bool initialize(const std::string& can_interface);
    void start();
    void stop();
    void update();
    
    bool isInitialized() const { return is_initialized_; }
    
    // 电机参数
    double wheel_base;
    double wheel_radius;
    double max_speed;
    double max_acceleration;
    
    // 电机状态
    const std::vector<MotorStatus>& getMotorStatuses() const { return motor_statuses_; }
};
} // namespace lawnmower
#endif // LAWNMOWER_MOTOR_MOTORCONTROLLER_H
