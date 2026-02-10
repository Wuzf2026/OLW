#include "lawnmower/motor/MotorController.h"
#include <can_msgs/Frame.h>
#include <linux/can.h>
#include <sys/ioctl.h>
#include <net/if.h>
namespace lawnmower {
MotorController::MotorController() {
    is_initialized_ = false;
    wheel_base = 0.5; // 米
    wheel_radius = 0.1; // 米
    max_speed = 2.0; // 米/秒
    max_acceleration = 1.0; // 米/秒²
}
MotorController::~MotorController() {
    stop();
}
bool MotorController::initialize(const std::string& can_interface) {
    try {
        // 创建CAN接口
        int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock < 0) {
            ROS_ERROR("Failed to create CAN socket!");
            return false;
        }
        
        // 设置CAN接口
        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_interface.c_str());
        if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
            ROS_ERROR("Failed to get CAN interface index!");
            close(sock);
            return false;
        }
        
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            ROS_ERROR("Failed to bind CAN socket!");
            close(sock);
            return false;
        }
        
        // 创建CAN消息发布者
        can_frame_pub_ = nh_.advertise<can_msgs::Frame>("canbus/outgoing", 10);
        
        // 创建速度订阅者
        twist_sub_ = nh_.subscribe("cmd_vel", 10, &MotorController::twistCallback, this);
        
        // 创建电机状态发布者
        motor_status_pub_ = nh_.advertise<MotorStatusArray>("motor_status", 10);
        
        is_initialized_ = true;
        ROS_INFO("Motor controller initialized successfully!");
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Motor controller initialization failed: %s", e.what());
        return false;
    }
}
void MotorController::start() {
    if (is_initialized_) {
        ROS_INFO("Motor controller started.");
    }
}
void MotorController::stop() {
    if (is_initialized_) {
        // 发送停止命令
        MotorCommand stop_cmd;
        stop_cmd.id = 0x01;
        stop_cmd.command = MOTOR_CMD_STOP;
        sendMotorCommand(stop_cmd);
        
        ROS_INFO("Motor controller stopped.");
    }
}
void MotorController::update() {
    if (!is_initialized_) return;
    
    try {
        // 读取CAN总线上的电机状态
        can_frame frame;
        if (read(sock, &frame, sizeof(can_frame)) > 0) {
            // 解析电机状态
            MotorStatus status;
            status.id = frame.can_id & 0x1F; // 假设ID在0-31范围内
            
            // 解析数据（示例解析）
            status.speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) * 0.1; // RPM
            status.current = frame.data[2] * 0.1; // 安培
            status.temperature = frame.data[3]; // 摄氏度
            status.status_code = frame.data[4];
            status.is_enabled = (frame.data[5] & 0x01) != 0;
            status.is_fault = (frame.data[5] & 0x02) != 0;
            
            // 保存状态
            motor_statuses_[status.id] = status;
            
            // 发布状态消息
            MotorStatusArray status_array;
            status_array.statuses = motor_statuses_;
            motor_status_pub_.publish(status_array);
        }
    } catch (const std::exception& e) {
        ROS_WARN("Motor controller update failed: %s", e.what());
    }
}
void MotorController::twistCallback(const geometry_msgs/Twist::ConstPtr& msg) {
    if (!is_initialized_) return;
    
    // 差速运动学逆解
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;
    
    // 限制速度
    linear_x = std::max(std::min(linear_x, max_speed), -max_speed);
    angular_z = std::max(std::min(angular_z, max_speed/wheel_base), -max_speed/wheel_base);
    
    // 计算左右轮速度（弧度/秒）
    double left_speed = (linear_x - angular_z * wheel_base / 2) / wheel_radius;
    double right_speed = (linear_x + angular_z * wheel_base / 2) / wheel_radius;
    
    // 创建电机命令
    MotorCommand left_cmd, right_cmd;
    
    left_cmd.id = 0x01; // 左轮ID
    left_cmd.command = MOTOR_CMD_SPEED;
    left_cmd.speed = left_speed;
    
    right_cmd.id = 0x02; // 右轮ID
    right_cmd.command = MOTOR_CMD_SPEED;
    right_cmd.speed = right_speed;
    
    // 发送命令
    sendMotorCommand(left_cmd);
    sendMotorCommand(right_cmd);
}
void MotorController::sendMotorCommand(const MotorCommand& cmd) {
    can_frame frame;
    frame.can_id = cmd.id | 0x80; // 设置为远程帧
    frame.can_dlc = 8;
    
    // 打包命令数据
    frame.data[0] = (cmd.command >> 8) & 0xFF;
    frame.data[1] = cmd.command & 0xFF;
    frame.data[2] = (int16_t)(cmd.speed * 10) >> 8;
    frame.data[3] = (int16_t)(cmd.speed * 10) & 0xFF;
    frame.data[4] = cmd.direction;
    frame.data[5] = cmd.acceleration;
    frame.data[6] = cmd.deceleration;
    frame.data[7] = 0x00;
    
    // 发送CAN帧
    write(sock, &frame, sizeof(can_frame));
    
    // 发布CAN消息
    can_msgs::Frame ros_frame;
    ros_frame.header.stamp = ros::Time::now();
    ros_frame.id = frame.can_id;
    ros_frame.dlc = frame.can_dlc;
    ros_frame.data.assign(frame.data, frame.data + frame.can_dlc);
    
    can_frame_pub_.publish(ros_frame);
}
} // namespace lawnmower
