#include "lawnmower/sensor/RTKGPS.h"
namespace lawnmower {
RTKGPS::RTKGPS() {
    is_initialized_ = false;
    is_fixed = false;
}
RTKGPS::~RTKGPS() {
    stop();
}
bool RTKGPS::initialize(const std::string& port, int baudrate) {
    try {
        // 创建RTK GPS对象
        rtk_gps_.reset(new RTK_GPS());
        
        // 配置串口参数
        rtk_gps_->setPort(port);
        rtk_gps_->setBaudrate(baudrate);
        
        // 初始化
        if (!rtk_gps_->init()) {
            ROS_ERROR("RTK GPS initialization failed!");
            return false;
        }
        
        // 创建ROS发布者
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("gps/fix", 10);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("gps/imu", 10);
        
        is_initialized_ = true;
        ROS_INFO("T-RTK UM982 initialized successfully!");
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("RTK GPS initialization failed: %s", e.what());
        return false;
    }
}
void RTKGPS::start() {
    if (is_initialized_) {
        rtk_gps_->start();
        ROS_INFO("RTK GPS started.");
    }
}
void RTKGPS::stop() {
    if (is_initialized_) {
        rtk_gps_->stop();
        ROS_INFO("RTK GPS stopped.");
    }
}
void RTKGPS::update() {
    if (!is_initialized_) return;
    
    try {
        // 更新定位数据
        rtk_gps_->update();
        
        // 获取定位状态
        is_fixed = rtk_gps_->isFixed();
        
        if (is_fixed) {
            // 发布GPS数据
            sensor_msgs::NavSatFix gps_msg;
            gps_msg.header.stamp = ros::Time::now();
            gps_msg.header.frame_id = "gps_link";
            
            gps_msg.latitude = rtk_gps_->getLatitude();
            gps_msg.longitude = rtk_gps_->getLongitude();
            gps_msg.altitude = rtk_gps_->getAltitude();
            
            gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
            gps_msg.position_covariance = {
                rtk_gps_->getHdop() * rtk_gps_->getHdop(), 0, 0,
                0, rtk_gps_->getHdop() * rtk_gps_->getHdop(), 0,
                0, 0, rtk_gps_->getVdop() * rtk_gps_->getVdop()
            };
            
            gps_pub_.publish(gps_msg);
            
            // 发布IMU数据
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "gps_link";
            
            imu_msg.orientation.x = rtk_gps_->getRoll();
            imu_msg.orientation.y = rtk_gps_->getPitch();
            imu_msg.orientation.z = rtk_gps_->getYaw();
            imu_msg.orientation.w = 1.0;
            
            imu_msg.angular_velocity.x = 0;
            imu_msg.angular_velocity.y = 0;
            imu_msg.angular_velocity.z = rtk_gps_->getHeadingRate();
            
            imu_msg.linear_acceleration.x = 0;
            imu_msg.linear_acceleration.y = 0;
            imu_msg.linear_acceleration.z = 0;
            
            imu_pub_.publish(imu_msg);
            
            // 更新状态变量
            latitude = rtk_gps_->getLatitude();
            longitude = rtk_gps_->getLongitude();
            altitude = rtk_gps_->getAltitude();
            hdop = rtk_gps_->getHdop();
            vdop = rtk_gps_->getVdop();
            speed = rtk_gps_->getSpeed();
            heading = rtk_gps_->getHeading();
        }
    } catch (const std::exception& e) {
        ROS_WARN("RTK GPS update failed: %s", e.what());
    }
}
} // namespace lawnmower
