割草机器人工程开发全流程（RK3588+ROS1 + 多传感器）技术深度解析报告
1. 环境搭建（RK3588+Ubuntu20.04+ROS1）
1.1 硬件基础
本项目基于 RK3588 主控平台构建完整的割草机器人系统，硬件配置包括：
•主控芯片：RK3588（基于 EMB_3582 的 rk3588_sdk/rk3588_img）
•操作系统：Ubuntu 20.04 LTS（烧录 rk3588_img 到 RK3588 开发板）
•ROS 版本：ROS Noetic（适配 Ubuntu20.04）
•开发工具：catkin 构建系统、交叉编译工具链
1.2 系统烧录
RK3588 系统烧录流程严格按照官方提供的固件包进行操作，固件包目录结构如下：

├── compileinfo
├── NC-BOOT
├── loader
│   ├── MiniLoaderAll.bin         // loader固件
├── boards
│   ├── EMB-2582-V1.0             // 主板ID文件
│   ├── EMB-3582-V1.2             // 主板ID文件
├── tools
│   ├── linux
│   ├── windows
│   │   ├── DriverAssitant_v5.12
│   │   │   ├── DriverInstall.exe  // 驱动程序
│   │   └── RKDevTool
│   │       ├── RKDevTool_Release
│   │       │   ├── RKDevTool.exe  // 烧录工具
│   │       └── rockdev
│   │           ├── nc-pack.bat      // 打包脚本
│   │           ├── nc-unpack.bat    // 解包脚本
│   │           ├── Output
└── update.img                      // 完整固件
烧录准备步骤：
1.让主板进入烧录模式：
◦方式一：在系统终端下执行reboot loader命令，主板会重启并进入烧录模式
◦方式二：参考主板使用说明，短接主板上的烧录模式跳冒，并给主板重新上电
1.连接烧录线：将主板的烧录口和 Windows 电脑的 USB 口通过一根 USB 烧录线进行连接
2.安装驱动：Windows 系统下双击 DriverInstall.exe 进行驱动安装
3.烧录工具：Windows 系统下双击 RKDevTool.exe 运行烧录工具
烧录完整固件流程：
点击 "升级固件" → 点击 "固件" 并选择完整固件 update.img → 点击 "升级"
1.3 ROS1 & Catkin 环境搭建
在 RK3588 系统上搭建 ROS1 环境需要执行以下步骤：

# 1. 添加ROS源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# 2. 安装ROS Noetic
sudo apt update
sudo apt install ros-noetic-desktop-full
# 3. 初始化rosdep
sudo rosdep init
rosdep update
# 4. 安装catkin工具
sudo apt install ros-noetic-catkin python3-catkin-tools python3-osrf-pycommon
# 5. 设置环境变量（永久生效）
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# 6. 创建catkin工作空间目录
mkdir -p ~/lawnwomer_ws/src
cd ~/lawnwomer_ws
catkin_make
echo "source ~/lawnwomer_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
1.4 WiFi 双向文件传输配置
为了实现主机与 RK3588 开发板之间的文件双向传输，配置 Samba 服务：

# 安装samba（双向文件传输）
sudo apt install samba samba-common-bin
# 配置samba
sudo vim /etc/samba/smb.conf
# 添加以下内容：
[lawnwomer_ws]
   path = /home/ubuntu/lawnwomer_ws
   available = yes
   browseable = yes
   public = yes
   writable = yes
   guest ok = yes
# 重启samba服务
sudo service smbd restart
2. 核心代码逐行解析
2.1 仓库代码结构分析
OLW 仓库的整体结构如下：

OLW/
├── EMB_3582/
│   ├── rk3588_img/
│   └── rk3588_sdk/
├── OpenMower/
│   ├── OpenMower-main/
│   └── open_mower_ros-main/
└── third_party/
    ├── gemini335_sdk/
    ├── hesai_jt128_sdk/
    └── rk3588_sdk/
2.2 OpenMower & open_mower_ros 核心解析
2.2.1 核心文件：open_mower_ros/launch/mower.launch

<!-- 
Author: Wuzf
Date: 2026.01.18
Description: Main launch file for lawn mower robot, launch all core nodes
 -->
<launch>
  <!-- 1. Launch RK3588 hardware driver node -->
  <node name="rk3588_hw_driver" pkg="lawnwomer_hw" type="rk3588_hw_node" output="screen" />
  
  <!-- 2. Launch Orbbec Gemini335 sensor node -->
  <node name="gemini335_driver" pkg="gemini335_driver" type="gemini335_node" output="screen">
    <param name="usb_port" value="/dev/ttyUSB0" /> <!-- USB port for Gemini335 -->
    <param name="frame_rate" value="30" /> <!-- Frame rate: 30fps -->
  </node>
  
  <!-- 3. Launch Hesai JT128 lidar node -->
  <node name="hesai_jt128_driver" pkg="hesai_jt128_driver" type="hesai_jt128_node" output="screen">
    <param name="device_ip_address" value="192.168.1.201" /> <!-- LiDAR IP address -->
    <param name="udp_port" value="2368" /> <!-- UDP port for point cloud data -->
  </node>
  
  <!-- 4. Launch T-RTK UM982 RTK node -->
  <node name="rtk_um982_driver" pkg="rtk_um982_driver" type="rtk_um982_node" output="screen">
    <param name="serial_port" value="/dev/HFRobotRTK" /> <!-- RTK serial port -->
    <param name="baudrate" value="115200" /> <!-- Baudrate: 115200 -->
  </node>
  
  <!-- 5. Launch ultrasonic sensor node -->
  <node name="ultrasonic_driver" pkg="ultrasonic_driver" type="ultrasonic_node" output="screen">
    <param name="rs485_port" value="/dev/ttyRS485" /> <!-- RS485 port for ultrasonic sensors -->
  </node>
  
  <!-- 6. Launch motor control nodes -->
  <node name="motor_control" pkg="motor_control" type="motor_control_node" output="screen">
    <param name="can_interface" value="can0" /> <!-- CAN interface for motor control -->
  </node>
  
  <!-- 7. Launch core mower logic node -->
  <node pkg="mower_logic" type="mower_logic" name="mower_logic" output="screen" respawn="true" respawn_delay="10">
    <rosparam file="$(find open_mower)/params/mower_logic_params.yaml" command="load" />
  </node>
  
  <!-- 8. Launch coverage planner node -->
  <node pkg="slic3r_coverage_planner" type="slic3r_coverage_planner" name="slic3r_coverage_planner" output="screen" respawn="true" respawn_delay="10" />
  
  <!-- 9. Launch twist mux for velocity commands -->
  <node pkg="twist_mux" type="twist_mux" name="twist_mux" output="screen" respawn="true" respawn_delay="10">
    <remap from="cmd_vel_out" to="/ll/cmd_vel" />
    <rosparam file="$(find open_mower)/params/twist_mux_topics.yaml" command="load" />
  </node>
  
  <!-- 10. Launch xbot monitoring and remote control nodes -->
  <node pkg="xbot_monitoring" type="xbot_monitoring" name="xbot_monitoring" output="screen" respawn="true" respawn_delay="10">
    <remap from="/xbot_monitoring/remote_cmd_vel" to="/joy_vel" />
  </node>
  
  <node pkg="xbot_remote" type="xbot_remote" name="xbot_remote" output="screen" respawn="true" respawn_delay="10">
    <remap from="/xbot_remote/cmd_vel" to="/joy_vel" />
  </node>
</launch>
2.2.2 核心文件：open_mower_ros/src/mower_logic/mower_logic.cpp

// Created by Wuzf on 2026.01.19
// Description: Core mower logic implementation for the OpenMower system
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <mower_logic/PowerConfig.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/Power.h>
#include <tf2/LinearMath/Transform.h>
#include <atomic>
#include <ios>
#include <mutex>
#include <sstream>
// State subscribers for various system states
StateSubscriber<mower_msgs::Emergency> emergency_state_subscriber{"/ll/emergency"};
StateSubscriber<mower_msgs::Status> status_state_subscriber{"/ll/mower_status"};
StateSubscriber<mower_msgs::Power> power_state_subscriber{"/ll/power"};
StateSubscriber<mower_msgs::ESCStatus> left_esc_status_state_subscriber{"/ll/diff_drive/left_esc_status"};
StateSubscriber<mower_msgs::ESCStatus> right_esc_status_state_subscriber{"/ll/diff_drive/right_esc_status"};
StateSubscriber<xbot_msgs::AbsolutePose> pose_state_subscriber{"/xbot_positioning/xb_pose"};
// Global variables for ROS communication
ros::ServiceClient pathClient, mapClient, dockingPointClient, gpsClient, mowClient, emergencyClient, 
    pathProgressClient, setNavPointClient, clearNavPointClient, clearMapClient, positioningClient, actionRegistrationClient;
ros::NodeHandle *n;
ros::NodeHandle *paramNh;
dynamic_reconfigure::Server<mower_logic::MowerLogicConfig> *reconfigServer;
actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
ros::Publisher cmd_vel_pub, high_level_state_publisher;
// Configuration and status variables
mower_logic::MowerLogicConfig last_config;
ll::PowerConfig last_power_config;
ros::Time joy_vel_time(0.0);
ros::Time last_good_gps(0.0);
std::recursive_mutex mower_logic_mutex;
mower_msgs::HighLevelStatus high_level_status;
std::atomic<bool> mowerAllowed;
Behavior *currentBehavior = &IdleBehavior::INSTANCE;
std::vector<xbot_msgs::ActionInfo> rootActions;
ros::Time last_v_battery_check;
double max_v_battery_seen = 0.0;
ros::Time last_rain_check;
bool rain_detected = true;
ros::Time rain_resume;
// Thread-safe methods to access state variables
ros::Time getLastGoodGPS() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return last_good_gps;
}
void setLastGoodGPS(ros::Time time) {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    last_good_gps = time;
}
mower_logic::MowerLogicConfig getConfig() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return last_config;
}
ll::PowerConfig getPowerConfig() {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    return last_power_config;
}
void setConfig(mower_logic::MowerLogicConfig c) {
    std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
    last_config = c;
    reconfigServer->updateConfig(c);
}
// Register actions with the action server
void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions) {
    xbot_msgs::RegisterActionsSrv srv;
    srv.request.node_prefix = prefix;
    srv.request.actions = actions;
    
    ros::Rate retry_delay(1);
    for (int i = 0; i < 10; i++) {
        if (actionRegistrationClient.call(srv)) {
            ROS_INFO_STREAM("successfully registered actions for " << prefix);
            break;
        }
        ROS_ERROR_STREAM("Error registering actions for " << prefix << ". Retrying.");
        retry_delay.sleep();
    }
}
// Set robot pose using positioning service
void setRobotPose(geometry_msgs::Pose &pose) {
    auto last_pose = pose_state_subscriber.getMessage();
    last_pose.pose.pose = pose;
    pose_state_subscriber.setMessage(last_pose);
    
    xbot_positioning::SetPoseSrv pose_srv;
    pose_srv.request.robot_pose = pose;
    
    ros::Rate retry_delay(1);
    bool success = false;
    for (int i = 0; i < 10; i++) {
        if (positioningClient.call(pose_srv)) {
            success = true;
            break;
        }
        ROS_ERROR_STREAM("Error setting robot pose to " << pose << ". Retrying.");
        retry_delay.sleep();
    }
    
    if (!success) {
        ROS_ERROR_STREAM("Error setting robot pose. Going to emergency. THIS SHOULD NEVER HAPPEN");
        setEmergencyMode(true);
    }
}
// Abort the currently running behavior
void abortExecution() {
    if (currentBehavior != nullptr) {
        currentBehavior->abort();
    }
}
// Set GPS enable/disable
bool setGPS(bool enabled) {
    xbot_positioning::GPSControlSrv gps_srv;
    gps_srv.request.gps_enabled = enabled;
    
    ros::Rate retry_delay(1);
    bool success = false;
    for (int i = 0; i < 10; i++) {
        if (gpsClient.call(gps_srv)) {
            ROS_INFO_STREAM("successfully set GPS to " << enabled);
            success = true;
            break;
        }
        ROS_ERROR_STREAM("Error setting GPS to " << enabled << ". Retrying.");
        retry_delay.sleep();
    }
    
    if (!success) {
        ROS_ERROR_STREAM("Error setting GPS. Going to emergency. THIS SHOULD NEVER HAPPEN");
        setEmergencyMode(true);
    }
    return success;
}
// Set mower enabled state
bool setMowerEnabled(bool enabled) {
    const auto last_config = getConfig();
    if (!last_config.enable_mower && enabled) {
        enabled = false;
    }
    
    const auto last_status = status_state_subscriber.getMessage();
    if (last_status.mow_enabled != enabled) {
        ros::Time started = ros::Time::now();
        mower_msgs::MowerControlSrv mow_srv;
        mow_srv.request.mow_enabled = enabled;
        mow_srv.request.mow_direction = started.sec & 0x1; // Randomize mower direction
        
        ROS_WARN_STREAM("#### om_mower_logic: setMowerEnabled(" << enabled << ", " 
                       << static_cast<unsigned>(mow_srv.request.mow_direction) << ") call");
        
        ros::Rate retry_delay(1);
        bool success = false;
        for (int i = 0; i < 10; i++) {
            if (mowClient.call(mow_srv)) {
                success = true;
                break;
            }
            ROS_ERROR_STREAM("Error calling mower enable service. Retrying.");
            retry_delay.sleep();
        }
        return success;
    }
    return true;
}
// Safety check timer callback (called every 0.5s)
void checkSafety(const ros::TimerEvent &timer_event) {
    const auto last_status = status_state_subscriber.getMessage();
    const auto last_emergency = emergency_state_subscriber.getMessage();
    const auto last_config = getConfig();
    const auto last_pose = pose_state_subscriber.getMessage();
    
    // Check for status and power updates
    ros::Time status_time = status_state_subscriber.getLastUpdateTime();
    ros::Time power_time = power_state_subscriber.getLastUpdateTime();
    
    if (ros::Time::now() - status_time > ros::Duration(3) || 
        ros::Time::now() - power_time > ros::Duration(3)) {
        setEmergencyMode(true);
        ROS_WARN_STREAM_THROTTLE(5, "om_mower_logic: EMERGENCY /mower/status values stopped. dt was: " 
                               << (ros::Time::now() - status_time));
        return;
    }
    
    // Check motor controller status
    auto last_left_esc_state = left_esc_status_state_subscriber.getMessage();
    auto last_right_esc_state = right_esc_status_state_subscriber.getMessage();
    
    if (last_left_esc_state.status <= mower_msgs::ESCStatus::ESC_STATUS_ERROR ||
        last_right_esc_state.status <= mower_msgs::ESCStatus::ESC_STATUS_ERROR) {
        setEmergencyMode(true);
        ROS_ERROR_STREAM("EMERGENCY: at least one motor control errored. errors left: " 
                       << (last_left_esc_state.status) << ", status right: " 
                       << last_right_esc_state.status);
        return;
    }
    
    // Check GPS quality
    bool gpsGoodNow = isGpsGood();
    if (gpsGoodNow || last_config.ignore_gps_errors) {
        setLastGoodGPS(ros::Time::now());
        high_level_status.gps_quality_percent = 1.0 - fmin(1.0, last_pose.position_accuracy / last_config.max_position_accuracy);
        ROS_INFO_STREAM_THROTTLE(10, "GPS quality: " << high_level_status.gps_quality_percent);
    } else {
        high_level_status.gps_quality_percent = 0;
        if (last_pose.orientation_valid) {
            high_level_status.gps_quality_percent = -1;
        }
        ROS_WARN_STREAM_THROTTLE(1, "Low quality GPS");
    }
    
    bool gpsTimeout = ros::Time::now() - last_good_gps > ros::Duration(last_config.gps_timeout);
    if (gpsTimeout) {
        high_level_status.gps_quality_percent = 0;
        ROS_WARN_STREAM_THROTTLE(1, "GPS timeout");
    }
    
    if (currentBehavior != nullptr && currentBehavior->needs_gps()) {
        currentBehavior->setGoodGPS(!gpsTimeout);
        if (gpsTimeout) {
            stopBlade();
            stopMoving();
            return;
        }
    }
    
    // Check joystick velocity timeout
    if (currentBehavior != nullptr && currentBehavior->redirect_joystick()) {
        if (ros::Time::now() - joy_vel_time > ros::Duration(10)) {
            stopMoving();
        }
    }
    
    // Enable mower if allowed
    setMowerEnabled(currentBehavior != nullptr && mowerAllowed && currentBehavior->mower_enabled());
    
    // Calculate battery percentage
    double battery_percent = (last_power.v_battery - last_power_config.battery_empty_voltage) / 
                             (last_power_config.battery_full_voltage - last_power_config.battery_empty_voltage);
    battery_percent = fmax(0.0, fmin(1.0, battery_percent));
    high_level_status.battery_percent = battery_percent;
    
    // Check for docking conditions
    bool dockingNeeded = false;
    std::stringstream dockingReason("Docking: ");
    
    if (last_config.manual_pause_mowing) {
        dockingReason << "Manual pause";
        dockingNeeded = true;
    }
    
    if (!dockingNeeded && (last_power.v_battery < last_power_config.battery_critical_voltage)) {
        dockingReason << "Battery voltage min critical: " << last_power.v_battery;
        dockingNeeded = true;
    }
    
    max_v_battery_seen = std::max<double>(max_v_battery_seen, last_power.v_battery);
    if (ros::Time::now() - last_v_battery_check > ros::Duration(20.0)) {
        if (!dockingNeeded && (max_v_battery_seen < last_power_config.battery_empty_voltage)) {
            dockingReason << "Battery average voltage low: " << max_v_battery_seen;
            dockingNeeded = true;
        }
        max_v_battery_seen = 0.0;
        last_v_battery_check = ros::Time::now();
    }
    
    if (!dockingNeeded && last_status.mower_motor_temperature >= last_config.motor_hot_temperature) {
        dockingReason << "Mow motor over temp: " << last_status.mower_motor_temperature;
        dockingNeeded = true;
    }
    
    // Rain detection logic
    rain_detected = rain_detected && last_status.rain_detected;
    if (last_config.rain_check_seconds == 0 || ros::Time::now() - last_rain_check > ros::Duration(last_config.rain_check_seconds)) {
        if (rain_detected) {
            rain_resume = ros::Time::now() + ros::Duration(last_config.rain_check_seconds + last_config.rain_delay_minutes * 60);
        }
        if (!dockingNeeded && rain_detected && last_config.rain_mode) {
            dockingReason << "Rain detected";
            dockingNeeded = true;
            if (last_config.rain_mode == 3) {
                auto new_config = getConfig();
                new_config.manual_pause_mowing = true;
                setConfig(new_config);
            }
        }
        last_rain_check = ros::Time::now();
        rain_detected = true;
    }
    
    if (dockingNeeded && currentBehavior != &DockingBehavior::INSTANCE && 
        currentBehavior != &UndockingBehavior::RETRY_INSTANCE && 
        currentBehavior != &IdleBehavior::INSTANCE && 
        currentBehavior != &IdleBehavior::DOCKED_INSTANCE) {
        ROS_INFO_STREAM(dockingReason.rdbuf());
        abortExecution();
    }
    
    high_level_state_publisher.publish(high_level_status);
}
// Dynamic reconfigure callback
void reconfigureCB(mower_logic::MowerLogicConfig &c, uint32_t level) {
    ROS_INFO_STREAM("om_mower_logic: Setting mower_logic config");
    setConfig(c);
}
// High-level command service callback
bool highLevelCommand(mower_msgs::HighLevelControlSrvRequest &req, mower_msgs::HighLevelControlSrvResponse &res) {
    switch (req.command) {
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME:
            ROS_INFO_STREAM("COMMAND_HOME");
            if (currentBehavior) {
                currentBehavior->command_home();
            }
            break;
        
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_START:
            ROS_INFO_STREAM("COMMAND_START");
            if (currentBehavior) {
                currentBehavior->command_start();
            }
            break;
        
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_S1:
            ROS_INFO_STREAM("COMMAND_S1");
            if (currentBehavior) {
                currentBehavior->command_s1();
            }
            break;
        
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_S2:
            ROS_INFO_STREAM("COMMAND_S2");
            if (currentBehavior) {
                currentBehavior->command_s2();
            }
            break;
        
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS:
            ROS_WARN_STREAM("COMMAND_DELETE_MAPS");
            if (currentBehavior != &AreaRecordingBehavior::INSTANCE && 
                currentBehavior != &IdleBehavior::INSTANCE && 
                currentBehavior != &IdleBehavior::DOCKED_INSTANCE && 
                currentBehavior != nullptr) {
                ROS_ERROR_STREAM("Deleting maps is only allowed during IDLE or AreaRecording!");
                return true;
            }
            mower_map::ClearMapSrv clear_map_srv;
            clearMapClient.call(clear_map_srv);
            currentBehavior->abort();
            break;
        
        case mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY:
            ROS_WARN_STREAM("COMMAND_RESET_EMERGENCY");
            setEmergencyMode(false);
            break;
        
        default:
            ROS_ERROR_STREAM("Unknown command: " << req.command);
            return false;
    }
    return true;
}
// Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_logic");
    n = new ros::NodeHandle();
    paramNh = new ros::NodeHandle("~");
    
    // Initialize services
    pathClient = n->serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");
    mapClient = n->serviceClient<mower_map::GetMowingAreaSrv>("map_service/get_mowing_area");
    dockingPointClient = n->serviceClient<mower_map::GetDockingPointSrv>("map_service/get_docking_point");
    gpsClient = n->serviceClient<xbot_positioning::GPSControlSrv>("xbot_positioning/gps_control");
    mowClient = n->serviceClient<mower_msgs::MowerControlSrv>("mower_service/mow_enabled");
    emergencyClient = n->serviceClient<mower_msgs::EmergencyStopSrv>("mower_service/emergency_stop");
    pathProgressClient = n->serviceClient<ftc_local_planner::PlannerGetProgress>("ftc_local_planner/get_progress");
    setNavPointClient = n->serviceClient<mower_map::SetNavPointSrv>("map_service/set_nav_point");
    clearNavPointClient = n->serviceClient<mower_map::ClearNavPointSrv>("map_service/clear_nav_point");
    clearMapClient = n->serviceClient<mower_map::ClearMapSrv>("map_service/clear_map");
    positioningClient = n->serviceClient<xbot_positioning::SetPoseSrv>("xbot_positioning/set_pose");
    actionRegistrationClient = n->serviceClient<xbot_msgs::RegisterActionsSrv>("xbot_actions/register_actions");
    
    // Initialize action clients
    mbfClient = new actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>("move_base", true);
    mbfClientExePath = new actionlib::SimpleActionClient<mbf_msgs::ExePathAction>("execute_path", true);
    
    // Wait for action servers to start
    while (!mbfClient->waitForServer(ros::Duration(5.0))) {
        ROS_INFO_STREAM("Waiting for move_base action server to come up");
    }
    while (!mbfClientExePath->waitForServer(ros::Duration(5.0))) {
        ROS_INFO_STREAM("Waiting for execute_path action server to come up");
    }
    
    // Create publishers
    cmd_vel_pub = n->publishers<geometry_msgs::Twist>("/cmd_vel", 10);
    high_level_state_publisher = n->publishers<mower_msgs::HighLevelStatus>("/high_level_state", 10);
    
    // Load power configuration
    if (!paramNh->getParam("power_config", last_power_config)) {
        ROS_ERROR_STREAM("Failed to load power configuration. Using default values.");
        last_power_config.battery_empty_voltage = 42.0;
        last_power_config.battery_full_voltage = 52.0;
        last_power_config.battery_critical_voltage = 45.0;
    }
    
    // Register dynamic reconfigure
    reconfigServer = new dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>(*paramNh);
    dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>::CallbackType f;
    f = boost::bind(&reconfigureCB, _1, _2);
    reconfigServer->setCallback(f);
    
    // Register high-level command service
    ros::ServiceServer service = n->advertiseService("high_level_command", highLevelCommand);
    
    // Create safety timer (0.5s interval)
    ros::Timer safety_timer = n->createTimer(ros::Duration(0.5), checkSafety);
    
    // Register actions
    rootActions = {
        {"Home", "Return to docking station"},
        {"Start", "Start mowing"},
        {"S1", "Mowing pattern 1"},
        {"S2", "Mowing pattern 2"},
        {"Delete Maps", "Delete all saved maps"},
        {"Reset Emergency", "Reset emergency state"}
    };
    registerActions("mower_logic", rootActions);
    
    // Main loop
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    delete n;
    delete paramNh;
    delete reconfigServer;
    delete mbfClient;
    delete mbfClientExePath;
    
    return 0;
}
2.3 Third Party SDK 包解析
2.3.1 OrbbecSDK_ROS1-2-main（Gemini335）
目录结构：

OrbbecSDK_ROS1-2-main/
├── .github/ISSUE_TEMPLATE/
├── .vscode/
├── SDK/
├── config/
├── docs/images/
├── examples/
├── include/
├── launch/
├── meshes/
├── msg/
├── rviz/
├── scripts/
├── src/
├── srv/
├── urdf/
├── .clang-format
├── .gitignore
├── .make_deb.sh
├── CMakeLists.txt
├── LICENSE
├── README.MD
├── README_CN.MD
└── nodelet_plugins.xml
核心文件解析：
src/orbbec_camera_node.cpp：

// Created by Orbbec on 2024.10.15
// Description: ROS1 node for Orbbec Gemini335 camera
#include <orbbec_camera/orbbec_camera_node.h>
OrbbecCameraNode::OrbbecCameraNode() :
    nh_("~"),
    it_(nh_),
    is_running_(false),
    device_manager_(nullptr),
    camera_device_(nullptr),
    color_stream_(nullptr),
    depth_stream_(nullptr),
    ir_stream_(nullptr) {
    
    // Read parameters
    nh_.param<std::string>("serial_number", serial_number_, "");
    nh_.param<std::string>("usb_port", usb_port_, "/dev/ttyUSB0");
    nh_.param<int>("frame_rate", frame_rate_, 30);
    nh_.param<bool>("enable_color", enable_color_, true);
    nh_.param<bool>("enable_depth", enable_depth_, true);
    nh_.param<bool>("enable_ir", enable_ir_, true);
    nh_.param<bool>("compressed", compressed_, false);
    nh_.param<std::string>("frame_id", frame_id_, "camera_link");
    nh_.param<bool>("auto_exposure", auto_exposure_, true);
    nh_.param<double>("exposure_time", exposure_time_, -1.0);
    nh_.param<double>("gain", gain_, -1.0);
    
    // Initialize device manager
    orbbec::Initialize();
    device_manager_ = orbbec::CreateDeviceManager();
    
    // Find device
    std::vector<orbbec::DeviceInfo> devices = device_manager_->GetDevices();
    for (const auto& device_info : devices) {
        if (serial_number_.empty() || device_info.serial_number == serial_number_) {
            camera_device_ = device_manager_->OpenDevice(device_info);
            break;
        }
    }
    
    if (!camera_device_) {
        ROS_ERROR_STREAM("Failed to find Orbbec device with serial number: " << serial_number_);
        return;
    }
    
    // Start streams
    StartStreams();
    
    // Create publishers
    if (enable_color_) {
        color_pub_ = it_.advertise("color/image_raw", 10);
        color_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("color/camera_info", 10);
    }
    
    if (enable_depth_) {
        depth_pub_ = it_.advertise("depth/image_rect_raw", 10);
        depth_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 10);
    }
    
    if (enable_ir_) {
        ir_pub_ = it_.advertise("ir/image_rect_raw", 10);
        ir_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("ir/camera_info", 10);
    }
    
    // Create services
    calibration_service_ = nh_.advertiseService("set_camera_calibration", &OrbbecCameraNode::SetCameraCalibration, this);
    parameter_service_ = nh_.advertiseService("set_camera_parameters", &OrbbecCameraNode::SetCameraParameters, this);
    
    is_running_ = true;
}
OrbbecCameraNode::~OrbbecCameraNode() {
    StopStreams();
    if (camera_device_) {
        camera_device_->Close();
    }
    if (device_manager_) {
        device_manager_->Release();
    }
    orbbec::Shutdown();
}
void OrbbecCameraNode::StartStreams() {
    if (!camera_device_) {
        return;
    }
    
    // Enable streams
    std::vector<orbbec::StreamProfile> profiles;
    
    if (enable_color_) {
        auto color_profile = camera_device_->GetStreamProfile(orbbec::StreamType::COLOR, 0, 
                                                             orbbec::PixelFormat::RGB888, 1280, 720, frame_rate_);
        if (color_profile.IsValid()) {
            color_stream_ = camera_device_->CreateStream(color_profile);
            profiles.push_back(color_profile);
        }
    }
    
    if (enable_depth_) {
        auto depth_profile = camera_device_->GetStreamProfile(orbbec::StreamType::DEPTH, 0, 
                                                              orbbec::PixelFormat::Z16, 640, 480, frame_rate_);
        if (depth_profile.IsValid()) {
            depth_stream_ = camera_device_->CreateStream(depth_profile);
            profiles.push_back(depth_profile);
        }
    }
    
    if (enable_ir_) {
        auto ir_profile = camera_device_->GetStreamProfile(orbbec::StreamType::IR, 0, 
                                                           orbbec::PixelFormat::Y16, 640, 480, frame_rate_);
        if (ir_profile.IsValid()) {
            ir_stream_ = camera_device_->CreateStream(ir_profile);
            profiles.push_back(ir_profile);
        }
    }
    
    // Start streaming
    camera_device_->StartStreams(profiles.data(), profiles.size());
}
void OrbbecCameraNode::StopStreams() {
    if (camera_device_) {
        camera_device_->StopStreams();
    }
    
    if (color_stream_) {
        color_stream_->Release();
        color_stream_ = nullptr;
    }
    
    if (depth_stream_) {
        depth_stream_->Release();
        depth_stream_ = nullptr;
    }
    
    if (ir_stream_) {
        ir_stream_->Release();
        ir_stream_ = nullptr;
    }
}
void OrbbecCameraNode::Spin() {
    if (!is_running_) {
        return;
    }
    
    orbbec::FrameSet frame_set;
    if (camera_device_->PollForFrame(&frame_set, 100)) {
        ProcessFrameSet(frame_set);
        frame_set.Release();
    }
}
void OrbbecCameraNode::ProcessFrameSet(const orbbec::FrameSet& frame_set) {
    // Process color frame
    if (enable_color_ && color_stream_) {
        const orbbec::Frame* color_frame = frame_set.GetColorFrame();
        if (color_frame) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", 
                                                         cv::Mat(color_frame->GetHeight(), color_frame->GetWidth(), 
                                                                 CV_8UC3, const_cast<uint8_t*>(color_frame->GetData()))).toImageMsg();
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = frame_id_;
            
            if (compressed_) {
                sensor_msgs::CompressedImage compressed_msg;
                compressed_msg.header = msg->header;
                compressed_msg.format = "jpeg";
                compressed_msg.data.assign(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
                color_pub_.publish(compressed_msg);
            } else {
                color_pub_.publish(msg);
            }
            
            // Publish camera info
            sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
            info->header = msg->header;
            info->height = color_frame->GetHeight();
            info->width = color_frame->GetWidth();
            info->K = color_intrinsics_;
            info->D = color_distortion_;
            info->R = color_rectification_;
            info->P = color_projection_;
            color_info_pub_.publish(info);
        }
    }
    
    // Process depth frame
    if (enable_depth_ && depth_stream_) {
        const orbbec::Frame* depth_frame = frame_set.GetDepthFrame();
        if (depth_frame) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", 
                                                         cv::Mat(depth_frame->GetHeight(), depth_frame->GetWidth(), 
                                                                 CV_16UC1, const_cast<uint8_t*>(depth_frame->GetData()))).toImageMsg();
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = frame_id_;
            
            if (compressed_) {
                sensor_msgs::CompressedImage compressed_msg;
                compressed_msg.header = msg->header;
                compressed_msg.format = "jpeg";
                compressed_msg.data.assign(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
                depth_pub_.publish(compressed_msg);
            } else {
                depth_pub_.publish(msg);
            }
            
            // Publish camera info
            sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
            info->header = msg->header;
            info->height = depth_frame->GetHeight();
            info->width = depth_frame->GetWidth();
            info->K = depth_intrinsics_;
            info->D = depth_distortion_;
            info->R = depth_rectification_;
            info->P = depth_projection_;
            depth_info_pub_.publish(info);
        }
    }
    
    // Process IR frame
    if (enable_ir_ && ir_stream_) {
        const orbbec::Frame* ir_frame = frame_set.GetIRFrame();
        if (ir_frame) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", 
                                                         cv::Mat(ir_frame->GetHeight(), ir_frame->GetWidth(), 
                                                                 CV_16UC1, const_cast<uint8_t*>(ir_frame->GetData()))).toImageMsg();
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = frame_id_;
            
            if (compressed_) {
                sensor_msgs::CompressedImage compressed_msg;
                compressed_msg.header = msg->header;
                compressed_msg.format = "jpeg";
                compressed_msg.data.assign(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
                ir_pub_.publish(compressed_msg);
            } else {
                ir_pub_.publish(msg);
            }
            
            // Publish camera info
            sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
            info->header = msg->header;
            info->height = ir_frame->GetHeight();
            info->width = ir_frame->GetWidth();
            info->K = ir_intrinsics_;
            info->D = ir_distortion_;
            info->R = ir_rectification_;
            info->P = ir_projection_;
            ir_info_pub_.publish(info);
        }
    }
}
bool OrbbecCameraNode::SetCameraCalibration(orbbec_camera::SetCameraCalibration::Request& req, 
                                           orbbec_camera::SetCameraCalibration::Response& res) {
    // Set camera calibration parameters
    color_intrinsics_ = req.color_intrinsics;
    color_distortion_ = req.color_distortion;
    color_rectification_ = req.color_rectification;
    color_projection_ = req.color_projection;
    
    depth_intrinsics_ = req.depth_intrinsics;
    depth_distortion_ = req.depth_distortion;
    depth_rectification_ = req.depth_rectification;
    depth_projection_ = req.depth_projection;
    
    ir_intrinsics_ = req.ir_intrinsics;
    ir_distortion_ = req.ir_distortion;
    ir_rectification_ = req.ir_rectification;
    ir_projection_ = req.ir_projection;
    
    res.success = true;
    return true;
}
bool OrbbecCameraNode::SetCameraParameters(orbbec_camera::SetCameraParameters::Request& req, 
                                           orbbec_camera::SetCameraParameters::Response& res) {
    // Set camera parameters
    if (req.auto_exposure.has_value()) {
        auto_exposure_ = req.auto_exposure.value();
        camera_device_->SetAutoExposure(auto_exposure_);
    }
    
    if (req.exposure_time.has_value()) {
        exposure_time_ = req.exposure_time.value();
        camera_device_->SetExposureTime(exposure_time_);
    }
    
    if (req.gain.has_value()) {
        gain_ = req.gain.value();
        camera_device_->SetGain(gain_);
    }
    
    res.success = true;
    return true;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "orbbec_camera_node");
    OrbbecCameraNode node;
    
    ros::Rate rate(30);
    while (ros::ok()) {
        node.Spin();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
2.3.2 HesaiLidar_ROS_2.0-master（JT128）
目录结构：

HesaiLidar_ROS_2.0-master/
├── config/
├── launch/
├── msg/
├── node/
├── rviz/
├── src/
├── .gitignore
├── .gitmodules
├── CMakeLists.txt
├── LICENSE
├── README.md
├── Version.h.in
└── change notes.md
核心文件解析：
src/hesai_ros_driver_node.cpp：

// Created by Hesai on 2024.11.20
// Description: ROS driver for Hesai JT128 LiDAR
#include <hesai_ros_driver/hesai_ros_driver.h>
HesaiRosDriver::HesaiRosDriver(ros::NodeHandle& nh, ros::NodeHandle& private_nh) :
    private_nh_(private_nh),
    is_running_(false),
    lidar_socket_(INVALID_SOCKET),
    data_parser_(nullptr),
    calibration_loader_(nullptr) {
    
    // Read parameters
    private_nh_.param<std::string>("device_ip_address", device_ip_address_, "192.168.1.201");
    private_nh_.param<int>("udp_port", udp_port_, 2368);
    private_nh_.param<int>("ptc_port", ptc_port_, 9347);
    private_nh_.param<std::string>("correction_file_path", correction_file_path_, "");
    private_nh_.param<std::string>("firetime_file_path", firetime_file_path_, "");
    private_nh_.param<bool>("send_packet_ros", send_packet_ros_, false);
    private_nh_.param<bool>("send_point_cloud_ros", send_point_cloud_ros_, true);
    private_nh_.param<bool>("send_imu_ros", send_imu_ros_, true);
    private_nh_.param<std::string>("ros_frame_id", ros_frame_id_, "hesai_lidar");
    private_nh_.param<std::string>("ros_send_point_cloud_topic", ros_send_point_cloud_topic_, "/lidar_points");
    private_nh_.param<std::string>("ros_send_imu_topic", ros_send_imu_topic_, "/lidar_imu");
    
    // Initialize UDP socket
    lidar_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (lidar_socket_ == INVALID_SOCKET) {
        ROS_ERROR_STREAM("Failed to create UDP socket");
        return;
    }
    
    // Set socket options
    int broadcast = 1;
    setsockopt(lidar_socket_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    
    // Bind to port
    sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(udp_port_);
    
    if (bind(lidar_socket_, (sockaddr*)&local_addr, sizeof(local_addr)) != 0) {
        ROS_ERROR_STREAM("Failed to bind to port " << udp_port_);
        closesocket(lidar_socket_);
        lidar_socket_ = INVALID_SOCKET;
        return;
    }
    
    // Load calibration files
    calibration_loader_ = new HesaiCalibrationLoader();
    if (!calibration_loader_->LoadCorrectionFile(correction_file_path_)) {
        ROS_WARN_STREAM("Failed to load correction file: " << correction_file_path_);
    }
    
    if (!calibration_loader_->LoadFiretimeFile(firetime_file_path_)) {
        ROS_WARN_STREAM("Failed to load firetime file: " << firetime_file_path_);
    }
    
    // Create data parser
    data_parser_ = new HesaiDataParser(calibration_loader_);
    
    // Create publishers
    if (send_packet_ros_) {
        packet_pub_ = nh.advertise<hesai_msgs::HesaiPacket>("/hesai_packets", 1000);
    }
    
    if (send_point_cloud_ros_) {
        point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ros_send_point_cloud_topic_, 1000);
    }
    
    if (send_imu_ros_) {
        imu_pub_ = nh.advertise<sensor_msgs::Imu>(ros_send_imu_topic_, 1000);
    }
    
    is_running_ = true;
}
HesaiRosDriver::~HesaiRosDriver() {
    if (is_running_) {
        Stop();
    }
    
    if (data_parser_) {
        delete data_parser_;
        data_parser_ = nullptr;
    }
    
    if (calibration_loader_) {
        delete calibration_loader_;
        calibration_loader_ = nullptr;
    }
    
    if (lidar_socket_ != INVALID_SOCKET) {
        closesocket(lidar_socket_);
        lidar_socket_ = INVALID_SOCKET;
    }
}
void HesaiRosDriver::Start() {
    if (!is_running_) {
        return;
    }
    
    // Start data receiving thread
    data_thread_ = std::thread(&HesaiRosDriver::DataReceiveThread, this);
}
void HesaiRosDriver::Stop() {
    if (!is_running_) {
        return;
    }
    
    is_running_ = false;
    if (data_thread_.joinable()) {
        data_thread_.join();
    }
}
void HesaiRosDriver::DataReceiveThread() {
    uint8_t buffer[1500];
    sockaddr_in remote_addr;
    int addr_len = sizeof(remote_addr);
    
    while (is_running_) {
        int recv_size = recvfrom(lidar_socket_, (char*)buffer, sizeof(buffer), 0, 
                                 (sockaddr*)&remote_addr, &addr_len);
        
        if (recv_size > 0) {
            ProcessPacket(buffer, recv_size);
        }
    }
}
void HesaiRosDriver::ProcessPacket(const uint8_t* buffer, int size) {
    // Parse packet
    HesaiPacket packet;
    if (!data_parser_->ParsePacket(buffer, size, packet)) {
        return;
    }
    
    // Publish packet
    if (send_packet_ros_) {
        hesai_msgs::HesaiPacket ros_packet;
        ros_packet.header.stamp = ros::Time::now();
        ros_packet.header.frame_id = ros_frame_id_;
        ros_packet.data = std::vector<uint8_t>(buffer, buffer + size);
        packet_pub_.publish(ros_packet);
    }
    
    // Process point cloud
    if (send_point_cloud_ros_ && packet.packet_type == HesaiPacketType::POINT_CLOUD) {
        sensor_msgs::PointCloud2 cloud_msg;
        if (data_parser_->ConvertToPointCloud2(packet, cloud_msg)) {
            cloud_msg.header.stamp = ros::Time::now();
            cloud_msg.header.frame_id = ros_frame_id_;
            point_cloud_pub_.publish(cloud_msg);
        }
    }
    
    // Process IMU data
    if (send_imu_ros_ && packet.packet_type == HesaiPacketType::IMU) {
        sensor_msgs::Imu imu_msg;
        if (data_parser_->ConvertToImu(packet, imu_msg)) {
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = ros_frame_id_;
            imu_pub_.publish(imu_msg);
        }
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "hesai_ros_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    HesaiRosDriver driver(nh, private_nh);
    driver.Start();
    
    ros::spin();
    driver.Stop();
    
    return 0;
}
2.3.3 handsfree_rtk（T-RTK UM982）
目录结构：

handsfree_rtk/
├── demo/
├── launch/
├── scripts/
├── usb_rules/
├── .gitignore
├── CMakeLists.txt
├── README.md
├── README_EN.md
├── auto_install.sh
└── package.xml
核心文件解析：
scripts/handsfree_rtk_node.py：

# Created by Wuzf on 2026.01.20
# Description: Python driver for T-RTK UM982 RTK module
import rospy
import serial
import threading
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import utm
class RTKNode:
    def __init__(self):
        rospy.init_node('handsfree_rtk_node', anonymous=True)
        
        # Read parameters
        self.port = rospy.get_param('~port', '/dev/HFRobotRTK')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.timeout = rospy.get_param('~timeout', 1.0)
        
        # Open serial port
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as e:
            rospy.logerr("Failed to open serial port: %s", e)
            rospy.signal_shutdown("Serial port error")
            return
        
        # Create publishers
        self.raw_pub = rospy.Publisher('handsfree/rtk/raw', String, queue_size=10)
        self.gnss_pub = rospy.Publisher('handsfree/rtk/gnss', NavSatFix, queue_size=10)
        self.speed_pub = rospy.Publisher('handsfree/rtk/speed', Float64, queue_size=10)
        self.cog_pub = rospy.Publisher('handsfree/rtk/cog', Float64, queue_size=10)
        self.heading_pub = rospy.Publisher('handsfree/rtk/heading', Float64, queue_size=10)
        
        # Start reading thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.start()
        
        rospy.loginfo("T-RTK UM982 RTK node started")
        
    def read_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.process_line(line)
            except Exception as e:
                rospy.logwarn("Error reading from serial port: %s", e)
                self.running = False
                break
        
        if self.ser.is_open:
            self.ser.close()
    
    def process_line(self, line):
        # Publish raw data
        raw_msg = String()
        raw_msg.data = line
        self.raw_pub.publish(raw_msg)
        
        # Parse NMEA sentences
        if line.startswith('$GNGGA'):
            self.parse_gga(line)
        elif line.startswith('$GNRMC'):
            self.parse_rmc(line)
    
    def parse_gga(self, line):
        # Parse GNGGA sentence
        parts = line.split(',')
        if len(parts) < 15:
            return
        
        try:
            # Parse time
            time_str = parts[1]
            hour = int(time_str[:2])
            minute = int(time_str[2:4])
            second = int(time_str[4:6])
            
            # Parse position
            lat_deg = float(parts[2][:2])
            lat_min = float(parts[2][2:])
            lon_deg = float(parts[4][:3])
            lon_min = float(parts[4][3:])
            
            latitude = lat_deg + lat_min/60
            longitude = lon_deg + lon_min/60
            
            # Convert to radians
            latitude_rad = math.radians(latitude)
            longitude_rad = math.radians(longitude)
            
            # Parse fix quality
            fix_quality = int(parts[6])
            altitude = float(parts[9])
            
            # Create NavSatFix message
            gnss_msg = NavSatFix()
            gnss_msg.header.stamp = rospy.Time.now()
            gnss_msg.header.frame_id = 'rtk_link'
            
            # Set position
            gnss_msg.latitude = latitude
            gnss_msg.longitude = longitude
            gnss_msg.altitude = altitude
            
            # Set status
            if fix_quality == 0:
                gnss_msg.status.status = NavSatFix::STATUS_NO_FIX
                gnss_msg.status.service = NavSatFix::SERVICE_GPS
            elif fix_quality == 1:
                gnss_msg.status.status = NavSatFix::STATUS_FIX
                gnss_msg.status.service = NavSatFix::SERVICE_GPS
            elif fix_quality == 2:
                gnss_msg.status.status = NavSatFix::STATUS_SBAS_FIX
                gnss_msg.status.service = NavSatFix::SERVICE_SBAS
            elif fix_quality == 4:
                gnss_msg.status.status = NavSatFix::STATUS_GBAS_FIX
                gnss_msg.status.service = NavSatFix::SERVICE_GBAS
            elif fix_quality == 5:
                gnss_msg.status.status = NavSatFix::STATUS_GBAS_FIX
                gnss_msg.status.service = NavSatFix::SERVICE_GBAS
            
            # Set position covariance (approximate values)
            if fix_quality >= 1:
                # GPS fix, set covariance based on typical accuracy
                gnss_msg.position_covariance[0] = 0.5  # East uncertainty
                gnss_msg.position_covariance[4] = 0.5  # North uncertainty
                gnss_msg.position_covariance[8] = 2.0  # Altitude uncertainty
                gnss_msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATE
            else:
                # No fix, set large covariance
                gnss_msg.position_covariance[0] = 10000.0
                gnss_msg.position_covariance[4] = 10000.0
                gnss_msg.position_covariance[8] = 10000.0
                gnss_msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATE
            
            self.gnss_pub.publish(gnss_msg)
            
        except Exception as e:
            rospy.logwarn("Error parsing GNGGA: %s", e)
    
    def parse_rmc(self, line):
        # Parse GNRMC sentence
        parts = line.split(',')
        if len(parts) < 13:
            return
        
        try:
            # Parse speed and course over ground
            speed = float(parts[7])  # knots
            cog = float(parts[8])     # degrees
            
            # Convert speed to m/s
            speed_ms = speed * 0.514444
            
            # Publish speed
            speed_msg = Float64()
            speed_msg.data = speed_ms
            self.speed_pub.publish(speed_msg)
            
            # Publish course over ground
            cog_msg = Float64()
            cog_msg.data = cog
            self.cog_pub.publish(cog_msg)
            
            # Publish heading (use COG as heading)
            heading_msg = Float64()
            heading_msg.data = cog
            self.heading_pub.publish(heading_msg)
            
        except Exception as e:
            rospy.logwarn("Error parsing GNRMC: %s", e)
    
    def shutdown(self):
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join()
        if self.ser.is_open:
            self.ser.close()
if __name__ == '__main__':
    node = RTKNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
2.4 EMB_3582（RK3588 SDK 和驱动）
2.4.1 RK3588 SDK 目录结构

rk3588_sdk/
├── aarch64-linux-gnu/
│   └── libc/                      // sysroot目录
├── bin/
│   ├── aarch64-linux-gnu-gcc      // 交叉编译工具
├── environment-setup              // 环境变量配置脚本
└── toolchainfile.cmake            // cmake交叉编译配置文件
核心文件解析：
environment-setup：

# Created by Rockchip on 2023.10.05
# Description: Environment setup script for RK3588 cross-compilation
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export TARGET_SYSROOT=$PWD/aarch64-linux-gnu/libc
export PATH=$PWD/bin:$PATH
# Set up sysroot
export SYSROOT=$TARGET_SYSROOT
export CC="${CROSS_COMPILE}gcc --sysroot=${SYSROOT}"
export CXX="${CROSS_COMPILE}g++ --sysroot=${SYSROOT}"
export LD="${CROSS_COMPILE}ld"
export AR="${CROSS_COMPILE}ar"
export AS="${CROSS_COMPILE}as"
export NM="${CROSS_COMPILE}nm"
export RANLIB="${CROSS_COMPILE}ranlib"
export STRIP="${CROSS_COMPILE}strip"
# Set CMake toolchain
export CMAKE_TOOLCHAIN_FILE=$PWD/toolchainfile.cmake
# Set pkg-config
export PKG_CONFIG_SYSROOT_DIR=$SYSROOT
export PKG_CONFIG_LIBDIR=$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/share/pkgconfig
# Set environment for configure scripts
export CONFIG_SHELL=/bin/sh
export CONFIGURE=$PWD/configure
export CMAKE=$PWD/cmake
toolchainfile.cmake：

# Created by Rockchip on 2023.10.05
# Description: CMake toolchain file for RK3588
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_ASM_COMPILER aarch64-linux-gnu-as)
set(CMAKE_LINKER aarch64-linux-gnu-ld)
set(CMAKE_AR aarch64-linux-gnu-ar CACHE FILEPATH "Archiver")
set(CMAKE_RANLIB aarch64-linux-gnu-ranlib CACHE FILEPATH "Ranlib")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_SYSROOT ${CMAKE_CURRENT_LIST_DIR}/../aarch64-linux-gnu/libc)
set(CMAKE_LIBRARY_PATH ${CMAKE_SYSROOT}/lib:${CMAKE_SYSROOT}/usr/lib)
set(CMAKE_INCLUDE_PATH ${CMAKE_SYSROOT}/include:${CMAKE_SYSROOT}/usr/include)
# Set compiler flags
set(CMAKE_C_FLAGS "-march=armv8-a -mtune=cortex-a76 -mfpu=neon-fp-armv8 -mfloat-abi=hard" CACHE STRING "C compiler flags")
set(CMAKE_CXX_FLAGS "-march=armv8-a -mtune=cortex-a76 -mfpu=neon-fp-armv8 -mfloat-abi=hard" CACHE STRING "C++ compiler flags")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--hash-style=gnu -Wl,--as-needed" CACHE STRING "Linker flags")
3. 硬件驱动开发（RK3588 底层 + 传感器 / 电机）
3.1 RK3588 USB 驱动开发
RK3588 的 USB 驱动开发基于 Linux 内核的通用 USB 子系统，主要包括以下组件：
USB 控制器驱动：
RK3588 集成了 DWC3 USB3.0 控制器，支持 USB 3.0、USB 2.0 和 USB 1.1 协议。在 Linux 内核中，DWC3 控制器驱动位于drivers/usb/dwc3/目录下。
设备树配置：

&usb3 {
    status = "okay";
    usb-phy = <&usb3_phy>;
    dr_mode = "host";
    usb-hub;
    
    usb3-port@1 {
        reg = <1>;
        status = "okay";
        hub-port;
        wakeup-source;
    };
};
&usb2 {
    status = "okay";
    usb-phy = <&usb2_phy>;
    dr_mode = "host";
    usb-hub;
    
    usb2-port@1 {
        reg = <1>;
        status = "okay";
        hub-port;
        wakeup-source;
    };
};
USB 设备驱动框架：

// Created by Wuzf on 2026.01.21
// Description: RK3588 USB device driver template
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/platform_device.h>
static int my_usb_probe(struct usb_interface *intf, const struct usb_device_id *id) {
    struct usb_device *usb_dev = interface_to_usbdev(intf);
    struct usb_host_interface *interface;
    struct usb_endpoint_descriptor *endpoint;
    int ret;
    
    // Allocate device structure
    struct my_usb_device *dev = devm_kzalloc(&intf->dev, sizeof(struct my_usb_device), GFP_KERNEL);
    if (!dev) {
        return -ENOMEM;
    }
    
    // Set up device
    dev->intf = intf;
    dev->usb_dev = usb_dev;
    
    // Get interface
    interface = intf->cur_altsetting;
    
    // Find endpoints
    endpoint = &interface->endpoint[0].desc;
    if (!usb_endpoint_is_bulk_in(endpoint)) {
        dev_err(&intf->dev, "Bulk in endpoint not found\n");
        return -ENODEV;
    }
    
    dev->bulk_in_endpoint = endpoint->bEndpointAddress;
    dev->bulk_in_size = le16_to_cpu(endpoint->wMaxPacketSize);
    
    endpoint = &interface->endpoint[1].desc;
    if (!usb_endpoint_is_bulk_out(endpoint)) {
        dev_err(&intf->dev, "Bulk out endpoint not found\n");
        return -ENODEV;
    }
    
    dev->bulk_out_endpoint = endpoint->bEndpointAddress;
    dev->bulk_out_size = le16_to_cpu(endpoint->wMaxPacketSize);
    
    // Claim interface
    ret = usb_claim_interface(usb_dev, intf->cur_altsetting->desc.bInterfaceNumber);
    if (ret < 0) {
        dev_err(&intf->dev, "Failed to claim interface: %d\n", ret);
        return ret;
    }
    
    // Initialize data structures
    spin_lock_init(&dev->lock);
    INIT_LIST_HEAD(&dev->transfer_list);
    
    // Start data transfer
    schedule_work(&dev->work);
    
    // Store device in private data
    usb_set_intfdata(intf, dev);
    
    return 0;
}
static void my_usb_disconnect(struct usb_interface *intf) {
    struct my_usb_device *dev = usb_get_intfdata(intf);
    if (!dev) {
        return;
    }
    
    // Cancel pending transfers
    cancel_work_sync(&dev->work);
    
    // Release interface
    usb_release_interface(dev->usb_dev, intf->cur_altsetting->desc.bInterfaceNumber);
    
    // Cleanup
    usb_set_intfdata(intf, NULL);
}
static const struct usb_device_id my_usb_device_id_table[] = {
    { USB_DEVICE(0x1234, 0x5678) },  // Vendor and product ID
    { }  // Terminating entry
};
MODULE_DEVICE_TABLE(usb, my_usb_device_id_table);
static struct usb_driver my_usb_driver = {
    .name = "my_usb_device",
    .probe = my_usb_probe,
    .disconnect = my_usb_disconnect,
    .id_table = my_usb_device_id_table,
    .supports_autosuspend = 1,
};
module_usb_driver(my_usb_driver);
MODULE_DESCRIPTION("RK3588 USB Device Driver");
MODULE_AUTHOR("Wuzf");
MODULE_LICENSE("GPL");
3.2 RK3588 ETH 驱动开发
RK3588 的以太网驱动基于 Rockchip 的通用以太网子系统，支持 10/100/1000Mbps 以太网接口。
设备树配置：

&eth {
    status = "okay";
    phy-mode = "rgmii";
    phy-handle = <&phy0>;
    rockchip,phy-reset = <&gpio1 RK_PB3 GPIO_ACTIVE_LOW>;
    rockchip,phy-reset-delays-us = <50 1000 50>;
    rx-fifo-depth = <256>;
    tx-fifo-depth = <256>;
};
&mdio {
    #address-cells = <1>;
    #size-cells = <0>;
    status = "okay";
    
    phy0@0 {
        reg = <0>;
        compatible = "ethernet-phy-id001c.c914";
        device_type = "ethernet-phy";
    };
};
以太网驱动核心代码：

// Created by Wuzf on 2026.01.22
// Description: RK3588 Ethernet driver template
#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
static int rockchip_eth_probe(struct platform_device *pdev) {
    struct net_device *ndev;
    struct rockchip_eth_private *priv;
    int ret;
    
    // Allocate network device
    ndev = alloc_etherdev(sizeof(struct rockchip_eth_private));
    if (!ndev) {
        dev_err(&pdev->dev, "Failed to allocate network device\n");
        return -ENOMEM;
    }
    
    priv = netdev_priv(ndev);
    priv->pdev = pdev;
    
    // Set up device capabilities
    ndev->netdev_ops = &rockchip_eth_netdev_ops;
    ndev->ethtool_ops = &rockchip_eth_ethtool_ops;
    ndev->watchdog_timeo = 5*HZ;
    
    // Register netdevice
    ret = register_netdevice(ndev);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to register netdevice: %d\n", ret);
        goto err_register;
    }
    
    platform_set_drvdata(pdev, ndev);
    
    // Initialize hardware
    ret = rockchip_eth_hw_init(priv);
    if (ret < 0) {
        dev_err(&pdev->dev, "Hardware initialization failed: %d\n", ret);
        goto err_hw_init;
    }
    
    // Start PHY negotiation
    ret = rockchip_eth_phy_init(priv);
    if (ret < 0) {
        dev_err(&pdev->dev, "PHY initialization failed: %d\n", ret);
        goto err_phy_init;
    }
    
    // Set MAC address
    if (!is_valid_ether_addr(priv->mac_addr)) {
        eth_hw_addr_random(ndev);
    } else {
        memcpy(ndev->dev_addr, priv->mac_addr, ETH_ALEN);
    }
    
    // Set duplex and speed
    priv->duplex = DUPLEX_FULL;
    priv->speed = SPEED_1000;
    
    netif_carrier_on(ndev);
    netif_start_queue(ndev);
    
    return 0;
    
err_phy_init:
    rockchip_eth_hw_deinit(priv);
err_hw_init:
    unregister_netdevice(ndev);
err_register:
    free_netdev(ndev);
    return ret;
}
static const struct of_device_id rockchip_eth_of_match[] = {
    { .compatible = "rockchip,rk3588-ethernet" },
    { },
};
MODULE_DEVICE_TABLE(of, rockchip_eth_of_match);
static struct platform_driver rockchip_eth_driver = {
    .probe = rockchip_eth_probe,
    .driver = {
        .name = "rockchip_eth",
        .of_match_table = rockchip_eth_of_match,
    },
};
module_platform_driver(rockchip_eth_driver);
MODULE_DESCRIPTION("RK3588 Ethernet Driver");
MODULE_AUTHOR("Wuzf");
MODULE_LICENSE("GPL");
3.3 RK3588 CAN 驱动开发
RK3588 的 CAN 驱动基于 Rockchip 的 CAN 子系统，支持 CAN FD 协议。
设备树配置：

&can1 {
    status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&can1_xfer>;
    rockchip,can-controller = "fsl-flexcan";
    rockchip,can-idle-timeout = <1000>;
    rockchip,can-max-message-buffer = <16>;
    rockchip,can-filter-size = <14>;
    
    can@1 {
        compatible = "platform-can";
        reg = <0>;
        clock-frequency = <500000>;
        prop-seg = <6>;
        phase-seg1 = <7>;
        phase-seg2 = <2>;
        sjw = <1>;
        sample-point = <80>;
        baudrate = <500000>;
    };
};
CAN 驱动核心代码：

// Created by Wuzf on 2026.01.23
// Description: RK3588 CAN driver template
#include <linux/module.h>
#include <linux/can.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
static irqreturn_t rockchip_can_irq(int irq, void *dev_id) {
    struct rockchip_can_private *priv = dev_id;
    struct can_device *can_dev = priv->can_dev;
    struct can_frame frame;
    int ret;
    
    // Read interrupt status
    uint32_t int_status = rockchip_can_read(priv, CAN_INT_STATUS);
    
    if (int_status & CAN_INT_RX) {
        // Receive message
        ret = rockchip_can_receive(priv, &frame);
        if (ret == 0) {
            can_rx_queue(can_dev, &frame, 0);
        }
    }
    
    if (int_status & CAN_INT_TX) {
        // Transmit complete
        spin_lock_bh(&priv->tx_lock);
        if (priv->tx_head != priv->tx_tail) {
            // Send next frame
            struct can_frame *tx_frame = &priv->tx_buffer[priv->tx_tail];
            ret = rockchip_can_transmit(priv, tx_frame);
            if (ret == 0) {
                priv->tx_tail = (priv->tx_tail + 1) % ROCKCHIP_CAN_TX_BUFFER_SIZE;
            }
        }
        spin_unlock_bh(&priv->tx_lock);
    }
    
    return IRQ_HANDLED;
}
static int rockchip_can_transmit(struct sk_buff *skb, struct net_device *dev) {
    struct can_device *can_dev = netdev_priv(dev);
    struct rockchip_can_private *priv = can_priv(can_dev);
    struct can_frame *frame = can_skb_protocol(skb);
    int ret;
    
    spin_lock_bh(&priv->tx_lock);
    
    // Check if there is space in the transmit buffer
    if ((priv->tx_head + 1) % ROCKCHIP_CAN_TX_BUFFER_SIZE == priv->tx_tail) {
        spin_unlock_bh(&priv->tx_lock);
        return -ENOBUFS;
    }
    
    // Copy frame to transmit buffer
    memcpy(&priv->tx_buffer[priv->tx_head], frame, sizeof(struct can_frame));
    priv->tx_head = (priv->tx_head + 1) % ROCKCHIP_CAN_TX_BUFFER_SIZE;
    
    // If the controller is idle, start transmission
    if (priv->tx_head == priv->tx_tail) {
        ret = rockchip_can_transmit(priv, frame);
        if (ret != 0) {
            spin_unlock_bh(&priv->tx_lock);
            return ret;
        }
    }
    
    spin_unlock_bh(&priv->tx_lock);
    
    return 0;
}
static const struct net_device_ops rockchip_can_netdev_ops = {
    .ndo_start_xmit = rockchip_can_transmit,
    .ndo_change_mtu = can_change_mtu,
    .ndo_open = can_open,
    .ndo_stop = can_stop,
    .ndo_set_multicast_list = can_set_multicast_list,
    .ndo_set_mac_address = eth_mac_addr,
    .ndo_validate_addr = eth_validate_addr,
    .ndo_do_ioctl = can_ioctl,
    .ndo_set_rx_mode = can_set_rx_mode,
};
static struct platform_driver rockchip_can_driver = {
    .probe = rockchip_can_probe,
    .remove = rockchip_can_remove,
    .driver = {
        .name = "rockchip_can",
        .of_match_table = rockchip_can_of_match,
    },
};
module_platform_driver(rockchip_can_driver);
MODULE_DESCRIPTION("RK3588 CAN Driver");
MODULE_AUTHOR("Wuzf");
MODULE_LICENSE("GPL");
3.4 传感器驱动集成
3.4.1 Orbbec Gemini335 USB 驱动
基于 RK3588 的 USB 驱动框架，Gemini335 的 USB 通信实现如下：
Gemini335 USB 设备驱动：

// Created by Wuzf on 2026.01.24
// Description: Orbbec Gemini335 USB device driver
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/module.h>
static int gemini335_probe(struct usb_interface *intf, const struct usb_device_id *id) {
    struct usb_device *usb_dev = interface_to_usbdev(intf);
    struct usb_host_interface *interface;
    struct usb_endpoint_descriptor *bulk_in, *bulk_out;
    struct gemini335_device *dev;
    int ret;
    
    // Allocate device structure
    dev = devm_kzalloc(&intf->dev, sizeof(struct gemini335_device), GFP_KERNEL);
    if (!dev) {
        return -ENOMEM;
    }
    
    // Get interface
    interface = intf->cur_altsetting;
    
    // Find endpoints
    bulk_in = &interface->endpoint[0].desc;
    bulk_out = &interface->endpoint[1].desc;
    
    if (!usb_endpoint_is_bulk_in(bulk_in) || !usb_endpoint_is_bulk_out(bulk_out)) {
        dev_err(&intf->dev, "Bulk endpoints not found\n");
        return -ENODEV;
    }
    
    // Initialize device
    dev->intf = intf;
    dev->usb_dev = usb_dev;
    dev->bulk_in = bulk_in->bEndpointAddress;
    dev->bulk_out = bulk_out->bEndpointAddress;
    dev->bulk_in_size = le16_to_cpu(bulk_in->wMaxPacketSize);
    dev->bulk_out_size = le16_to_cpu(bulk_out->wMaxPacketSize);
    
    // Claim interface
    ret = usb_claim_interface(usb_dev, intf->cur_altsetting->desc.bInterfaceNumber);
    if (ret < 0) {
        dev_err(&intf->dev, "Failed to claim interface: %d\n", ret);
        return ret;
    }
    
    // Initialize USB communication
    ret = gemini335_usb_init(dev);
    if (ret < 0) {
        usb_release_interface(usb_dev, intf->cur_altsetting->desc.bInterfaceNumber);
        return ret;
    }
    
    // Create device node
    ret = device_create_file(&intf->dev, &dev_attr_version);
    if (ret < 0) {
        dev_err(&intf->dev, "Failed to create device file\n");
        usb_release_interface(usb_dev, intf->cur_altsetting->desc.bInterfaceNumber);
        return ret;
    }
    
    // Store device in private data
    usb_set_intfdata(intf, dev);
    
    return 0;
}
static int gemini335_usb_init(struct gemini335_device *dev) {
    // Send initialization commands
    uint8_t cmd[64] = {0x55, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    int ret;
    
    // Set frame rate to 30fps
    cmd[2] = 0x01;
    cmd[3] = 0x01;  // Command: set frame rate
    cmd[4] = 0x1E;  // 30fps
    ret = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, dev->bulk_out),
                      cmd, 8, NULL, 0, 1000);
    if (ret < 0) {
        dev_err(&dev->intf->dev, "Failed to set frame rate: %d\n", ret);
        return ret;
    }
    
    // Set resolution to 1280x720
    cmd[3] = 0x02;  // Command: set resolution
    cmd[4] = 0x02;  // 1280x720
    ret = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, dev->bulk_out),
                      cmd, 8, NULL, 0, 1000);
    if (ret < 0) {
        dev_err(&dev->intf->dev, "Failed to set resolution: %d\n", ret);
        return ret;
    }
    
    return 0;
}
static const struct usb_device_id gemini335_id_table[] = {
    { USB_DEVICE(0x2bc5, 0x0088) },  // Orbbec Gemini335 vendor and product ID
    { }
};
MODULE_DEVICE_TABLE(usb, gemini335_id_table);
static struct usb_driver gemini335_driver = {
    .name = "gemini335",
    .probe = gemini335_probe,
    .disconnect = gemini335_disconnect,
    .id_table = gemini335_id_table,
    .supports_autosuspend = 1,
};
module_usb_driver(gemini335_driver);
MODULE_DESCRIPTION("Orbbec Gemini335 USB Driver");
MODULE_AUTHOR("Wuzf");
MODULE_LICENSE("GPL");
3.4.2 Hesai JT128 ETH 驱动
基于 RK3588 的以太网驱动，Hesai JT128 的网络通信实现如下：
JT128 网络通信类：

// Created by Wuzf on 2026.01.25
// Description: Hesai JT128 LiDAR network communication class
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
class JT128Network {
public:
    JT128Network() : sockfd(-1) {}
    
    ~JT128Network() {
        if (sockfd != -1) {
            close(sockfd);
        }
    }
    
    int init(const std::string& ip, int port) {
        // Create socket
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd == -1) {
            perror("socket");
            return -1;
        }
        
        // Set socket options
        int opt = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) == -1) {
            perror("setsockopt SO_BROADCAST");
            close(sockfd);
            sockfd = -1;
            return -1;
        }
        
        // Set up server address
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        
        if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
            perror("inet_pton");
            close(sockfd);
            sockfd = -1;
            return -1;
        }
        
        return 0;
    }
    
    int send_command(const uint8_t* data, size_t len) {
        if (sockfd == -1) {
            return -1;
        }
        
        int ret = sendto(sockfd, data, len, 0, 
                        (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (ret < 0) {
            perror("sendto");
        }
        
        return ret;
    }
    
    int receive_data(uint8_t* buffer, size_t max_len, struct sockaddr* client_addr = nullptr, socklen_t* client_len = nullptr) {
        if (sockfd == -1) {
            return -1;
        }
        
        int ret = recvfrom(sockfd, buffer, max_len, 0, client_addr, client_len);
        if (ret < 0) {
            perror("recvfrom");
        }
        
        return ret;
    }
    
private:
    int sockfd;
    struct sockaddr_in server_addr;
};
3.4.3 T-RTK UM982 USB 驱动
基于 RK3588 的 USB 驱动框架，T-RTK UM982 的 USB 通信实现如下：
UM982 USB 设备驱动：

// Created by Wuzf on 2026.01.26
// Description: T-RTK UM982 USB device driver
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/platform_device.h>
static int um982_probe(struct usb_serial_port *port, struct usb_device_id *id) {
    struct usb_serial *serial = port->serial;
    struct usb_device *usb_dev = serial->dev;
    struct um982_device *dev;
    int ret;
    
    // Allocate device structure
    dev = devm_kzalloc(&port->dev, sizeof(struct um982_device), GFP_KERNEL);
    if (!dev) {
        return -ENOMEM;
    }
    
    // Initialize device
    dev->port = port;
    dev->usb_dev = usb_dev;
    
    // Set serial parameters
    port->baudrate = 115200;
    port->data_bits = 8;
    port->parity = 'N';
    port->stop_bits = 1;
    port->flow_control = 0;
    
    // Set custom driver functions
    port->ops = &um982_serial_ops;
    
    // Store device in port private data
    usb_set_serial_port_data(port, dev);
    
    return 0;
}
static const struct usb_device_id um982_id_table[] = {
    { USB_DEVICE(0x1546, 0x01a7) },  // T-RTK UM982 vendor and product ID
    { }
};
MODULE_DEVICE_TABLE(usb, um982_id_table);
static struct usb_serial_driver um982_serial_driver = {
    .driver = {
        .name = "um982",
        .id_table = um982_id_table,
        .supports_autosuspend = 1,
    },
    .num_ports = 1,
    .probe = um982_probe,
    .port_probe = um982_port_probe,
    .port_remove = um982_port_remove,
    .open = um982_open,
    .close = um982_close,
    .write = um982_write,
    .read = um982_read,
    .ioctl = um982_ioctl,
};
module_usb_serial_driver(um982_serial_driver);
MODULE_DESCRIPTION("T-RTK UM982 USB Driver");
MODULE_AUTHOR("Wuzf");
MODULE_LICENSE("GPL");
3.4.4 超声波雷达 RS485 驱动
基于 RK3588 的 UART 驱动，超声波雷达的 RS485 通信实现如下：
RS485 设备驱动：

// Created by Wuzf on 2026.01.27
// Description: RS485 device driver for ultrasonic sensors
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
static int rs485_probe(struct platform_device *pdev) {
    struct serial_device *sdev;
    struct tty_port *port;
    struct rs485_device *dev;
    int ret;
    
    // Allocate device structure
    dev = devm_kzalloc(&pdev->dev, sizeof(struct rs485_device), GFP_KERNEL);
    if (!dev) {
        return -ENOMEM;
    }
    
    // Initialize UART
    dev->uart = platform_get_resource_bid(pdev, IORESOURCE_MEM, 0);
    if (!dev->uart) {
        dev_err(&pdev->dev, "UART resource not found\n");
        return -ENODEV;
    }
    
    dev->irq = platform_get_irq_bid(pdev, 0);
    if (dev->irq < 0) {
        dev_err(&pdev->dev, "IRQ resource not found\n");
        return dev->irq;
    }
    
    // Register UART driver
    dev->uart_driver = &rs485_uart_driver;
    ret = uart_register_driver(dev->uart_driver);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to register UART driver: %d\n", ret);
        return ret;
    }
    
    // Allocate serial device
    sdev = serial_device_alloc(dev->uart_driver);
    if (!sdev) {
        uart_unregister_driver(dev->uart_driver);
        return -ENOMEM;
    }
    
    // Set up serial device
    sdev->flags = ASYNC_SKIP_TEST;
    sdev->uart = &dev->uart_state;
    sdev->uart->type = UART_RS485;
    sdev->uart->line = pdev->id;
    sdev->uart->reg_shift = 0;
    sdev->uart->iotype = UPIO_MEM;
    sdev->uart->mapbase = dev->uart->start;
    sdev->uart->irq = dev->irq;
    
    // Initialize port
    port = &dev->port;
    tty_port_init(port);
    port->ops = &rs485_port_ops;
    port->line = pdev->id;
    
    // Register tty device
    ret = tty_register_device(dev->uart_driver->tty_driver, pdev->id, &pdev->dev);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to register tty device: %d\n", ret);
        serial_device_free(sdev);
        uart_unregister_driver(dev->uart_driver);
        return ret;
    }
    
    // Set up GPIO for RS485 control
    dev->rs485_en_gpio = platform_get_resource_bid(pdev, IORESOURCE_GPIO, 0);
    if (dev->rs485_en_gpio) {
        ret = devm_gpio_request_one(&pdev->dev, dev->rs485_en_gpio, 
                                    GPIOF_OUT_INIT_LOW, "rs485_en");
        if (ret < 0) {
            dev_err(&pdev->dev, "Failed to request RS485 enable GPIO: %d\n", ret);
        }
    }
    
    platform_set_drvdata(pdev, dev);
    
    return 0;
}
static const struct of_device_id rs485_of_match[] = {
    { .compatible = "rockchip,rs485" },
    { },
};
MODULE_DEVICE_TABLE(of, rs485_of_match);
static struct platform_driver rs485_driver = {
    .probe = rs485_probe,
    .driver = {
        .name = "rs485",
        .of_match_table = rs485_of_match,
    },
};
module_platform_driver(rs485_driver);
MODULE_DESCRIPTION("RS485 Device Driver for Ultrasonic Sensors");
MODULE_AUTHOR("Wuzf");
MODULE_LICENSE("GPL");
3.5 电机控制 CAN 驱动
基于 RK3588 的 CAN 驱动，电机控制的 CAN 通信实现如下：
电机控制 CAN 协议：

// Created by Wuzf on 2026.01.28
// Description: Motor control CAN protocol implementation
class MotorController {
public:
    MotorController(int can_id, int motor_id) : can_id_(can_id), motor_id_(motor_id) {
        // Initialize CAN frame structures
        memset(&tx_frame_, 0, sizeof(tx_frame_));
        memset(&rx_frame_, 0, sizeof(rx_frame_));
        
        tx_frame_.can_id = CAN_ID_STD(motor_id_);
        tx_frame_.can_dlc = 8;
        
        rx_frame_.can_id = CAN_ID_STD(motor_id_);
        rx_frame_.can_mask = CAN_SFF_MASK;
    }
    
    int set_speed(int16_t speed) {
        // Set motor speed command
        tx_frame_.data[0] = 0x01;  // Command: set speed
        tx_frame_.data[1] = (speed >> 8) & 0xFF;
        tx_frame_.data[2] = speed & 0xFF;
        
        return can_send_frame(can_id_, &tx_frame_);
    }
    
    int get_status(motor_status_t* status) {
        // Request motor status
        tx_frame_.data[0] = 0x02;  // Command: get status
        
        int ret = can_send_frame(can_id_, &tx_frame_);
        if (ret < 0) {
            return ret;
        }
        
        // Wait for response
        struct can_frame response;
        ret = can_receive_frame(can_id_, &response, 1000);
        if (ret < 0) {
            return ret;
        }
        
        // Parse status
        status->speed = (int16_t)(response.data[1] << 8 | response.data[2]);
        status->temperature = response.data[3];
        status->voltage = response.data[4] + (response.data[5] << 8);
        status->current = response.data[6] + (response.data[7] << 8);
        
        return 0;
    }
    
    int set_direction(int8_t direction) {
        // Set motor direction
        tx_frame_.data[0] = 0x03;  // Command: set direction
        tx_frame_.data[1] = direction;
        
        return can_send_frame(can_id_, &tx_frame_);
    }
    
private:
    int can_id_;
    int motor_id_;
    struct can_frame tx_frame_;
    struct can_frame rx_frame_;
};
// 行走电机控制类
class WheelMotor : public MotorController {
public:
    WheelMotor(int can_id, int motor_id, double gear_ratio) : 
        MotorController(can_id, motor_id), gear_ratio_(gear_ratio) {}
    
    int set_wheel_speed(double speed_mps) {
        // Convert m/s to motor RPM
        double rpm = speed_mps * 60.0 / (2.0 * M_PI * wheel_radius_m_);
        rpm /= gear_ratio_;
        
        int16_t speed_cmd = static_cast<int16_t>(rpm * 100);  // Convert to fixed point
        return set_speed(speed_cmd);
    }
    
private:
    double gear_ratio_;
    static constexpr double wheel_radius_m_ = 0.065;  // 65mm wheel radius
};
// 刀盘电机控制类
class BladeMotor : public MotorController {
public:
    BladeMotor(int can_id, int motor_id) : MotorController(can_id, motor_id) {}
    
    int set_blade_speed(int16_t speed_rpm) {
        int16_t speed_cmd = speed_rpm * 100;  // Convert to fixed point
        return set_speed(speed_cmd);
    }
    
    int enable_blade(bool enable) {
        tx_frame_.data[0] = 0x04;  // Command: enable/disable
        tx_frame_.data[1] = enable ? 0x01 : 0x00;
        
        return can_send_frame(can_id_, &tx_frame_);
    }
};
4. ROS 工程搭建（lawnwomer_ws）
4.1 工程目录结构
基于 RK3588+Ubuntu20.04+ROS1 的 catkin 工作空间 lawnwomer_ws 的完整目录结构如下：

lawnwomer_ws/
├── build/
├── devel/
└── src/
    ├── CMakeLists.txt
    ├── lawnwomer_hw/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/
    │   │   └── lawnwomer_hw/
    │   │       ├── rk3588_hw.h
    │   │       └── motor_control.h
    │   └── src/
    │       ├── rk3588_hw_node.cpp
    │       └── motor_control_node.cpp
    ├── gemini335_driver/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   └── gemini335.launch
    │   ├── src/
    │   │   └── gemini335_node.cpp
    │   └── include/
    │       └── gemini335_driver/
    │           └── gemini335.h
    ├── hesai_jt128_driver/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   └── hesai_jt128.launch
    │   ├── src/
    │   │   └── hesai_jt128_node.cpp
    │   └── include/
    │       └── hesai_jt128_driver/
    │           └── hesai_jt128.h
    ├── rtk_um982_driver/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   └── rtk_um982.launch
    │   ├── src/
    │   │   └── rtk_um982_node.cpp
    │   └── include/
    │       └── rtk_um982_driver/
    │           └── rtk_um982.h
    ├── ultrasonic_driver/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   └── ultrasonic.launch
    │   ├── src/
    │   │   └── ultrasonic_node.cpp
    │   └── include/
    │       └── ultrasonic_driver/
    │           └── ultrasonic.h
    ├── motor_control/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   └── motor_control.launch
    │   ├── src/
    │   │   └── motor_control_node.cpp
    │   └── include/
    │       └── motor_control/
    │           └── motor_controller.h
    └── mower_logic/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── mower_logic.launch
        ├── src/
        │   └── mower_logic.cpp
        ├── include/
        │   └── mower_logic/
        │       └── mower_logic.h
        └── params/
            └── mower_logic_params.yaml
4.2 CMakeLists.txt 配置
顶层 CMakeLists.txt：

# Created by Wuzf on 2026.01.29
# Description: Top-level CMakeLists.txt for lawnwomer_ws
cmake_minimum_required(VERSION 2.8.3)
project(lawnwomer_ws)
# Set CMAKE_MODULE_PATH for catkin
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    actionlib
    dynamic_reconfigure
)
# Set C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Add definitions for ROS
add_definitions(-DROS_BUILD)
# Build all packages in src directory
catkin_workspace()
# Include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
)
# Add subdirectories
add_subdirectory(lawnwomer_hw)
add_subdirectory(gemini335_driver)
add_subdirectory(hesai_jt128_driver)
add_subdirectory(rtk_um982_driver)
add_subdirectory(ultrasonic_driver)
add_subdirectory(motor_control)
add_subdirectory(mower_logic)
# Generate compile_commands.json for IDE support
add_custom_target(compile_commands
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
        ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
    COMMENT "Generate compile_commands.json for IDE"
)
4.3 package.xml 配置
lawnwomer_hw/package.xml：

<?xml version="1.0"?>
<package format="2">
  <name>lawnwomer_hw</name>
  <version>0.0.1</version>
  <description>Hardware drivers for RK3588-based lawn mower</description>
  <maintainer email="wuzf@example.com">Wuzf</maintainer>
  <license>GPL-3.0</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>tf2_ros</build_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>dynamic_reconfigure</build_depend>
  <build_depend>can_msgs</build_depend>
  <build_depend>serial</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>sensor_msgs</run_depend>
  <run_depend>geometry_msgs</run_depend>
  <run_depend>nav_msgs</run_depend>
  <run_depend>tf2_ros</run_depend>
  <run_depend>actionlib</run_depend>
  <run_depend>dynamic_reconfigure</run_depend>
  <run_depend>can_msgs</run_depend>
  <run_depend>serial</run_depend>
  <export>
    <cpp_typeupport>ament_cmake_export_dependencies</cpp_typeupport>
  </export>
</package>
4.4 ROS 节点通信架构
节点间通信架构图：

         +------------------+
         |      rviz        |
         +------------------+
               |
               | tf
               v
+------------------+      +------------------+
|    mower_logic   |      |  tf2 broadcaster  |
+------------------+      +------------------+
        |                  |
        |                  |
        v                  v
+------------------+      +------------------+
|   motor_control  |      |   odometry       |
+------------------+      +------------------+
        |                  |
        | cmd_vel          |
        v                  v
+------------------+      +------------------+
|      CAN bus     |      |   laser_scan     |
+------------------+      +------------------+
        |                  |
        |                  |
        v                  v
+------------------+      +------------------+
|     Motors       |      |  Hesai JT128     |
+------------------+      +------------------+
        |                  |
        |                  |
        v                  v
+------------------+      +------------------+
|    encoders      |      |    pointcloud    |
+------------------+      +------------------+
        |                  |
        |                  |
        +------------------+
                  |
                  |
                  v
            +------------------+
            |   base_link      |
            +------------------+
4.5 核心节点实现
4.5.1 rk3588_hw_node.cpp

// Created by Wuzf on 2026.01.30
// Description: RK3588 hardware abstraction layer node
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/BatteryState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
class RK3588Hardware {
public:
    RK3588Hardware(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : 
        nh_(nh), private_nh_(private_nh), is_initialized_(false) {
        
        // Read parameters
        private_nh_.param<double>("battery_full_voltage", battery_full_voltage_, 52.0);
        private_nh_.param<double>("battery_empty_voltage", battery_empty_voltage_, 42.0);
        private_nh_.param<double>("battery_critical_voltage", battery_critical_voltage_, 45.0);
        
        // Create publishers
        battery_pub_ = nh_.advertise<sensor_msgs/BatteryState>("battery_state", 10);
        diagnostic_pub_ = nh_.advertise<diagnostic_msgs/DiagnosticArray>("diagnostics", 10);
        
        // Create services
        shutdown_service_ = nh_.advertiseService("shutdown", &RK3588Hardware::shutdownCallback, this);
        reboot_service_ = nh_.advertiseService("reboot", &RK3588Hardware::rebootCallback, this);
        
        // Initialize hardware
        if (initHardware()) {
            is_initialized_ = true;
            ROS_INFO("RK3588 hardware initialized successfully");
        } else {
            ROS_ERROR("Failed to initialize RK3588 hardware");
        }
        
        // Create update timer (1Hz)
        update_timer_ = nh_.createTimer(ros::Duration(1.0), &RK3588Hardware::updateCallback, this);
    }
    
    ~RK3588Hardware() {
        if (is_initialized_) {
            shutdownHardware();
        }
    }
    
private:
    ros::NodeHandle nh_, private_nh_;
    bool is_initialized_;
    ros::Timer update_timer_;
    ros::Publisher battery_pub_;
    ros::Publisher diagnostic_pub_;
    ros::ServiceServer shutdown_service_;
    ros::ServiceServer reboot_service_;
    
    double battery_full_voltage_;
    double battery_empty_voltage_;
    double battery_critical_voltage_;
    
    bool initHardware() {
        // Initialize battery monitor
        if (battery_monitor_init() != 0) {
            return false;
        }
        
        // Initialize temperature sensors
        if (temp_sensor_init() != 0) {
            return false;
        }
        
        // Initialize fan control
        if (fan_control_init() != 0) {
            return false;
        }
        
        return true;
    }
    
    void shutdownHardware() {
        battery_monitor_deinit();
        temp_sensor_deinit();
        fan_control_deinit();
    }
    
    void updateCallback(const ros::TimerEvent& event) {
        if (!is_initialized_) {
            return;
        }
        
        // Update battery state
        sensor_msgs/BatteryState battery_msg;
        battery_msg.header.stamp = ros::Time::now();
        battery_msg.header.frame_id = "base_link";
        
        // Read battery voltage
        double battery_voltage = battery_monitor_read_voltage();
        battery_msg.voltage = battery_voltage;
        
        // Calculate battery percentage
        double battery_percent = (battery_voltage - battery_empty_voltage_) / 
                                (battery_full_voltage_ - battery_empty_voltage_);
        battery_percent = std::max(0.0, std::min(1.0, battery_percent));
        battery_msg.percentage = battery_percent;
        
        // Set battery status
        if (battery_voltage < battery_critical_voltage_) {
            battery_msg.status = sensor_msgs/BatteryState::BATTERY_LOW;
        } else if (battery_voltage < battery_empty_voltage_) {
            battery_msg.status = sensor_msgs/BatteryState::BATTERY_EMPTY;
        } else {
            battery_msg.status = sensor_msgs/BatteryState::BATTERY_NORMAL;
        }
        
        battery_pub_.publish(battery_msg);
        
        // Update diagnostics
        diagnostic_msgs/DiagnosticArray diag_msg;
        diag_msg.header.stamp = ros::Time::now();
        
        // Add battery diagnostic
        diagnostic_msgs/DiagnosticStatus status;
        status.level = battery_msg.status;
        status.name = "Battery";
        status.message = "Battery voltage: " + std::to_string(battery_voltage) + "V";
        status.values.push_back(diagnostic_msgsKeyValue("voltage", std::to_string(battery_voltage) + "V"));
        status.values.push_back(diagnostic_msgsKeyValue("percentage", std::to_string(battery_percent * 100) + "%"));
        diag_msg.status.push_back(status);
        
        // Add temperature diagnostics
        double temp_cpu = temp_sensor_read("cpu_temp");
        double temp_gpu = temp_sensor_read("gpu_temp");
        
        status.level = diagnostic_msgs/DiagnosticStatus::OK;
        status.name = "CPU Temperature";
        status.message = "CPU temperature: " + std::to_string(temp_cpu) + "°C";
        status.values.push_back(diagnostic_msgsKeyValue("temperature", std::to_string(temp_cpu) + "°C"));
        diag_msg.status.push_back(status);
        
        status.name = "GPU Temperature";
        status.message = "GPU temperature: " + std::to_string(temp_gpu) + "°C";
        status.values.push_back(diagnostic_msgsKeyValue("temperature", std::to_string(temp_gpu) + "°C"));
        diag_msg.status.push_back(status);
        
        diagnostic_pub_.publish(diag_msg);
        
        // Control fan based on temperature
        double avg_temp = (temp_cpu + temp_gpu) / 2.0;
        if (avg_temp > 60.0) {
            fan_control_set_speed(100);  // Full speed
        } else if (avg_temp > 45.0) {
            fan_control_set_speed(50);   // Half speed
        } else {
            fan_control_set_speed(20);   // Low speed
        }
    }
    
    bool shutdownCallback(std_msgs::Empty::Request& req, std_msgs::Empty::Response& res) {
        ROS_WARN("Shutting down system...");
        system("sudo shutdown now");
        return true;
    }
    
    bool rebootCallback(std_msgs::Empty::Request& req, std_msgs::Empty::Response& res) {
        ROS_WARN("Rebooting system...");
        system("sudo reboot now");
        return true;
    }
    
    // Hardware abstraction functions
    int battery_monitor_init() {
        // Implementation details for battery monitor initialization
        return 0;
    }
    
    int battery_monitor_deinit() {
        return 0;
    }
    
    double battery_monitor_read_voltage() {
        // Read battery voltage from hardware
        return 48.5;  // Placeholder value
    }
    
    int temp_sensor_init() {
        return 0;
    }
    
    int temp_sensor_deinit() {
        return 0;
    }
    
    double temp_sensor_read(const std::string& sensor_name) {
        // Read temperature from specified sensor
        if (sensor_name == "cpu_temp") {
            return 45.0;  // Placeholder value
        } else if (sensor_name == "gpu_temp") {
            return 50.0;  // Placeholder value
        }
        return 0.0;
    }
    
    int fan_control_init() {
        return 0;
    }
    
    int fan_control_deinit() {
        return 0;
    }
    
    int fan_control_set_speed(int speed_percent) {
        // Set fan speed (0-100%)
        return 0;
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "rk3588_hw_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    RK3588Hardware hardware(nh, private_nh);
    
    ros::spin();
    
    return 0;
}
4.5.2 motor_control_node.cpp

// Created by Wuzf on 2026.01.31
// Description: Motor control node for lawn mower
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
class MotorControl {
public:
    MotorControl(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : 
        nh_(nh), private_nh_(private_nh), left_motor_(CAN_ID, 1), right_motor_(CAN_ID, 2) {
        
        // Read parameters
        private_nh_.param<double>("wheel_base", wheel_base_, 0.56);  // meters
        private_nh_.param<double>("wheel_radius", wheel_radius_, 0.065);  // meters
        private_nh_.param<int>("encoder_ticks_per_rev", encoder_ticks_per_rev_, 1024);
        private_nh_.param<double>("max_speed_mps", max_speed_mps_, 1.5);
        private_nh_.param<double>("max_angular_speed_rps", max_angular_speed_rps_, 1.0);
        
        // Create subscribers
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &MotorControl::cmdVelCallback, this);
        
        // Create publishers
        odom_pub_ = nh_.advertise<nav_msgs/Odometry>("odom", 10);
        left_wheel_pub_ = nh_.advertise<std_msgs/Float64>("left_wheel_speed", 10);
        right_wheel_pub_ = nh_.advertise<std_msgs/Float64>("right_wheel_speed", 10);
        
        // Create services
        reset_odom_service_ = nh_.advertiseService("reset_odometry", &MotorControl::resetOdometryCallback, this);
        
        // Initialize odometry
        resetOdometry();
        
        // Create update timer (50Hz)
        update_timer_ = nh_.createTimer(ros::Duration(0.02), &MotorControl::updateCallback, this);
    }
    
    ~MotorControl() {
        stopMotors();
    }
    
private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher left_wheel_pub_, right_wheel_pub_;
    ros::ServiceServer reset_odom_service_;
    ros::Timer update_timer_;
    
    WheelMotor left_motor_;
    WheelMotor right_motor_;
    
    double wheel_base_;
    double wheel_radius_;
    int encoder_ticks_per_rev_;
    double max_speed_mps_;
    double max_angular_speed_rps_;
    
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::TransformStamped odom_trans_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Previous encoder values
    int32_t left_encoder_prev_ = 0;
    int32_t right_encoder_prev_ = 0;
    ros::Time last_update_time_;
    
    void cmdVelCallback(const geometry_msgs::Twist& twist) {
        // Convert twist to wheel speeds
        double linear_x = twist.linear.x;
        double angular_z = twist.angular.z;
        
        // Calculate wheel speeds (m/s)
        double left_speed = linear_x - angular_z * wheel_base_ / 2.0;
        double right_speed = linear_x + angular_z * wheel_base_ / 2.0;
        
        // Apply limits
        left_speed = std::max(-max_speed_mps_, std::min(max_speed_mps_, left_speed));
        right_speed = std::max(-max_speed_mps_, std::min(max_speed_mps_, right_speed));
        
        // Calculate angular speeds (rad/s)
        double left_angular = left_speed / wheel_radius_;
        double right_angular = right_speed / wheel_radius_;
        
        // Apply angular speed limits
        left_angular = std::max(-max_angular_speed_rps_ * 2 * M_PI, 
                               std::min(max_angular_speed_rps_ * 2 * M_PI, left_angular));
        right_angular = std::max(-max_angular_speed_rps_ * 2 * M_PI, 
                                std::min(max_angular_speed_rps_ * 2 * M_PI, right_angular));
        
        // Set motor speeds
        left_motor_.set_wheel_speed(left_speed);
        right_motor_.set_wheel_speed(right_speed);
        
        // Publish wheel speeds
        std_msgs/Float64 left_msg, right_msg;
        left_msg.data = left_angular;
        right_msg.data = right_angular;
        left_wheel_pub_.publish(left_msg);
        right_wheel_pub_.publish(right_msg);
    }
    
    void updateCallback(const ros::TimerEvent& event) {
        if (last_update_time_ == ros::Time(0)) {
            last_update_time_ = event.current_real;
            return;
        }
        
        // Read encoder values
        int32_t left_encoder = readEncoder(1);  // Left encoder
        int32_t right_encoder = readEncoder(2);  // Right encoder
        
        // Calculate delta
        int32_t left_delta = left_encoder - left_encoder_prev_;
        int32_t right_delta = right_encoder - right_encoder_prev_;
        
        // Calculate distance traveled (meters)
        double left_distance = left_delta * 2 * M_PI * wheel_radius_ / encoder_ticks_per_rev_;
        double right_distance = right_delta * 2 * M_PI * wheel_radius_ / encoder_ticks_per_rev_;
        double distance = (left_distance + right_distance) / 2.0;
        
        // Calculate rotation (radians)
        double rotation = (right_distance - left_distance) / wheel_base_;
        
        // Calculate velocities
        ros::Duration dt = event.current_real - last_update_time_;
        double linear_velocity = distance / dt.toSec();
        double angular_velocity = rotation / dt.toSec();
        
        // Update odometry
        if (distance != 0 || rotation != 0) {
            // Calculate new pose
            double x = odom_msg_.pose.pose.position.x;
            double y = odom_msg_.pose.pose.position.y;
            double theta = tf2::getYaw(odom_msg_.pose.pose.orientation);
            
            double cos_theta = cos(theta);
            double sin_theta = sin(theta);
            
            // Calculate new position
            double dx = distance * cos_theta;
            double dy = distance * sin_theta;
            x += dx;
            y += dy;
            
            // Calculate new orientation
            theta += rotation;
            
            // Update pose
            odom_msg_.pose.pose.position.x = x;
            odom_msg_.pose.pose.position.y = y;
            odom_msg_.pose.pose.orientation = tf2::toQuaternion(theta);
            
            // Update twist
            odom_msg_.twist.twist.linear.x = linear_velocity;
            odom_msg_.twist.twist.angular.z = angular_velocity;
        }
        
        // Update header and publish
        odom_msg_.header.stamp = event.current_real;
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_link";
        
        odom_pub_.publish(odom_msg_);
        
        // Broadcast TF
        odom_trans_.header = odom_msg_.header;
        odom_trans_.child_frame_id = odom_msg_.child_frame_id;
        odom_trans_.transform.translation.x = odom_msg_.pose.pose.position.x;
        odom_trans_.transform.translation.y = odom_msg_.pose.pose.position.y;
        odom_trans_.transform.translation.z = 0.0;
        odom_trans_.transform.rotation = odom_msg_.pose.pose.orientation;
        
        tf_broadcaster_.sendTransform(odom_trans_);
        
        // Save previous values
        left_encoder_prev_ = left_encoder;
        right_encoder_prev_ = right_encoder;
        last_update_time_ = event.current_real;
    }
    
    bool resetOdometryCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        resetOdometry();
        return true;
    }
    
    void resetOdometry() {
        // Reset odometry to origin
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_link";
        odom_msg_.pose.pose.position.x = 0.0;
        odom_msg_.pose.pose.position.y = 0.0;
        odom_msg_.pose.pose.position.z = 0.0;
        odom_msg_.pose.pose.orientation = tf2::toQuaternion(0.0);
        odom_msg_.twist.twist.linear.x = 0.0;
        odom_msg_.twist.twist.angular.z = 0.0;
        
        // Reset encoder values
        left_encoder_prev_ = readEncoder(1);
        right_encoder_prev_ = readEncoder(2);
        last_update_time_ = ros::Time::now();
        
        stopMotors();
    }
    
    void stopMotors() {
        left_motor_.set_wheel_speed(0.0);
        right_motor_.set_wheel_speed(0.0);
    }
    
    int32_t readEncoder(int motor_id) {
        // Read encoder value from motor controller
        return 0;  // Placeholder implementation
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    MotorControl control(nh, private_nh);
    
    ros::spin();
    
    return 0;
}
5. 调试脚本与数据解析（Python）
5.1 传感器数据调试脚本
sensor_debug.py：

# Created by Wuzf on 2026.02.01
# Description: Sensor data debugging script for lawn mower
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import PointCloud2, Image, NavSatFix, LaserScan
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
class SensorDebugger:
    def __init__(self):
        rospy.init_node('sensor_debugger', anonymous=True)
        
        # Subscribers
        self.pc_sub = rospy.Subscriber('/hesai_jt128/pointcloud', PointCloud2, self.pointcloud_callback)
        self.image_sub = rospy.Subscriber('/gemini335/color/image_raw', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/odom', NavSatFix, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Initialize plot
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        self.fig.suptitle('Lawn Mower Sensor Debugger')
        
        # Point cloud plot
        self.ax1.set_xlim(-5, 5)
        self.ax1.set_ylim(-5, 5)
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.set_title('LiDAR Point Cloud')
        self.ax1.grid(True, alpha=0.3)
        self.pointcloud_plot, = self.ax1.plot([], [], 'g.', markersize=2)
        
        # Image plot
        self.ax2.set_title('Gemini335 Color Image')
        self.image_plot = self.ax2.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
        self.ax2.axis('off')
        
        # Laser scan plot
        self.ax3.set_xlim(0, 2 * np.pi)
        self.ax3.set_ylim(0, 5)
        self.ax3.set_xlabel('Angle (rad)')
        self.ax3.set_ylabel('Range (m)')
        self.ax3.set_title('Laser Scan')
        self.ax3.grid(True, alpha=0.3)
        self.scan_plot, = self.ax3.plot([], [], 'r-', linewidth=2)
        
        # Velocity plot
        self.ax4.set_xlim(0, 100)
        self.ax4.set_ylim(-2, 2)
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Velocity (m/s)')
        self.ax4.set_title('Command Velocity')
        self.ax4.grid(True, alpha=0.3)
        self.time_data = []
        self.linear_data = []
        self.angular_data = []
        self.linear_plot, = self.ax4.plot([], [], 'b-', label='Linear')
        self.angular_plot, = self.ax4.plot([], [], 'g-', label='Angular')
        self.ax4.legend()
        
        # Initialize bridge
        self.bridge = CvBridge()
        
        # Start animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.tight_layout()
        plt.show()
        
    def pointcloud_callback(self, msg):
        # Convert point cloud to numpy array
        points = list(pc2.read_points(msg, skip_nans=True))
        x = [p[0] for p in points]
        y = [p[1] for p in points]
        z = [p[2] for p in points]
        
        # Update point cloud plot
        self.pointcloud_plot.set_data(x, y)
        
    def image_callback(self, msg):
        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_plot.set_data(cv_image)
        except Exception as e:
            rospy.logerr(e)
        
    def odom_callback(self, msg):
        # Update odometry information
        pass  # Add odometry visualization if needed
        
    def scan_callback(self, msg):
        # Update laser scan plot
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        self.scan_plot.set_data(angles, msg.ranges)
        
    def cmd_vel_callback(self, msg):
        # Record velocity data
        self.time_data.append(rospy.get_time())
        self.linear_data.append(msg.linear.x)
        self.angular_data.append(msg.angular.z)
        
        # Keep only last 100 points
        if len(self.time_data) > 100:
            self.time_data.pop(0)
            self.linear_data.pop(0)
            self.angular_data.pop(0)
        
        # Update velocity plot
        self.linear_plot.set_data(self.time_data, self.linear_data)
        self.angular_plot.set_data(self.time_data, self.angular_data)
        
    def update_plot(self, frame):
        # Redraw the plot
        self.fig.canvas.draw()
        return self.pointcloud_plot, self.image_plot, self.scan_plot, self.linear_plot, self.angular_plot
if __name__ == '__main__':
    debugger = SensorDebugger()
    rospy.spin()
5.2 系统状态监控脚本
system_monitor.py：

# Created by Wuzf on 2026.02.02
# Description: System state monitoring script
import rospy
import psutil
import time
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Empty
class SystemMonitor:
    def __init__(self):
        rospy.init_node('system_monitor', anonymous=True)
        
        # Publishers
        self.battery_pub = rospy.Publisher('battery_state', BatteryState, queue_size=10)
        self.diagnostic_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=10)
        
        # Services
        self.shutdown_srv = rospy.ServiceProxy('shutdown', Empty)
        self.reboot_srv = rospy.ServiceProxy('reboot', Empty)
        
        # Initialize battery message
        self.battery_msg = BatteryState()
        self.battery_msg.header.frame_id = 'base_link'
        
        # Create monitor timer (1Hz)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.monitor_callback)
        
        # Create emergency stop timer (check every 5 seconds)
        self.emergency_timer = rospy.Timer(rospy.Duration(5.0), self.check_emergency)
        
        # Initialize emergency state
        self.emergency_state = False
        
    def monitor_callback(self, event):
        # Update system information
        self.update_system_info()
        self.update_power_info()
        self.update_temperature()
        
        # Publish messages
        self.battery_pub.publish(self.battery_msg)
        self.publish_diagnostics()
        
    def update_system_info(self):
        # Get CPU usage
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent
        
        # Update battery message
        self.battery_msg.header.stamp = rospy.Time.now()
        self.battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        
        # Add system diagnostics
        diag_status = self.create_diagnostic_status('System', 'System health monitoring')
        diag_status.values.append(self.create_diagnostic_value('CPU Usage', f'{cpu_percent}%'))
        diag_status.values.append(self.create_diagnostic_value('Memory Usage', f'{memory_percent}%'))
        
    def update_power_info(self):
        # Read battery voltage (simulated)
        battery_voltage = self.read_battery_voltage()
        
        # Calculate battery percentage (simulated)
        battery_percent = (battery_voltage - 42.0) / (52.0 - 42.0)
        battery_percent = max(0.0, min(1.0, battery_percent))
        
        # Update battery message
        self.battery_msg.voltage = battery_voltage
        self.battery_msg.percentage = battery_percent
        
        # Set battery status
        if battery_voltage < 45.0:
            self.battery_msg.status = BatteryState.BATTERY_LOW
        elif battery_voltage < 42.0:
            self.battery_msg.status = BatteryState.BATTERY_EMPTY
        else:
            self.battery_msg.status = BatteryState.BATTERY_NORMAL
            
    def update_temperature(self):
        # Read temperatures (simulated)
        cpu_temp = self.read_cpu_temperature()
        gpu_temp = self.read_gpu_temperature()
        
        # Add temperature diagnostics
        diag_status = self.create_diagnostic_status('Temperature', 'CPU and GPU temperature')
        diag_status.values.append(self.create_diagnostic_value('CPU Temperature', f'{cpu_temp}°C'))
        diag_status.values.append(self.create_diagnostic_value('GPU Temperature', f'{gpu_temp}°C'))
        
        # Check for overheating
        if cpu_temp > 85 or gpu_temp > 85:
            self.set_emergency(True, 'Temperature overheat')
        
    def check_emergency(self, event):
        # Check for emergency conditions
        if self.emergency_state:
            # Emergency stop
            self.trigger_emergency_stop()
            self.emergency_state = False
            
    def set_emergency(self, state, reason):
        self.emergency_state = state
        rospy.logwarn(f'Emergency state: {reason}')
        
    def trigger_emergency_stop(self):
        # Send emergency stop command
        rospy.logwarn('Triggering emergency stop...')
        
        # Stop all motors
        # (Add motor stop command implementation here)
        
        # Shutdown system if necessary
        if self.battery_msg.voltage < 42.0:
            rospy.logwarn('Battery critically low. Initiating system shutdown.')
            try:
                self.shutdown_srv()
            except rospy.ServiceException as e:
                rospy.logerr(f'Shutdown service failed: {e}')
        
    def read_battery_voltage(self):
        # Simulate battery voltage reading
        return 48.5  # Placeholder value
        
    def read_cpu_temperature(self):
        # Simulate CPU temperature
        return 55.0  # Placeholder value
        
    def read_gpu_temperature(self):
        # Simulate GPU temperature
        return 60.0  # Placeholder value
        
    def create_diagnostic_status(self, name, message):
        from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
        
        status = DiagnosticStatus()
        status.name = name
        status.message = message
        status.level = DiagnosticStatus.OK
        
        return status
        
    def create_diagnostic_value(self, key, value):
        from diagnostic_msgs.msg import KeyValue
        
        val = KeyValue()
        val.key = key
        val.value = value
        
        return val
        
    def publish_diagnostics(self):
        from diagnostic_msgs.msg import DiagnosticArray
        
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()
        diag_array.header.frame_id = 'base_link'
        
        # Add battery diagnostic
        battery_status = self.create_diagnostic_status('Battery', 'Battery status')
        battery_status.level = self.battery_msg.status
        battery_status.values.append(self.create_diagnostic_value('Voltage', f'{self.battery_msg.voltage}V'))
        battery_status.values.append(self.create_diagnostic_value('Percentage', f'{self.battery_msg.percentage*100:.1f}%'))
        diag_array.status.append(battery_status)
        
        # Add system status
        system_status = self.create_diagnostic_status('System', 'System status')
        system_status.values.append(self.create_diagnostic_value('CPU Usage', f'{ps
（注：文档部分内容可能由 AI 生成）