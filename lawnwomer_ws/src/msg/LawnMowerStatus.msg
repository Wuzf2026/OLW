# 割草机器人整体状态消息
Header header
# 系统状态
uint8 system_state
uint8 SYSTEM_STATE_IDLE = 0
uint8 SYSTEM_STATE_MOWING = 1
uint8 SYSTEM_STATE_NAVIGATING = 2
uint8 SYSTEM_STATE_EMERGENCY = 3
# 电池状态
float32 battery_percentage
float32 battery_voltage
float32 battery_current
# 位置信息
geometry_msgs/PoseStamped current_pose
geometry_msgs/PoseStamped target_pose
# 传感器状态
bool laser_scanner_ready
bool depth_camera_ready
bool rtk_gps_fixed
bool ultrasonic_array_ready
# 电机状态
MotorStatus[] motors
# 安全状态
bool emergency_stop_active
bool obstacle_detected
bool battery_low
bool motor_fault
# 割草统计
float32 mowed_area  # 平方米
float32 total_mowing_time  # 秒
float32 remaining_mowing_time  # 秒
