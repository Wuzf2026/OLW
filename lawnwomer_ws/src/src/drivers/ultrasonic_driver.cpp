#include "lawnmower/sensor/UltrasonicArray.h"
namespace lawnmower {
UltrasonicArray::UltrasonicArray() {
    is_initialized_ = false;
    num_sensors = 8;
    max_range = 5.0;
    min_range = 0.02;
    field_of_view = 0.3; // 弧度
}
UltrasonicArray::~UltrasonicArray() {
    stop();
}
bool UltrasonicArray::initialize(const std::string& port, int baudrate, int num_sensors) {
    try {
        this->num_sensors = num_sensors;
        
        // 配置串口
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
        serial_port_.setPort(port);
        serial_port_.setBaudrate(baudrate);
        serial_port_.setTimeout(timeout);
        
        // 打开串口
        serial_port_.open();
        if (!serial_port_.isOpen()) {
            ROS_ERROR("Failed to open serial port!");
            return false;
        }
        
        // 创建ROS发布者
        for (int i = 0; i < num_sensors; ++i) {
            std::string topic_name = "ultrasonic/" + std::to_string(i) + "/range";
            range_pubs_.push_back(nh_.advertise<sensor_msgs::Range>(topic_name, 10));
        }
        
        is_initialized_ = true;
        ROS_INFO("Ultrasonic array initialized successfully!");
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Ultrasonic array initialization failed: %s", e.what());
        return false;
    }
}
void UltrasonicArray::start() {
    if (is_initialized_) {
        ROS_INFO("Ultrasonic array started.");
    }
}
void UltrasonicArray::stop() {
    if (is_initialized_) {
        serial_port_.close();
        ROS_INFO("Ultrasonic array stopped.");
    }
}
void UltrasonicArray::update() {
    if (!is_initialized_) return;
    
    try {
        // 发送测距请求（示例协议）
        uint8_t request[2] = {0xAA, 0x01}; // 开始测距命令
        serial_port_.write(request, sizeof(request));
        
        // 读取响应数据
        uint8_t response[10]; // 假设每个传感器返回10字节数据
        if (serial_port_.available() >= num_sensors * 10) {
            serial_port_.read(response, num_sensors * 10);
            
            // 解析数据
            for (int i = 0; i < num_sensors; ++i) {
                // 示例解析：前2字节为距离（毫米）
                uint16_t distance_mm = (response[i*10 + 1] << 8) | response[i*10];
                double distance_m = distance_mm / 1000.0;
                
                // 发布测距数据
                sensor_msgs::Range range_msg;
                range_msg.header.stamp = ros::Time::now();
                range_msg.header.frame_id = "ultrasonic_" + std::to_string(i);
                
                range_msg.range = distance_m;
                range_msg.min_range = min_range;
                range_msg.max_range = max_range;
                range_msg.field_of_view = field_of_view;
                
                range_pubs_[i].publish(range_msg);
            }
        }
    } catch (const std::exception& e) {
        ROS_WARN("Ultrasonic update failed: %s", e.what());
    }
}
} // namespace lawnmower
