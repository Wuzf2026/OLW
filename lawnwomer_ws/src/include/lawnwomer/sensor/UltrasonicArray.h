#ifndef LAWNMOWER_SENSOR_ULTRASONICARRAY_H
#define LAWNMOWER_SENSOR_ULTRASONICARRAY_H
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <vector>
#include <serial/serial.h>
namespace lawnmower {
class UltrasonicArray {
private:
    std::vector<ros::Publisher> range_pubs_;
    serial::Serial serial_port_;
    
    bool is_initialized_;
    std::mutex mtx_;
    
public:
    UltrasonicArray();
    ~UltrasonicArray();
    
    bool initialize(const std::string& port, int baudrate, int num_sensors);
    void start();
    void stop();
    void update();
    
    bool isInitialized() const { return is_initialized_; }
    
    // 传感器参数
    int num_sensors;
    double max_range;
    double min_range;
    double field_of_view;
};
} // namespace lawnmower
#endif // LAWNMOWER_SENSOR_ULTRASONICARRAY_H
