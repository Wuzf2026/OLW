#ifndef LAWNMOWER_SENSOR_DEPTHCAMERA_H
#define LAWNMOWER_SENSOR_DEPTHCAMERA_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo>
#include <image_transport/image_transport.h>
#include "orbbec/ObSensor.h"
namespace lawnmower {
class DepthCamera {
private:
    image_transport::ImageTransport it_;
    image_transport::Publisher color_pub_;
    image_transport::Publisher depth_pub_;
    ros::Publisher camera_info_pub_;
    
    std::unique_ptr<ObSensor> ob_sensor_;
    
    bool is_initialized_;
    std::mutex mtx_;
    
public:
    DepthCamera(ros::NodeHandle& nh);
    ~DepthCamera();
    
    bool initialize();
    void start();
    void stop();
    void update();
    
    bool isInitialized() const { return is_initialized_; }
    
    // 相机参数
    int width;
    int height;
    double fps;
    std::string serial_number;
};
} // namespace lawnmower
#endif // LAWNMOWER_SENSOR_DEPTHCAMERA_H
