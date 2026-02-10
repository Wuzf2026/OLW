#include <ros/ros.h>
#include <lawnwomer/SensorConfig.h>
#include <dynamic_reconfigure/server.h>
#include <lawnwomer/SensorConfig.h>
class SensorConfigServer {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer config_service_;
    
    // 传感器配置参数（示例）
    std::map<std::string, std::string> laser_config_;
    std::map<std::string, std::string> camera_config_;
    std::map<std::string, std::string> gps_config_;
    
public:
    SensorConfigServer() {
        config_service_ = nh_.advertiseService("sensor_config", &SensorConfigServer::configCallback, this);
        
        // 初始化默认配置
        laser_config_["device_ip"] = "192.168.1.201";
        laser_config_["udp_port"] = "2368";
        laser_config_["ptc_port"] = "9347";
        
        camera_config_["width"] = "1280";
        camera_config_["height"] = "800";
        camera_config_["fps"] = "30";
        
        gps_config_["port"] = "/dev/ttyUSB0";
        gps_config_["baudrate"] = "115200";
    }
    
    bool configCallback(lawnwomer::SensorConfig::Request &req, 
                       lawnwomer::SensorConfig::Response &res) {
        try {
            if (req.sensor_type == "laser") {
                if (laser_config_.find(req.parameter) != laser_config_.end()) {
                    laser_config_[req.parameter] = req.value;
                    res.success = true;
                    res.message = "Laser configuration updated successfully";
                } else {
                    res.success = false;
                    res.message = "Invalid laser parameter: " + req.parameter;
                }
            } else if (req.sensor_type == "camera") {
                if (camera_config_.find(req.parameter) != camera_config_.end()) {
                    camera_config_[req.parameter] = req.value;
                    res.success = true;
                    res.message = "Camera configuration updated successfully";
                } else {
                    res.success = false;
                    res.message = "Invalid camera parameter: " + req.parameter;
                }
            } else if (req.sensor_type == "gps") {
                if (gps_config_.find(req.parameter) != gps_config_.end()) {
                    gps_config_[req.parameter] = req.value;
                    res.success = true;
                    res.message = "GPS configuration updated successfully";
                } else {
                    res.success = false;
                    res.message = "Invalid GPS parameter: " + req.parameter;
                }
            } else {
                res.success = false;
                res.message = "Unknown sensor type: " + req.sensor_type;
            }
        } catch (const std::exception& e) {
            res.success = false;
            res.message = "Configuration failed: " + std::string(e.what());
        }
        
        return true;
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_config_server");
    SensorConfigServer server;
    ros::spin();
    return 0;
}
