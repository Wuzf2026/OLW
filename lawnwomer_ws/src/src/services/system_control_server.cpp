#include <ros/ros.h>
#include <lawnwomer/SystemControl.h>
#include <std_msgs/Bool.h>
class SystemControlServer {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer control_service_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher start_mowing_pub_;
    
    bool is_emergency_stop_ = false;
    bool is_mowing_ = false;
    
public:
    SystemControlServer() {
        control_service_ = nh_.advertiseService("system_control", &SystemControlServer::controlCallback, this);
        emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("emergency_stop", 10);
        start_mowing_pub_ = nh_.advertise<std_msgs::Bool>("start_mowing", 10);
    }
    
    bool controlCallback(lawnwomer::SystemControl::Request &req, 
                        lawnwomer::SystemControl::Response &res) {
        try {
            switch (req.command) {
                case lawnwomer::SystemControl::Request::COMMAND_START_MOWING:
                    startMowing();
                    res.success = true;
                    res.message = "Mowing started";
                    break;
                    
                case lawnwomer::SystemControl::Request::COMMAND_STOP_MOWING:
                    stopMowing();
                    res.success = true;
                    res.message = "Mowing stopped";
                    break;
                    
                case lawnwomer::SystemControl::Request::COMMAND_RETURN_HOME:
                    returnHome();
                    res.success = true;
                    res.message = "Returning home";
                    break;
                    
                case lawnwomer::SystemControl::Request::COMMAND_EMERGENCY_STOP:
                    emergencyStop();
                    res.success = true;
                    res.message = "Emergency stop activated";
                    break;
                    
                case lawnwomer::SystemControl::Request::COMMAND_RESET:
                    resetSystem();
                    res.success = true;
                    res.message = "System reset";
                    break;
                    
                default:
                    res.success = false;
                    res.message = "Unknown command: " + std::to_string(req.command);
            }
        } catch (const std::exception& e) {
            res.success = false;
            res.message = "Command failed: " + std::string(e.what());
        }
        
        return true;
    }
    
    void startMowing() {
        is_mowing_ = true;
        std_msgs::Bool msg;
        msg.data = true;
        start_mowing_pub_.publish(msg);
    }
    
    void stopMowing() {
        is_mowing_ = false;
        std_msgs::Bool msg;
        msg.data = false;
        start_mowing_pub_.publish(msg);
    }
    
    void returnHome() {
        // 实现返回充电座逻辑
        ROS_INFO("Returning to home base...");
    }
    
    void emergencyStop() {
        is_emergency_stop_ = true;
        std_msgs::Bool msg;
        msg.data = true;
        emergency_stop_pub_.publish(msg);
        
        // 停止所有运动
        stopMowing();
        ROS_WARN("EMERGENCY STOP ACTIVATED!");
    }
    
    void resetSystem() {
        is_emergency_stop_ = false;
        std_msgs::Bool msg;
        msg.data = false;
        emergency_stop_pub_.publish(msg);
        
        ROS_INFO("System reset completed.");
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "system_control_server");
    SystemControlServer server;
    ros::spin();
    return 0;
}
