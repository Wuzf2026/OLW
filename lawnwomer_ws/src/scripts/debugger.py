import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image, NavSatFix, Range
from geometry_msgs.msg import Twist
from lawnmower.msg import LawnMowerStatus, MotorStatus
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import cv2
from cv_bridge import CvBridge
class LawnMowerDebugger:
    def __init__(self):
        rospy.init_node('lawnmower_debugger', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅主题
        rospy.Subscriber('/lawnmower_status', LawnMowerStatus, self.status_callback)
        rospy.Subscriber('/points_raw', PointCloud2, self.laser_callback)
        rospy.Subscriber('/color/image_raw', Image, self.color_image_callback)
        rospy.Subscriber('/depth/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 创建图形界面
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # 初始化数据
        self.laser_data = None
        self.color_image = None
        self.depth_image = None
        self.gps_data = None
        self.cmd_vel_data = None
        
        # 动画更新
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        
    def status_callback(self, msg):
        self.status_data = msg
        
    def laser_callback(self, msg):
        # 转换点云数据
        self.laser_data = np.array(msg.data, dtype=np.uint8)
        
    def color_image_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)
            
    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "mono16")
        except Exception as e:
            print(e)
            
    def gps_callback(self, msg):
        self.gps_data = msg
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = msg
        
    def update_plot(self, frame):
        # 清空子图
        self.ax1.cla()
        self.ax2.cla()
        self.ax3.cla()
        self.ax4.cla()
        
        # 绘制状态信息
        if hasattr(self, 'status_data'):
            status_text = f"System State: {self.status_data.system_state}\n" + \
                         f"Battery: {self.status_data.battery_percentage:.1f}%\n" + \
                         f"GPS: {self.status_data.rtk_gps_fixed}\n" + \
                         f"Motors: {len(self.status_data.motors)} active"
            self.ax1.text(0.05, 0.95, status_text, transform=self.ax1.transAxes, 
                         verticalalignment='top', fontsize=10, 
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            self.ax1.set_title('System Status')
            self.ax1.axis('off')
            
        # 绘制激光雷达数据
        if self.laser_data is not None:
            # 简单可视化（显示前100个点）
            points = self.laser_data[:100]
            self.ax2.scatter(points[::3], points[1::3], s=1, c=points[2::3], cmap='viridis')
            self.ax2.set_xlabel('X (m)')
            self.ax2.set_ylabel('Y (m)')
            self.ax2.set_title('Laser Scanner Points')
            self.ax2.axis('equal')
            
        # 绘制彩色图像
        if self.color_image is not None:
            self.ax3.imshow(cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB))
            self.ax3.set_title('Color Camera')
            self.ax3.axis('off')
            
        # 绘制深度图像
        if self.depth_image is not None:
            self.ax4.imshow(self.depth_image, cmap='jet', vmin=0, vmax=2000)
            self.ax4.set_title('Depth Image (mm)')
            self.ax4.axis('off')
            
        plt.tight_layout()
        return self.ani
    
    def run(self):
        plt.show()
if __name__ == '__main__':
    debugger = LawnMowerDebugger()
    debugger.run()
