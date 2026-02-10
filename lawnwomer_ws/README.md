lawnwomer_ws/
├── CMakeLists.txt          # 工程顶层CMakeLists.txt（catkin_make编译入口）
├── package.xml             # ROS包描述文件（依赖、版本、作者等）
├── include/                # 头文件目录（按模块分类）
│   └── lawnmower/
│       ├── sensor/         # 传感器驱动头文件
│       ├── motor/          # 电机控制头文件
│       ├── utils/          # 工具函数头文件
│       └── lawnmower_node.h# 主节点头文件
├── src/                    # 源文件目录（按功能模块分类）
│   ├── drivers/            # 硬件驱动实现
│   │   ├── gemini335_driver.cpp    # Gemini335深度相机驱动
│   │   ├── hesai_lidar_driver.cpp  # Hesai JT128激光雷达驱动
│   │   ├── rtk_um982_driver.cpp    # T-RTK UM982定位模块驱动
│   │   ├── ultrasonic_driver.cpp   # 超声波雷达阵列驱动
│   │   └── motor_can_driver.cpp    # CAN总线电机控制驱动
│   ├── services/           # ROS服务实现
│   │   ├── sensor_config_server.cpp # 传感器配置服务
│   │   └── system_control_server.cpp # 系统控制服务
│   └── nodes/              # ROS节点实现
│       └── lawnmower_node.cpp       # 工程主节点（系统集成）
├── scripts/                # Python脚本（调试工具）
│   └── debugger.py         # 实时数据监控、可视化调试工具
├── config/                 # 配置文件目录（传感器、机器人、控制参数）
│   ├── sensor_params.yaml  # 传感器标定/通信参数
│   ├── robot_params.yaml   # 机器人几何/运动参数
│   └── control_params.yaml # 电机控制/速度限制参数
├── launch/                 # ROS启动文件（一键启动所有节点）
│   └── bringup.launch      # 工程总启动文件（驱动+服务+主节点）
├── srv/                    # ROS服务定义文件
│   ├── SensorConfig.srv    # 传感器配置服务
│   └── SystemControl.srv   # 系统控制服务
├── msg/                    # ROS自定义消息文件
│   ├── MotorStatus.msg     # 电机状态消息
│   └── LawnMowerStatus.msg # 机器人整体状态消息
└── README.md               # 工程说明（编译、部署、使用方法）
