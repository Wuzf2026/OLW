# 电机状态消息定义
uint8 id
string name
float32 speed  # RPM
float32 current  # 安培
float32 temperature  # 摄氏度
uint8 status_code
bool is_enabled
bool is_fault
