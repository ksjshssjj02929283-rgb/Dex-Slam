std_msgs/Header header

int32 id
string name
uint8 point_type

float32 x
float32 y
float32 theta

uint8 FREE_GOAL = 0 # 自由目标点
uint8 STATION = 1   # 站点
uint8 CHARGING = 2  # 充电点
uint8 PARKING = 3   # 停车点
uint8 PICKUP = 4    # 取货点
uint8 LIFT = 5      # 电梯点