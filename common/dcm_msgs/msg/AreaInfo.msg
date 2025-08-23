uint32 VIRTUAL_WALL = 1            # 虚拟墙
uint32 RESTRICTED_AREA = 2         # 禁行区
uint32 DETOUR_AREA = 3             # 绕行区
uint32 NO_DETOUR_AREA = 4          # 禁止绕障区
uint32 SPEED_LIMIT_AREA = 5        # 限速区
uint32 CARPET_AREA = 6             # 地毯区
uint32 TRACKSLIP_AREA = 7          # 易打滑区域
uint32 ELEVATOR_AREA = 8           # 电梯区

string                  name       # 区域名称
uint32                  type       # 区域类型
geometry_msgs/Pose2D[]  polygon    # 区域点列表
string                  attribute  # 区域属性
