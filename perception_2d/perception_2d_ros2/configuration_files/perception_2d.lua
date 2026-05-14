-- perception_2d ROS2 节点顶层配置
-- 由 node_main.cpp 通过 -configuration_directory / -configuration_basename gflag 加载
-- 装载顺序：本文件（Lua 默认值） → params_perception_2d.yaml（yaml 覆盖同名 key）
include "rec_by_pre_point.lua"

options = {
    laser_perception_2d              = LASER_PERCEPTION_2D,
    use_odometry                     = true,
    num_laser_scans                  = 2,    -- W1 PRO 双雷达（前 + 后）
    num_subdivisions_per_laser_scan  = 1,
    tracking_frame                   = "base_link",
    lookup_transform_timeout_sec     = 0.1,
}

return options
