-- perception_2d 算法层参数表
-- ⚠️ 该表与 perception_2d_ros2/params/params_perception_2d.yaml 同步维护
-- 装载顺序：本 Lua → params_perception_2d.yaml（yaml 同名 key 覆盖此默认值）
LASER_PERCEPTION_2D = {
    -- 检测器开关
    predict_rec = true,           -- 预设点识别（充电桩 / 角点）
    reflect_rec = true,           -- 反光板识别
    line_rec    = true,           -- 直线检测（墙体 / 玻璃边缘）
    shape_rec   = false,          -- 形状匹配（货架 / 人腿 / 障碍物）；打开需 model.yaml 模型
    vmark_rec   = false,          -- visual marker（vmarker.yaml）
    angle_rec   = false,          -- 角点识别（保留字段，未连入主流程）

    -- 数据预处理
    intensities_min     = 900,    -- 反光板强度阈值
    connect_threshold   = 0.05,   -- 聚类连接距离
    range_max           = 20.0,
    range_min           = 0.1,
    min_cluster_points  = 2,

    -- 匹配阈值
    search_distance     = 0.4,
    search_angle        = 0.2,
    max_error           = 0.04,
    start_rec_dis       = 4.0,
    min_select_points   = 10,

    -- 反光板
    reflector_error     = 0.02,
    reflector_radius    = 0.055,

    -- 直线检测
    min_line_length     = 0.35,

    -- 形状匹配（shape_rec 打开后生效）
    shape_match_score         = 0.7,
    shape_match_close_degree  = 0.05,
    shape_dis_to_robot_center = 0.335,

    -- 备用模型文件路径（留空走 model_basename gflag）
    model_file = "",
}
