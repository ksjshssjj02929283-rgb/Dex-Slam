include "W1-2d_mapping_indoor.lua"
options.provide_odom_frame = true
options.use_odometry = false

-- added by hz
-- 是否使用手动设置的初始位置，如果为true，必须手动设置初始值之后才会启动定位
options.sub_initial_pose = false 
options.use_predefined_init_pose = true
options.sub_apriltag_pose = false

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 80

return options