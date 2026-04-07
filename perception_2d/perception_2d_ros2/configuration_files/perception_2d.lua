include "rec_by_pre_point.lua"
options = {
    laser_perception_2d = LASER_PERCPETION_2D,
    use_odometry = true,
    num_laser_scans = 1,
    num_subdivisions_per_laser_scan = 1,
    tracking_frame = "base_link",
    lookup_transform_timeout_sec = 0.2,
}

return options