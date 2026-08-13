#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <filesystem>
#include <fstream>

#include "common/options.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"
#include "wrapper/rviz_publisher.h"

namespace lightning {

LaserMapping::~LaserMapping() {
    scan_down_body_ = nullptr;
    scan_undistort_ = nullptr;
    scan_down_world_ = nullptr;
    LOG(INFO) << "laser mapping deconstruct";
}

bool LaserMapping::Init(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    if (use_voxel_map_) {
        voxel_map_ = std::make_shared<VoxelMapType>(vmap_config_);
    }
    if (use_ivox_) {
        // 将 ivox_options_ 转换为 IVoxConfig，统一通过 IVoxManager 管理
        ivox_config_.resolution = ivox_options_.resolution_;
        ivox_config_.nearby_type = ivox_options_.nearby_type_;
        ivox_config_.capacity = ivox_options_.capacity_;
        ivox_config_.filter_size_map_min = static_cast<float>(filter_size_map_min_);
        ivox_config_.esti_plane_threshold = fasterlio::ESTI_PLANE_THRESHOLD;
        ivox_manager_ = std::make_shared<IVoxManagerType>(ivox_config_);
    }
    LOG(INFO) << "LaserMapping: use_ivox=" << use_ivox_ << " use_voxel_map=" << use_voxel_map_;

    // esekf init
    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, 23, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;
    Vec3d lidar_T_wrt_IMU;
    Mat3d lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();
        extrinsic_est_en_ = yaml["fasterlio"]["extrinsic_est_en"].as<bool>();
        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        // VoxelMap 参数
        if (yaml["fasterlio"]["voxel_map"]) {
            auto vm = yaml["fasterlio"]["voxel_map"];
            if (vm["voxel_size"]) vmap_config_.voxel_size = vm["voxel_size"].as<double>();
            if (vm["max_layer"]) vmap_config_.max_layer = vm["max_layer"].as<int>();
            if (vm["layer_point_size"]) vmap_config_.layer_point_size = vm["layer_point_size"].as<std::vector<int>>();
            if (vm["max_points_size"]) vmap_config_.max_points_size = vm["max_points_size"].as<int>();
            if (vm["max_cov_points_size"]) vmap_config_.max_cov_points_size = vm["max_cov_points_size"].as<int>();
            if (vm["planer_threshold"]) vmap_config_.planer_threshold = vm["planer_threshold"].as<float>();
            if (vm["sigma_num"]) vmap_config_.sigma_num = vm["sigma_num"].as<double>();
            if (vm["ranging_cov"]) vmap_config_.ranging_cov = vm["ranging_cov"].as<double>();
            if (vm["angle_cov"]) vmap_config_.angle_cov = vm["angle_cov"].as<double>();
            if (vm["pub_max_voxel_layer"]) vmap_config_.pub_max_voxel_layer = vm["pub_max_voxel_layer"].as<int>();
        }

        // IVox 参数（可选，有默认值）
        if (yaml["fasterlio"]["use_ivox"]) use_ivox_ = yaml["fasterlio"]["use_ivox"].as<bool>();
        if (yaml["fasterlio"]["ivox_grid_resolution"])
            ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        if (yaml["fasterlio"]["ivox_nearby_type"]) {
            int ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
            if (ivox_nearby_type == 0) ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
            else if (ivox_nearby_type == 6)
                ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
            else if (ivox_nearby_type == 18)
                ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
            else if (ivox_nearby_type == 26)
                ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
            else {
                LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
                ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
            }
        }
        if (yaml["fasterlio"]["use_voxel_map"]) use_voxel_map_ = yaml["fasterlio"]["use_voxel_map"].as<bool>();

        if (yaml["fasterlio"]["voxel_map"] && yaml["fasterlio"]["voxel_map"]["pub_voxel_map"])
            pub_voxel_map_ = yaml["fasterlio"]["voxel_map"]["pub_voxel_map"].as<bool>();

        // IVox 范围删除参数
        if (yaml["fasterlio"]["ivox_range_delete"])
            ivox_config_.enable_range_delete = yaml["fasterlio"]["ivox_range_delete"].as<bool>();
        if (yaml["fasterlio"]["ivox_max_range"])
            ivox_config_.max_map_range = yaml["fasterlio"]["ivox_max_range"].as<float>();

        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        lidar_process_interval_ = yaml["fasterlio"]["lidar_process_interval"].as<int>();
        enable_skip_lidar_ = lidar_process_interval_ > 1;  // 1=逐帧处理，N>1=每N帧处理一次

        options_.kf_dis_th_ = yaml["fasterlio"]["kf_dis_th"].as<double>();
        options_.kf_angle_th_ = yaml["fasterlio"]["kf_angle_th"].as<double>() * M_PI / 180.0;

    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::AIRY);
        LOG(INFO) << "Using RoboSense M1 (AIRY) Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    lidar_type_ = lidar_type;

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    lidar_T_wrt_IMU = math::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = math::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));

    return true;
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    publish_count_++;

    auto imu_refined = p_imu_->TransformImuToLidar(imu);

    double timestamp = imu_refined->timestamp;

    // livox和airy的重力加速度的单位都为G
    if (lidar_type_ == 1 || lidar_type_ == 4) {
        imu_refined->linear_acceleration *= G_m_s2;
    }

    UL lock(mtx_imu_buffer_);
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    if (p_imu_->IsIMUInited()) {
        /// 更新最新imu状态
        kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, imu_refined->angular_velocity,
                        imu_refined->linear_acceleration);

        // LOG(INFO) << "newest wrt lidar: " << timestamp - kf_.GetX().timestamp_;

        /// 更新ui
        if (ui_) {
            ui_->UpdateNavState(kf_imu_.GetX());
        }
    }

    last_timestamp_imu_ = timestamp;

    imu_buffer_.emplace_back(imu_refined);
}

bool LaserMapping::Run() {
    if (!SyncPackages()) {
        return false;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);

    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    // IMU 初始化未完成时，只推进 IMU 初始化流程，不进入后续激光建图流程。
    if (!p_imu_->IsIMUInited()) {
        LOG_EVERY_N(WARNING, 50) << "waiting for imu init, skip laser mapping";
        return false;
    }

    /// the first scan
    if (flg_first_scan_) {
        LOG(INFO) << "first scan pts: " << scan_undistort_->size();

        state_point_ = kf_.GetX();

        // 构造首帧世界坐标变换
        Eigen::Matrix3d R_wl = (state_point_.rot_ * state_point_.offset_R_lidar_).matrix();
        Eigen::Vector3d t_wl = state_point_.rot_ * state_point_.offset_t_lidar_ + state_point_.pos_;

        // VoxelMap：用原始点云（未降采样）构造带协方差的点集并初始化地图
        if (use_voxel_map_) {
            // 首帧使用单位阵作为状态协方差（尚无 EKF 估计）
            Eigen::Matrix3d cov_rot = Eigen::Matrix3d::Identity() * 1e-6;
            Eigen::Matrix3d cov_trans = Eigen::Matrix3d::Identity() * 1e-6;

            std::vector<voxel_map::pointWithCov> pv_first;
            pv_first.reserve(scan_undistort_->size());
            for (size_t i = 0; i < scan_undistort_->size(); ++i) {
                voxel_map::pointWithCov pv;
                Eigen::Vector3d p_body = scan_undistort_->points[i].getVector3fMap().cast<double>();
                Eigen::Vector3d p_world = R_wl * p_body + t_wl;
                pv.point_body = p_body;
                pv.point_world = p_world;
                pv.point = p_world;  // OctoTree 内部使用世界坐标
                Eigen::Matrix3d body_cov = voxel_map_->CalcBodyCov(p_body);
                pv.cov = VoxelMapType::PropagateCovToWorld(body_cov, p_body, R_wl, cov_rot, cov_trans);
                pv_first.push_back(pv);
            }
            voxel_map_->InitializeMap(pv_first);
        }

        // IVox：用降采样后的点云初始化，保持与后续帧一致的分辨率
        if (use_ivox_) {
            // 对首帧点云做降采样
            CloudPtr scan_first_down(new PointCloudType);
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_first_down);

            int first_pts = static_cast<int>(scan_first_down->size());
            scan_down_world_->resize(first_pts);
            for (int i = 0; i < first_pts; ++i) {
                const auto &pb = scan_first_down->points[i];
                Eigen::Vector3d p_world = R_wl * pb.getVector3fMap().cast<double>() + t_wl;
                scan_down_world_->points[i].x = static_cast<float>(p_world(0));
                scan_down_world_->points[i].y = static_cast<float>(p_world(1));
                scan_down_world_->points[i].z = static_cast<float>(p_world(2));
                scan_down_world_->points[i].intensity = pb.intensity;
            }
            ivox_manager_->GetIVox()->AddPoints(scan_down_world_->points);
            LOG(INFO) << "IVox first frame init: " << first_pts << " pts";
        }

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        flg_first_scan_ = false;
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % lidar_process_interval_;

        if (skip_lidar_cnt_ != 0) {
            /// 更新UI中的内容
            if (ui_) {
                ui_->UpdateNavState(kf_.GetX());
                ui_->UpdateScan(scan_undistort_, kf_.GetX().GetPose());
            }

            return false;
        }
    }

    // LOG(INFO) << "LIO get cloud at beg: " << std::setprecision(14) << measures_.lidar_begin_time_
    //           << ", end: " << measures_.lidar_end_time_;

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LOG(ERROR) << "检测到雷达断流，时长：" << (measures_.lidar_begin_time_ - last_lidar_time_);
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    /// downsample
    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }
    scan_down_world_->resize(cur_pts);

    // ====== IEKF 外一次性预计算（VoxelMap 需要 body_cov；IVox 需要 nearest_points_ 空间）======
    if (use_voxel_map_) {
        // VoxelMap 需要协方差，预计算 body_cov
        body_points_list_.resize(cur_pts);
        body_cov_list_.resize(cur_pts);
        std::vector<size_t> idx(cur_pts);
        for (int i = 0; i < cur_pts; ++i) idx[i] = i;
        std::for_each(std::execution::par_unseq, idx.begin(), idx.end(), [&](const size_t &i) {
            body_points_list_[i] = scan_down_body_->points[i].getVector3fMap().cast<double>();
            body_cov_list_[i] = voxel_map_->CalcBodyCov(body_points_list_[i]);
        });
        iekf_first_iter_ = true;
    }
    if (use_ivox_) {
        // IVox 需要 nearest_points_ / residuals_（其余临时数组由 IVoxManager 内部管理）
        nearest_points_.resize(cur_pts);
        residuals_.resize(cur_pts, 0.0f);
    }

    Timer::Evaluate(
        [&, this]() {
            auto old_state = kf_.GetX();

            kf_.Update(ESKF::ObsType::LIDAR, 1e-3);
            state_point_ = kf_.GetX();

            if (keep_first_imu_estimation_ && all_keyframes_.size() < 5 &&
                (old_state.rot_.inverse() * state_point_.rot_).log().norm() > 0.3 * M_PI / 180) {
                kf_.ChangeX(old_state);
                state_point_ = old_state;

                // 状态回退：pv_list_ 中的世界坐标是基于被拒绝的姿态计算的，
                // 需要用正确的预测状态（old_state）重建 pv_list_ 以正确更新地图
                if (use_voxel_map_) {
                    Eigen::Matrix3d R_wl_ok = (old_state.rot_ * old_state.offset_R_lidar_).matrix();
                    Eigen::Vector3d t_wl_ok = old_state.rot_ * old_state.offset_t_lidar_ + old_state.pos_;
                    Eigen::Matrix3d cov_rot_ok = Eigen::Matrix3d::Identity() * 1e-4;
                    Eigen::Matrix3d cov_trans_ok = Eigen::Matrix3d::Identity() * 1e-4;
                    int cnt = scan_down_body_->size();
                    pv_list_.resize(cnt);
                    for (int ii = 0; ii < cnt; ++ii) {
                        const Eigen::Vector3d &pb = body_points_list_[ii];
                        Eigen::Vector3d pw = R_wl_ok * pb + t_wl_ok;
                        pv_list_[ii].point_body = pb;
                        pv_list_[ii].point_world = pw;
                        pv_list_[ii].point = pw;
                        // 使用预计算的 body_cov
                        pv_list_[ii].cov = VoxelMapType::PropagateCovToWorld(body_cov_list_[ii], pb, R_wl_ok,
                                                                             cov_rot_ok, cov_trans_ok);
                    }
                }

                LOG(INFO) << "set state as prediction";
            }

            // LOG(INFO) << "old yaw: " << old_state.rot_.angleZ() << ", new: " << state_point_.rot_.angleZ();

            state_point_.timestamp_ = measures_.lidar_end_time_;
            euler_cur_ = state_point_.rot_;
            pos_lidar_ = state_point_.pos_ + state_point_.rot_ * state_point_.offset_t_lidar_;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    // LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
    //           << " Map voxel num: " << voxel_map_->NumVoxels() << " effect num : " << effect_feat_num_;

    /// keyframes
    if (last_kf_ == nullptr) {
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = state_point_.GetPose();
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } else if (!options_.is_in_slam_mode_ && (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        }
    }

    /// 更新kf_imu_：从 kf_ 同步状态，并对 imu_buffer_ 中剩余 IMU 做预测
    {
        UL lock_imu(mtx_imu_buffer_);
        kf_imu_ = kf_;
        if (!measures_.imu_.empty()) {
            double t = measures_.imu_.back()->timestamp;
            for (auto &imu : imu_buffer_) {
                double dt = imu->timestamp - t;
                kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);
                t = imu->timestamp;
            }
        }
    }

    if (ui_) {
        ui_->UpdateScan(scan_undistort_, state_point_.GetPose());
    }

    // 发布 VoxelMap 平面（UI 和 RViz），仅在 use_voxel_map_ 且 pub_voxel_map_ 启用时执行
    if (use_voxel_map_ && pub_voxel_map_) {
        auto planes = voxel_map_->GetUpdatedPlanesForVis(state_point_.pos_, 50.0);
        if (!planes.empty()) {
            if (ui_) {
                ui_->UpdateVoxelMapPlanes(planes);
            }
            if (rviz_publisher_) {
                rviz_publisher_->UpdateVoxelMap(planes);
            }
        }
    }

    return true;
}

void LaserMapping::MakeKF() {
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_);

    // opt_pose 统一用 LIO pose 占位，正式传播由 HandleKF（后端线程，PoseOptimization 串行之后）完成。
    // 这样无论 SLAM 模式还是非 SLAM 模式，MakeKF 都不读取任何其他帧的 opt_pose，彻底避免数据竞争。
    kf->SetOptPose(kf->GetLIOPose());

    kf->SetState(state_point_);

    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    scan_count_++;
    double timestamp = ToSec(msg->header.stamp);

    CloudPtr cloud(new PointCloudType());
    preprocess_->Process(msg, cloud);

    UL lock(mtx_lidar_buffer_);
    if (timestamp < last_timestamp_lidar_) {
        LOG(ERROR) << "lidar loop back, clear buffer";
        lidar_buffer_.clear();
        time_buffer_.clear();
    }

    lidar_buffer_.push_back(cloud);
    time_buffer_.push_back(timestamp);
    last_timestamp_lidar_ = timestamp;
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    scan_count_++;
    double timestamp = ToSec(msg->header.stamp);

    CloudPtr cloud(new PointCloudType());
    preprocess_->Process(msg, cloud);

    UL lock(mtx_lidar_buffer_);
    if (timestamp < last_timestamp_lidar_) {
        LOG(ERROR) << "lidar loop back, clear buffer";
        lidar_buffer_.clear();
        time_buffer_.clear();
    }

    lidar_buffer_.push_back(cloud);
    time_buffer_.push_back(timestamp);
    last_timestamp_lidar_ = timestamp;
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    scan_count_++;
    double timestamp = math::ToSec(cloud->header.stamp);

    UL lock(mtx_lidar_buffer_);
    if (timestamp < last_timestamp_lidar_) {
        LOG(ERROR) << "lidar loop back, clear buffer";
        lidar_buffer_.clear();
        time_buffer_.clear();
    }

    lidar_buffer_.push_back(cloud);
    time_buffer_.push_back(timestamp);
    last_timestamp_lidar_ = timestamp;
}

bool LaserMapping::SyncPackages() {
    // 按固定顺序加锁避免死锁：先 lidar 后 imu
    UL lock_lidar(mtx_lidar_buffer_);
    if (lidar_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);
            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        lo::lidar_time_interval = lidar_mean_scantime_;

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    // 读取 last_timestamp_imu_ 需要 imu 锁
    {
        UL lock_imu(mtx_imu_buffer_);
        if (last_timestamp_imu_ < lidar_end_time_) {
            return false;
        }

        /*** push imu_ data, and pop from imu_ buffer ***/
        double imu_time = imu_buffer_.front()->timestamp;
        measures_.imu_.clear();

        // 检查 IMU 是否覆盖完整激光周期 [lidar_begin_time_, lidar_end_time_]
        if (imu_time > lidar_end_time_) {
            // IMU 全部晚于激光结束时间，无法提供任何 IMU 数据
            LOG(WARNING) << std::setprecision(12) << "IMU data too late for lidar: imu_front=" << imu_time
                         << ", lidar_end=" << lidar_end_time_ << ", diff=" << (imu_time - lidar_end_time_) * 1000.0
                         << "ms"
                         << ", skipping this lidar frame";
            lidar_buffer_.pop_front();
            time_buffer_.pop_front();
            lidar_pushed_ = false;
            return false;
        }

        if ((imu_time - measures_.lidar_begin_time_) * 1000 > 20) {
            // IMU 不覆盖激光开始时间，部分区间缺失 IMU (超过20ms)
            LOG(WARNING) << std::setprecision(12) << "IMU does not cover full lidar period: imu_front=" << imu_time
                         << ", lidar_begin=" << measures_.lidar_begin_time_ << ", lidar_end=" << lidar_end_time_
                         << ", missing " << (imu_time - measures_.lidar_begin_time_) * 1000.0 << "ms at start";
        }

        while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
            imu_time = imu_buffer_.front()->timestamp;
            if (imu_time > lidar_end_time_) {
                break;
            }

            measures_.imu_.push_back(imu_buffer_.front());

            imu_buffer_.pop_front();
        }
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // LOG(INFO) << "sync: " << std::setprecision(14) << measures_.lidar_begin_time_ << ", " <<
    // measures_.lidar_end_time_;

    return true;
}

void LaserMapping::MapIncremental() {
    // VoxelMap 地图更新（受 use_voxel_map_ 开关控制）
    if (use_voxel_map_) {
        if (!pv_list_.empty()) {
            Timer::Evaluate([&, this]() { voxel_map_->UpdateMap(pv_list_); }, "    VoxelMap Update");
        }
    }

    // IVox 地图更新（受 use_ivox_ 开关控制）
    // 通过 IVoxManager::UpdateMap 统一管理，含最近邻去重滤波
    if (use_ivox_) {
        Timer::Evaluate(
            [&, this]() {
                ivox_manager_->UpdateMap(scan_down_world_->points, nearest_points_, flg_EKF_inited_,
                                         pos_lidar_.cast<float>());
            },
            "    IVox Add Points");
    }
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 *
 * 性能优化策略（参考 VoxelMap 原始代码和 Voxel-SLAM）：
 *   1. body_cov 在 IEKF 外预计算一次（仅依赖 body 坐标，不变）
 *   2. 首次 IEKF 迭代做完整 hash 查找（FindCorrespondences）
 *   3. 后续迭代复用首次的匹配对，只重算 point_world 和残差/Jacobian
 *
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) index[i] = i;

    // ----------------------------------------------------------------
    //  IVox 匹配：通过 IVoxManager::FindCorrespondences 统一管理
    //  包含坐标变换 + kNN 查询 + 平面拟合 + 马氏距离检验
    //  nearest_points_ 同步更新，供本帧 EKF 收敛后 MapIncremental 使用
    // ----------------------------------------------------------------
    if (use_ivox_) {
        Timer::Evaluate(
            [&, this]() {
                auto R_wl = (s.rot_ * s.offset_R_lidar_).cast<float>();
                auto t_wl = (s.rot_ * s.offset_t_lidar_ + s.pos_).cast<float>();

                ivox_manager_->FindCorrespondences(scan_down_body_, scan_down_world_, R_wl.matrix(), t_wl, corr_pts_,
                                                   corr_norm_, residuals_, nearest_points_);
            },
            "    ObsModel (IVox Match)");
    } else {
        corr_pts_.clear();
        corr_norm_.clear();
    }

    // ----------------------------------------------------------------
    //  VoxelMap 匹配：首次迭代做完整 hash 查找，后续迭代仅 reproject
    // ----------------------------------------------------------------
    if (use_voxel_map_) {
        Eigen::Matrix3d R_wl_d = (s.rot_ * s.offset_R_lidar_).matrix();
        Eigen::Vector3d t_wl_d = s.rot_ * s.offset_t_lidar_ + s.pos_;

        if (iekf_first_iter_) {
            iekf_first_iter_ = false;

            auto &P = kf_.GetP();
            Eigen::Matrix3d cov_rot = P.block<3, 3>(3, 3);
            Eigen::Matrix3d cov_trans = P.block<3, 3>(0, 0);

            Timer::Evaluate(
                [&, this]() {
                    pv_list_.resize(cnt_pts);
                    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                        const Eigen::Vector3d &p_body = body_points_list_[i];
                        Eigen::Vector3d p_world = R_wl_d * p_body + t_wl_d;
                        pv_list_[i].point_body = p_body;
                        pv_list_[i].point_world = p_world;
                        pv_list_[i].point = p_world;
                        pv_list_[i].cov =
                            VoxelMapType::PropagateCovToWorld(body_cov_list_[i], p_body, R_wl_d, cov_rot, cov_trans);
                    });
                },
                "    ObsModel (Prepare Points)");

            Timer::Evaluate(
                [&, this]() {
                    ptpl_list_.clear();
                    voxel_map_->FindCorrespondences(pv_list_, ptpl_list_);
                },
                "    ObsModel (VoxelMap Match)");
        } else {
            // 后续迭代：复用首次匹配对，只重算 world 坐标
            Timer::Evaluate(
                [&, this]() {
                    int n = static_cast<int>(ptpl_list_.size());
                    std::vector<size_t> vmap_idx(n);
                    for (int i = 0; i < n; ++i) vmap_idx[i] = i;
                    std::for_each(std::execution::par_unseq, vmap_idx.begin(), vmap_idx.end(), [&](const size_t &i) {
                        ptpl_list_[i].point_world = R_wl_d * ptpl_list_[i].point + t_wl_d;
                    });
                },
                "    ObsModel (Reproject Points)");
        }
    } else {
        ptpl_list_.clear();
    }

    int ivox_feat_num = use_ivox_ ? static_cast<int>(corr_pts_.size()) : 0;
    int vmap_feat_num = use_voxel_map_ ? static_cast<int>(ptpl_list_.size()) : 0;
    effect_feat_num_ = ivox_feat_num + vmap_feat_num;

    if (effect_feat_num_ < 1) {
        obs.valid_ = false;
        LOG(WARNING) << "No Effective Points! vmap=" << vmap_feat_num << " ivox=" << ivox_feat_num
                     << " pos=" << s.pos_.transpose();
        return;
    }

    // ---------- 构建 Jacobian 和残差 ----------
    Timer::Evaluate(
        [&, this]() {
            obs.h_x_ = Eigen::MatrixXd::Zero(effect_feat_num_, 12);
            obs.residual_.resize(effect_feat_num_);

            const Mat3f off_R = s.offset_R_lidar_.matrix().cast<float>();
            const Vec3f off_t = s.offset_t_lidar_.cast<float>();
            const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

            // --- 前段：IVox ---
            if (use_ivox_ && ivox_feat_num > 0) {
                std::vector<size_t> ivox_idx(ivox_feat_num);
                for (int i = 0; i < ivox_feat_num; ++i) ivox_idx[i] = i;

                std::for_each(std::execution::par_unseq, ivox_idx.begin(), ivox_idx.end(), [&](const size_t &i) {
                    Vec3f point_this_be = corr_pts_[i].head<3>();
                    Mat3f point_be_crossmat = math::SKEW_SYM_MATRIX(point_this_be);
                    Vec3f point_this = off_R * point_this_be + off_t;
                    Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);
                    Vec3f norm_vec = corr_norm_[i].head<3>();
                    Vec3f C(Rt * norm_vec);
                    Vec3f A(point_crossmat * C);

                    if (extrinsic_est_en_) {
                        Vec3f B(point_be_crossmat * off_R.transpose() * C);
                        obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                            B[1], B[2], C[0], C[1], C[2];
                    } else {
                        obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0;
                    }
                    obs.residual_(i) = -corr_pts_[i][3];
                });
            }

            // --- 后段：VoxelMap ---
            if (use_voxel_map_ && vmap_feat_num > 0) {
                std::vector<size_t> vmap_idx(vmap_feat_num);
                for (int i = 0; i < vmap_feat_num; ++i) vmap_idx[i] = i;

                std::for_each(std::execution::par_unseq, vmap_idx.begin(), vmap_idx.end(), [&](const size_t &i) {
                    const int row = ivox_feat_num + static_cast<int>(i);
                    const auto &match = ptpl_list_[i];

                    Vec3f point_this_be = match.point.cast<float>();
                    Mat3f point_be_crossmat = math::SKEW_SYM_MATRIX(point_this_be);
                    Vec3f point_this = off_R * point_this_be + off_t;
                    Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);
                    Vec3f norm_vec = match.normal.cast<float>();
                    Vec3f C(Rt * norm_vec);
                    Vec3f A(point_crossmat * C);

                    if (extrinsic_est_en_) {
                        Vec3f B(point_be_crossmat * off_R.transpose() * C);
                        obs.h_x_.block<1, 12>(row, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                            B[1], B[2], C[0], C[1], C[2];
                    } else {
                        obs.h_x_.block<1, 12>(row, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0;
                    }
                    float res = static_cast<float>(match.normal.dot(match.point_world) + match.d);
                    obs.residual_(row) = -res;
                });
            }
        },
        "    ObsModel (IEKF Build Jacobian)");

    // 中位数平方误差统计
    std::vector<double> res_sq2;
    res_sq2.reserve(effect_feat_num_);
    if (use_ivox_) {
        for (int i = 0; i < ivox_feat_num; ++i) {
            double r = corr_pts_[i][3];
            res_sq2.emplace_back(r * r);
        }
    }
    if (use_voxel_map_) {
        for (int i = 0; i < vmap_feat_num; ++i) {
            double r = ptpl_list_[i].normal.dot(ptpl_list_[i].point_world) + ptpl_list_[i].d;
            res_sq2.emplace_back(r * r);
        }
    }
    std::sort(res_sq2.begin(), res_sq2.end());
    obs.lidar_residual_mean_ = res_sq2[res_sq2.size() / 2];
    obs.lidar_residual_max_ = res_sq2[res_sq2.size() - 1];
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    for (auto &kf : all_keyframes_) {
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        if (use_lio_pose) {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetLIOPose().matrix());
        } else {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetOptPose().matrix());
        }

        *global_map += *cloud_trans;

        LOG(INFO) << "kf " << kf->GetID() << ", pose: " << kf->GetOptPose().translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    LOG(INFO) << "global map: " << global_map_filtered->size();

    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    /// 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    LOG(INFO) << "lio map is saved to ./data/lio.pcd";
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

}  // namespace lightning