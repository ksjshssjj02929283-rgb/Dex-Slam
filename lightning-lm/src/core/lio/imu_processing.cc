#include "imu_processing.h"

namespace lightning {

namespace {
Mat3d ProjectToSO3(const Mat3d& R) {
    Eigen::JacobiSVD<Mat3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3d U = svd.matrixU();
    Mat3d V = svd.matrixV();
    Mat3d R_ortho = U * V.transpose();
    if (R_ortho.determinant() < 0.0) {
        U.col(2) *= -1.0;
        R_ortho = U * V.transpose();
    }
    return R_ortho;
}
}  // namespace

ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_.setZero();
    Q_.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5;
    cov_acc_ = Vec3d(0.1, 0.1, 0.1);
    cov_gyr_ = Vec3d(0.1, 0.1, 0.1);
    cov_bias_gyr_ = Vec3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = Vec3d(0.0001, 0.0001, 0.0001);
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    last_imu_.reset(new lightning::IMU());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    angvel_last_.setZero();

    imu_need_init_ = true;
    init_iter_num_ = 1;
    imu_queue_.clear();
    imu_pose_.clear();
    last_imu_.reset(new lightning::IMU());
    cur_pcl_un_.reset(new PointCloudType());
}

void ImuProcess::SetExtrinsic(const Vec3d &transl, const Mat3d &rot) {
    R_imu_from_lidar_ = ProjectToSO3(rot);
    R_lidar_from_imu_ = R_imu_from_lidar_.transpose();
    t_lidar_in_imu_ = transl;
    // 将 IMU 系中的平移外参转换到 LiDAR 系，后续状态在 LiDAR 旋转系下仅保留平移项
    t_refined_imu_in_lidar_ = R_lidar_from_imu_ * t_lidar_in_imu_;
}

void ImuProcess::SetGyrCov(const Vec3d &scaler) { cov_gyr_scale_ = scaler; }

void ImuProcess::SetAccCov(const Vec3d &scaler) { cov_acc_scale_ = scaler; }

void ImuProcess::SetGyrBiasCov(const Vec3d &b_g) { cov_bias_gyr_ = b_g; }

void ImuProcess::SetAccBiasCov(const Vec3d &b_a) { cov_bias_acc_ = b_a; }

IMUPtr ImuProcess::TransformImuToLidar(const IMUPtr &imu) const {
    auto imu_lidar = std::make_shared<IMU>(*imu);
    imu_lidar->angular_velocity = R_lidar_from_imu_ * imu->angular_velocity;
    imu_lidar->linear_acceleration = R_lidar_from_imu_ * imu->linear_acceleration;
    return imu_lidar;
}

Mat3d ImuProcess::AlignGravity(const Vec3d &gravity_vector, Vec3d &aligned_gravity) {
    // 归一化重力向量
    Vec3d n0 = gravity_vector / gravity_vector.norm();
    
    // 目标重力方向（世界坐标系z轴）
    Vec3d n1(0, 0, 1);
    if (n0[2] < 0) {
        n1[2] = -1;  // 如果重力向上，则目标方向也向上
    }
    
    // 计算旋转轴和角度
    Vec3d rotvec = n0.cross(n1);
    double rnorm = rotvec.norm();
    
    Mat3d rot = Mat3d::Identity();
    if (rnorm > 1e-6) {  // 避免除零
        rotvec = rotvec / rnorm;
        Eigen::AngleAxisd angaxis(std::asin(rnorm), rotvec);
        rot = ProjectToSO3(angaxis.matrix());
        
        std::cout << "Gravity alignment: rotation angle = " << std::asin(rnorm) * 180.0 / M_PI 
                  << " deg, axis = " << rotvec.transpose() << std::endl;
    } else {
        std::cout << "Gravity already aligned, no rotation needed" << std::endl;
    }
    
    // 计算对齐后的重力向量
    aligned_gravity = rot * gravity_vector;
    
    std::cout << "Gravity before alignment: " << gravity_vector.transpose() << std::endl;
    std::cout << "Gravity after alignment:  " << aligned_gravity.transpose() << std::endl;
    
    return rot;
}

void ImuProcess::IMUInit(const MeasureGroup &meas, ESKF &kf_state, int &N) {
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_ **/

    Vec3d cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu_.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu_.front()->angular_velocity;
        mean_acc_ = imu_acc;
        mean_gyr_ = gyr_acc;
    }

    for (const auto &imu : meas.imu_) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc = imu_acc;
        cur_gyr = gyr_acc;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ =
            cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ =
            cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }
    
    // 静止判断：检查加速度和角速度方差
    const double acc_cov_threshold = 0.05;   // 加速度方差阈值
    const double gyr_cov_threshold = 0.05;  // 角速度方差阈值
    
    double acc_cov_norm = cov_acc_.norm();
    double gyr_cov_norm = cov_gyr_.norm();
    
    std::cout << "IMU static check: acc_cov = " << acc_cov_norm 
              << ", gyr_cov = " << gyr_cov_norm << std::endl;
    
    if (acc_cov_norm > acc_cov_threshold || gyr_cov_norm > gyr_cov_threshold) {
        std::cout << "IMU not static! acc_cov = " << acc_cov_norm 
                  << " (threshold: " << acc_cov_threshold << "), gyr_cov = " << gyr_cov_norm
                  << " (threshold: " << gyr_cov_threshold << ")" << std::endl;
        // 重置初始化状态，等待下一帧
        b_first_frame_ = true;
        N = 1;
        return;
    }
    
    std::cout << "IMU static check passed!" << std::endl;

    auto init_state = kf_state.GetX();
    init_state.timestamp_ = meas.imu_.back()->timestamp;
    std::cout << "IMU Init mean_acc_: " << mean_acc_.transpose() << ", mean_gyr_: " << mean_gyr_.transpose()
              << ", N: " << N - 1 << std::endl;
    
    // 计算初始重力向量（加速度计测量的是-g）
    Vec3d gravity_init = -mean_acc_ / mean_acc_.norm() * G_m_s2;
    std::cout << "Initial gravity estimate: " << gravity_init.transpose() << std::endl;
    
    // 使用重力对齐方法获取旋转矩阵
    Vec3d aligned_gravity;
    Mat3d R_align = AlignGravity(gravity_init, aligned_gravity);
    
    // 设置对齐后的重力向量和初始旋转
    init_state.grav_ = S2(aligned_gravity);
    init_state.rot_ = SO3(ProjectToSO3(R_align));
    init_state.bg_ = mean_gyr_;
    
    init_state.offset_t_lidar_ = t_refined_imu_in_lidar_;
    init_state.offset_R_lidar_ = SO3();
    
    kf_state.ChangeX(init_state);
    std::cout << "init_state.grav_: " << init_state.grav_.vec_.transpose() << std::endl;
    std::cout << "init_state.rot_: \n" << init_state.rot_.matrix() << std::endl;

    auto init_P = kf_state.GetP();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.ChangeP(init_P);

    last_imu_ = meas.imu_.back();
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &pcl_out) {
    /*** add the imu_ of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);
    const double &imu_end_time = v_imu.back()->timestamp;

    const double &pcl_beg_time = meas.lidar_begin_time_;
    const double &pcl_end_time = meas.lidar_end_time_;

    /*** Initialize IMU pose ***/
    auto imu_state = kf_state.GetX();
    imu_pose_.clear();
    imu_pose_.emplace_back(0.0, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix());

    /*** forward propagation at each imu_ point ***/
    Vec3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    Mat3d R_imu;

    double dt = 0;
    Vec3d acc = Vec3d::Zero();
    Vec3d gyro = Vec3d::Zero();

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (tail->timestamp < last_lidar_end_time_) {
            continue;
        }

        angvel_avr = .5 * (head->angular_velocity + tail->angular_velocity);
        acc_avr = .5 * (head->linear_acceleration + tail->linear_acceleration);

        acc_avr = acc_avr * G_m_s2 / mean_acc_.norm();  // - state_inout.ba;

        if (head->timestamp < last_lidar_end_time_) {
            dt = tail->timestamp - last_lidar_end_time_;
        } else {
            dt = tail->timestamp - head->timestamp;
        }

        acc = acc_avr;
        gyro = angvel_avr;

        if (dt > 0.1) {
            LOG(ERROR) << "get abnormal dt: " << dt;
            kf_state.SetTime((*it_imu)->timestamp);
            break;
        }

        Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
        Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
        Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
        Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;

        kf_state.Predict(dt, Q_, gyro, acc);

        /* save the poses at each IMU measurements */
        imu_state = kf_state.GetX();
        angvel_last_ = angvel_avr - imu_state.bg_;
        acc_s_last_ = imu_state.rot_ * (acc_avr - imu_state.ba_);
        for (int i = 0; i < 3; i++) {
            acc_s_last_[i] += imu_state.grav_[i];
        }

        double &&offs_t = tail->timestamp - pcl_beg_time;
        imu_pose_.emplace_back(
            Pose6D(offs_t, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.Predict(dt, Q_, gyro, acc);

    imu_state = kf_state.GetX();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** sort point clouds by offset time ***/
    pcl_out = meas.scan_;
    std::sort(pcl_out->points.begin(), pcl_out->points.end(),
              [](const PointType &p1, const PointType &p2) { return p1.time < p2.time; });

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out->empty()) {
        return;
    }

    auto it_pcl = pcl_out->points.end() - 1;
    for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = (head->rot);
        vel_imu = (head->vel);
        pos_imu = (head->pos);
        acc_imu = (tail->acc);
        angvel_avr = (tail->gyr);

        for (; it_pcl->time / double(1000) > head->offset_time; it_pcl--) {
            dt = it_pcl->time / double(1000) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * p_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            Mat3d R_i(R_imu * math::exp(angvel_avr, dt).matrix());

            Vec3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            Vec3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos_);
            Vec3d p_compensate = imu_state.offset_R_lidar_.inverse() *
                                 (imu_state.rot_.inverse() *
                                      (R_i * (imu_state.offset_R_lidar_ * P_i + imu_state.offset_t_lidar_) + T_ei) -
                                  imu_state.offset_t_lidar_);  // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            if (it_pcl == pcl_out->points.begin()) {
                break;
            }
        }
    }
}

void ImuProcess::Process(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &scan) {
    if (meas.imu_.empty()) {
        return;
    }

    if (imu_need_init_) {
        /// The very first lidar frame
        IMUInit(meas, kf_state, init_iter_num_);

        imu_need_init_ = true;

        last_imu_ = meas.imu_.back();

        auto imu_state = kf_state.GetX();
        if (init_iter_num_ > max_init_count_) {
            cov_acc_ *= pow(G_m_s2 / mean_acc_.norm(), 2);
            imu_need_init_ = false;

            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            LOG(INFO) << "imu init done, bg: " << imu_state.bg_.transpose() << ", ba: " << imu_state.ba_.transpose();
        } else {
            LOG(INFO) << "waiting for imu init ... " << init_iter_num_;
        }

        return;
    }

    Timer::Evaluate([&, this]() { UndistortPcl(meas, kf_state, scan); }, "Undistort Pcl");
}

}  // namespace lightning
