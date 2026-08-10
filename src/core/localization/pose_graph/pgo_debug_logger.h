#pragma once

#include <fstream>
#include <mutex>
#include <string>

#include "common/eigen_types.h"
#include "common/nav_state.h"

namespace lightning::loc {

/**
 * PGO调试日志记录器
 * 记录融合过程中所有中间位姿，便于分析和可视化
 */
class PGODebugLogger {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static PGODebugLogger& Instance() {
        static PGODebugLogger instance;
        return instance;
    }

    /**
     * 初始化日志文件
     * @param output_dir 输出目录
     * @return 是否成功
     */
    bool Init(const std::string& output_dir = "/tmp/pgo_debug");

    /**
     * 记录DR输入位姿
     */
    void LogDRInput(double timestamp, const SE3& pose);

    /**
     * 记录LO输入位姿
     */
    void LogLOInput(double timestamp, const SE3& pose);

    /**
     * 记录LidarLoc输入位姿
     */
    void LogLidarLocInput(double timestamp, const SE3& pose, double confidence);

    /**
     * 记录内插后的位姿（给到优化器的值）
     * @param dr_valid DR内插是否成功
     * @param lo_valid LO内插是否成功
     * @param loc_valid LidarLoc内插是否成功（通常直接来自上游，不需要内插）
     */
    void LogInterpolatedPoses(double timestamp, const SE3& dr_pose, bool dr_valid, const SE3& lo_pose, bool lo_valid,
                              const SE3& lidar_loc_pose, bool loc_valid);

    /**
     * 记录PGO优化后的位姿
     */
    void LogOptimizedPose(double timestamp, const SE3& pose);

    /**
     * 记录外推后的位姿
     */
    void LogExtrapolatedPose(double timestamp, const SE3& pose, double dt_extrap);

    /**
     * 记录平滑后的最终输出位姿
     */
    void LogSmoothedOutput(double timestamp, const SE3& pose, double smooth_factor);

    /**
     * 关闭日志文件
     */
    void Close();

   private:
    PGODebugLogger() = default;
    ~PGODebugLogger() { Close(); }

    PGODebugLogger(const PGODebugLogger&) = delete;
    PGODebugLogger& operator=(const PGODebugLogger&) = delete;

    void WriteHeader();
    std::string PoseToString(const SE3& pose);

    std::mutex mutex_;
    std::ofstream csv_file_;
    bool initialized_ = false;
};

}  // namespace lightning::loc