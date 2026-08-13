#include "pgo_debug_logger.h"

#include <filesystem>
#include <iomanip>
#include <iostream>

#include <glog/logging.h>

namespace lightning::loc {

bool PGODebugLogger::Init(const std::string& output_dir) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        LOG(WARNING) << "PGODebugLogger already initialized";
        return true;
    }

    // 创建输出目录
    try {
        std::filesystem::create_directories(output_dir);
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to create output directory: " << e.what();
        return false;
    }

    // 生成文件名（带时间戳）
    std::string filename = output_dir + "/pgo_debug_" + std::to_string(std::time(nullptr)) + ".csv";

    csv_file_.open(filename, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        LOG(ERROR) << "Failed to open file: " << filename;
        return false;
    }

    WriteHeader();
    initialized_ = true;

    LOG(INFO) << "PGODebugLogger initialized, output to: " << filename;
    return true;
}

void PGODebugLogger::LogDRInput(double timestamp, const SE3& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // CSV列结构（总共33列）：
    // 1-2: timestamp, stage
    // 3-9: tx,ty,tz,qx,qy,qz,qw (主位姿)
    // 10-11: confidence, smooth_factor (空)
    // 12-18: dr_tx~dr_qw (空)
    // 19-25: lo_tx~lo_qw (空)
    // 26-32: loc_tx~loc_qw (空)
    // 33: dt_extrap (空)
    // 逗号计数: 1(ts)+1(stage)+6(pose内部)+1+2+21 = 32个逗号 → 33列

    csv_file_ << std::fixed << std::setprecision(6) << timestamp << ","
              << "DR_INPUT" << "," << PoseToString(pose) << ","  // 列3-9: 主位姿 + 分隔符
              << ",,"                                            // 列10-11: confidence, smooth_factor (空)
              << ",,,,,,,,,,,,,,,,,,,,,"                         // 列12-32: 21个空值 + 分隔符
              << "\n";                                           // 列33: dt_extrap (空)
    csv_file_.flush();
}

void PGODebugLogger::LogLOInput(double timestamp, const SE3& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // CSV列结构（总共33列）：同DR_INPUT

    csv_file_ << std::fixed << std::setprecision(6) << timestamp << ","
              << "LO_INPUT" << "," << PoseToString(pose) << ","  // 列3-9: 主位姿 + 分隔符
              << ",,"                                            // 列10-11: confidence, smooth_factor (空)
              << ",,,,,,,,,,,,,,,,,,,,,"                         // 列12-32: 21个空值 + 分隔符
              << "\n";                                           // 列33: dt_extrap (空)
    csv_file_.flush();
}

void PGODebugLogger::LogLidarLocInput(double timestamp, const SE3& pose, double confidence) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // CSV列结构（总共33列）：
    // 1-2: timestamp, stage
    // 3-9: tx,ty,tz,qx,qy,qz,qw (主位姿)
    // 10: confidence
    // 11: smooth_factor (空)
    // 12-18: dr_tx~dr_qw (空)
    // 19-25: lo_tx~lo_qw (空)
    // 26-32: loc_tx~loc_qw (空)
    // 33: dt_extrap (空)
    // 逗号计数: 1+1+6+1+1+1+21 = 32个逗号 → 33列

    csv_file_ << std::fixed << std::setprecision(6) << timestamp << ","
              << "LIDARLOC_INPUT" << "," << PoseToString(pose) << ","  // 列3-9: 主位姿 + 分隔符
              << confidence << ","                                     // 列10: confidence + 分隔符
              << ","                                                   // 列11: smooth_factor (空)
              << ",,,,,,,,,,,,,,,,,,,,,"                               // 列12-32: 21个空值 + 分隔符
              << "\n";                                                 // 列33: dt_extrap (空)
    csv_file_.flush();
}

void PGODebugLogger::LogInterpolatedPoses(double timestamp, const SE3& dr_pose, bool dr_valid, const SE3& lo_pose,
                                          bool lo_valid, const SE3& lidar_loc_pose, bool loc_valid) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // CSV列结构（总共33列）：
    // 1-2: timestamp, stage
    // 3-9: tx,ty,tz,qx,qy,qz,qw (空，INTERPOLATED阶段主位姿不使用)
    // 10-11: confidence, smooth_factor (空)
    // 12-18: dr_tx,dr_ty,dr_tz,dr_qx,dr_qy,dr_qz,dr_qw (内插失败时为空)
    // 19-25: lo_tx,lo_ty,lo_tz,lo_qx,lo_qy,lo_qz,lo_qw (内插失败时为空)
    // 26-32: loc_tx,loc_ty,loc_tz,loc_qx,loc_qy,loc_qz,loc_qw (内插失败时为空)
    // 33: dt_extrap (空)
    //
    // 内插失败处理策略：
    // - 成功：记录实际位姿值
    // - 失败：保持CSV字段为空（不记录0值），避免可视化时误导分析
    // 逗号计数: 1+1+7+2+6/7*3+2/1 = 32个逗号 → 33列

    // 内插失败时，输出空值而非0值
    std::string dr_str = dr_valid ? PoseToString(dr_pose) : ",,,,,,";
    std::string lo_str = lo_valid ? PoseToString(lo_pose) : ",,,,,,";
    std::string loc_str = loc_valid ? PoseToString(lidar_loc_pose) : ",,,,,,";

    csv_file_ << std::fixed << std::setprecision(6) << timestamp << ","
              << "INTERPOLATED" << ","
              << ",,,,,,,"         // 列3-9: 7个空值 (主位姿)
              << ",,"              // 列10-11: confidence, smooth_factor (空)
              << dr_str << ","     // 列12-18: dr位姿 + 分隔符
              << lo_str << ","     // 列19-25: lo位姿 + 分隔符
              << loc_str << "\n";  // 列26-32: loc位姿 (列33自动为空)
    csv_file_.flush();
}

void PGODebugLogger::LogOptimizedPose(double timestamp, const SE3& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // CSV列结构（总共33列）：同DR_INPUT

    csv_file_ << std::fixed << std::setprecision(6) << timestamp << ","
              << "OPTIMIZED" << "," << PoseToString(pose) << ","  // 列3-9: 主位姿 + 分隔符
              << ",,"                                             // 列10-11: confidence, smooth_factor (空)
              << ",,,,,,,,,,,,,,,,,,,,,"                          // 列12-32: 21个空值 + 分隔符
              << "\n";                                            // 列33: dt_extrap (空)
    csv_file_.flush();
}

void PGODebugLogger::LogExtrapolatedPose(double timestamp, const SE3& pose, double dt_extrap) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // CSV列结构（总共33列）：
    // 1-2: timestamp, stage
    // 3-9: tx,ty,tz,qx,qy,qz,qw (主位姿)
    // 10-11: confidence, smooth_factor (空)
    // 12-18: dr_tx~dr_qw (空)
    // 19-25: lo_tx~lo_qw (空)
    // 26-32: loc_tx~loc_qw (空)
    // 33: dt_extrap
    // 逗号计数: 1+1+6+1+2+20+1 = 32个逗号 → 33列

    csv_file_ << std::fixed << std::setprecision(6) << timestamp << ","
              << "EXTRAPOLATED" << "," << PoseToString(pose) << ","  // 列3-9: 主位姿 + 分隔符
              << ",,"                                                // 列10-11: confidence, smooth_factor (空)
              << ",,,,,,,,,,,,,,,,,,,,"                              // 列12-32: 20个空值
              << dt_extrap << "\n";                                  // 列33: dt_extrap
    csv_file_.flush();
}

void PGODebugLogger::LogSmoothedOutput(double timestamp, const SE3& pose, double smooth_factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // CSV列结构（总共33列）：
    // 1-2: timestamp, stage
    // 3-9: tx,ty,tz,qx,qy,qz,qw (主位姿)
    // 10: confidence (空)
    // 11: smooth_factor
    // 12-18: dr_tx~dr_qw (空)
    // 19-25: lo_tx~lo_qw (空)
    // 26-32: loc_tx~loc_qw (空)
    // 33: dt_extrap (空)
    // 逗号计数: 1+1+6+1+1+1+20 = 32个逗号 → 33列

    csv_file_ << std::fixed << std::setprecision(6) << timestamp << ","
              << "SMOOTHED_OUTPUT" << "," << PoseToString(pose) << ","  // 列3-9: 主位姿 + 分隔符
              << ","                                                    // 列10: confidence (空)
              << smooth_factor << ","                                   // 列11: smooth_factor + 分隔符
              << ",,,,,,,,,,,,,,,,,,,,"                                 // 列12-32: 20个空值
              << "\n";                                                  // 列33: dt_extrap (空)
    csv_file_.flush();
}

void PGODebugLogger::Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (csv_file_.is_open()) {
        csv_file_.close();
        LOG(INFO) << "PGODebugLogger closed";
    }
    initialized_ = false;
}

void PGODebugLogger::WriteHeader() {
    csv_file_ << "# PGO Debug Log - Intermediate Poses\n"
              << "# Format: CSV\n"
              << "#\n"
              << "# Columns:\n"
              << "#   timestamp       - 时间戳 (秒)\n"
              << "#   stage           - 融合阶段\n"
              << "#   tx,ty,tz        - 平移 (米)\n"
              << "#   qx,qy,qz,qw     - 四元数\n"
              << "#   confidence      - LidarLoc置信度\n"
              << "#   smooth_factor   - 平滑因子\n"
              << "#   dr_tx,dr_ty,dr_tz,dr_qx,dr_qy,dr_qz,dr_qw     - 内插后DR位姿\n"
              << "#   lo_tx,lo_ty,lo_tz,lo_qx,lo_qy,lo_qz,lo_qw     - 内插后LO位姿\n"
              << "#   loc_tx,loc_ty,loc_tz,loc_qx,loc_qy,loc_qz,loc_qw - 内插后LidarLoc位姿\n"
              << "#   dt_extrap       - 外推时间差 (秒)\n"
              << "#\n"
              << "# Stages:\n"
              << "#   DR_INPUT        - DR输入位姿\n"
              << "#   LO_INPUT        - LO输入位姿\n"
              << "#   LIDARLOC_INPUT  - LidarLoc输入位姿\n"
              << "#   INTERPOLATED    - 内插后给优化器的位姿\n"
              << "#   OPTIMIZED       - PGO优化后位姿\n"
              << "#   EXTRAPOLATED    - 外推后位姿\n"
              << "#   SMOOTHED_OUTPUT - 平滑后最终输出\n"
              << "#\n";

    // CSV 表头
    csv_file_ << "timestamp,stage,"
              << "tx,ty,tz,qx,qy,qz,qw,"
              << "confidence,smooth_factor,"
              << "dr_tx,dr_ty,dr_tz,dr_qx,dr_qy,dr_qz,dr_qw,"
              << "lo_tx,lo_ty,lo_tz,lo_qx,lo_qy,lo_qz,lo_qw,"
              << "loc_tx,loc_ty,loc_tz,loc_qx,loc_qy,loc_qz,loc_qw,"
              << "dt_extrap\n";
}

std::string PGODebugLogger::PoseToString(const SE3& pose) {
    auto t = pose.translation();
    auto q = pose.unit_quaternion();

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << t.x() << "," << t.y() << "," << t.z() << "," << q.x() << "," << q.y()
        << "," << q.z() << "," << q.w();
    return oss.str();
}

}  // namespace lightning::loc