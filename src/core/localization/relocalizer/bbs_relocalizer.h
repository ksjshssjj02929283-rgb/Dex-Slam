#pragma once

#include <optional>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/eigen_types.h"
#include "common/point_def.h"

#ifdef BUILD_CUDA
#include <gpu_bbs3d/bbs3d.cuh>
namespace bbs_impl = gpu;
using BBSFloat = float;
using BBSVec3 = Eigen::Vector3f;
#else
#include <cpu_bbs3d/bbs3d.hpp>
namespace bbs_impl = cpu;
using BBSFloat = double;
using BBSVec3 = Eigen::Vector3d;
#endif

namespace lightning::loc {

/**
 * 3D-BBS 全局重定位包装器 (GPU版)
 *
 * 用途：在无初始位姿（冷启动）时，对单帧 LiDAR 扫描做
 *       branch-and-bound 全局搜索，给出 map 系下的粗位姿。
 *       该粗位姿随后作为 NDT 精配准的初值交给 LidarLoc。
 *
 * 数据来源：建图时 slam3d 已保存 global.pcd，本类直接加载它构建
 *           层级体素图（GPU 版，依赖 CUDA + thrust）。
 */
class BBSRelocalizer {
   public:
    struct Options {
        Options() {}

        bool enable_ = false;                // 是否启用 BBS 重定位
        std::string global_pcd_path_;       // global.pcd 路径（空=不加载地图）
        std::string voxelmaps_path_;        // 预构建体素图路径（空=运行时构建）
        double min_level_res_ = 1.0;        // 最细层分辨率 [m]
        int max_level_ = 6;                  // 层级数
        double min_roll_ = -0.02;
        double max_roll_ = 0.02;
        double min_pitch_ = -0.02;
        double max_pitch_ = 0.02;
        double min_yaw_ = 0.0;
        double max_yaw_ = 6.28;             // 6.28 自动转为 2π
        double score_threshold_percentage_ = 0.4;
        double src_leaf_size_ = 2.0;        // 源点云降采样 [m]（0=不降采样）
        int timeout_msec_ = 240000;         // 超时保护，0=不超时
        int num_threads_ = 8;               // GPU版不使用，保留兼容配置
    };

    BBSRelocalizer() = default;
    ~BBSRelocalizer() = default;

    /// 初始化：加载 global.pcd 并构建体素图（或加载预构建体素图）
    bool Init(const Options& options);

    /// 对单帧扫描做全局重定位，返回 map 系 SE3（失败返回 nullopt）
    std::optional<SE3> Relocalize(CloudPtr scan);

    /// 体素图是否已就绪
    bool Ready() const { return ready_; }

    /// 最近一次重定位耗时 [msec]
    double LastElapsedMsec() const { return last_elapsed_msec_; }

    /// 最近一次重定位得分百分比 [0,1]
    double LastScorePercentage() const { return last_score_percentage_; }

   private:
    Options options_;
    std::unique_ptr<bbs_impl::BBS3D> bbs_;
    bool ready_ = false;
    double last_elapsed_msec_ = 0.0;
    double last_score_percentage_ = 0.0;
};

}  // namespace lightning::loc
