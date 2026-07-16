#include "bbs_relocalizer.h"

#include <glog/logging.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "common/eigen_types.h"

namespace lightning::loc {

bool BBSRelocalizer::Init(const Options& options) {
    options_ = options;
    if (!options_.enable_) {
        return false;
    }

    bbs_ = std::make_unique<gpu::BBS3D>();
    bbs_->set_score_threshold_percentage(static_cast<float>(options_.score_threshold_percentage_));
    bbs_->set_voxel_expantion_rate(2.0f);

    if (options_.timeout_msec_ > 0) {
        bbs_->enable_timeout();
        bbs_->set_timeout_duration_in_msec(options_.timeout_msec_);
    } else {
        bbs_->disable_timeout();
    }

    // 角度搜索范围（roll/pitch 小范围，yaw 全范围）
    Eigen::Vector3f min_rpy(options_.min_roll_, options_.min_pitch_, options_.min_yaw_);
    Eigen::Vector3f max_rpy(options_.max_roll_, options_.max_pitch_, options_.max_yaw_);
    if (max_rpy.z() >= 6.28f) {
        max_rpy.z() = 2.0f * M_PI;
    }
    bbs_->set_angular_search_range(min_rpy, max_rpy);

    // 优先加载预构建体素图（快），否则从 global.pcd 运行时构建
    if (!options_.voxelmaps_path_.empty() && bbs_->set_voxelmaps_coords(options_.voxelmaps_path_)) {
        LOG(INFO) << "[BBS] Loaded prebuilt voxelmaps from " << options_.voxelmaps_path_;
        ready_ = true;
        return true;
    }

    if (options_.global_pcd_path_.empty()) {
        LOG(ERROR) << "[BBS] No global.pcd path and no prebuilt voxelmaps, BBS disabled";
        return false;
    }

    LOG(INFO) << "[BBS] Loading global map from " << options_.global_pcd_path_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(options_.global_pcd_path_, *map_cloud) == -1) {
        LOG(ERROR) << "[BBS] Failed to load global.pcd: " << options_.global_pcd_path_;
        return false;
    }

    std::vector<Eigen::Vector3f> tar_points;
    tar_points.reserve(map_cloud->size());
    for (const auto& pt : map_cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            tar_points.emplace_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
        }
    }
    LOG(INFO) << "[BBS] Building hierarchical voxel map from " << tar_points.size() << " points (GPU)...";

    bbs_->set_tar_points(tar_points, static_cast<float>(options_.min_level_res_), options_.max_level_);
    bbs_->set_trans_search_range(tar_points);

    // GPU版不支持save_voxel_params/save_voxelmaps_pcd，跳过保存
    // 如需预构建体素图加速启动，可用CPU版的voxelmaps_saver工具离线生成

    ready_ = true;
    LOG(INFO) << "[BBS] Voxel map ready (GPU)";
    return true;
}

std::optional<SE3> BBSRelocalizer::Relocalize(CloudPtr scan) {
    if (!ready_ || !bbs_) {
        return std::nullopt;
    }

    // 降采样源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    src_cloud->reserve(scan->size());
    for (const auto& pt : scan->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
          src_cloud->emplace_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }
    }

    if (options_.src_leaf_size_ > 0.0 && !src_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(options_.src_leaf_size_, options_.src_leaf_size_, options_.src_leaf_size_);
        voxel.setInputCloud(src_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel.filter(*filtered);
        *src_cloud = *filtered;
    }

    if (src_cloud->empty()) {
        LOG(WARNING) << "[BBS] Source cloud empty after filtering";
        return std::nullopt;
    }

    std::vector<Eigen::Vector3f> src_points;
    src_points.reserve(src_cloud->size());
    for (const auto& pt : src_cloud->points) {
        src_points.emplace_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
    }

    bbs_->set_src_points(src_points);
    bbs_->localize();

    last_elapsed_msec_ = bbs_->get_elapsed_time();
    last_score_percentage_ = bbs_->get_best_score_percentage();

    if (!bbs_->has_localized()) {
        if (bbs_->has_timed_out()) {
            LOG(WARNING) << "[BBS] Relocalization timed out (" << last_elapsed_msec_ << " ms)";
        } else {
            LOG(WARNING) << "[BBS] Relocalization failed, score " << bbs_->get_best_score()
                         << " (" << last_score_percentage_ * 100.0 << "%), below threshold";
        }
        return std::nullopt;
    }

    // GPU版返回 Matrix4f，转成 double
    Eigen::Matrix4f pose_mat = bbs_->get_global_pose();
    Eigen::Matrix3d R = pose_mat.block<3, 3>(0, 0).cast<double>();
    Eigen::Vector3d t = pose_mat.block<3, 1>(0, 3).cast<double>();
    Quatd q(R);
    q.normalize();

    LOG(INFO) << "[BBS] Relocalized! score " << bbs_->get_best_score()
              << " (" << last_score_percentage_ * 100.0 << "%), time " << last_elapsed_msec_ << " ms"
              << ", pos: " << t.transpose();

    return SE3(q, t);
}

}  // namespace lightning::loc
