//
// Created by xiang on 25-6-23.
//

#include "core/g2p5/g2p5.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <thread>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <opencv2/opencv.hpp>
#include "utils/timer.h"
#include "yaml-cpp/yaml.h"

namespace lightning::g2p5 {

// 前向声明（ApplyNavMapFilter 定义在 namespace 末尾）
static cv::Mat ApplyNavMapFilter(const cv::Mat& original, const std::string& filter_mode,
                                  bool save_debug_map, const std::string& debug_dir);

G2P5::~G2P5() { Quit(); }

void G2P5::Quit() {
    quit_flag_ = true;

    if (options_.online_mode_) {
        draw_frontend_map_thread_.Quit();
    }

    if (draw_backend_map_thread_.joinable()) {
        draw_backend_map_thread_.join();
    }
}

void G2P5::PushKeyframe(Keyframe::Ptr kf) {
    UL lock(kf_mutex_);
    all_keyframes_.emplace_back(kf);

    if (options_.online_mode_) {
        draw_frontend_map_thread_.AddMessage(kf);
    } else {
        RenderFront(kf);
    }
}

void G2P5::RenderFront(Keyframe::Ptr kf) {
    {
        UL lock(frontend_mutex_);
        frontend_current_ = kf;
    }

    {
        UL lock{newest_map_mutex_};

        lightning::Timer::Evaluate([&]() { AddKfToMap({kf}, frontend_map_); }, "G2P5 Occupancy Mapping", false);
        newest_map_ = frontend_map_;
    }

    /// 向外回调
    if (map_update_cb_) {
        map_update_cb_(newest_map_);
    }
}

void G2P5::RedrawGlobalMapBackend() { backend_redraw_flag_ = true; }

G2P5MapPtr G2P5::RedrawGlobalMapOffline() {
    LOG(INFO) << "RedrawGlobalMapOffline: start";

    std::vector<Keyframe::Ptr> all_keyframes;
    {
        UL lock(kf_mutex_);
        all_keyframes = all_keyframes_;
    }

    if (all_keyframes.empty()) {
        LOG(WARNING) << "RedrawGlobalMapOffline: no keyframes, skip";
        return nullptr;
    }

    G2P5Map::Options opt;
    opt.resolution_ = options_.grid_map_resolution_;
    auto offline_map = std::make_shared<G2P5Map>(opt);

    AddKfToMap(all_keyframes, offline_map);

    LOG(INFO) << "RedrawGlobalMapOffline: done, " << all_keyframes.size() << " keyframes";

    return offline_map;
}

void G2P5::RenderBack() {
    while (!quit_flag_) {
        while (!backend_redraw_flag_ && !quit_flag_) {
            sleep(1);
        }

        if (quit_flag_) {
            break;
        }

        is_busy_ = true;

        /// 后端重绘被触发
        backend_redraw_flag_ = false;

        /// 重新绘制整张地图，如果中间过程又被重绘了，则退出
        std::vector<Keyframe::Ptr> all_keyframes;
        {
            UL lock(kf_mutex_);
            all_keyframes = all_keyframes_;
        }

        if (all_keyframes.empty()) {
            is_busy_ = false;
            continue;
        }

        G2P5Map::Options opt;
        opt.resolution_ = options_.grid_map_resolution_;
        backend_map_ = std::make_shared<G2P5Map>(opt);

        /// 预先用全部关键帧对空地图做一次 ResizeMap，确定最终包围盒
        /// 这样后续逐帧 AddKfToMap 时 ResizeMap 不会再触发 Resize()，
        /// 避免反复搬移已有数据产生坐标累积误差（重影）
        ResizeMap(all_keyframes, backend_map_);

        bool abort = false;
        LOG(INFO) << "[G2P5] backend redraw start, total " << all_keyframes.size() << " keyframes";
        for (auto cur_kf = all_keyframes.begin(); cur_kf != all_keyframes.end(); ++cur_kf) {
            AddKfToMap({*cur_kf}, backend_map_);
            if (backend_redraw_flag_) {
                LOG(INFO) << "backend redraw triggered in process, abort";
                abort = true;
                break;
            }
            if (quit_flag_) {
                abort = true;
                break;
            }
        }

        if (abort) {
            /// 继续重绘
            is_busy_ = false;
            continue;
        }

        /// 处理重绘期间新到来的帧（按向量顺序追加，避免用 GetID() 当下标的歧义）
        // std::vector<Keyframe::Ptr> new_kfs;
        // {
        //     UL lock(kf_mutex_);
        //     if (all_keyframes_.size() > all_keyframes.size()) {
        //         new_kfs.assign(all_keyframes_.begin() + all_keyframes.size(), all_keyframes_.end());
        //     }
        // }
        // if (!new_kfs.empty()) {
        //     AddKfToMap(new_kfs, backend_map_);
        // }

        {
            /// 同步前后端地图，替换newest map
            UL lock{newest_map_mutex_};
            frontend_map_ = backend_map_;
            newest_map_ = frontend_map_;
        }

        /// 向外回调
        if (map_update_cb_) {
            map_update_cb_(newest_map_);
        }

        is_busy_ = false;
    }

    LOG(INFO) << "backend render quit";
}

bool G2P5::ResizeMap(const std::vector<Keyframe::Ptr> &kfs, G2P5MapPtr &map) {
    /// 重设地图大小
    float init_min_x, init_min_y, init_max_x, init_max_y;
    float min_x, min_y, max_x, max_y;
    map->GetMinAndMax(init_min_x, init_min_y, init_max_x, init_max_y);
    min_x = init_min_x;
    min_y = init_min_y;
    max_x = init_max_x;
    max_y = init_max_y;

    /// 从后往前迭代
    for (auto it = kfs.begin(); it != kfs.end(); ++it) {
        if (quit_flag_) {
            return true;
        }

        auto kf = *it;
        SE3 pose = kf->GetOptPose();
        auto cloud = kf->GetCloud();

        if (pose.translation().x() < min_x) {
            min_x = pose.translation().x();
        }

        if (pose.translation().y() < min_y) {
            min_y = pose.translation().y();
        }

        if (pose.translation().x() > max_x) {
            max_x = pose.translation().x();
        }

        if (pose.translation().y() > max_y) {
            max_y = pose.translation().y();
        }

        if (min_x > max_x || min_y > max_y) {
            return false;
        }

        for (size_t i = 0; i < cloud->points.size(); i += 10) {
            float range = cloud->points[i].getVector3fMap().norm();

            if (range > options_.usable_scan_range_ || range <= 0.01 || std::isnan(range)) {
                continue;
            }

            Vec3d point = pose * cloud->points[i].getVector3fMap().cast<double>();

            if ((point.x() - 1) < min_x) {
                min_x = point.x() - 1;
            }
            if ((point.y() - 1) < min_y) {
                min_y = point.y() - 1;
            }
            if ((point.x() + 1) > max_x) {
                max_x = point.x() + 1;
            }
            if ((point.y() + 1) > max_y) {
                max_y = point.y() + 1;
            }
        }
    }

    if (min_x > max_x || min_y > max_y) {
        return false;
    }

    /// resize map if necessary
    if (min_x < init_min_x || min_y < init_min_y || max_x > init_max_x || max_y > init_max_y) {
        min_x = (min_x > init_min_x) ? init_min_x : min_x;
        min_y = (min_y > init_min_y) ? init_min_y : min_y;
        max_x = (max_x < init_max_x) ? init_max_x : max_x;
        max_y = (max_y < init_max_y) ? init_max_y : max_y;

        float r = map->GetGridResolution();
        min_x = static_cast<int>((floor)(min_x / r)) * r;
        min_y = static_cast<int>((floor)(min_y / r)) * r;
        max_x = static_cast<int>((ceil)(max_x / r)) * r;
        max_y = static_cast<int>((ceil)(max_y / r)) * r;
        map->Resize(min_x, min_y, max_x, max_y);

        // LOG(INFO) << "map resized to " << min_x << ", " << min_y << ", " << max_x << ", " << max_y;
    }

    return true;
}

bool G2P5::AddKfToMap(const std::vector<Keyframe::Ptr> &kfs, G2P5MapPtr &map) {
    /// 如果需要，更新地图大小
    ResizeMap(kfs, map);

    for (const auto &kf : kfs) {
        Convert3DTo2DScan(kf, map);
    }

    return true;
}

void G2P5::SaveGridMap(const std::string& save_path) {
    LOG(INFO) << "SaveGridMap: start, path=" << save_path;

    G2P5MapPtr offline_map = RedrawGlobalMapOffline();
    if (!offline_map) {
        LOG(WARNING) << "SaveGridMap: no map data, skip";
        return;
    }

    // ToROS 转换：OccupancyGrid → cv::Mat（ROS 惯例：Y 轴翻转，Free=255, Occupied=0, Unknown=128）
    auto ros_map = offline_map->ToROS();
    const int width  = static_cast<int>(ros_map.info.width);
    const int height = static_cast<int>(ros_map.info.height);

    cv::Mat nav_image(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
        const int row = y * width;
        for (int x = 0; x < width; ++x) {
            int8_t data = ros_map.data[row + x];
            if (data == 0) {
                nav_image.at<uchar>(height - 1 - y, x) = 255;  // Free → White
            } else if (data == 100) {
                nav_image.at<uchar>(height - 1 - y, x) = 0;    // Occupied → Black
            } else {
                nav_image.at<uchar>(height - 1 - y, x) = 128;  // Unknown → Gray
            }
        }
    }

    // 可选：噪声过滤生成导航地图
    if (options_.build_nav_map_) {
        std::string debug_dir = options_.save_debug_map_ ? save_path + "/debug_map" : "";
        nav_image = ApplyNavMapFilter(nav_image, options_.nav_map_filter_mode_,
                                      options_.save_debug_map_, debug_dir);
        LOG(INFO) << "SaveGridMap: nav map filter applied (mode=" << options_.nav_map_filter_mode_ << ")";
    }

    // 保存为 PNG 格式
    if (!cv::imwrite(save_path + "/map.png", nav_image)) {
        LOG(ERROR) << "SaveGridMap: failed to write map.png";
        return;
    }
    LOG(INFO) << "SaveGridMap: map.png saved";

    // 保存 map.yaml
    std::ofstream yaml_file(save_path + "/map.yaml");
    if (!yaml_file.is_open()) {
        LOG(ERROR) << "SaveGridMap: failed to open map.yaml";
        return;
    }

    try {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "image"            << YAML::Value << "map.png";
        emitter << YAML::Key << "mode"             << YAML::Value << "trinary";
        emitter << YAML::Key << "width"            << YAML::Value << width;
        emitter << YAML::Key << "height"           << YAML::Value << height;
        emitter << YAML::Key << "resolution"       << YAML::Value << ros_map.info.resolution;
        std::vector<double> orig{ros_map.info.origin.position.x,
                                 ros_map.info.origin.position.y, 0.0};
        emitter << YAML::Key << "origin"           << YAML::Value << orig;
        emitter << YAML::Key << "negate"           << YAML::Value << 0;
        emitter << YAML::Key << "occupied_thresh"  << YAML::Value << 0.65;
        emitter << YAML::Key << "free_thresh"      << YAML::Value << 0.25;
        emitter << YAML::EndMap;
        yaml_file << emitter.c_str();
        yaml_file.close();
    } catch (...) {
        yaml_file.close();
        LOG(ERROR) << "SaveGridMap: failed to write map.yaml";
        return;
    }

    LOG(INFO) << "SaveGridMap: map.yaml saved";
}

G2P5MapPtr G2P5::GetNewestMap() {
    UL lock{newest_map_mutex_};
    LOG(INFO) << "getting newest map";
    if (newest_map_ == nullptr) {
        return nullptr;
    }

    return newest_map_->MakeDeepCopy();
}

void G2P5::Convert3DTo2DScan(Keyframe::Ptr kf, G2P5MapPtr &map) {
    SE3 Twb = kf->GetOptPose();
    Vec3d orig = Twb.translation();
    double min_th = options_.min_th_floor_;
    double max_th = options_.max_th_floor_;
    auto cloud = kf->GetCloud();

    // ── 地面高度估计（世界 Z 轴，与倾角无关）───────────────────────────────
    if (options_.esti_floor_) {
        if (!EstimateFloorZWorld(kf, Twb, floor_z_world_)) {
            floor_z_world_ = orig.z() - options_.lidar_height_;
        }
        if (options_.verbose_) {
            LOG(INFO) << "floor_z_world: " << floor_z_world_;
        }
    } else {
        floor_z_world_ = orig.z() + options_.default_floor_height_;
    }
    double lidar_h = orig.z() - floor_z_world_;  // LiDAR 世界系离地高度

    // ── 0.1° 角度分箱 miss 射线去重（按世界 XY 方向）────────────────────
    constexpr int kNumAngleBins = 3600;
    constexpr double kBinStep = 2.0 * M_PI / kNumAngleBins;
    struct MissCandidate {
        double dis_xy = std::numeric_limits<double>::max();
        Vec3d p_world;
        double dis_floor = 0.0;
    };
    std::vector<MissCandidate> miss_bins(kNumAngleBins);

    // 第一遍：标记 hit，收集 miss 候选
    for (const auto &pt : cloud->points) {
        if (quit_flag_) return;
        Vec3d pc(pt.x, pt.y, pt.z);
        double range = pc.norm();  // 3D 距离过滤
        if (range > options_.usable_scan_range_ || range < 0.01) continue;

        Vec3d p_world = Twb * pc;
        double dis_floor = p_world.z() - floor_z_world_;  // 世界 Z 离地高度

        bool is_obstacle = (dis_floor > min_th && dis_floor < max_th);
        bool is_floor    = (!is_obstacle && dis_floor > -min_th && dis_floor < min_th);

        if (is_obstacle) {
            map->SetHitPoint(p_world[0], p_world[1], true, dis_floor);
        }
        if (is_obstacle || is_floor) {
            // 世界 XY 方向角（miss 射线方向）
            double dx = p_world[0] - orig[0];
            double dy = p_world[1] - orig[1];
            double dis_xy = std::sqrt(dx * dx + dy * dy);
            if (dis_xy < 0.01) continue;
            double angle = std::atan2(dy, dx);
            if (angle < 0.0) angle += 2.0 * M_PI;
            int bin = static_cast<int>(angle / kBinStep);
            if (bin >= kNumAngleBins) bin = kNumAngleBins - 1;
            if (dis_xy < miss_bins[bin].dis_xy) {
                miss_bins[bin] = {dis_xy, p_world, dis_floor};
            }
        }
    }

    // 第二遍：发射 miss 射线（每 0.1° 最近点）
    for (int b = 0; b < kNumAngleBins; ++b) {
        if (miss_bins[b].dis_xy >= std::numeric_limits<double>::max()) continue;
        if (quit_flag_) return;
        const auto &c = miss_bins[b];
        map->SetMissPoint(c.p_world[0], c.p_world[1],
                          orig[0], orig[1],
                          c.dis_floor, lidar_h);
    }
}

bool G2P5::EstimateFloorZWorld(Keyframe::Ptr kf, const SE3& Twb, double& floor_z_world) {
    auto cloud = kf->GetCloud();
    Vec3d orig = Twb.translation();

    double expected_floor_z = orig.z() - options_.lidar_height_;
    double search_half = std::max(options_.lidar_height_ * 1.5, 0.5);

    std::vector<float> z_vals;
    z_vals.reserve(cloud->points.size() / 5);

    for (const auto& pt : cloud->points) {
        Vec3d pc(pt.x, pt.y, pt.z);
        float range = static_cast<float>(pc.norm());
        if (range > 20.0f || range < 0.1f) continue;

        Vec3d pw = Twb * pc;
        if (pw.z() > expected_floor_z - search_half &&
            pw.z() < expected_floor_z + search_half * 0.5) {
            z_vals.push_back(static_cast<float>(pw.z()));
        }
    }

    if (z_vals.size() < 50) return false;

    // 取底部 15% 分位数作为地面 Z（比 min 鲁棒，比 mean 不受天花板影响）
    auto it = z_vals.begin() + z_vals.size() * 15 / 100;
    std::nth_element(z_vals.begin(), it, z_vals.end());
    floor_z_world = *it;
    return true;
}

bool G2P5::DetectPlaneCoeffs(Keyframe::Ptr kf) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.25);

    CloudPtr cloud(new PointCloudType);
    for (auto &pt : kf->GetCloud()->points) {
        if (pt.z < options_.lidar_height_ + options_.default_floor_height_) {
            cloud->points.push_back(pt);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    if (cloud->size() < 200) {
        // LOG(ERROR) << "not enough points cloud->size(): " << cloud->size();
        return false;
    }

    // if (options_.verbose_) {
    //     pcl::io::savePCDFile("./data/floor_candi.pcd", *cloud);
    // }

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (coefficients->values[2] < 0.99) {
        LOG(ERROR) << "floor is not horizontal. ";
        return false;
    }

    if (inliers->indices.size() < 100) {
        LOG(ERROR) << "cannot get enough points on floor: " << inliers->indices.size();
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        floor_coeffs_[i] = coefficients->values[i];
    }
    cloud->clear();

    return true;
}

void G2P5::Init(std::string yaml_path) {
    auto yaml = YAML::LoadFile(yaml_path);
    options_.esti_floor_ = yaml["g2p5"]["esti_floor"].as<bool>();
    options_.min_th_floor_ = yaml["g2p5"]["min_th_floor"].as<float>();
    options_.max_th_floor_ = yaml["g2p5"]["max_th_floor"].as<float>();
    options_.lidar_height_ = yaml["g2p5"]["lidar_height"].as<float>();
    options_.grid_map_resolution_ = yaml["g2p5"]["grid_map_resolution"].as<float>();
    options_.default_floor_height_ = yaml["g2p5"]["floor_height"].as<float>();
    if (yaml["g2p5"]["usable_scan_range"]) {
        options_.usable_scan_range_ = yaml["g2p5"]["usable_scan_range"].as<float>();
    }
    if (yaml["g2p5"]["build_nav_map"]) {
        options_.build_nav_map_ = yaml["g2p5"]["build_nav_map"].as<bool>();
    }
    if (yaml["g2p5"]["navigation_map_filter_mode"]) {
        options_.nav_map_filter_mode_ = yaml["g2p5"]["navigation_map_filter_mode"].as<std::string>();
    }
    if (yaml["g2p5"]["save_debug_map"]) {
        options_.save_debug_map_ = yaml["g2p5"]["save_debug_map"].as<bool>();
    }

    G2P5Map::Options opt;
    opt.resolution_ = options_.grid_map_resolution_;
    frontend_map_ = std::make_shared<G2P5Map>(opt);

    if (options_.online_mode_) {
        draw_frontend_map_thread_.SetProcFunc([this](Keyframe::Ptr kf) { RenderFront(kf); });
        draw_frontend_map_thread_.Start();
    }

    draw_backend_map_thread_ = std::thread([this]() { RenderBack(); });
}

/**
 * 对占据栅格图像（White=自由, Black=障碍, Gray=未知）做连通域噪声过滤，
 * 移除面积小于阈值且无灰色邻居的孤立黑色小区域（噪点）。
 *
 * @param original       输入图像（CV_8UC1）
 * @param filter_mode    过滤强度: "normal"(面积<10), "high"(面积<50), "super"(面积<100)
 * @param save_debug_map 是否保存中间结果到 debug_dir
 * @param debug_dir      调试图片输出目录（save_debug_map=true 时有效）
 * @return               处理后的图像（CV_8UC1）
 */
static cv::Mat ApplyNavMapFilter(const cv::Mat& original,
                                  const std::string& filter_mode,
                                  bool save_debug_map,
                                  const std::string& debug_dir) {
    if (save_debug_map && !debug_dir.empty()) {
        std::filesystem::create_directories(debug_dir);
    }

    int min_noise_area = 10;
    if (filter_mode == "high") {
        min_noise_area = 50;
    } else if (filter_mode == "super") {
        min_noise_area = 100;
    }

    const int white_thresh = 250;
    const int black_thresh = 160;
    const int dilate_radius = 3;

    cv::Mat result = original.clone();

    cv::Mat white_mask, black_mask, gray_mask;
    cv::compare(original, white_thresh, white_mask, cv::CMP_GE);
    cv::compare(original, black_thresh, black_mask, cv::CMP_LE);
    cv::bitwise_and(~white_mask, ~black_mask, gray_mask);  // black_thresh < val < white_thresh

    if (save_debug_map && !debug_dir.empty()) {
        cv::imwrite(debug_dir + "/01_white_mask.png", white_mask);
        cv::imwrite(debug_dir + "/02_black_mask.png", black_mask);
        cv::imwrite(debug_dir + "/03_gray_mask.png", gray_mask);
    }

    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(black_mask, labels, stats, centroids, 8);

    if (save_debug_map && !debug_dir.empty()) {
        cv::Mat labels_vis;
        cv::normalize(labels, labels_vis, 0, 255, cv::NORM_MINMAX);
        labels_vis.convertTo(labels_vis, CV_8U);
        cv::applyColorMap(labels_vis, labels_vis, cv::COLORMAP_JET);
        cv::imwrite(debug_dir + "/04_connected_components.png", labels_vis);
    }

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * dilate_radius + 1, 2 * dilate_radius + 1));

    // ── 优化：O(3n) 替代原来的 O(K×5n) ────────────────────────────────────
    // 原方案：对每个小连通域逐一做全图 compare/dilate/bitwise_and（K 次全图操作）
    // 优化方案：
    //   1. 将 gray_mask 向外膨胀一次（等价于"gray 的邻域"）
    //   2. 第一遍扫描全图：凡是黑色像素落在 gray_expanded 内，其 label 标记为 keep
    //   3. 第二遍扫描全图：按 label_action LUT 批量应用结果
    // 总复杂度 O(3n)，与连通域数量 K 无关

    // Step 1: 将 gray 区域向外膨胀 dilate_radius，得到 gray 的邻域
    cv::Mat gray_expanded;
    cv::dilate(gray_mask, gray_expanded, kernel);

    // Step 2: 构建 label → action 的 LUT
    // action: 0 = keep（大连通域 or 有 gray 邻居）, 1 = remove（小孤立噪点）
    std::vector<uint8_t> label_action(num_labels, 0);
    for (int i = 1; i < num_labels; ++i) {
        if (stats.at<int>(i, cv::CC_STAT_AREA) < min_noise_area) {
            label_action[i] = 1;  // 暂定删除（小），后续若发现有 gray 邻居则改为 keep
        }
    }

    // Step 3: 第一遍扫描——找出触碰 gray 邻域的小连通域，将其标记为 keep
    const int height = original.rows;
    const int width  = original.cols;
    for (int y = 0; y < height; ++y) {
        const int32_t* label_row    = labels.ptr<int32_t>(y);
        const uint8_t* gray_exp_row = gray_expanded.ptr<uint8_t>(y);
        for (int x = 0; x < width; ++x) {
            int lbl = label_row[x];
            if (lbl > 0 && label_action[lbl] == 1 && gray_exp_row[x] > 0) {
                label_action[lbl] = 0;  // 有 gray 邻居 → keep（墙边缘噪点保留）
            }
        }
    }

    // Step 4: 第二遍扫描——按 LUT 批量应用（仅保存 debug 时才维护 removed/edge mask）
    cv::Mat removed_mask = save_debug_map ? cv::Mat::zeros(original.size(), CV_8U) : cv::Mat();
    cv::Mat edge_mask    = save_debug_map ? cv::Mat::zeros(original.size(), CV_8U) : cv::Mat();

    for (int y = 0; y < height; ++y) {
        const int32_t* label_row = labels.ptr<int32_t>(y);
        uint8_t*       result_row = result.ptr<uint8_t>(y);
        for (int x = 0; x < width; ++x) {
            int lbl = label_row[x];
            if (lbl > 0 && label_action[lbl] == 1) {
                result_row[x] = 255;  // 孤立小噪点 → 置白
                if (save_debug_map) removed_mask.at<uint8_t>(y, x) = 255;
            }
        }
    }

    // debug 模式下重建 edge_mask（触碰 gray 但被保留的小连通域）
    if (save_debug_map) {
        // 先收集"原本小但有 gray 邻居"的 label set
        std::vector<bool> is_small_kept(num_labels, false);
        for (int i = 1; i < num_labels; ++i) {
            if (stats.at<int>(i, cv::CC_STAT_AREA) < min_noise_area && label_action[i] == 0) {
                is_small_kept[i] = true;
            }
        }
        // 单遍扫描填充 edge_mask
        for (int y = 0; y < height; ++y) {
            const int32_t* label_row = labels.ptr<int32_t>(y);
            for (int x = 0; x < width; ++x) {
                int lbl = label_row[x];
                if (lbl > 0 && is_small_kept[lbl]) edge_mask.at<uint8_t>(y, x) = 255;
            }
        }
    }

    if (save_debug_map && !debug_dir.empty()) {
        cv::imwrite(debug_dir + "/05_edge_mask.png", edge_mask);

        cv::Mat annotated;
        cv::cvtColor(original, annotated, cv::COLOR_GRAY2BGR);

        cv::Mat removed_bgr = cv::Mat::zeros(original.size(), CV_8UC3);
        removed_bgr.setTo(cv::Scalar(255, 0, 0), removed_mask);
        cv::addWeighted(annotated, 1.0, removed_bgr, 1.0, 0, annotated);

        cv::Mat edge_bgr = cv::Mat::zeros(original.size(), CV_8UC3);
        edge_bgr.setTo(cv::Scalar(0, 255, 0), edge_mask);
        cv::addWeighted(annotated, 1.0, edge_bgr, 1.0, 0, annotated);

        cv::imwrite(debug_dir + "/06_annotated.png", annotated);
        cv::imwrite(debug_dir + "/07_final_result.png", result);

        cv::Mat comparison;
        cv::hconcat(original, result, comparison);
        cv::imwrite(debug_dir + "/08_comparison.png", comparison);
    }

    return result;
}

}  // namespace lightning::g2p5