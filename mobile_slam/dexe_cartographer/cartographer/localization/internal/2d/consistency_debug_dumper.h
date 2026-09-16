/*
 * Copyright 2026 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_CONSISTENCY_DEBUG_DUMPER_H_
#define CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_CONSISTENCY_DEBUG_DUMPER_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "opencv2/core.hpp"

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace localization {

// 一致性检查 / 恢复现场快照（**默认开启的基础功能**）。
//
// 用途：把「当前位姿下的点云」与「用于匹配的 local submap」叠加渲染到同一张 PNG，
// 让「匹配对不上」这件事从一个分数变成可以直接看的图。
//
// 为什么可以常开：快照只在**一致性检查失败**时产生，健康运行下一张都不写、零开销；
// 开销正比于故障率而非帧率。实测单张约 140 KB（±15 m local submap、scale=2），
// 环形保留 1000 张 ≈ 136 MB 稳态上限。
//
// 关键取舍：
//  * 栅格提取在调用线程完成（栅格生命周期归调用方），点云与元数据深拷贝，
//    **PNG 编码与落盘全部在后台线程**——定位主链路只付约 1-3 ms。
//    这一点是必需的：本工具服务于时序问题排查，若同步写盘就会改变被观测对象本身。
//  * 队列有界，满则丢弃新帧并计数（保留突发的**开头**，那里才有因果）。
//  * 落盘目录是**环形**的：写满 max_files 后删除最早的一张，长时间跑不会撑满磁盘，
//    留在盘上的永远是最近的现场。注意这与上一条取向相反且都是刻意的——
//    队列丢新是为了保住一次突发的**开头**，目录删旧是为了保住整次运行的**最近**。
//  * 各雷达合并前的去畸变点云分色绘制——若多雷达合并的时间对齐有问题，
//    两片点云会整体错开一个角度，肉眼可辨。
class ConsistencyDebugDumper {
 public:
  enum class Event {
    kCheckFailed,         // 一致性检查失败 → 触发验证/恢复流程
    kRecoverySucceeded,   // 局部地图恢复成功
    kRecoveryFailed,      // 全部恢复手段失败 → 定位丢失
    kFrame,               // 逐帧快照（debug_dump_every_frame 打开时，每帧匹配都存）
  };

  struct Options {
    bool enabled = true;        // 基础功能，默认开启
    std::string dir;            // 空则取 $HOME/.dexe/data/consistency_check
    int max_files = 1000;       // 单个目录内保留的 PNG 上限，超出则删最早；<=0 表示不限
    int scale = 2;              // 输出放大倍数，便于看清单点
    int queue_capacity = 8;     // 待写队列深度
    int point_radius = 2;       // 雷达点半径（像素，已含 scale），0 = 单像素
    bool every_frame = false;   // 每帧匹配都存一张（落到 frames/ 子目录，独立环形区）
  };

  // 一次事件的键值元数据（写进图片横幅与 index.csv）
  using Meta = std::vector<std::pair<std::string, std::string>>;

  ConsistencyDebugDumper();
  ~ConsistencyDebugDumper();

  ConsistencyDebugDumper(const ConsistencyDebugDumper&) = delete;
  ConsistencyDebugDumper& operator=(const ConsistencyDebugDumper&) = delete;

  // 幂等；enabled=false 时全部接口变为空操作。
  void Configure(const Options& options);

  bool enabled() const { return options_.enabled; }
  bool every_frame() const { return options_.enabled && options_.every_frame; }

  // 投递一次快照。栅格提取在本函数内（调用线程）完成，其余在后台线程。
  //  pose            : 与点云叠加的位姿（估计位姿 / 恢复后位姿），local 系
  //  predicted_pose  : 里程计预测位姿，用于对照；无则传 nullptr
  //  submap          : 本次匹配实际使用的 local submap
  //  matching_cloud  : 送进匹配器的点云（tracking 系，已去畸变/滤波）
  //  per_sensor      : 合并前各雷达的去畸变点云（tracking 系），与 sensor_ids 等长
  void Dump(Event event,
            const common::Time& time,
            const transform::Rigid2d& pose,
            const transform::Rigid2d* predicted_pose,
            const mapping::Submap2D& submap,
            const sensor::PointCloud& matching_cloud,
            const std::vector<sensor::RangeData>& per_sensor,
            const std::vector<std::string>& sensor_ids,
            const Meta& meta);

 private:
  // 一帧待渲染的数据：与调用方的栅格 / 点云对象再无关联。
  struct Snapshot {
    Event event;
    int64_t seq = 0;          // 本次运行内的计数，仅用于横幅与 index.csv（文件名用时间戳）
    double unix_time = 0.0;
    transform::Rigid2d pose = transform::Rigid2d::Identity();
    transform::Rigid2d predicted = transform::Rigid2d::Identity();
    bool has_predicted = false;

    cv::Mat grid;              // CV_8UC1，已是最终朝向（x 向右、y 向上）
    double resolution = 0.05;  // 世界→像素所需参数
    double max_x = 0.0, max_y = 0.0;
    int offset_x = 0, offset_y = 0, num_y_cells = 0;

    std::vector<std::pair<std::string, std::vector<Eigen::Vector2f>>> sensor_clouds;
    std::vector<Eigen::Vector2f> matching_cloud;
    Meta meta;
  };

  void Worker();
  void Render(const Snapshot& snap);
  // 扫描输出目录，接管已有文件（文件名以时间戳打头，字典序 = 时间序，重启天然衔接）
  void AdoptExistingFiles();
  // 超出 max_files 则删最早，直至不超过上限（按目录各自成环）
  void RotateIfNeeded(const std::string& dir);
  // 事件对应的输出目录：逐帧快照进 frames/，事件快照进主目录。
  // 两者各自成环，避免海量逐帧图把真正有价值的失败现场挤掉。
  std::string DirFor(Event event) const;

  // 世界坐标 → 像素（与栅格提取使用同一套公式，保证点云与地图严格对齐）
  static cv::Point2d WorldToPixel(const Snapshot& snap, double x, double y);

  static const char* EventTag(Event event);

  Options options_;
  std::string dir_;
  bool configured_ = false;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Snapshot> queue_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::atomic<int64_t> seq_{0};
  std::atomic<int64_t> written_{0};
  std::atomic<int64_t> dropped_{0};
  std::atomic<int64_t> rotated_{0};
  // 仅 worker 线程访问：各目录下本工具管理的 PNG，按文件名（= 时间）升序
  std::map<std::string, std::deque<std::string>> managed_files_;
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_CONSISTENCY_DEBUG_DUMPER_H_
