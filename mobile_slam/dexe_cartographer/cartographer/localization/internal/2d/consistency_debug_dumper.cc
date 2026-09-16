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

#include "cartographer/localization/internal/2d/consistency_debug_dumper.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "boost/filesystem.hpp"
#include "glog/logging.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/xy_index.h"

namespace cartographer {
namespace localization {
namespace {

// 与雷达点区分开的固定配色（BGR）。合并前各雷达一色，最多 4 路后循环。
const cv::Scalar kSensorColors[] = {
    cv::Scalar(60, 60, 255),    // 红   sensor[0]（按 sensor_id 排序，通常是前雷达）
    cv::Scalar(255, 180, 60),   // 蓝   sensor[1]（通常是后雷达）
    cv::Scalar(60, 255, 255),   // 黄
    cv::Scalar(255, 60, 255),   // 品红
};
const cv::Scalar kMatchingColor(80, 220, 80);    // 绿：真正送进匹配器的点
const cv::Scalar kPoseColor(255, 0, 255);        // 品红：本帧位姿
const cv::Scalar kPredictedColor(0, 200, 255);   // 橙：里程计预测位姿
const cv::Scalar kBannerBg(32, 32, 32);
const cv::Scalar kBannerFg(235, 235, 235);

std::string FormatStamp(double unix_time, const char* fmt) {
  const std::time_t secs = static_cast<std::time_t>(unix_time);
  std::tm tm_buf{};
  localtime_r(&secs, &tm_buf);
  char buf[40];
  std::strftime(buf, sizeof(buf), fmt, &tm_buf);
  const int millis =
      static_cast<int>((unix_time - static_cast<double>(secs)) * 1000.0);
  std::ostringstream oss;
  oss << buf << "." << std::setw(3) << std::setfill('0') << millis;
  return oss.str();
}

// 文件名用：YYYYmmdd_HHMMSS.mmm —— 带日期才不会跨零点错序，字典序即时间序，
// 轮转「删最早」直接按文件名排序即可，重启也不需要额外衔接。
std::string FileStamp(double unix_time) {
  return FormatStamp(unix_time, "%Y%m%d_%H%M%S");
}

// 横幅用：可读优先
std::string BannerTime(double unix_time) {
  return FormatStamp(unix_time, "%H:%M:%S");
}

// 文件名前缀是否形如 YYYYmmdd_HHMMSS.mmm（据此判断是不是本工具生成的）
bool HasTimestampPrefix(const std::string& name) {
  if (name.size() < 19) return false;
  for (int i = 0; i < 19; ++i) {
    const char ch = name[i];
    if (i == 8) { if (ch != '_') return false; continue; }
    if (i == 15) { if (ch != '.') return false; continue; }
    if (ch < '0' || ch > '9') return false;
  }
  return name[19] == '_';
}

double ToUnixSeconds(const common::Time& time) {
  return static_cast<double>(common::ToUniversal(time)) * 1e-7 -
         static_cast<double>(common::kUtsEpochOffsetFromUnixEpochInSeconds);
}

std::string Fmt(double v, int precision = 3) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << v;
  return oss.str();
}

}  // namespace

ConsistencyDebugDumper::ConsistencyDebugDumper() = default;

ConsistencyDebugDumper::~ConsistencyDebugDumper() {
  stop_ = true;
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

const char* ConsistencyDebugDumper::EventTag(Event event) {
  switch (event) {
    case Event::kCheckFailed:       return "check_failed";
    case Event::kRecoverySucceeded: return "recovery_ok";
    case Event::kRecoveryFailed:    return "recovery_failed";
    case Event::kFrame:             return "frame";
  }
  return "unknown";
}

void ConsistencyDebugDumper::Configure(const Options& options) {
  if (configured_) return;
  configured_ = true;
  options_ = options;
  if (!options_.enabled) return;

  dir_ = options_.dir;
  if (dir_.empty()) {
    const char* home = std::getenv("HOME");
    dir_ = std::string(home ? home : "/tmp") + "/.dexe/data/consistency_check";
  }
  boost::system::error_code ec;
  boost::filesystem::create_directories(dir_, ec);
  if (!ec && options_.every_frame) {
    boost::filesystem::create_directories(dir_ + "/frames", ec);
  }
  if (ec) {
    LOG(ERROR) << "ConsistencyDebugDumper: cannot create " << dir_ << ": "
               << ec.message() << " — disabled.";
    options_.enabled = false;
    return;
  }
  if (options_.scale < 1) options_.scale = 1;
  if (options_.queue_capacity < 1) options_.queue_capacity = 1;
  if (options_.point_radius < 0) options_.point_radius = 0;

  AdoptExistingFiles();

  worker_ = std::thread(&ConsistencyDebugDumper::Worker, this);
  // 常开的基础功能：只在一致性检查失败时产图，健康运行下不写盘。
  LOG(INFO) << "ConsistencyDebugDumper active — consistency/recovery snapshots to "
            << dir_ << " (ring buffer: keep newest " << options_.max_files
            << " png, oldest deleted on overflow; adopted "
            << managed_files_.size() << " from previous runs, scale="
            << options_.scale
            << ", point_radius=" << options_.point_radius << ")."
            << (options_.every_frame
                    ? " EVERY-FRAME dump ON: one png per matched frame under"
                      " frames/ (separate ring) — heavy, diagnosis only."
                    : " Images are produced only when a consistency check fails.");
}

// 启动时扫一遍目录接管已有 PNG：文件名以 YYYYmmdd_HHMMSS.mmm 打头，
// 字典序即时间序，所以跨重启也能正确判断「谁最早」，无需任何额外状态。
std::string ConsistencyDebugDumper::DirFor(Event event) const {
  return event == Event::kFrame ? dir_ + "/frames" : dir_;
}

void ConsistencyDebugDumper::AdoptExistingFiles() {
  std::vector<std::string> dirs{dir_};
  if (options_.every_frame) dirs.push_back(dir_ + "/frames");
  for (const auto& dir : dirs) {
    std::vector<std::string> names;
    boost::system::error_code ec;
    for (boost::filesystem::directory_iterator it(dir, ec), end;
         !ec && it != end; it.increment(ec)) {
      if (!boost::filesystem::is_regular_file(it->status())) continue;
      const std::string name = it->path().filename().string();
      if (name.size() < 5 || name.substr(name.size() - 4) != ".png") continue;
      if (!HasTimestampPrefix(name)) continue;  // 只接管本工具生成的文件
      names.push_back(name);
    }
    std::sort(names.begin(), names.end());
    auto& q = managed_files_[dir];
    for (auto& n : names) q.push_back(std::move(n));
    RotateIfNeeded(dir);  // 上次运行的上限可能更大，先收敛到当前配置
  }
}

// 环形保留：超出 max_files 就从最早的开始删。
void ConsistencyDebugDumper::RotateIfNeeded(const std::string& dir) {
  if (options_.max_files <= 0) return;
  auto& q = managed_files_[dir];
  while (static_cast<int>(q.size()) > options_.max_files) {
    const std::string victim = dir + "/" + q.front();
    q.pop_front();
    boost::system::error_code ec;
    boost::filesystem::remove(victim, ec);
    if (ec) {
      LOG_FIRST_N(WARNING, 3)
          << "ConsistencyDebugDumper: failed to rotate out " << victim << ": "
          << ec.message();
      continue;
    }
    ++rotated_;
  }
}

void ConsistencyDebugDumper::Dump(
    Event event, const common::Time& time, const transform::Rigid2d& pose,
    const transform::Rigid2d* predicted_pose, const mapping::Submap2D& submap,
    const sensor::PointCloud& matching_cloud,
    const std::vector<sensor::RangeData>& per_sensor,
    const std::vector<std::string>& sensor_ids, const Meta& meta) {
  if (!options_.enabled) return;

  const auto* grid =
      dynamic_cast<const mapping::ProbabilityGrid*>(submap.grid());
  if (grid == nullptr) {
    LOG_FIRST_N(WARNING, 1)
        << "ConsistencyDebugDumper: submap grid is not a ProbabilityGrid, "
           "skipping dumps.";
    return;
  }

  // 满队列时丢弃新帧：突发场景里最早那几帧才含因果，保住它们。
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (static_cast<int>(queue_.size()) >= options_.queue_capacity) {
      ++dropped_;
      return;
    }
  }

  Snapshot snap;
  snap.event = event;
  snap.seq = ++seq_;
  snap.unix_time = ToUnixSeconds(time);
  snap.pose = pose;
  if (predicted_pose != nullptr) {
    snap.predicted = *predicted_pose;
    snap.has_predicted = true;
  }
  snap.meta = meta;

  // ---- 栅格提取：必须在调用线程做（栅格归调用方所有，随后可能被重建/写入）----
  Eigen::Array2i offset;
  mapping::CellLimits cell_limits;
  grid->ComputeCroppedLimits(&offset, &cell_limits);
  if (cell_limits.num_x_cells <= 0 || cell_limits.num_y_cells <= 0) {
    return;
  }
  const auto& limits = grid->limits();
  snap.resolution = limits.resolution();
  snap.max_x = limits.max().x();
  snap.max_y = limits.max().y();
  snap.offset_x = offset.x();
  snap.offset_y = offset.y();
  snap.num_y_cells = cell_limits.num_y_cells;

  // 直接生成最终朝向（x 向右、y 向上），不再走 DrawAsImage 的「先画后旋转」，
  // 以免点云叠加时还要跟着推导一次旋转，两处公式一旦不一致就会看见「点云整体错位」的假象。
  //   row = gi.x() - offset.x()                      （gi.x() 由世界 y 决定，y 增 → row 减）
  //   col = (num_y_cells-1) - (gi.y() - offset.y())  （gi.y() 由世界 x 决定，x 增 → col 增）
  snap.grid = cv::Mat(cell_limits.num_x_cells, cell_limits.num_y_cells, CV_8UC1,
                      cv::Scalar(127));  // 127 = 未知，与既有 local_map.png 一致
  for (const Eigen::Array2i& xy : mapping::XYIndexRangeIterator(cell_limits)) {
    const Eigen::Array2i gi = xy + offset;
    if (!grid->IsKnown(gi)) continue;
    const int row = xy.x();
    const int col = cell_limits.num_y_cells - 1 - xy.y();
    snap.grid.at<uchar>(row, col) =
        static_cast<uchar>(grid->GetProbability(gi) * 255.0f);
  }

  // ---- 点云深拷贝（tracking 系；投影到 local 系放到后台做）----
  snap.matching_cloud.reserve(matching_cloud.size());
  for (const auto& p : matching_cloud) {
    snap.matching_cloud.emplace_back(p.position.x(), p.position.y());
  }
  for (size_t i = 0; i < per_sensor.size(); ++i) {
    std::vector<Eigen::Vector2f> pts;
    pts.reserve(per_sensor[i].returns.size());
    for (const auto& p : per_sensor[i].returns) {
      pts.emplace_back(p.position.x(), p.position.y());
    }
    const std::string id =
        i < sensor_ids.size() ? sensor_ids[i] : ("sensor" + std::to_string(i));
    snap.sensor_clouds.emplace_back(id, std::move(pts));
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(std::move(snap));
  }
  cv_.notify_one();
}

void ConsistencyDebugDumper::Worker() {
  for (;;) {
    Snapshot snap;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
      if (queue_.empty()) {
        if (stop_) break;
        continue;
      }
      snap = std::move(queue_.front());
      queue_.pop_front();
    }
    Render(snap);
  }
  LOG(WARNING) << "ConsistencyDebugDumper: wrote " << written_
               << " png, rotated out " << rotated_ << ", dropped " << dropped_
               << " (queue full); " << managed_files_[dir_].size() << " kept in "
               << dir_ << ", " << managed_files_[dir_ + "/frames"].size()
               << " in frames/.";
}

cv::Point2d ConsistencyDebugDumper::WorldToPixel(const Snapshot& snap, double x,
                                                 double y) {
  const double gix = (snap.max_y - y) / snap.resolution - 0.5;
  const double giy = (snap.max_x - x) / snap.resolution - 0.5;
  const double row = gix - snap.offset_x;
  const double col = (snap.num_y_cells - 1) - (giy - snap.offset_y);
  return cv::Point2d(col, row);  // (x=col, y=row)
}

void ConsistencyDebugDumper::Render(const Snapshot& snap) {
  const int s = options_.scale;

  cv::Mat canvas;
  cv::cvtColor(snap.grid, canvas, cv::COLOR_GRAY2BGR);
  if (s > 1) {
    cv::resize(canvas, canvas, cv::Size(canvas.cols * s, canvas.rows * s), 0, 0,
               cv::INTER_NEAREST);
  }

  const double c = std::cos(snap.pose.rotation().angle());
  const double sn = std::sin(snap.pose.rotation().angle());
  const double tx = snap.pose.translation().x();
  const double ty = snap.pose.translation().y();
  auto to_px = [&](double px, double py) {
    const cv::Point2d q = WorldToPixel(snap, px, py);
    return cv::Point(static_cast<int>(std::lround(q.x * s)),
                     static_cast<int>(std::lround(q.y * s)));
  };

  // 合并前各雷达的去畸变点云：分色。若多雷达时间对齐有问题，
  // 两片点云会整体相差一个角度——这是本工具最直接的判据。
  for (size_t i = 0; i < snap.sensor_clouds.size(); ++i) {
    const cv::Scalar& color =
        kSensorColors[i % (sizeof(kSensorColors) / sizeof(kSensorColors[0]))];
    for (const auto& p : snap.sensor_clouds[i].second) {
      const double wx = c * p.x() - sn * p.y() + tx;
      const double wy = sn * p.x() + c * p.y() + ty;
      const cv::Point q = to_px(wx, wy);
      if (q.x < 0 || q.y < 0 || q.x >= canvas.cols || q.y >= canvas.rows) continue;
      cv::circle(canvas, q, options_.point_radius, color, -1, cv::LINE_8);
    }
  }
  // 真正送进匹配器的点（滤波后）
  for (const auto& p : snap.matching_cloud) {
    const double wx = c * p.x() - sn * p.y() + tx;
    const double wy = sn * p.x() + c * p.y() + ty;
    const cv::Point q = to_px(wx, wy);
    if (q.x < 0 || q.y < 0 || q.x >= canvas.cols || q.y >= canvas.rows) continue;
    // 匹配点比雷达点小一档，压在上面仍能看出下面是哪一路雷达
    cv::circle(canvas, q, std::max(0, options_.point_radius - 1), kMatchingColor,
               -1, cv::LINE_8);
  }

  auto draw_pose = [&](const transform::Rigid2d& p, const cv::Scalar& color) {
    const cv::Point o = to_px(p.translation().x(), p.translation().y());
    const double a = p.rotation().angle();
    const cv::Point tip = to_px(p.translation().x() + 0.5 * std::cos(a),
                               p.translation().y() + 0.5 * std::sin(a));
    cv::circle(canvas, o, 4 * s, color, 1, cv::LINE_AA);
    cv::line(canvas, o, tip, color, std::max(1, s), cv::LINE_AA);
  };
  if (snap.has_predicted) draw_pose(snap.predicted, kPredictedColor);
  draw_pose(snap.pose, kPoseColor);

  // ---- 横幅：把这张图自解释所需的数字全部写上 ----
  std::ostringstream l1, l2;
  l1 << "[" << snap.seq << "] " << EventTag(snap.event) << "  t="
     << BannerTime(snap.unix_time) << "  pose=(" << Fmt(snap.pose.translation().x())
     << ", " << Fmt(snap.pose.translation().y()) << ", "
     << Fmt(snap.pose.rotation().angle()) << "rad)";
  if (snap.has_predicted) {
    l1 << "  pred=(" << Fmt(snap.predicted.translation().x()) << ", "
       << Fmt(snap.predicted.translation().y()) << ", "
       << Fmt(snap.predicted.rotation().angle()) << ")";
  }
  for (const auto& kv : snap.meta) l2 << kv.first << "=" << kv.second << "  ";

  std::ostringstream l3;
  l3 << "legend: ";
  for (size_t i = 0; i < snap.sensor_clouds.size(); ++i) {
    l3 << snap.sensor_clouds[i].first << "(" << snap.sensor_clouds[i].second.size()
       << "pts) ";
  }
  l3 << "| green=matching(" << snap.matching_cloud.size()
     << ") magenta=pose orange=odom-pred | map: white=occupied black=free"
     << " gray=unknown | res=" << Fmt(snap.resolution, 3) << "m";

  const int line_h = 18;
  const int banner_h = line_h * 3 + 8;
  // 横幅按最长一行的实际排版宽度撑开——否则 reason 这类长文本会被裁掉，
  // 而那恰恰是判断失败原因最关键的一行。
  int text_w = 0;
  for (const std::string& t : {l1.str(), l2.str(), l3.str()}) {
    int baseline = 0;
    const cv::Size sz =
        cv::getTextSize(t, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &baseline);
    text_w = std::max(text_w, sz.width);
  }
  cv::Mat out(canvas.rows + banner_h, std::max(canvas.cols, text_w + 14),
              CV_8UC3, kBannerBg);
  canvas.copyTo(out(cv::Rect(0, banner_h, canvas.cols, canvas.rows)));
  const auto put = [&](const std::string& text, int line, const cv::Scalar& col) {
    cv::putText(out, text, cv::Point(6, 6 + line_h * line + 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv::LINE_AA);
  };
  put(l1.str(), 0, kBannerFg);
  put(l2.str(), 1, cv::Scalar(120, 220, 255));
  put(l3.str(), 2, cv::Scalar(180, 180, 180));

  // 文件名 = 当前帧时间戳 + 事件名。同一帧里「检查失败」与「恢复成功/失败」共用一个
  // 时间戳，靠事件名区分；再撞名（同事件同毫秒）才追加序号兜底。
  std::ostringstream base;
  base << FileStamp(snap.unix_time) << "_" << EventTag(snap.event);
  for (const auto& kv : snap.meta) {
    if (kv.first == "conf" || kv.first == "score") base << "_" << kv.first << kv.second;
  }
  std::string filename = base.str() + ".png";
  const std::string dir = DirFor(snap.event);
  for (int dup = 2; boost::filesystem::exists(dir + "/" + filename) && dup < 100; ++dup) {
    filename = base.str() + "_" + std::to_string(dup) + ".png";
  }
  const std::string path = dir + "/" + filename;

  if (!cv::imwrite(path, out)) {
    LOG_FIRST_N(ERROR, 3) << "ConsistencyDebugDumper: failed to write " << path;
    return;
  }
  ++written_;
  managed_files_[dir].push_back(filename);
  RotateIfNeeded(dir);

  // 索引表：全量流水，便于按分数/事件排序定位，无需逐张打开。
  // 它**不随 PNG 轮转**——早期行对应的图可能已被删除，这是刻意的：
  // 图会滚掉，但「那一刻发生过什么、分数多少」的记录留下才有回溯价值。
  const std::string index = dir + "/index.csv";
  const bool need_header = !boost::filesystem::exists(index);
  std::ofstream ofs(index, std::ios::app);
  if (ofs) {
    if (need_header) {
      ofs << "seq,time,event,pose_x,pose_y,pose_yaw,pred_x,pred_y,pred_yaw,"
             "matching_pts,meta,file\n";
    }
    ofs << snap.seq << "," << FileStamp(snap.unix_time) << "," << EventTag(snap.event)
        << "," << Fmt(snap.pose.translation().x()) << ","
        << Fmt(snap.pose.translation().y()) << ","
        << Fmt(snap.pose.rotation().angle()) << ",";
    if (snap.has_predicted) {
      ofs << Fmt(snap.predicted.translation().x()) << ","
          << Fmt(snap.predicted.translation().y()) << ","
          << Fmt(snap.predicted.rotation().angle()) << ",";
    } else {
      ofs << ",,,";
    }
    ofs << snap.matching_cloud.size() << ",\"";
    for (const auto& kv : snap.meta) ofs << kv.first << "=" << kv.second << " ";
    ofs << "\"," << filename << "\n";
  }
}

}  // namespace localization
}  // namespace cartographer
