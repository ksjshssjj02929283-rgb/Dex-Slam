/*
 * Copyright 2017 The Cartographer Authors
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
#include "cartographer_ros/mapping/ros_map.h"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <map>
#include <set>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"

namespace cartographer_ros {

void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer,
              double occ_thresh, double free_thresh)

{
  const std::string header =
      absl::StrCat("P5\n# Cartographer map; ", resolution, " m/pixel\n",
                   image.width(), " ", image.height(), "\n255\n");
  uchar occ_thresh_val = 255 * occ_thresh;
  uchar free_thresh_val = 255 * free_thresh;

  file_writer->Write(header.data(), header.size());
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const char color = image.GetPixel(x, y)[0];
      u_char* uc_value = (u_char*)malloc(sizeof(u_char));
      memcpy(uc_value, &color, sizeof(char));
      if (*uc_value > occ_thresh_val) {
        *uc_value = 255;
      } else if (*uc_value < free_thresh_val) {
        *uc_value = 0;
      }
      char* c_value = (char*)malloc(sizeof(char));
      memcpy(c_value, uc_value, sizeof(u_char));
      file_writer->Write(c_value, 1);
    }
  }
}

// 本地时间 ISO 8601（秒级，无时区后缀），如 2026-09-14T16:30:00
std::string LocalTimeIso8601() {
  const std::time_t now = std::time(nullptr);
  std::tm tm_local{};
  localtime_r(&now, &tm_local);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_local);
  return buf;
}

GridMapStats ComputeGridMapStats(const cv::Mat& rendered_mat,
                                 const cv::Mat& filtered_mat,
                                 const double resolution,
                                 const double free_thresh) {
  GridMapStats stats;
  stats.resolution = resolution;
  // total 取渲染图外框（含画布 padding），与写出的 map.png 尺寸一致。
  stats.total_cells =
      rendered_mat.empty()
          ? 0u
          : static_cast<uint32_t>(rendered_mat.cols) *
                static_cast<uint32_t>(rendered_mat.rows);
  // free 与 map_server 的判定一致：occupancy = (255 - p) / 255 < free_thresh。
  const int free_pixel_thresh = static_cast<int>(255.0 * (1.0 - free_thresh));
  stats.free_cells =
      filtered_mat.empty()
          ? 0u
          : static_cast<uint32_t>(
                cv::countNonZero(filtered_mat > free_pixel_thresh));
  return stats;
}

namespace {

// map.yaml 的内存镜像：整份载入，按键读改，整份写出。
// 内部仍按行保存——没改过的行（含注释、空行、行尾注释）写回时逐字节不变。
// 不用 yaml-cpp 正是为此：它重新输出会丢注释、改浮点格式和 origin 的行内列表写法。
// 只处理扁平的单行 "key: value"（map.yaml 全由我方生成，就是这个形状）；
// 其它行（注释、空行、缩进的续行）当作无键行原样保留。
class MapYamlDoc {
 public:
  // 路径为空或文件不存在 = 空文档。重复键只认第一次出现，其后的同名行丢弃。
  static MapYamlDoc Load(const std::string& path) {
    MapYamlDoc doc;
    if (path.empty()) return doc;
    std::ifstream in(path);
    for (std::string text; std::getline(in, text);) {
      const std::string key = KeyOf(text);
      if (!key.empty() && doc.index_.count(key) > 0) continue;
      if (!key.empty()) doc.index_[key] = doc.lines_.size();
      doc.lines_.push_back({key, text, /*touched=*/false});
    }
    return doc;
  }

  bool Has(const std::string& key) const { return index_.count(key) > 0; }

  absl::optional<std::string> Get(const std::string& key) const {
    const auto it = index_.find(key);
    if (it == index_.end()) return absl::nullopt;
    const std::string& text = lines_[it->second].text;
    const size_t b = text.find_first_not_of(" \t", text.find(':') + 1);
    const size_t e = text.find_last_not_of(" \t\r");
    if (b == std::string::npos || e < b) return std::string();
    return text.substr(b, e - b + 1);
  }

  absl::optional<double> GetDouble(const std::string& key) const {
    const auto value = Get(key);
    if (!value.has_value()) return absl::nullopt;
    char* end = nullptr;
    const double v = std::strtod(value->c_str(), &end);
    if (end == value->c_str() || !std::isfinite(v)) return absl::nullopt;
    return v;
  }

  // 已有：原位改值；没有：追加到末尾。
  void Set(const std::string& key, const std::string& value) {
    const std::string text = absl::StrCat(key, ": ", value);
    const auto it = index_.find(key);
    if (it != index_.end()) {
      lines_[it->second].text = text;
      lines_[it->second].touched = true;
      return;
    }
    index_[key] = lines_.size();
    lines_.push_back({key, text, /*touched=*/true});
  }

  void Erase(const std::string& key) {
    const auto it = index_.find(key);
    if (it == index_.end()) return;
    lines_[it->second].erased = true;
    index_.erase(it);
  }

  // 载入时就有、本次没被 Set / Erase 动过的键，用于日志。
  std::vector<std::string> UntouchedKeys() const {
    std::vector<std::string> keys;
    for (const auto& line : lines_) {
      if (!line.key.empty() && !line.touched && !line.erased) keys.push_back(line.key);
    }
    return keys;
  }

  std::string ToString() const {
    std::string out;
    for (const auto& line : lines_) {
      if (!line.erased) absl::StrAppend(&out, line.text, "\n");
    }
    return out;
  }

 private:
  struct Line {
    std::string key;  // 空 = 注释 / 空行 / 续行
    std::string text;
    bool touched = false;
    bool erased = false;
  };

  // 顶层键行 = 行首非空白、非 '#'，且冒号前有键名。
  static std::string KeyOf(const std::string& text) {
    if (text.empty() || text[0] == ' ' || text[0] == '\t' || text[0] == '#') return "";
    const size_t colon = text.find(':');
    if (colon == std::string::npos || colon == 0) return "";
    const size_t e = text.find_last_not_of(" \t", colon - 1);
    return e == std::string::npos ? "" : text.substr(0, e + 1);
  }

  std::vector<Line> lines_;
  std::map<std::string, size_t> index_;  // 键 → lines_ 下标（已删除的不在内）
};

double NormalizeAngle(const double a) {
  double r = std::remainder(a, 2.0 * M_PI);  // [-π, π]
  if (r <= -M_PI) r += 2.0 * M_PI;            // 归到 (-π, π]
  return r;
}

// 角度会被反复读回再叠加补偿，6 位有效数字（StrCat 默认）会累积误差，这里给足 10 位。
std::string FormatAngle(const double rad) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.10g", rad);
  return buf;
}

std::string DirOf(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
}

// 目录级排它锁：同一目录下所有写 map.yaml 的进程（SLAM 节点、离线工具、map_manager）
// 用同一个锁文件互斥，读改写之间不会被别人插进来。拿不到锁不阻止写盘，只告警。
class MapYamlDirLock {
 public:
  explicit MapYamlDirLock(const std::string& dir) {
    const std::string path = dir + "/.map.yaml.lock";
    fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX) != 0) {
      LOG(WARNING) << "map.yaml lock unavailable (" << path << "): " << std::strerror(errno);
    }
  }
  ~MapYamlDirLock() {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }
  MapYamlDirLock(const MapYamlDirLock&) = delete;
  MapYamlDirLock& operator=(const MapYamlDirLock&) = delete;

 private:
  int fd_ = -1;
};

bool WriteFileAtomically(const std::string& path, const std::string& content) {
  const std::string tmp = path + ".tmp";
  FILE* f = std::fopen(tmp.c_str(), "w");
  if (f == nullptr) {
    LOG(ERROR) << "Failed to open " << tmp << ": " << std::strerror(errno);
    return false;
  }
  const bool ok = std::fwrite(content.data(), 1, content.size(), f) == content.size() &&
                  std::fflush(f) == 0 && ::fsync(::fileno(f)) == 0;
  std::fclose(f);
  if (!ok || std::rename(tmp.c_str(), path.c_str()) != 0) {
    LOG(ERROR) << "Failed to write " << path << ": " << std::strerror(errno);
    std::remove(tmp.c_str());
    return false;
  }
  return true;
}

// 写入方负责的键依次写进文档；data_version 放最后，保证追加时排在末尾
// （与新建时的顺序一致：其它键 → created_at → last_edited_at → data_version）。
void SetEntriesExceptVersion(const MapYamlEntries& entries, MapYamlDoc* yaml) {
  for (const auto& kv : entries) {
    if (kv.first != "data_version") yaml->Set(kv.first, kv.second);
  }
}

void SetVersion(const MapYamlEntries& entries, MapYamlDoc* yaml) {
  for (const auto& kv : entries) {
    if (kv.first == "data_version") yaml->Set(kv.first, kv.second);
  }
}

bool HasEntry(const MapYamlEntries& entries, const std::string& key) {
  for (const auto& kv : entries) {
    if (kv.first == key) return true;
  }
  return false;
}

}  // namespace

std::string ReadMapCreatedAt(const std::string& yaml_path) {
  std::string value = MapYamlDoc::Load(yaml_path).Get("created_at").value_or("");
  // 去掉可能的引号（两种写法都接受）
  if (value.size() >= 2 && (value.front() == '"' || value.front() == '\'') &&
      value.back() == value.front()) {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

MapYamlEntries MakeMapYamlEntries(const double resolution, const Eigen::Vector2d& origin,
                                  const std::string& map_file_name,
                                  const double occ_thresh, const double free_thresh,
                                  const absl::optional<std::string>& data_version,
                                  const GridMapStats* stats) {
  // yaml 内引用的文件路径使用相对路径（仅 basename），不写绝对路径。
  // 这样无论地图目录位于何处（staging / 正式 / backup / 被整体移动），
  // 加载侧（map_loader）都按"yaml 所在目录 + 相对路径"解析，路径恒有效。
  const size_t slash = map_file_name.find_last_of('/');
  const std::string base =
      (slash == std::string::npos) ? map_file_name : map_file_name.substr(slash + 1);
  // 数值常量与写法沿用 ros map_saver：
  // https://github.com/ros-planning/navigation/blob/ac41d2480c4cf1602daf39a6e9629142731d92b0/map_server/src/map_saver.cpp#L114
  MapYamlEntries entries = {
      {"image", base + ".png"},
      {"line_map", base + "_line_map.json"},
      {"cloud_map", base + ".pcd"},
      {"highlight_cloud_map", base + "_highlight.pcd"},
      {"resolution", absl::StrCat(resolution)},
      {"origin", absl::StrCat("[", origin.x(), ", ", origin.y(), ", 0.0]")},
      {"negate", "0"},
      {"occupied_thresh", absl::StrCat(occ_thresh)},
      {"free_thresh", absl::StrCat(free_thresh)}};
  if (stats != nullptr) {
    // 地图面积统计（口径见 GridMapStats）；读取侧对多余键不报错，旧地图无此段亦可加载
    entries.emplace_back("total_area_m2", absl::StrCat(stats->TotalAreaM2()));
    entries.emplace_back("effective_area_m2", absl::StrCat(stats->EffectiveAreaM2()));
  }
  // 同目录 map.pbstream 的数据版本（语义见 ros_map.h WriteYaml 注释）。
  if (data_version.has_value()) {
    entries.emplace_back("data_version",
                         data_version->empty() ? std::string("legacy") : *data_version);
  }
  return entries;
}
bool WriteMapYamlMerged(const std::string& out_yaml_path, const MapYamlEntries& entries,
                        const MapYamlMergeOptions& opts) {
  // 锁加在"底"所在目录：读改写要防的是底在读和写之间被别人改掉。
  // 定位侧保存的输出在 staging 目录、底是正式目录，两者不同时也以底为准。
  MapYamlDirLock lock(
      DirOf(opts.base_yaml_path.empty() ? out_yaml_path : opts.base_yaml_path));
  MapYamlDoc yaml = MapYamlDoc::Load(opts.base_yaml_path);

  // 1. 本方负责的键：原位覆盖，底里没有的追加
  SetEntriesExceptVersion(entries, &yaml);

  // 2. 面积依赖底图像素：重画了底图却没算面积，旧值必须删掉，宁缺勿错
  if (!HasEntry(entries, "total_area_m2")) {
    yaml.Erase("total_area_m2");
    yaml.Erase("effective_area_m2");
  }

  // 3. created_at：显式给了用给的（空串 = 现在）；没给就沿用底，底也没有才取现在
  if (opts.created_at.has_value()) {
    yaml.Set("created_at", opts.created_at->empty() ? LocalTimeIso8601() : *opts.created_at);
  } else if (!yaml.Has("created_at")) {
    yaml.Set("created_at", LocalTimeIso8601());
  }

  // 4. display_yaw：坐标系转了就补偿，界面上的朝向保持不变；底里没有不新增
  if (opts.display_yaw_delta != 0.0 && yaml.Has("display_yaw")) {
    const auto yaw = yaml.GetDouble("display_yaw");
    if (yaw.has_value()) {
      yaml.Set("display_yaw", FormatAngle(NormalizeAngle(*yaw + opts.display_yaw_delta)));
    } else {
      LOG(WARNING) << "map.yaml display_yaw is not a number, kept as is: '"
                   << yaml.Get("display_yaw").value_or("") << "'";
    }
  }

  // 5. last_edited_at：底图最后一次被编辑的时刻。走到这里就是底图被重写了，一律取现在
  yaml.Set("last_edited_at", LocalTimeIso8601());

  // 6. data_version 最后写
  SetVersion(entries, &yaml);

  const std::vector<std::string> kept = yaml.UntouchedKeys();
  if (!kept.empty()) {
    LOG(INFO) << "map.yaml merged (" << out_yaml_path << "), kept keys not owned by this writer: "
              << absl::StrJoin(kept, ", ");
  }
  return WriteFileAtomically(out_yaml_path, yaml.ToString());
}

void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& map_file_name,
               ::cartographer::io::FileWriter* file_writer,
               double occ_thresh, double free_thresh,
               const absl::optional<std::string>& data_version,
               const GridMapStats* stats, const std::string& created_at)
{
  // 新建地图：从空文档按 其它键 → created_at → last_edited_at → data_version 的顺序写。
  const MapYamlEntries entries = MakeMapYamlEntries(resolution, origin, map_file_name,
                                                    occ_thresh, free_thresh, data_version, stats);
  MapYamlDoc yaml;
  SetEntriesExceptVersion(entries, &yaml);
  // 地图创建时刻，机器人本地时间 ISO 8601、无时区后缀；map_manager get_maps 以 MapInfo.created_at 透传给上层。
  const std::string created = created_at.empty() ? LocalTimeIso8601() : created_at;
  yaml.Set("created_at", created);
  // 新建即首次编辑，两者相同；之后只有底图编辑会推进 last_edited_at
  yaml.Set("last_edited_at", created);
  SetVersion(entries, &yaml);
  const std::string output = yaml.ToString();
  file_writer->Write(output.data(), output.size());
}

void WritePng(const ::cartographer::io::Image& image, 
              const std::string file_name,
              double occ_thresh, double free_thresh)
{
  cv::imwrite(file_name, GeneratePngMat(image, occ_thresh, free_thresh));
}

cv::Mat GeneratePngMat(const ::cartographer::io::Image& image,
                       double occ_thresh, double free_thresh)
{
  cv::Mat map_mat(image.height(), image.width(), CV_8UC1);
  uchar occ_thresh_val = 255 * occ_thresh;
  uchar free_thresh_val = 255 * free_thresh;
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      uchar value = image.GetPixel(x, y)[0];
      if (value > occ_thresh_val) {
        value = 255;
      } else if (value < free_thresh_val) {
        value = 0;
      }
      map_mat.at<uchar>(y, x) = value;
    }
  }
  return map_mat;
}

void WriteNavigationMap(const std::string pgm_file_name,
                       const std::string output_file_name,
                       const std::string& filter_mode,
                       bool save_debug_map,
                       const std::string& map_dir)
{
  cv::Mat original = cv::imread(pgm_file_name, cv::IMREAD_GRAYSCALE);
  if (original.empty()) {
    LOG(ERROR) << "Failed to read PNG file: " << pgm_file_name;
    return;
  }
  WriteNavigationMap(original, output_file_name, filter_mode, save_debug_map, map_dir);
}

void WriteNavigationMap(const cv::Mat& original,
                       const std::string output_file_name,
                       const std::string& filter_mode,
                       bool save_debug_map,
                       const std::string& map_dir,
                       cv::Mat* filtered_out)
{
  LOG(INFO) << "Starting WriteNavigationMap with noise removal for " << output_file_name;
  LOG(INFO) << "Input mat dimensions: " << original.cols << "x" << original.rows;

  // 创建debug_map目录（在地图目录下）
  std::string debug_dir = map_dir + "/debug_map";
  if (save_debug_map) {
    std::string mkdir_cmd = "mkdir -p " + debug_dir;
    int ret = system(mkdir_cmd.c_str());
    if (ret != 0) {
      LOG(WARNING) << "Failed to create debug directory: " << debug_dir;
    } else {
      LOG(INFO) << "Created debug directory: " << debug_dir;
    }
  }
  
  // 获取滤波模式参数
  bool use_area_filter = true;
  int min_noise_area = 10;

  if (filter_mode == "normal") {
    use_area_filter = true;
    min_noise_area = 10;
    LOG(INFO) << "Using normal filter mode.";
  } else if (filter_mode == "high") {
    use_area_filter = true;
    min_noise_area = 50;
    LOG(INFO) << "Using high filter mode.";
  } else if (filter_mode == "super") {
    use_area_filter = true;
    min_noise_area = 100;
    LOG(INFO) << "Using super filter mode.";
  } else {
    LOG(WARNING) << "Unknown filter mode: " << filter_mode << ", using default (normal)";
    use_area_filter = true;
    min_noise_area = 10;
  }
  
  // 固定参数（与Python版本debug.py完全一致）
  const int white_thresh = 250;
  const int gray_thresh = 250;
  const int black_thresh = 160;
  const int dilate_radius = 3;
  
  // 步骤2: 对PNG图像进行噪声处理
  cv::Mat result = original.clone();
  
  // 生成三类区域mask（与debug.py完全一致）
  cv::Mat white_mask = (original >= white_thresh) * 255;  // 转为255显示
  cv::Mat black_mask = (original <= black_thresh) * 255;  // 转为255显示
  cv::Mat gray_mask = ((original > black_thresh) & (original < gray_thresh)) * 255;  // 转为255显示

  // 保存mask图像（仅在debug模式）
  if (save_debug_map) {
    cv::imwrite(debug_dir + "/01_white_mask.png", white_mask );  // 转为255显示
    cv::imwrite(debug_dir + "/02_black_mask.png", black_mask);
    cv::imwrite(debug_dir + "/03_gray_mask.png", gray_mask);
  }
  
  // 对黑色区域做连通域分析
  cv::Mat labels, stats, centroids;
  int num_labels = cv::connectedComponentsWithStats(black_mask, labels, stats, centroids, 8);
  
  LOG(INFO) << "Found " << (num_labels - 1) << " connected components in black regions";
  
  // 保存连通域标签可视化（仅在debug模式）
  if (save_debug_map) {
    cv::Mat labels_vis;
    cv::normalize(labels, labels_vis, 0, 255, cv::NORM_MINMAX);
    labels_vis.convertTo(labels_vis, CV_8U);
    cv::applyColorMap(labels_vis, labels_vis, cv::COLORMAP_JET);
    cv::imwrite(debug_dir + "/04_connected_components.png", labels_vis);
  }
  
  // 构造膨胀核
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, 
      cv::Size(2 * dilate_radius + 1, 2 * dilate_radius + 1));
  
  int removed_count = 0;
  int kept_by_gray = 0;
  int kept_by_size = 0;
  
  // 创建中间处理结果可视化图像（仅 debug 模式才分配）
  cv::Mat removed_mask, edge_mask;
  if (save_debug_map) {
    removed_mask = cv::Mat::zeros(original.size(), CV_8U);
    edge_mask    = cv::Mat::zeros(original.size(), CV_8U);
  }
  
  // 处理每个连通域
  // 优化：利用 stats bounding box，将所有操作限制在小 ROI 内，
  // 避免每次对整图（3534x4649）做全量扫描操作（原方案为 O(N×W×H)）
  for (int i = 1; i < num_labels; ++i) {
    // 条件①：面积约束优先判断（无需创建 mask，直接跳过大连通域）
    if (use_area_filter) {
      int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area >= min_noise_area) {
        kept_by_size++;
        continue;
      }
    }

    // 从 stats 获取 tight bounding box
    const int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    const int y = stats.at<int>(i, cv::CC_STAT_TOP);
    const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
    const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    // 带 padding 的 ROI（为膨胀操作留出边界）
    const int pad = dilate_radius + 1;
    const int x0 = std::max(0, x - pad);
    const int y0 = std::max(0, y - pad);
    const int x1 = std::min(original.cols, x + w + pad);
    const int y1 = std::min(original.rows, y + h + pad);
    const cv::Rect padded_roi(x0, y0, x1 - x0, y1 - y0);

    // 条件②：灰色邻居约束 —— 仅在小 ROI 上操作
    cv::Mat component_mask_roi;
    cv::compare(labels(padded_roi), i, component_mask_roi, cv::CMP_EQ);

    cv::Mat dilated_roi;
    cv::dilate(component_mask_roi, dilated_roi, kernel, cv::Point(-1, -1), 1);

    cv::Mat neighbor_roi;
    cv::bitwise_and(dilated_roi, ~component_mask_roi, neighbor_roi);
    cv::Mat gray_neighbors_roi;
    cv::bitwise_and(neighbor_roi, gray_mask(padded_roi), gray_neighbors_roi);

    const bool has_gray_neighbor = cv::countNonZero(gray_neighbors_roi) > 0;

    // 从 padded ROI 中提取 tight 区域的 mask，用于写回全图
    const cv::Rect tight_roi(x, y, w, h);
    const cv::Mat tight_mask = component_mask_roi(cv::Rect(x - x0, y - y0, w, h));

    if (has_gray_neighbor) {
      kept_by_gray++;
      if (save_debug_map) edge_mask(tight_roi).setTo(255, tight_mask);
    } else {
      // 判定为噪点，清除（改为白色 255）
      result(tight_roi).setTo(255, tight_mask);
      if (save_debug_map) removed_mask(tight_roi).setTo(255, tight_mask);
      removed_count++;
    }
  }
  
  // 保存debug图片（仅在save_debug_map=true时）
  if (save_debug_map) {
    // 05_edge_mask.png - 显示灰色邻居区域（边缘检测）
    cv::imwrite(debug_dir + "/05_edge_mask.png", edge_mask);
    
    // 06_annotated.png - 标注图：原图 + 不同颜色的标记
    cv::Mat annotated;
    cv::cvtColor(original, annotated, cv::COLOR_GRAY2BGR);  // 转为彩色图
    
    // 蓝色标记：被移除的噪点
    cv::Mat removed_bgr = cv::Mat::zeros(original.size(), CV_8UC3);
    removed_bgr.setTo(cv::Scalar(255, 0, 0), removed_mask);  // 蓝色 (BGR)
    cv::addWeighted(annotated, 1.0, removed_bgr, 1.0, 0, annotated);
    
    // 绿色标记：边缘区域（灰色邻居）
    cv::Mat edge_bgr = cv::Mat::zeros(original.size(), CV_8UC3);
    edge_bgr.setTo(cv::Scalar(0, 255, 0), edge_mask);  // 绿色 (BGR)
    cv::addWeighted(annotated, 1.0, edge_bgr, 1.0, 0, annotated);
    
    cv::imwrite(debug_dir + "/06_annotated.png", annotated);
  }
  
  LOG(INFO) << "Noise removal statistics:";
  LOG(INFO) << "  Filter mode: " << filter_mode;
  LOG(INFO) << "  Removed components (noise): " << removed_count;
  LOG(INFO) << "  Kept components (has gray neighbors): " << kept_by_gray;
  
  // 步骤3: 保存为PNG文件
  if (filtered_out != nullptr) {
    *filtered_out = result;
  }
  if (!cv::imwrite(output_file_name, result)) {
    LOG(ERROR) << "Failed to write navigation map to " << output_file_name;
    return;
  }
  
  // 保存最终结果和对比图（仅在debug模式）
  if (save_debug_map) {
    // 07_final_result.png - 最终处理后的结果
    cv::imwrite(debug_dir + "/07_final_result.png", result);
    
    // 08_comparison.png - 左右对比：原始 vs 最终结果
    cv::Mat comparison;
    cv::hconcat(original, result, comparison);
    cv::imwrite(debug_dir + "/08_comparison.png", comparison);
  }
  
  LOG(INFO) << "Successfully saved navigation map to " << output_file_name;
}

}  // namespace cartographer_ros
