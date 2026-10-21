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

/*
 * pbstream_export_tool —— 从 map.pbstream 导出地图内容（只读，不改地图）
 *
 * 点云、高亮点云、静态底图、线特征都固化在 pbstream 的 global_static_submap 里，
 * 地图目录不再单独保存 pcd / 线特征文件；需要查看或给其它工具用时用本工具导出。
 *
 * 用法：
 *   ros2 run cartographer_ros pbstream_export_tool --map_name <图名> [--maps_root <根目录>]
 *   ros2 run cartographer_ros pbstream_export_tool --map_dir <含 map.pbstream 的目录>
 *   ros2 run cartographer_ros pbstream_export_tool --pbstream <文件>
 *     可选 --output_dir <目录>（默认：给地图时为 <map>/export，给文件时为文件所在目录下的 export）
 *          --export all | cloud,grid,lines,overlay,info（逗号分隔，默认 all）
 *          --occ_thresh / --free_thresh（默认取同目录 map.yaml，没有则 0.65 / 0.196）
 *          --cloud_voxel <米>（点云体素降采样，默认 0 = 不降采样）
 *          --line_thickness <像素>（叠加图线宽，默认 2）、--label_lines（叠加图上标线段 local_id）
 *
 * 产物（output_dir 下）：
 *   cloud.pcd            点云 + 高亮点云合在一个文件：PointXYZRGBL，普通点灰色 label=0，高亮点红色 label=1
 *   static_map.png/.yaml 静态底图渲染（三值灰度，未经导航用滤波），yaml 为 map_server 格式
 *   lines.json           线特征：端点、长度、权重、法向、观测数等，坐标系为 map
 *   lines_overlay.png    线特征画在静态底图上（红线，端点蓝点）
 *   info.json            pbstream 概要：数据版本、底图尺寸 / 分辨率 / 原点、点数、线数、各产物路径
 *   最后一行标准输出：[PBSTREAM_EXPORT_RESULT] {json}；退出码 0 成功 / 1 失败 / 2 参数错误
 */

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "absl/strings/str_split.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer_ros/tools/map_ops/map_ops.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

DEFINE_string(map_name, "", "地图名（在 --maps_root 下查找，先 <名>/slam/ 再 <名>/）");
DEFINE_string(maps_root, "", "地图根目录，默认 ~/.dexe/data/maps");
DEFINE_string(map_dir, "", "含 map.pbstream 的目录（或地图目录，自动找 slam/）");
DEFINE_string(pbstream, "", "直接指定 pbstream 文件");
DEFINE_string(output_dir, "", "输出目录，默认见文件头说明");
DEFINE_string(export, "all", "导出项：all 或 cloud,grid,lines,overlay,info 的组合");
DEFINE_double(occ_thresh, -1.0, "渲染占用阈值，<0 取同目录 map.yaml，没有则 0.65");
DEFINE_double(free_thresh, -1.0, "渲染空闲阈值，<0 取同目录 map.yaml，没有则 0.196");
DEFINE_double(cloud_voxel, 0.0, "点云体素降采样尺寸（米），0 = 不降采样");
DEFINE_int32(line_thickness, 2, "叠加图线宽（像素）");
DEFINE_bool(label_lines, false, "叠加图上标出线段 local_id（线段在地图里的唯一编号）");

namespace fs = std::filesystem;
namespace carto = ::cartographer;

namespace {

std::string JsonEscape(const std::string& s) {
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') out.push_back('\\');
    out.push_back(c);
  }
  return out;
}

std::string Num(double v) {
  std::ostringstream os;
  os << std::setprecision(10) << v;
  return os.str();
}

// 最后一行结果：调用方（脚本 / command_board 风格的编排）按这一行解析
int Finish(bool ok, const std::string& reason, const std::string& message, const std::string& extra_json,
           int code) {
  std::cout << "[PBSTREAM_EXPORT_RESULT] {\"ok\": " << (ok ? "true" : "false") << ", \"reason\": \""
            << JsonEscape(reason) << "\", \"message\": \"" << JsonEscape(message) << "\""
            << (extra_json.empty() ? "" : ", " + extra_json) << "}" << std::endl;
  return code;
}

struct Loaded {
  carto::mapping::proto::SerializationHeader header;
  bool has_static = false;
  carto::mapping::proto::Submap2D static_submap;
};

// 只读需要的记录：头、位姿图、轨迹选项三条固定记录之后，找 global_static_submap
bool LoadStaticSubmap(const std::string& pbstream, Loaded* out, std::string* error) {
  carto::io::ProtoStreamReader reader(pbstream);
  if (!reader.ReadProto(&out->header)) {
    *error = "failed to read SerializationHeader";
    return false;
  }
  carto::mapping::proto::SerializedData record;
  for (int i = 0; i < 2; ++i) {  // PoseGraph、AllTrajectoryBuilderOptions
    if (!reader.ReadProto(&record)) {
      *error = "pbstream truncated before trajectory options";
      return false;
    }
    record.Clear();
  }
  while (reader.ReadProto(&record)) {
    if (record.has_global_static_submap()) {
      out->static_submap = record.global_static_submap();
      out->has_static = true;
      return true;
    }
    record.Clear();
  }
  return true;
}

struct CloudCounts {
  size_t normal = 0;
  size_t highlight = 0;
};

void AppendCloud(const carto::mapping::proto::PointCloudData& proto, uint8_t r, uint8_t g, uint8_t b,
                 uint32_t label, double voxel, pcl::PointCloud<pcl::PointXYZRGBL>* cloud, size_t* count) {
  carto::sensor::PointCloud pts;
  for (const auto& p : proto.points()) pts.push_back({Eigen::Vector3f(p.x(), p.y(), p.z())});
  if (voxel > 0.0) pts = carto::sensor::VoxelFilter(pts, static_cast<float>(voxel));
  for (const auto& p : pts.points()) {
    pcl::PointXYZRGBL q;
    q.x = p.position.x();
    q.y = p.position.y();
    q.z = p.position.z();
    q.r = r;
    q.g = g;
    q.b = b;
    q.label = label;
    cloud->push_back(q);
  }
  *count = pts.size();
}

// 世界坐标（map 系，米）→ 渲染图像素（origin 为图像左下角，行序自上而下）
cv::Point2d WorldToPixel(double x, double y, const Eigen::Vector2d& origin, double resolution, int height) {
  return {(x - origin.x()) / resolution, height - (y - origin.y()) / resolution};
}

// 找含 map.pbstream 的目录：分层布局 <dir>/slam/ 优先，再回退 <dir>/；都没有返回空
// （不用 map_ops::ResolveMapDir，它找不到时直接 abort，这里要给调用方一个退出码）
std::string FindSlamDir(const fs::path& dir) {
  std::error_code ec;
  for (const fs::path& d : {dir / "slam", dir}) {
    if (fs::is_regular_file(d / "map.pbstream", ec)) return d.string();
  }
  return "";
}

}  // namespace

int main(int argc, char** argv) {
  google::AllowCommandLineReparsing();
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // ── 参数与路径 ─────────────────────────────────────────────────────
  const int given = (FLAGS_map_name.empty() ? 0 : 1) + (FLAGS_map_dir.empty() ? 0 : 1) +
                    (FLAGS_pbstream.empty() ? 0 : 1);
  if (given != 1) {
    return Finish(false, "bad_args", "need exactly one of --map_name / --map_dir / --pbstream", "", 2);
  }
  std::set<std::string> what;
  for (const auto& item : absl::StrSplit(FLAGS_export, ',', absl::SkipWhitespace())) {
    what.insert(std::string(item));
  }
  if (what.count("all")) what = {"cloud", "grid", "lines", "overlay", "info"};
  for (const auto& w : what) {
    if (w != "cloud" && w != "grid" && w != "lines" && w != "overlay" && w != "info") {
      return Finish(false, "bad_args", "unknown --export item: " + w, "", 2);
    }
  }

  std::string pbstream = FLAGS_pbstream;
  std::string map_root_dir;  // 地图目录（slam/ 的上一级），用于默认输出位置
  if (pbstream.empty()) {
    std::string slam_dir;
    if (!FLAGS_map_name.empty()) {
      if (FLAGS_map_name.find('/') != std::string::npos || FLAGS_map_name == "." || FLAGS_map_name == "..") {
        return Finish(false, "bad_args", "map_name must be a plain name: " + FLAGS_map_name, "", 2);
      }
      std::string root = FLAGS_maps_root;
      if (root.empty()) {
        const char* home = std::getenv("HOME");
        root = std::string(home ? home : "") + "/.dexe/data/maps";
      }
      slam_dir = FindSlamDir(fs::path(root) / FLAGS_map_name);
      if (slam_dir.empty()) {
        return Finish(false, "map_not_found", "no map.pbstream for map " + FLAGS_map_name + " under " + root, "", 1);
      }
    } else {
      slam_dir = FindSlamDir(FLAGS_map_dir);
      if (slam_dir.empty()) {
        return Finish(false, "map_not_found", "no map.pbstream in " + FLAGS_map_dir + " or its slam/", "", 1);
      }
    }
    pbstream = (fs::path(slam_dir) / "map.pbstream").string();
    map_root_dir = fs::path(slam_dir).filename() == "slam" ? fs::path(slam_dir).parent_path().string() : slam_dir;
  }
  std::error_code ec;
  if (!fs::is_regular_file(pbstream, ec)) {
    return Finish(false, "pbstream_not_found", "pbstream not found: " + pbstream, "", 1);
  }
  const fs::path pb_dir = fs::path(pbstream).parent_path();
  fs::path out_dir = FLAGS_output_dir.empty()
                         ? (map_root_dir.empty() ? pb_dir / "export" : fs::path(map_root_dir) / "export")
                         : fs::path(FLAGS_output_dir);
  fs::create_directories(out_dir, ec);
  if (ec) return Finish(false, "output_dir_failed", "cannot create " + out_dir.string() + ": " + ec.message(), "", 1);

  // 渲染阈值：命令行 > 同目录 map.yaml > 默认
  const auto yaml_cfg = ::cartographer_ros::map_ops::ReadMapYamlConfig((pb_dir / "map.yaml").string());
  const double occ = FLAGS_occ_thresh >= 0 ? FLAGS_occ_thresh : (yaml_cfg.has_occ_thresh ? yaml_cfg.occ_thresh : 0.65);
  const double free = FLAGS_free_thresh >= 0 ? FLAGS_free_thresh
                                             : (yaml_cfg.has_free_thresh ? yaml_cfg.free_thresh : 0.196);

  // ── 读取 ───────────────────────────────────────────────────────────
  Loaded loaded;
  std::string error;
  if (!LoadStaticSubmap(pbstream, &loaded, &error)) return Finish(false, "pbstream_read_failed", error, "", 1);
  if (!loaded.has_static) {
    return Finish(false, "no_static_submap",
                  "pbstream has no global_static_submap (legacy map?), nothing to export", "", 1);
  }
  const auto& sm = loaded.static_submap;
  std::vector<std::string> outputs;
  std::ostringstream extra;

  // ── 点云：普通点与高亮点合在一个 PCD，用颜色和 label 区分 ──────────
  CloudCounts counts;
  if (what.count("cloud")) {
    pcl::PointCloud<pcl::PointXYZRGBL> cloud;
    if (sm.has_cloud()) AppendCloud(sm.cloud(), 160, 160, 160, 0, FLAGS_cloud_voxel, &cloud, &counts.normal);
    if (sm.has_highlight_cloud()) {
      AppendCloud(sm.highlight_cloud(), 255, 0, 0, 1, FLAGS_cloud_voxel, &cloud, &counts.highlight);
    }
    if (cloud.empty()) {
      LOG(WARNING) << "static submap has no point cloud (pbstream older than data_version 0.3.0?)";
    } else {
      cloud.width = cloud.size();
      cloud.height = 1;
      const std::string f = (out_dir / "cloud.pcd").string();
      if (pcl::io::savePCDFileBinary(f, cloud) != 0) return Finish(false, "write_failed", "cannot write " + f, "", 1);
      outputs.push_back(f);
      LOG(INFO) << "cloud.pcd: " << counts.normal << " points + " << counts.highlight << " highlight points";
    }
  }

  // ── 静态底图渲染 ───────────────────────────────────────────────────
  cv::Mat grid;
  Eigen::Vector2d origin(0, 0);
  double resolution = 0.0;
  const bool need_grid = what.count("grid") || what.count("overlay") || what.count("info");
  if (need_grid) {
    if (!sm.has_grid()) return Finish(false, "no_grid", "global_static_submap has no grid", "", 1);
    ::cartographer_ros::map_ops::RenderSingleStaticSubmap(sm, occ, free, &grid, &origin, &resolution);
  }
  if (what.count("grid")) {
    const std::string png = (out_dir / "static_map.png").string();
    const std::string yaml = (out_dir / "static_map.yaml").string();
    if (!cv::imwrite(png, grid)) return Finish(false, "write_failed", "cannot write " + png, "", 1);
    std::ofstream y(yaml);
    y << "image: static_map.png\n"
      << "resolution: " << Num(resolution) << "\n"
      << "origin: [" << Num(origin.x()) << ", " << Num(origin.y()) << ", 0.0]\n"
      << "negate: 0\n"
      << "occupied_thresh: " << Num(occ) << "\n"
      << "free_thresh: " << Num(free) << "\n";
    if (!y) return Finish(false, "write_failed", "cannot write " + yaml, "", 1);
    outputs.push_back(png);
    outputs.push_back(yaml);
  }

  // ── 线特征 ─────────────────────────────────────────────────────────
  const auto& lines = sm.line_map_2d_list().line_map_data();
  if (what.count("lines")) {
    const std::string f = (out_dir / "lines.json").string();
    std::ofstream o(f);
    o << "{\n  \"frame\": \"map\",\n  \"count\": " << lines.size() << ",\n  \"lines\": [";
    bool first = true;
    for (const auto& ml : lines) {
      const auto& l = ml.line_data();
      const double len = std::hypot(l.end_x() - l.begin_x(), l.end_y() - l.begin_y());
      o << (first ? "\n" : ",\n") << "    {\"global_id\": " << ml.global_id() << ", \"local_id\": " << ml.local_id()
        << ", \"begin\": [" << Num(l.begin_x()) << ", " << Num(l.begin_y()) << "], \"end\": [" << Num(l.end_x())
        << ", " << Num(l.end_y()) << "], \"length\": " << Num(len) << ", \"weight\": " << Num(l.weight())
        << ", \"normal\": "
        << (l.has_normal() ? "[" + Num(l.normal_x()) + ", " + Num(l.normal_y()) + "]" : std::string("null"))
        << ", \"inliers\": " << ml.inliers() << ", \"lose_time\": " << ml.lose_time()
        << ", \"observations\": " << ml.line_data_list_size() << "}";
      first = false;
    }
    o << (lines.empty() ? "]\n}\n" : "\n  ]\n}\n");
    if (!o) return Finish(false, "write_failed", "cannot write " + f, "", 1);
    outputs.push_back(f);
  }

  // ── 线特征叠加到底图 ───────────────────────────────────────────────
  if (what.count("overlay")) {
    cv::Mat color;
    cv::cvtColor(grid, color, cv::COLOR_GRAY2BGR);
    for (const auto& ml : lines) {
      const auto& l = ml.line_data();
      const auto a = WorldToPixel(l.begin_x(), l.begin_y(), origin, resolution, grid.rows);
      const auto b = WorldToPixel(l.end_x(), l.end_y(), origin, resolution, grid.rows);
      cv::line(color, a, b, cv::Scalar(0, 0, 255), FLAGS_line_thickness, cv::LINE_AA);
      cv::circle(color, a, FLAGS_line_thickness + 1, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
      cv::circle(color, b, FLAGS_line_thickness + 1, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
      if (FLAGS_label_lines) {
        cv::putText(color, std::to_string(ml.local_id()), (a + b) * 0.5, cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(0, 128, 0), 1, cv::LINE_AA);
      }
    }
    const std::string f = (out_dir / "lines_overlay.png").string();
    if (!cv::imwrite(f, color)) return Finish(false, "write_failed", "cannot write " + f, "", 1);
    outputs.push_back(f);
  }

  // ── 概要 ───────────────────────────────────────────────────────────
  std::ostringstream files;
  for (size_t i = 0; i < outputs.size(); ++i) files << (i ? ", " : "") << "\"" << JsonEscape(outputs[i]) << "\"";
  extra << "\"pbstream\": \"" << JsonEscape(pbstream) << "\", \"output_dir\": \"" << JsonEscape(out_dir.string())
        << "\", \"data_version\": \"" << JsonEscape(loaded.header.data_version()) << "\", \"cloud_points\": "
        << sm.cloud().points_size() << ", \"highlight_points\": " << sm.highlight_cloud().points_size()
        << ", \"lines\": " << lines.size();
  if (need_grid) {
    extra << ", \"grid\": {\"width\": " << grid.cols << ", \"height\": " << grid.rows
          << ", \"resolution\": " << Num(resolution) << ", \"origin\": [" << Num(origin.x()) << ", "
          << Num(origin.y()) << "], \"occupied_thresh\": " << Num(occ) << ", \"free_thresh\": " << Num(free) << "}";
  }
  if (what.count("info")) {
    const std::string f = (out_dir / "info.json").string();
    std::ofstream o(f);
    o << "{" << extra.str() << ", \"files\": [" << files.str() << ", \"" << JsonEscape(f) << "\"]}\n";
    if (!o) return Finish(false, "write_failed", "cannot write " + f, "", 1);
    outputs.push_back(f);
  }
  std::ostringstream all_files;
  for (size_t i = 0; i < outputs.size(); ++i) {
    all_files << (i ? ", " : "") << "\"" << JsonEscape(outputs[i]) << "\"";
  }
  return Finish(true, "", "exported " + std::to_string(outputs.size()) + " files",
                extra.str() + ", \"files\": [" + all_files.str() + "]", 0);
}
