/*
 * Copyright 2024 The Cartographer Authors
 *
 * Standalone test binary for map.yaml 合并写出（WriteMapYamlMerged / WriteYaml）
 *
 *   运行：`ros2 run cartographer_ros map_yaml_merge_test`
 *
 *   测试矩阵：
 *     M0 - 新建（无底）：键齐全、created_at 写当前时间、data_version 在末尾
 *     M1 - 未知键 / 注释 / 空行原样保留，顺序不变
 *     M2 - 本方键原位覆盖，底里缺的本方键追加在末尾
 *     M3 - created_at：默认沿用底；显式给值用给的；显式空串取当前时间；底没有则补当前时间
 *     M4 - display_yaw：有键时加补偿并归一化到 (-π, π]；无键不新增；补偿为 0 原样
 *     M5 - 本次不带面积时删掉底里的旧面积
 *     M6 - 重复键只留第一次出现
 *     M7 - 输出与底不同路径：底不被改动；输出目录无 .tmp 残留
 *     M8 - WriteYaml（新建图）与对照实现逐字节一致
 *     M9 - 底不存在时等价于新建
 *     M10 - ReadMapCreatedAt：带引号 / 不带引号 / 无键 / 文件不存在
 *     M11 - last_edited_at：新建与 created_at 相同；合并重写一律取当前时间、已有键原位覆盖
 *     M12 - 不写 line_map / cloud_map / highlight_cloud_map；合并时删掉旧地图里的这三个键，其余行不动
 */

#include "cartographer_ros/mapping/ros_map.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "cartographer/io/file_writer.h"
#include "glog/logging.h"

namespace fs = std::filesystem;
using cartographer_ros::GridMapStats;
using cartographer_ros::MakeMapYamlEntries;
using cartographer_ros::MapYamlMergeOptions;
using cartographer_ros::WriteMapYamlMerged;

// ---------------------------------------------------------------------------
// 测试框架（轻量级，自带计数 + 断言）
// ---------------------------------------------------------------------------

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT(cond, msg) do { \
  if (!(cond)) { \
    std::cerr << "  ✘ FAIL: " << msg << " (line " << __LINE__ << ")\n"; \
    ++g_fail; \
    return; \
  } \
} while (0)

#define RUN_TEST(fn) do { \
  std::cout << "▶ " #fn "\n"; \
  int fail_before = g_fail; \
  fn(); \
  if (g_fail == fail_before) { \
    std::cout << "  ✓ pass\n"; \
    ++g_pass; \
  } \
} while (0)

// ---------------------------------------------------------------------------
// 工具函数
// ---------------------------------------------------------------------------

static fs::path g_root;

static fs::path NewDir(const std::string& name) {
  const fs::path d = g_root / name;
  fs::remove_all(d);
  fs::create_directories(d);
  return d;
}

static void WriteText(const fs::path& p, const std::string& s) {
  std::ofstream(p) << s;
}

static std::string ReadText(const fs::path& p) {
  std::ifstream in(p);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

static std::vector<std::string> Lines(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  for (std::string line; std::getline(ss, line);) out.push_back(line);
  return out;
}

static std::string ValueOf(const std::string& text, const std::string& key) {
  for (const auto& line : Lines(text)) {
    if (line.rfind(key + ": ", 0) == 0) return line.substr(key.size() + 2);
  }
  return "<absent>";
}

static int CountKey(const std::string& text, const std::string& key) {
  int n = 0;
  for (const auto& line : Lines(text)) {
    if (line.rfind(key + ":", 0) == 0) ++n;
  }
  return n;
}

static bool LooksLikeNow(const std::string& v) {
  return std::regex_match(v, std::regex(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})"));
}

static GridMapStats Stats() {
  GridMapStats s;
  s.resolution = 0.05;
  s.total_cells = 400;
  s.free_cells = 100;
  return s;
}

static cartographer_ros::MapYamlEntries Entries(const GridMapStats* stats,
                                                const char* version = "0.3.0") {
  return MakeMapYamlEntries(0.05, Eigen::Vector2d(-1.5, 2.25), "/x/slam/map", 0.65,
                            0.196, std::string(version), stats);
}

// 一份"建图侧写出 + 被人手工加过东西"的旧 yaml
static const char* kBaseYaml =
    "# map written by mapping node\n"
    "image: map.png\n"
    "line_map: map_line_map.json\n"
    "cloud_map: map.pcd\n"
    "highlight_cloud_map: map_highlight.pcd\n"
    "resolution: 0.05\n"
    "origin: [-9, 9, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n"
    "total_area_m2: 999\n"
    "effective_area_m2: 888\n"
    "\n"
    "vendor_note: keep me  # trailing comment\n"
    "created_at: 2026-01-02T03:04:05\n"
    "display_yaw: 3.0\n"
    "data_version: 0.2.0\n";

// ---------------------------------------------------------------------------
// 测试用例
// ---------------------------------------------------------------------------

void TestM0_FreshWrite() {
  const fs::path d = NewDir("m0");
  const auto stats = Stats();
  EXPECT(WriteMapYamlMerged((d / "map.yaml").string(), Entries(&stats), {}), "write failed");
  const std::string t = ReadText(d / "map.yaml");
  const auto ls = Lines(t);
  EXPECT(ls.size() == 11, "expect 11 lines, got " << ls.size());
  EXPECT(ls.front() == "image: map.png", "first line " << ls.front());
  EXPECT(ValueOf(t, "origin") == "[-1.5, 2.25, 0.0]", "origin " << ValueOf(t, "origin"));
  EXPECT(ValueOf(t, "total_area_m2") == "1", "total " << ValueOf(t, "total_area_m2"));
  EXPECT(ValueOf(t, "effective_area_m2") == "0.25", "eff " << ValueOf(t, "effective_area_m2"));
  EXPECT(LooksLikeNow(ValueOf(t, "created_at")), "created_at " << ValueOf(t, "created_at"));
  EXPECT(ls[8].rfind("created_at: ", 0) == 0, "created_at should be 3rd last");
  EXPECT(ls[9].rfind("last_edited_at: ", 0) == 0, "last_edited_at should be 2nd last");
  for (const char* k : {"line_map", "cloud_map", "highlight_cloud_map"}) {
    EXPECT(CountKey(t, k) == 0, "file reference key must not be written: " << k);
  }
  EXPECT(ls.back() == "data_version: 0.3.0", "last line " << ls.back());
  EXPECT(CountKey(t, "display_yaw") == 0, "display_yaw must not be invented");
}

void TestM1_PreserveUnknown() {
  const fs::path d = NewDir("m1");
  WriteText(d / "map.yaml", kBaseYaml);
  const auto stats = Stats();
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "map.yaml").string();
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  const auto ls = Lines(ReadText(d / "map.yaml"));
  // 底里的 line_map / cloud_map / highlight_cloud_map 三行被删
  EXPECT(ls.size() == 15, "line count " << ls.size());
  EXPECT(ls[0] == "# map written by mapping node", "comment lost: " << ls[0]);
  EXPECT(ls[9].empty(), "blank line lost");
  EXPECT(ls[10] == "vendor_note: keep me  # trailing comment", "unknown key altered: " << ls[10]);
  EXPECT(ls[12] == "display_yaw: 3.0", "display_yaw altered: " << ls[12]);
}

void TestM2_OverrideAndAppend() {
  const fs::path d = NewDir("m2");
  WriteText(d / "map.yaml",
            "foo: 1\n"
            "origin: [-9, 9, 0.0]\n"
            "resolution: 0.1\n"
            "bar: 2\n");
  const auto stats = Stats();
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "map.yaml").string();
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  const std::string t = ReadText(d / "map.yaml");
  const auto ls = Lines(t);
  EXPECT(ls[0] == "foo: 1", "foo moved");
  EXPECT(ls[1] == "origin: [-1.5, 2.25, 0.0]", "origin not overridden in place: " << ls[1]);
  EXPECT(ls[2] == "resolution: 0.05", "resolution not overridden in place: " << ls[2]);
  EXPECT(ls[3] == "bar: 2", "bar moved");
  EXPECT(ls[4] == "image: map.png", "first appended key: " << ls[4]);
  EXPECT(ls.back() == "data_version: 0.3.0", "data_version not last: " << ls.back());
  EXPECT(CountKey(t, "free_thresh") == 1 && CountKey(t, "created_at") == 1, "missing appended keys");
}

void TestM3_CreatedAt() {
  const fs::path d = NewDir("m3");
  const auto stats = Stats();
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "map.yaml").string();

  WriteText(d / "map.yaml", kBaseYaml);
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "inherit write failed");
  EXPECT(ValueOf(ReadText(d / "map.yaml"), "created_at") == "2026-01-02T03:04:05", "not inherited");

  WriteText(d / "map.yaml", kBaseYaml);
  o.created_at = std::string("2025-05-05T05:05:05");
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "override write failed");
  EXPECT(ValueOf(ReadText(d / "map.yaml"), "created_at") == "2025-05-05T05:05:05", "not overridden");

  WriteText(d / "map.yaml", kBaseYaml);
  o.created_at = std::string();
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "now write failed");
  const std::string now = ValueOf(ReadText(d / "map.yaml"), "created_at");
  EXPECT(LooksLikeNow(now) && now != "2026-01-02T03:04:05", "empty override should be now: " << now);

  WriteText(d / "map.yaml", "image: map.png\n");
  o.created_at.reset();
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "absent write failed");
  EXPECT(LooksLikeNow(ValueOf(ReadText(d / "map.yaml"), "created_at")), "absent should get now");
}

void TestM4_DisplayYaw() {
  const fs::path d = NewDir("m4");
  const auto stats = Stats();
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "map.yaml").string();

  WriteText(d / "map.yaml", kBaseYaml);  // display_yaw: 3.0
  o.display_yaw_delta = 0.5;              // 3.5 → 3.5 - 2π
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  const double yaw = std::stod(ValueOf(ReadText(d / "map.yaml"), "display_yaw"));
  EXPECT(std::abs(yaw - (3.5 - 2.0 * M_PI)) < 1e-9, "not normalized: " << yaw);

  WriteText(d / "map.yaml", "display_yaw: -1.0\n");
  o.display_yaw_delta = -2.0 * M_PI;  // 转一整圈 → 原值
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  EXPECT(std::abs(std::stod(ValueOf(ReadText(d / "map.yaml"), "display_yaw")) + 1.0) < 1e-9,
         "full turn should be identity");

  WriteText(d / "map.yaml", "display_yaw: " + absl::StrCat(-M_PI) + "\n");
  o.display_yaw_delta = 1e-12;  // 落在 -π 边上 → 归到 +π 一侧附近
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  const double edge = std::stod(ValueOf(ReadText(d / "map.yaml"), "display_yaw"));
  EXPECT(edge > -M_PI && edge <= M_PI, "edge out of (-π, π]: " << edge);

  WriteText(d / "map.yaml", "image: map.png\n");
  o.display_yaw_delta = 0.7;
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  EXPECT(CountKey(ReadText(d / "map.yaml"), "display_yaw") == 0, "absent key must not be added");

  WriteText(d / "map.yaml", "display_yaw: 0.10\n");
  o.display_yaw_delta = 0.0;
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  EXPECT(ValueOf(ReadText(d / "map.yaml"), "display_yaw") == "0.10", "zero delta must keep bytes");
}

void TestM5_StaleAreaErased() {
  const fs::path d = NewDir("m5");
  WriteText(d / "map.yaml", kBaseYaml);
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "map.yaml").string();
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(nullptr), o), "write failed");
  const std::string t = ReadText(d / "map.yaml");
  EXPECT(CountKey(t, "total_area_m2") == 0 && CountKey(t, "effective_area_m2") == 0,
         "stale area keys kept");
  EXPECT(CountKey(t, "vendor_note") == 1, "unrelated key lost");
}

void TestM6_DedupeKeys() {
  const fs::path d = NewDir("m6");
  WriteText(d / "map.yaml", "foo: 1\nfoo: 2\nresolution: 1\nresolution: 2\n");
  const auto stats = Stats();
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "map.yaml").string();
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  const std::string t = ReadText(d / "map.yaml");
  EXPECT(CountKey(t, "foo") == 1 && ValueOf(t, "foo") == "1", "foo dedupe");
  EXPECT(CountKey(t, "resolution") == 1 && ValueOf(t, "resolution") == "0.05", "resolution dedupe");
}

void TestM7_SeparateOutput() {
  const fs::path live = NewDir("m7_live");
  const fs::path staging = NewDir("m7_staging");
  WriteText(live / "map.yaml", kBaseYaml);
  const auto stats = Stats();
  MapYamlMergeOptions o;
  o.base_yaml_path = (live / "map.yaml").string();
  o.display_yaw_delta = 0.1;
  EXPECT(WriteMapYamlMerged((staging / "map.yaml").string(), Entries(&stats), o), "write failed");
  EXPECT(ReadText(live / "map.yaml") == kBaseYaml, "base must stay untouched");
  const std::string t = ReadText(staging / "map.yaml");
  EXPECT(ValueOf(t, "created_at") == "2026-01-02T03:04:05", "created_at not carried");
  EXPECT(std::abs(std::stod(ValueOf(t, "display_yaw")) - 3.1) < 1e-9, "yaw not compensated");
  EXPECT(ValueOf(t, "vendor_note") == "keep me  # trailing comment", "unknown key not carried");
  EXPECT(!fs::exists(staging / "map.yaml.tmp"), "tmp left behind");
  EXPECT(fs::exists(live / ".map.yaml.lock"), "lock should sit next to the base");
}

// WriteYaml 的对照实现（改造前的拼接写法，去掉三个文件引用键，last_edited_at 紧跟 created_at）
static std::string LegacyWriteYaml(double resolution, const Eigen::Vector2d& origin,
                                   const std::string& map_file_name, double occ_thresh,
                                   double free_thresh,
                                   const absl::optional<std::string>& data_version,
                                   const GridMapStats* stats, const std::string& created_at) {
  const size_t slash = map_file_name.find_last_of('/');
  const std::string base =
      (slash == std::string::npos) ? map_file_name : map_file_name.substr(slash + 1);
  std::string output = absl::StrCat(
      "image: ", base + ".png", "\n",
      "resolution: ", resolution, "\n",
      "origin: [", origin.x(), ", ", origin.y(),
      ", 0.0]\nnegate: 0\noccupied_thresh: ", occ_thresh, "\nfree_thresh: ", free_thresh, "\n");
  if (stats != nullptr) {
    absl::StrAppend(&output,
        "total_area_m2: ", stats->TotalAreaM2(), "\n",
        "effective_area_m2: ", stats->EffectiveAreaM2(), "\n");
  }
  absl::StrAppend(&output, "created_at: ", created_at, "\n");
  absl::StrAppend(&output, "last_edited_at: ", created_at, "\n");
  if (data_version.has_value()) {
    absl::StrAppend(&output, "data_version: ",
                    data_version->empty() ? std::string("legacy") : *data_version, "\n");
  }
  return output;
}

void TestM8_WriteYamlByteIdentical() {
  const fs::path d = NewDir("m8");
  const auto stats = Stats();
  const float occ = 0.65f, fr = 0.196f;  // 节点侧传的是 float
  struct Case { const GridMapStats* s; absl::optional<std::string> v; };
  const std::vector<Case> cases = {{&stats, std::string("0.3.0")},
                                   {&stats, std::string()},
                                   {nullptr, absl::nullopt}};
  int i = 0;
  for (const auto& c : cases) {
    const fs::path p = d / absl::StrCat("map", i++, ".yaml");
    {
      cartographer::io::StreamFileWriter w(p.string());
      cartographer_ros::WriteYaml(0.05, Eigen::Vector2d(-12.345, 6.7), "/a/b/map", &w, occ, fr,
                                  c.v, c.s, "2026-01-02T03:04:05");
    }
    const std::string want = LegacyWriteYaml(0.05, Eigen::Vector2d(-12.345, 6.7), "/a/b/map",
                                             occ, fr, c.v, c.s, "2026-01-02T03:04:05");
    EXPECT(ReadText(p) == want, "case " << i << " differs:\n" << ReadText(p) << "---\n" << want);
  }
}

void TestM9_MissingBase() {
  const fs::path d = NewDir("m9");
  const auto stats = Stats();
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "nope.yaml").string();
  o.display_yaw_delta = 1.0;
  EXPECT(WriteMapYamlMerged((d / "map.yaml").string(), Entries(&stats), o), "write failed");
  const std::string t = ReadText(d / "map.yaml");
  EXPECT(Lines(t).size() == 11, "should equal fresh write");
  EXPECT(CountKey(t, "display_yaw") == 0, "display_yaw must not be invented");
}

void TestM10_ReadMapCreatedAt() {
  const fs::path d = NewDir("m10");
  WriteText(d / "a.yaml", "image: map.png\ncreated_at: \"2026-01-02T03:04:05\"\n");
  WriteText(d / "b.yaml", "# c\ncreated_at: 2026-01-02T03:04:05  \n");
  WriteText(d / "c.yaml", "image: map.png\n");
  EXPECT(cartographer_ros::ReadMapCreatedAt((d / "a.yaml").string()) == "2026-01-02T03:04:05",
         "quoted");
  EXPECT(cartographer_ros::ReadMapCreatedAt((d / "b.yaml").string()) == "2026-01-02T03:04:05",
         "plain with trailing spaces");
  EXPECT(cartographer_ros::ReadMapCreatedAt((d / "c.yaml").string()).empty(), "absent key");
  EXPECT(cartographer_ros::ReadMapCreatedAt((d / "nope.yaml").string()).empty(), "missing file");
}

void TestM11_LastEditedAt() {
  const fs::path d = NewDir("m11");
  const auto stats = Stats();
  {
    cartographer::io::StreamFileWriter w((d / "fresh.yaml").string());
    cartographer_ros::WriteYaml(0.05, Eigen::Vector2d(0, 0), "map", &w, 0.65, 0.196,
                                std::string("0.3.0"), &stats, "");
  }
  const std::string fresh = ReadText(d / "fresh.yaml");
  EXPECT(LooksLikeNow(ValueOf(fresh, "last_edited_at")) &&
             ValueOf(fresh, "last_edited_at") == ValueOf(fresh, "created_at"),
         "fresh last_edited_at should equal created_at");

  // 底里有旧值：原位覆盖成现在，created_at 不动
  WriteText(d / "map.yaml",
            "image: map.png\ncreated_at: 2026-01-02T03:04:05\n"
            "last_edited_at: 2026-01-03T00:00:00\nvendor: x\n");
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "map.yaml").string();
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "merge write failed");
  const std::string t = ReadText(d / "map.yaml");
  const auto ls = Lines(t);
  EXPECT(CountKey(t, "last_edited_at") == 1, "last_edited_at duplicated");
  EXPECT(ls[2].rfind("last_edited_at: ", 0) == 0, "not overridden in place: " << ls[2]);
  const std::string edited = ValueOf(t, "last_edited_at");
  EXPECT(LooksLikeNow(edited) && edited != "2026-01-03T00:00:00", "should be now: " << edited);
  EXPECT(ValueOf(t, "created_at") == "2026-01-02T03:04:05", "created_at must be kept");

  // 底里没有：追加
  WriteText(d / "map.yaml", kBaseYaml);
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "append write failed");
  EXPECT(LooksLikeNow(ValueOf(ReadText(d / "map.yaml"), "last_edited_at")), "absent should get now");
}

void TestM12_DropFileReferenceKeys() {
  const fs::path d = NewDir("m12");
  WriteText(d / "map.yaml", kBaseYaml);
  const auto stats = Stats();
  MapYamlMergeOptions o;
  o.base_yaml_path = (d / "map.yaml").string();
  EXPECT(WriteMapYamlMerged(o.base_yaml_path, Entries(&stats), o), "write failed");
  const std::string t = ReadText(d / "map.yaml");
  for (const char* k : {"line_map", "cloud_map", "highlight_cloud_map"}) {
    EXPECT(CountKey(t, k) == 0, "legacy key not erased: " << k);
  }
  EXPECT(ValueOf(t, "vendor_note") == "keep me  # trailing comment", "unknown key must survive");
  EXPECT(ValueOf(t, "image") == "map.png", "image kept");
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_minloglevel = 1;
  g_root = fs::temp_directory_path() / absl::StrCat("map_yaml_merge_test_", ::getpid());
  fs::create_directories(g_root);

  RUN_TEST(TestM0_FreshWrite);
  RUN_TEST(TestM1_PreserveUnknown);
  RUN_TEST(TestM2_OverrideAndAppend);
  RUN_TEST(TestM3_CreatedAt);
  RUN_TEST(TestM4_DisplayYaw);
  RUN_TEST(TestM5_StaleAreaErased);
  RUN_TEST(TestM6_DedupeKeys);
  RUN_TEST(TestM7_SeparateOutput);
  RUN_TEST(TestM8_WriteYamlByteIdentical);
  RUN_TEST(TestM9_MissingBase);
  RUN_TEST(TestM10_ReadMapCreatedAt);
  RUN_TEST(TestM11_LastEditedAt);
  RUN_TEST(TestM12_DropFileReferenceKeys);

  fs::remove_all(g_root);
  std::cout << "\n== " << g_pass << " passed, " << g_fail << " failed ==\n";
  return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
