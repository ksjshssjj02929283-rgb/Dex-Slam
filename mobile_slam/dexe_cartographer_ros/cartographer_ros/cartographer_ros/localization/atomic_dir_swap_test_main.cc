/*
 * Copyright 2024 The Cartographer Authors
 *
 * Standalone test binary for AtomicDirSwap (Phase 4 注入测试套件)
 *
 *   测试矩阵：
 *     T0 - 基础流程：BeginStage → 写文件 → CommitStage → ActivateAndCleanup
 *     T1 - Step 1 中断：staging 写一半 → RecoverIfInterrupted 清 staging
 *     T2 - Step 2 中断：sentinel 缺失 → 同上
 *     T3 - Step 2 完成、Step 3 之前中断：续做切换
 *     T4 - Step 3 完成、Step 4 之前中断：续做 step 4-5a
 *     T5 - Step 4 完成、Step 5a 之前中断：归档 swap_backup
 *     T6 - Step 5b 中断（tombstone 残留）：清扫
 *     T7 - 历史 backup 轮换：保存 7 次 max_backups=3 → 应剩 3 个
 *     T8 - 首次启动（全空）：RecoverIfInterrupted 返回 false
 *     T9 - 极端：仅 swap_backup → 还原为 target
 */

#include "cartographer_ros/localization/atomic_dir_swap.h"

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "glog/logging.h"

namespace fs = std::filesystem;
using cartographer_ros::AtomicDirSwap;

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
// 测试工具：为每个测试准备独立目录
// ---------------------------------------------------------------------------

static std::string MakeTestRoot() {
  static int counter = 0;
  ++counter;
  std::string path = std::string("/tmp/atomic_swap_test_") +
                     std::to_string(::getpid()) + "_" +
                     std::to_string(counter);
  fs::remove_all(path);
  fs::create_directories(path);
  return path;
}

static void WriteFile(const std::string& path, const std::string& content) {
  std::ofstream ofs(path);
  ofs << content;
}

static std::string ReadFile(const std::string& path) {
  std::ifstream ifs(path);
  std::string content((std::istreambuf_iterator<char>(ifs)),
                      std::istreambuf_iterator<char>());
  return content;
}

// 写一份"假地图"三件套到 staging
static bool WriteFakeMapToStaging(const std::string& staging,
                                  const std::string& tag) {
  WriteFile(staging + "/map.pbstream", "pbstream-content-" + tag);
  WriteFile(staging + "/map.png", "png-content-" + tag);
  WriteFile(staging + "/map.yaml", "yaml-content-" + tag);
  return true;
}

// 直接放一份完整地图（带 sentinel）到 target_dir（模拟"已正常生效的旧地图"）
static void PutValidMap(const std::string& target_dir,
                        const std::string& tag,
                        std::time_t ts_unix = 0) {
  fs::create_directories(target_dir);
  WriteFile(target_dir + "/map.pbstream", "pbstream-content-" + tag);
  WriteFile(target_dir + "/map.png", "png-content-" + tag);
  WriteFile(target_dir + "/map.yaml", "yaml-content-" + tag);
  if (ts_unix == 0) ts_unix = std::time(nullptr);
  std::ofstream ofs(target_dir + "/.complete");
  ofs << "version=1\ntimestamp_unix=" << ts_unix << "\n"
      << "files=map.pbstream,map.png,map.yaml\n";
}

// ---------------------------------------------------------------------------
// 测试用例
// ---------------------------------------------------------------------------

// T0: 基础流程
static void TestBasicFlow() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_a";

  // 第一次保存（无旧 target）
  AtomicDirSwap swap(target, /*max_backups=*/5);
  std::string staging = swap.BeginStage();
  EXPECT(!staging.empty(), "BeginStage should succeed");
  EXPECT(fs::exists(staging), "staging dir should exist");

  WriteFakeMapToStaging(staging, "v1");

  EXPECT(swap.CommitStage({"map.pbstream", "map.png", "map.yaml"}),
         "CommitStage should succeed");
  EXPECT(fs::exists(staging + "/" + AtomicDirSwap::kSentinelName),
         "sentinel should exist after commit");

  EXPECT(swap.ActivateAndCleanup(), "ActivateAndCleanup should succeed");
  EXPECT(fs::exists(target + "/map.pbstream"), "target should have pbstream");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-v1",
         "target pbstream content correct");
  EXPECT(!fs::exists(staging), "staging should be gone");
  EXPECT(!fs::exists(target + ".swap_backup"), "swap_backup should be gone");
}

// T1: 模拟 Step 1 中断（staging 无 sentinel）→ RecoverIfInterrupted 清 staging
static void TestRecoverFromStep1Interrupt() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_b";

  // 已有正常 target
  PutValidMap(target, "old");

  // 模拟中断：staging 存在但无 sentinel
  fs::create_directories(target + ".staging");
  WriteFile(target + ".staging/map.pbstream", "half-written");

  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover should return true");
  EXPECT(!fs::exists(target + ".staging"), "incomplete staging should be removed");
  EXPECT(fs::exists(target + "/map.pbstream"), "target still intact");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-old",
         "old target preserved");
}

// T2: Step 2 完成、Step 3 之前中断 → 续做切换
static void TestRecoverFromStep2Done() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_c";

  PutValidMap(target, "old");

  // staging 完整（含 sentinel）
  std::string staging = target + ".staging";
  fs::create_directories(staging);
  WriteFakeMapToStaging(staging, "new");
  std::ofstream(staging + "/.complete")
      << "version=1\ntimestamp_unix=" << std::time(nullptr) << "\n"
      << "files=map.pbstream,map.png,map.yaml\n";

  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover should succeed");
  EXPECT(fs::exists(target + "/map.pbstream"), "target should exist");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-new",
         "target should be the new content");
  EXPECT(!fs::exists(staging), "staging should be consumed");
  // 应有一个历史 backup（旧版本归档）
  auto backups = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(backups.size() == 1, "should have 1 historical backup");
  if (!backups.empty()) {
    EXPECT(ReadFile(backups[0] + "/map.pbstream") == "pbstream-content-old",
           "backup should contain old content");
  }
}

// T3: Step 3 完成、Step 4 之前中断 → 续做 step 4-5a
static void TestRecoverFromStep3Done() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_d";

  // target 已被 rename 到 swap_backup（step 3 已发生）
  PutValidMap(target + ".swap_backup", "old");

  // staging 完整（含 sentinel）
  std::string staging = target + ".staging";
  fs::create_directories(staging);
  WriteFakeMapToStaging(staging, "new");
  std::ofstream(staging + "/.complete")
      << "version=1\ntimestamp_unix=" << std::time(nullptr) << "\n"
      << "files=map.pbstream,map.png,map.yaml\n";

  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover should succeed");
  EXPECT(fs::exists(target + "/map.pbstream"), "target should exist (from staging)");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-new",
         "target = new");
  EXPECT(!fs::exists(target + ".swap_backup"), "swap_backup should be archived");
  EXPECT(!fs::exists(staging), "staging consumed");

  auto backups = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(backups.size() == 1, "should have 1 historical backup");
}

// T4: Step 4 完成、Step 5a 之前中断 → 归档 swap_backup
static void TestRecoverFromStep4Done() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_e";

  PutValidMap(target, "new");                          // step 4 完成
  PutValidMap(target + ".swap_backup", "old");         // 旧版本待归档

  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover should succeed");
  EXPECT(fs::exists(target + "/map.pbstream"), "target intact");
  EXPECT(!fs::exists(target + ".swap_backup"), "swap_backup archived");

  auto backups = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(backups.size() == 1, "1 historical backup");
}

// T5: tombstone 残留 → RecoverIfInterrupted 清扫
static void TestTombstoneCleanup() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_f";

  PutValidMap(target, "current");

  // v3: tombstone 在 <target>/backups/ 容器内（target 内部）
  const std::string container = target + "/backups";
  fs::create_directories(container + "/tombstone.0");
  WriteFile(container + "/tombstone.0/map.pbstream", "deleted-stuff");
  fs::create_directories(container + "/tombstone.1");

  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover succeeds");
  EXPECT(!fs::exists(container + "/tombstone.0"), "tombstone.0 cleaned");
  EXPECT(!fs::exists(container + "/tombstone.1"), "tombstone.1 cleaned");
  EXPECT(fs::exists(target + "/map.pbstream"), "target intact");
}

// T6: 历史 backup 轮换 — 保存 7 次 max_backups=3 → 应剩 3 个
static void TestRotation() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_g";

  // 第一次：先放一份初始 target，不算 backup
  PutValidMap(target, "v0");

  // 连续保存 7 次
  for (int i = 1; i <= 7; ++i) {
    AtomicDirSwap swap(target, /*max_backups=*/3);
    std::string staging = swap.BeginStage();
    WriteFakeMapToStaging(staging, "v" + std::to_string(i));
    EXPECT(swap.CommitStage({"map.pbstream", "map.png", "map.yaml"}),
           "commit v" + std::to_string(i));
    EXPECT(swap.ActivateAndCleanup(), "activate v" + std::to_string(i));
    // 同秒可能时间戳冲突；等 1.05 秒确保 backup 名不同
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  }

  auto backups = AtomicDirSwap::ListHistoricalBackups(target);
  std::cout << "  historical backups after 7 saves (max=3): " << backups.size() << "\n";
  for (const auto& b : backups) std::cout << "    " << b << "\n";
  EXPECT(backups.size() == 3, "should keep exactly 3 backups");
  // 当前 target 应是 v7
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-v7",
         "target is v7");
  // 最新 backup 应是 v6（target 的上一版本）
  EXPECT(ReadFile(backups[0] + "/map.pbstream") == "pbstream-content-v6",
         "newest backup is v6");
  EXPECT(ReadFile(backups[2] + "/map.pbstream") == "pbstream-content-v4",
         "oldest kept backup is v4");
}

// T7: 全空 → recover 返回 false
static void TestEmptyState() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_h";  // 不创建
  EXPECT(!AtomicDirSwap::RecoverIfInterrupted(target),
         "should return false on empty state");
}

// T8: 仅 swap_backup（极端） → 还原
static void TestOnlySwapBackup() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_i";

  PutValidMap(target + ".swap_backup", "old");

  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover succeeds");
  EXPECT(fs::exists(target + "/map.pbstream"), "target restored");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-old",
         "old content restored");
  EXPECT(!fs::exists(target + ".swap_backup"), "swap_backup gone");
}

// T9: max_backups=0 → 不保留任何历史
static void TestZeroBackups() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_j";

  PutValidMap(target, "v0");

  for (int i = 1; i <= 3; ++i) {
    AtomicDirSwap swap(target, /*max_backups=*/0);
    std::string staging = swap.BeginStage();
    WriteFakeMapToStaging(staging, "v" + std::to_string(i));
    EXPECT(swap.CommitStage({"map.pbstream", "map.png", "map.yaml"}),
           "commit v" + std::to_string(i));
    EXPECT(swap.ActivateAndCleanup(), "activate v" + std::to_string(i));
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  }

  auto backups = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(backups.size() == 0, "max_backups=0 should keep no backups");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-v3",
         "target is v3");
}

// ---------------------------------------------------------------------------
// 注入测试：用 fork() + _exit() 模拟 SIGKILL 中断
// ---------------------------------------------------------------------------

// 在子进程中执行 partial 保存，在 sentinel 写完之前 _exit() 模拟 kill
static void ForkInterruptAfterStaging() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_inj1";
  PutValidMap(target, "old");

  pid_t pid = fork();
  if (pid == 0) {
    // 子进程：写到 staging 一半就 _exit
    AtomicDirSwap swap(target, 5);
    std::string staging = swap.BeginStage();
    WriteFile(staging + "/map.pbstream", "partial");
    // 模拟在写 png 中途被 kill：直接 _exit 不调用 CommitStage
    _exit(0);
  }
  int status = 0;
  ::waitpid(pid, &status, 0);

  // 父进程：调 recover，期望原图完整
  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover succeeds after kill");
  EXPECT(fs::exists(target + "/map.pbstream"), "old target intact");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-old",
         "old content preserved");
  EXPECT(!fs::exists(target + ".staging"), "staging cleaned");
}

// 子进程完成到 CommitStage 后立刻 _exit（step 2 done、step 3 未开始）
static void ForkInterruptAfterCommit() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_inj2";
  PutValidMap(target, "old");

  pid_t pid = fork();
  if (pid == 0) {
    AtomicDirSwap swap(target, 5);
    std::string staging = swap.BeginStage();
    WriteFakeMapToStaging(staging, "new");
    swap.CommitStage({"map.pbstream", "map.png", "map.yaml"});
    _exit(0);  // 不调 ActivateAndCleanup
  }
  int status = 0;
  ::waitpid(pid, &status, 0);

  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover succeeds");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-new",
         "new content activated");
  auto backups = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(backups.size() == 1, "1 backup archived");
}

// ---------------------------------------------------------------------------
// v1.2: SHA256 sentinel 完整性校验测试
// ---------------------------------------------------------------------------

// T12: 篡改 staging/map.pbstream 内容 → recover 应识别出 sha256 不匹配，
//      清掉 staging，保留旧 target（未升级）。
static void TestChecksumMismatch() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_k";

  // 1. 先放一份完整 target（用 PutValidMap 写一份"旧地图"）
  PutValidMap(target, "old");

  // 2. 模拟 step 2 完成：staging 完整 + 真实 CommitStage 写入带 SHA256 的 sentinel
  {
    AtomicDirSwap swap(target, /*max_backups=*/5, /*enable_checksum=*/true);
    std::string staging = swap.BeginStage();
    EXPECT(!staging.empty(), "BeginStage staging");
    WriteFakeMapToStaging(staging, "new");
    EXPECT(swap.CommitStage({"map.pbstream", "map.png", "map.yaml"}),
           "CommitStage with checksum");
    // 注意：此处不调 ActivateAndCleanup，模拟 step 2 完成、step 3 之前中断
  }

  // 3. 篡改 staging/map.pbstream：写入与 sentinel 不一致的内容
  WriteFile(target + ".staging/map.pbstream", "tampered-bytes-XYZ");

  // 4. 调 RecoverIfInterrupted：应识别篡改，回到旧 target
  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target),
         "recover should still return true (old target preserved)");
  EXPECT(!fs::exists(target + ".staging"),
         "tampered staging should be removed");
  EXPECT(fs::exists(target + "/map.pbstream"), "old target intact");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-old",
         "old content preserved (not upgraded to tampered new)");
  // 不应产生历史 backup（因为切换没真正发生）
  auto backups = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(backups.size() == 0,
         "no historical backup should be created from rejected staging");
}

// T13: enable_checksum=false → sentinel 中不应有 sha256_ 字段；ActivateAndCleanup 仍成功
static void TestChecksumDisabled() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_l";

  AtomicDirSwap swap(target, /*max_backups=*/5, /*enable_checksum=*/false);
  std::string staging = swap.BeginStage();
  EXPECT(!staging.empty(), "BeginStage staging");
  WriteFakeMapToStaging(staging, "v1");
  EXPECT(swap.CommitStage({"map.pbstream", "map.png", "map.yaml"}),
         "CommitStage with checksum disabled");

  // 校验 sentinel 内容不含 sha256_
  const std::string sentinel_content =
      ReadFile(staging + "/" + AtomicDirSwap::kSentinelName);
  EXPECT(sentinel_content.find("sha256_") == std::string::npos,
         "sentinel must NOT contain sha256_ when checksum disabled");
  // 但仍应有 timestamp_unix / files 等字段
  EXPECT(sentinel_content.find("timestamp_unix=") != std::string::npos,
         "sentinel should still have timestamp_unix");
  EXPECT(sentinel_content.find("files=") != std::string::npos,
         "sentinel should still have files=");

  EXPECT(swap.ActivateAndCleanup(),
         "ActivateAndCleanup should succeed with checksum disabled");
  EXPECT(fs::exists(target + "/map.pbstream"), "target written");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-v1",
         "target content correct");
}

// ---------------------------------------------------------------------------
// v2 测试：rollback marker 协议
// ---------------------------------------------------------------------------

// T14: rollback 三件套替换中途崩溃（marker 存在 + 三件套写了一半）
//      → RecoverRollbackIfInterrupted 从 chosen 重做，三件套恢复一致；backups 不动
static void TestRollbackMarkerRedo() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_rb1";

  // 现有 target = "current" + 1 个历史 backup = "old"
  PutValidMap(target, "current");
  fs::create_directories(target + "/backups");
  const std::string chosen = target + "/backups/backup.20260101-100000";
  PutValidMap(chosen, "old");

  // 模拟 rollback 三件套替换中途崩溃：marker 已写 + pbstream 已换成 old 但 png/yaml 还是 current
  EXPECT(AtomicDirSwap::WriteRollbackMarker(target, chosen), "WriteRollbackMarker");
  WriteFile(target + "/map.pbstream", "pbstream-content-old");  // 只换了 pbstream（混版！）
  // png/yaml 仍是 current → 此刻三件套不一致

  // 启动恢复 — 从 chosen 重做全部三件套
  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover should succeed");

  // 验证：三件套全部变成 old（一致），backups 不动，marker 清除
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-old", "pbstream = old");
  EXPECT(ReadFile(target + "/map.png") == "png-content-old", "png = old (redone)");
  EXPECT(ReadFile(target + "/map.yaml") == "yaml-content-old", "yaml = old (redone)");
  EXPECT(fs::exists(chosen), "chosen backup preserved (not consumed)");
  EXPECT(ReadFile(chosen + "/map.pbstream") == "pbstream-content-old", "chosen content intact");
  EXPECT(!fs::exists(target + ".rollback_marker"), "marker cleaned");
  // 无 .rbtmp 残留
  std::error_code ec;
  for (auto const& e : fs::directory_iterator(target, ec)) {
    const std::string fn = e.path().filename().string();
    EXPECT(fn.find(".rbtmp") == std::string::npos, "no .rbtmp leftover: " + fn);
  }
}

// T15: rollback 崩溃后 chosen backup 消失（异常）→ 清理 marker + tmp，target 保持原样不损坏
static void TestRollbackMarkerChosenGone() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_rb2";

  PutValidMap(target, "current");
  fs::create_directories(target + "/backups");
  const std::string chosen = target + "/backups/backup.20260101-100000";
  PutValidMap(chosen, "old");

  EXPECT(AtomicDirSwap::WriteRollbackMarker(target, chosen), "marker");
  // 残留一个 .rbtmp（模拟崩溃在 phase 1）
  WriteFile(target + "/map.pbstream.rbtmp", "half-copied");
  // chosen 异常消失
  std::error_code ec;
  fs::remove_all(chosen, ec);

  EXPECT(AtomicDirSwap::RecoverIfInterrupted(target), "recover succeeds (cleanup path)");

  // 验证：chosen 没了无法重做，但 target 三件套保持原样（current），.rbtmp 清掉，marker 清掉
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-current",
         "target unchanged (current preserved, rollback never completed)");
  EXPECT(!fs::exists(target + "/map.pbstream.rbtmp"), ".rbtmp cleaned");
  EXPECT(!fs::exists(target + ".rollback_marker"), "marker cleaned");
}

// ---------------------------------------------------------------------------
// 地图目录自包含验证
// ---------------------------------------------------------------------------

// T16: 历史 backup 落在 <target>/backups/ 内（target 自包含）
//      含 step 5a "swap_backup 内 backups 抽出到新 target" 流程
static void TestSelfContainedBackups() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_v3";

  // 连续保存 3 次形成历史链
  AtomicDirSwap swap1(target, 5);
  std::string staging = swap1.BeginStage();
  WriteFakeMapToStaging(staging, "v1");
  EXPECT(swap1.CommitStage({"map.pbstream", "map.png", "map.yaml"}), "v1 commit");
  EXPECT(swap1.ActivateAndCleanup(), "v1 activate");

  std::this_thread::sleep_for(std::chrono::milliseconds(1100));

  AtomicDirSwap swap2(target, 5);
  staging = swap2.BeginStage();
  WriteFakeMapToStaging(staging, "v2");
  EXPECT(swap2.CommitStage({"map.pbstream", "map.png", "map.yaml"}), "v2 commit");
  EXPECT(swap2.ActivateAndCleanup(), "v2 activate");

  std::this_thread::sleep_for(std::chrono::milliseconds(1100));

  AtomicDirSwap swap3(target, 5);
  staging = swap3.BeginStage();
  WriteFakeMapToStaging(staging, "v3");
  EXPECT(swap3.CommitStage({"map.pbstream", "map.png", "map.yaml"}), "v3 commit");
  EXPECT(swap3.ActivateAndCleanup(), "v3 activate");

  // 关键断言 1：target 内有 backups 子目录（v3 自包含）
  EXPECT(fs::exists(target + "/backups"), "target/backups exists (self-contained)");

  // 关键断言 2：所有历史 backup 都在 target 内部
  auto backups = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(backups.size() == 2, "have 2 historical backups (v1 + v2)");
  for (const auto& b : backups) {
    EXPECT(b.find(target + "/backups/backup.") == 0,
           "backup path inside target: " + b);
  }

  // 关键断言 3：<map_folder> 顶级**没有** target.backup.* / target.backups/ 散落
  EXPECT(!fs::exists(target + ".backup.20260520-093045"), "no legacy v2.0 sibling");
  EXPECT(!fs::exists(target + ".backups"), "no v2.1 sibling container");

  // 关键断言 4：target 内三件套是 v3 内容（最新生效）
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-v3", "current is v3");

  // 关键断言 5：历史 backup 内容是 v1/v2
  EXPECT(ReadFile(backups[0] + "/map.pbstream") == "pbstream-content-v2", "newest backup is v2");
  EXPECT(ReadFile(backups[1] + "/map.pbstream") == "pbstream-content-v1", "oldest backup is v1");

  // 关键断言 6：删除整个 target 目录 → 所有历史一起被删（用户的"一个目录管理所有"诉求）
  fs::remove_all(target);
  EXPECT(!fs::exists(target), "target removed");
  EXPECT(!fs::exists(target + "/backups"), "backups removed with target");
  // 没有任何散落条目残留
  std::error_code ec;
  for (auto const& entry : fs::directory_iterator(root, ec)) {
    EXPECT(false, "unexpected leftover: " + entry.path().string());
  }
}

// T18: rollback 纯替换语义 — ReplaceCurrentMapFrom 只换三件套，backups 完全不动
static void TestReplaceCurrentMapFrom() {
  const std::string root = MakeTestRoot();
  const std::string target = root + "/map_rb_sem";

  // 保存 3 次形成历史（v1 → v2 → v3，backups 里有 2 个 = v1, v2）
  for (const char* tag : {"v1", "v2", "v3"}) {
    AtomicDirSwap swap(target, 5);
    std::string staging = swap.BeginStage();
    WriteFakeMapToStaging(staging, tag);
    EXPECT(swap.CommitStage({"map.pbstream", "map.png", "map.yaml"}), std::string("commit ") + tag);
    EXPECT(swap.ActivateAndCleanup(), std::string("activate ") + tag);
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  }

  auto before = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(before.size() == 2, "before rollback: 2 backups (v1, v2)");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-v3", "current = v3");

  // 记录 backups 目录的完整快照（用于验证"完全不动"）
  const std::string chosen = before[1];  // v1（最旧；before 降序 → [0]=v2, [1]=v1）
  EXPECT(ReadFile(chosen + "/map.pbstream") == "pbstream-content-v1", "chosen is v1");

  // rollback：纯三件套替换
  AtomicDirSwap swap_rb(target, 5);
  EXPECT(swap_rb.ReplaceCurrentMapFrom(chosen), "ReplaceCurrentMapFrom succeeds");

  // 验证 rollback 语义：
  // 1. target 三件套全部变成 v1
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-v1", "target pbstream = v1");
  EXPECT(ReadFile(target + "/map.png") == "png-content-v1", "target png = v1");
  EXPECT(ReadFile(target + "/map.yaml") == "yaml-content-v1", "target yaml = v1");
  // 2. backups 数量完全不变（不新增、不删除）
  auto after = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(after.size() == 2, "after rollback: still exactly 2 backups (untouched)");
  // 3. backups 内容完全不变（v2, v1 仍在）
  EXPECT(ReadFile(after[0] + "/map.pbstream") == "pbstream-content-v2", "backup[0] still v2");
  EXPECT(ReadFile(after[1] + "/map.pbstream") == "pbstream-content-v1", "backup[1] still v1");
  // 4. chosen 仍在（copy 不消耗，可重复 rollback）
  EXPECT(fs::exists(chosen), "chosen preserved");
  // 5. 无 swap_backup / .rbtmp / marker 残留
  EXPECT(!fs::exists(target + ".swap_backup"), "no swap_backup leftover");
  EXPECT(!fs::exists(target + ".rollback_marker"), "no marker leftover");
  std::error_code ec;
  for (auto const& e : fs::directory_iterator(target, ec)) {
    const std::string fn = e.path().filename().string();
    EXPECT(fn.find(".rbtmp") == std::string::npos, "no .rbtmp leftover: " + fn);
  }

  // 6. 可重复 rollback 到同一 chosen（验证 copy 语义）
  EXPECT(swap_rb.ReplaceCurrentMapFrom(chosen), "second rollback to same chosen succeeds");
  EXPECT(ReadFile(target + "/map.pbstream") == "pbstream-content-v1", "still v1 after 2nd rollback");
  auto after2 = AtomicDirSwap::ListHistoricalBackups(target);
  EXPECT(after2.size() == 2, "still 2 backups after 2nd rollback");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_minloglevel = 1;  // 隐藏 INFO，只看 WARNING/ERROR/FATAL

  std::cout << "═══ AtomicDirSwap 注入测试套件 ═══\n";

  RUN_TEST(TestBasicFlow);
  RUN_TEST(TestRecoverFromStep1Interrupt);
  RUN_TEST(TestRecoverFromStep2Done);
  RUN_TEST(TestRecoverFromStep3Done);
  RUN_TEST(TestRecoverFromStep4Done);
  RUN_TEST(TestTombstoneCleanup);
  RUN_TEST(TestRotation);
  RUN_TEST(TestEmptyState);
  RUN_TEST(TestOnlySwapBackup);
  RUN_TEST(TestZeroBackups);
  RUN_TEST(ForkInterruptAfterStaging);
  RUN_TEST(ForkInterruptAfterCommit);
  RUN_TEST(TestChecksumMismatch);
  RUN_TEST(TestChecksumDisabled);
  RUN_TEST(TestRollbackMarkerRedo);
  RUN_TEST(TestRollbackMarkerChosenGone);
  RUN_TEST(TestSelfContainedBackups);
  RUN_TEST(TestReplaceCurrentMapFrom);

  std::cout << "\n═══ 结果：" << g_pass << " pass / " << g_fail << " fail ═══\n";
  return g_fail == 0 ? 0 : 1;
}
