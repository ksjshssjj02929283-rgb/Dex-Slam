/*
 * Copyright 2024 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef CARTOGRAPHER_ROS_LOCALIZATION_ATOMIC_DIR_SWAP_H_
#define CARTOGRAPHER_ROS_LOCALIZATION_ATOMIC_DIR_SWAP_H_

#include <string>
#include <vector>

namespace cartographer_ros {

/**
 * @brief 地图持久化的崩溃安全工具：原子保存 + 历史 backup + rollback。
 *
 *   磁盘布局（target = <map_folder>/<map_name>）：
 *     <target>/                       当前生效地图（三件套 + sentinel；cartographer 直接加载）
 *       ├─ map.pbstream / map.png / map.yaml / .complete
 *       └─ backups/                   历史 backup 容器（自包含在 target 内）
 *          ├─ backup.<YYYYMMDD-HHMMSS>/   max_backups 个保留版本
 *          └─ tombstone.<idx>/            轮换删除瞬态
 *     <target>.staging/               写入瞬态（保存中）
 *     <target>.swap_backup/           切换瞬态（保存中）
 *     <target>.rollback_marker        rollback 崩溃恢复标志（rollback 中）
 *
 *   save_map 流程（崩溃安全，目录级原子 rename）：
 *     1. BeginStage()          → 准备 <target>.staging/
 *     2. (调用方写三件套 + fsync)
 *     3. CommitStage(files)    → 算 SHA256 + 写 sentinel + fsync
 *     4. ActivateAndCleanup()  → 切换 + 归档旧 target 到 backups/ + 轮换
 *
 *   rollback 流程（纯三件套替换，完全不碰 backups/）：
 *     ReplaceCurrentMapFrom(<target>/backups/backup.<ts>)
 *       → 只替换 <target> 下的三件套；backups/ 目录零接触
 *
 *   启动恢复：
 *     RecoverIfInterrupted(target_dir)  → 清残缺 staging / 续做 swap / 清 tombstone
 *                                          + 若有 rollback_marker 则重做三件套替换
 *
 *   所有目录必须位于同一文件系统（POSIX rename 同卷原子）。
 */
class AtomicDirSwap {
 public:
  // sentinel 文件名
  static constexpr const char* kSentinelName = ".complete";

  // target_dir       绝对路径，如 "/data/maps/dex_map_test"
  // max_backups      保留多少个历史 backup（含本次刚归档的）；0 = 不保留；负数视为 0
  // enable_checksum  是否在 sentinel 中写 SHA256（默认开启）；启动恢复时若 sentinel 含
  //                  sha256_<file>=<hex>，会自动校验文件完整性
  explicit AtomicDirSwap(std::string target_dir, int max_backups = 5,
                         bool enable_checksum = true);

  // Step 1: 清残留 staging + 创建空 staging 目录。失败返回空字符串。
  std::string BeginStage();

  // Step 2: 验证 expected_files 全部存在于 staging、逐个 fsync、写 sentinel、
  //         fsync sentinel + staging 目录。
  // expected_files 是文件名（不含路径），如 {"map.pbstream", "map.png", "map.yaml"}。
  // 若构造时 enable_checksum=true，sentinel 中会追加每个文件的 SHA256（sha256_<file>=<hex>）。
  bool CommitStage(const std::vector<std::string>& expected_files);

  // Step 3-5c（save_map 路径）：
  //   3. rename(target, swap_backup)            旧 target 整体移走（含 backups 子目录）
  //   4. rename(staging, target)                新三件套激活
  //   5a. 抽出 swap_backup/backups → target/backups   保留历史
  //   5b. rename(swap_backup, target/backups/backup.<ts>)  归档旧 target
  //   5c. 轮换历史 backup，保留最近 max_backups 个
  bool ActivateAndCleanup();

  // 启动恢复（静态）：扫描磁盘状态，按决策表完成切换或清残留。
  // 不动 backup.<ts>；返回 true 表示 target_dir 现在有效可加载。
  // 状态 3/4 中若 staging sentinel 含 sha256_*，会先调 ValidateSentinel 校验；
  // 不匹配 → 清掉 staging 当作残缺处理，加载旧 target。
  static bool RecoverIfInterrupted(const std::string& target_dir);

  // 列出 target_dir.backup.* 历史版本，按时间戳降序（最新在前）。
  static std::vector<std::string> ListHistoricalBackups(
      const std::string& target_dir);

  // v1.2: 校验 dir/.complete 中所有 sha256_<file>=<hex> 字段对应文件是否匹配。
  //   - sentinel 不含任何 sha256_* 字段（旧版本）→ 视为兼容 → 返回 true
  //   - 文件计算 SHA256 与 sentinel 不匹配 → LOG(WARNING) 列出文件名 + 返回 false
  //   - sentinel / 任一文件读不到 → 返回 false
  static bool ValidateSentinel(const std::string& dir);

  // v3.1 rollback 核心：用指定 backup 的三件套替换当前 target 三件套。
  //   **完全不碰 <target>/backups/ 目录**（不新增、不删除、不移动 backup）。
  //   - source_backup_dir：要回退到的 backup 目录（如 <target>/backups/backup.<ts>）
  //   - 崩溃安全：先写 marker，再"全部 cp 到 .rbtmp + fsync → 连续 rename 替换"；
  //     崩溃后启动 RecoverRollbackIfInterrupted 从 source 重做（幂等，source 始终完整保留）
  //   - 仅替换 map.pbstream / map.png / map.yaml / .complete 四个文件
  bool ReplaceCurrentMapFrom(const std::string& source_backup_dir);

  // rollback marker（v3.1 语义：记录 chosen backup 路径供崩溃重做）
  //   marker 文件 = <target_dir>.rollback_marker，内容 key=value:
  //     chosen_backup=<source backup 完整路径>
  //     started_unix=<unix timestamp>
  static bool WriteRollbackMarker(const std::string& target_dir,
                                  const std::string& chosen_backup);
  static bool RemoveRollbackMarker(const std::string& target_dir);
  // 启动期：若 marker 存在 → 从 chosen 重做三件套替换（幂等）；清理 .rbtmp 残留。
  // 返回 true 表示已完成恢复或无需恢复。
  static bool RecoverRollbackIfInterrupted(const std::string& target_dir);

  static constexpr const char* kRollbackMarkerSuffix = ".rollback_marker";

  // 路径工具
  std::string StagingPath() const { return target_dir_ + ".staging"; }
  std::string SwapBackupPath() const { return target_dir_ + ".swap_backup"; }
  // 历史 backup 容器（在 target 内部，自包含）：<target>/backups/
  std::string BackupsContainerPath() const { return target_dir_ + "/backups"; }
  const std::string& TargetPath() const { return target_dir_; }
  int MaxBackups() const { return max_backups_; }
  bool ChecksumEnabled() const { return enable_checksum_; }

 private:
  std::string target_dir_;
  int max_backups_;
  bool enable_checksum_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LOCALIZATION_ATOMIC_DIR_SWAP_H_
