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
 * @brief 目录级原子替换工具，带历史 backup 轮换。
 *
 *   流程（6 步）：
 *     1. BeginStage()          → 准备 <target>.staging/
 *     2. (调用方写文件 + fsync)
 *     3. CommitStage(files)    → 写 sentinel + fsync
 *     4. ActivateAndCleanup()  → 内含 step 3-5b：
 *        a. rename(target, swap_backup)
 *        b. rename(staging, target)
 *        c. rename(swap_backup, backup.<ts>)       ← 归档
 *        d. 轮换：扫描 backup.*，超过 max_backups 个的
 *                 rename→tombstone→rm
 *
 *   启动期：
 *     AtomicDirSwap::RecoverIfInterrupted(target_dir);
 *     按决策表清残缺 staging / 续做 swap / 清 tombstone；不动 backup.<ts>
 *
 *   命名空间区分：
 *     - <target>.staging/         瞬态：step 1-2 中存在
 *     - <target>.swap_backup/     瞬态：step 3-5a 中存在
 *     - <target>.backup.<ts>/     历史：max_backups 个保留版本
 *     - <target>.tombstone.<idx>/ 瞬态：step 5b 删除中存在
 *
 *   所有目录必须位于同一文件系统（POSIX rename 同卷原子）。
 */
class AtomicDirSwap {
 public:
  // sentinel 文件名
  static constexpr const char* kSentinelName = ".complete";

  // target_dir       绝对路径，如 "/data/maps/dex_map_test"
  // max_backups      保留多少个历史 backup（含本次刚归档的）；0 = 不保留；负数视为 0
  // enable_checksum  是否在 sentinel 中写 SHA256（v1.2，默认开启）；启动恢复时若 sentinel 含
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

  // Step 3-5b：
  //   3. rename(target, swap_backup)
  //   4. rename(staging, target)
  //   5a. rename(swap_backup, backup.<ts>)
  //   5b. 轮换历史 backup，保留最近 max_backups 个
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

  // 路径工具
  std::string StagingPath() const { return target_dir_ + ".staging"; }
  std::string SwapBackupPath() const { return target_dir_ + ".swap_backup"; }
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
