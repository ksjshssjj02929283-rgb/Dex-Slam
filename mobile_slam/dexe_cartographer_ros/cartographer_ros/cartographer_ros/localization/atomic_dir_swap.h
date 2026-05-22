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
 * @brief 目录级原子替换工具。
 *
 *   提供"先写 staging → 写 sentinel → 同卷 rename 替换 target → 清 backup"的
 *   崩溃安全工作流。staging / backup / target 必须位于同一文件系统
 *   （依赖 POSIX rename(2) 的同卷原子性）。
 *
 *   典型用法：
 *     AtomicDirSwap swap("/path/to/map_dir");
 *     std::string staging = swap.BeginStage();
 *     // 把若干文件写入 staging/<name>，每个文件 close 前 fsync
 *     swap.CommitStage({"map.pbstream", "map.png", "map.yaml"});
 *     swap.ActivateAndCleanup();
 *
 *   启动期：
 *     AtomicDirSwap::RecoverIfInterrupted("/path/to/map_dir");
 *   会扫描磁盘状态（target / target.staging / target.backup 三者的存在与
 *   sentinel 完整性），根据决策表自动完成中断的切换或清理残留。
 *
 *   决策表（8 种磁盘状态全覆盖）：见 atomic_dir_swap.cc 内文档。
 */
class AtomicDirSwap {
 public:
  // sentinel 文件名（位于 staging / target 目录根下）
  static constexpr const char* kSentinelName = ".complete";

  explicit AtomicDirSwap(std::string target_dir);

  // 准备 staging 目录（先清掉旧 staging，再创建空目录）。
  // 返回 staging 绝对路径供调用方逐文件写入；失败返回空字符串。
  std::string BeginStage();

  // 验证 expected_files 全部存在于 staging，fsync 每个文件，写 sentinel，
  // fsync sentinel + staging 目录。
  // expected_files 是文件名（不含路径），如 {"map.pbstream", "map.png", "map.yaml"}。
  bool CommitStage(const std::vector<std::string>& expected_files);

  // 执行 swap：
  //   1. rename(target, backup)（同卷原子）
  //   2. rename(staging, target)（同卷原子）
  //   3. rm -rf backup
  // 任一 rename 失败会尝试紧急回滚到 target。
  bool ActivateAndCleanup();

  // 静态启动恢复：扫描 target_dir / target_dir.staging / target_dir.backup
  // 的磁盘状态，按决策表完成中断的步骤或清理残留。
  // 返回 true 表示 target_dir 现在是有效可加载的状态。
  static bool RecoverIfInterrupted(const std::string& target_dir);

  // 路径工具
  std::string StagingPath() const { return target_dir_ + ".staging"; }
  std::string BackupPath() const { return target_dir_ + ".backup"; }
  const std::string& TargetPath() const { return target_dir_; }

 private:
  std::string target_dir_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LOCALIZATION_ATOMIC_DIR_SWAP_H_
