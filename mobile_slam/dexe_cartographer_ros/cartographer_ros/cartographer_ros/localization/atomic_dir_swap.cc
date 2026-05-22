/*
 * Copyright 2024 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

#include "cartographer_ros/localization/atomic_dir_swap.h"

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

#include <openssl/evp.h>

#include "glog/logging.h"

namespace cartographer_ros {
namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// 内部工具
// ---------------------------------------------------------------------------
namespace {

// fsync 一个文件 / 目录路径
bool FsyncPath(const std::string& path) {
  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG(WARNING) << "[AtomicDirSwap] open for fsync failed: " << path
                 << " errno=" << errno << " (" << std::strerror(errno) << ")";
    return false;
  }
  bool ok = (::fsync(fd) == 0);
  if (!ok) {
    LOG(WARNING) << "[AtomicDirSwap] fsync failed: " << path
                 << " errno=" << errno << " (" << std::strerror(errno) << ")";
  }
  ::close(fd);
  return ok;
}

bool RemoveDirIfExists(const std::string& path) {
  std::error_code ec;
  if (!fs::exists(path, ec)) return true;
  fs::remove_all(path, ec);
  if (ec) {
    LOG(WARNING) << "[AtomicDirSwap] remove_all(" << path
                 << ") failed: " << ec.message();
    return false;
  }
  return true;
}

bool CreateDir(const std::string& path) {
  std::error_code ec;
  fs::create_directories(path, ec);
  if (ec) {
    LOG(ERROR) << "[AtomicDirSwap] create_directories(" << path
               << ") failed: " << ec.message();
    return false;
  }
  return true;
}

// 把 unix 时间戳转换为 "YYYYMMDD-HHMMSS"（本地时间）
std::string FormatBackupTimestamp(std::time_t t) {
  struct tm tm_buf;
  ::localtime_r(&t, &tm_buf);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tm_buf);
  return std::string(buf);
}

// 把 unix 时间戳转换为 "YYYY-MM-DDTHH:MM:SS"
std::string FormatIsoTimestamp(std::time_t t) {
  struct tm tm_buf;
  ::localtime_r(&t, &tm_buf);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_buf);
  return std::string(buf);
}

// 从 sentinel 中读取 timestamp_unix 字段；失败返回当前时间作兜底
std::time_t ReadSentinelTimestamp(const std::string& sentinel_path) {
  std::ifstream ifs(sentinel_path);
  if (!ifs) {
    LOG(WARNING) << "[AtomicDirSwap] cannot read sentinel: " << sentinel_path;
    return std::time(nullptr);
  }
  std::string line;
  while (std::getline(ifs, line)) {
    const std::string key = "timestamp_unix=";
    if (line.rfind(key, 0) == 0) {
      try {
        return static_cast<std::time_t>(std::stoll(line.substr(key.size())));
      } catch (...) {
        break;
      }
    }
  }
  LOG(WARNING) << "[AtomicDirSwap] timestamp_unix missing in: " << sentinel_path;
  return std::time(nullptr);
}

// 列出所有 <target>.backup.* 目录（按字典序，等价时间序）
std::vector<fs::path> ListBackupPaths(const std::string& target_dir) {
  std::vector<fs::path> result;
  fs::path parent = fs::path(target_dir).parent_path();
  std::string name_prefix = fs::path(target_dir).filename().string() + ".backup.";
  if (parent.empty()) parent = ".";
  std::error_code ec;
  if (!fs::exists(parent, ec)) return result;
  for (auto const& entry : fs::directory_iterator(parent, ec)) {
    if (ec) break;
    if (!entry.is_directory()) continue;
    const std::string fname = entry.path().filename().string();
    if (fname.rfind(name_prefix, 0) == 0) {
      result.push_back(entry.path());
    }
  }
  std::sort(result.begin(), result.end());  // 字典序升序 = 时间升序
  return result;
}

// 列出所有 <target>.tombstone.* 目录
std::vector<fs::path> ListTombstonePaths(const std::string& target_dir) {
  std::vector<fs::path> result;
  fs::path parent = fs::path(target_dir).parent_path();
  std::string name_prefix = fs::path(target_dir).filename().string() + ".tombstone.";
  if (parent.empty()) parent = ".";
  std::error_code ec;
  if (!fs::exists(parent, ec)) return result;
  for (auto const& entry : fs::directory_iterator(parent, ec)) {
    if (ec) break;
    if (!entry.is_directory()) continue;
    const std::string fname = entry.path().filename().string();
    if (fname.rfind(name_prefix, 0) == 0) {
      result.push_back(entry.path());
    }
  }
  return result;
}

// 找一个未占用的 tombstone 索引
std::string NextTombstonePath(const std::string& target_dir) {
  for (int i = 0; i < 10000; ++i) {
    std::string path = target_dir + ".tombstone." + std::to_string(i);
    if (!fs::exists(path)) return path;
  }
  // 极端兜底：用 pid + 时间戳
  return target_dir + ".tombstone." + std::to_string(::getpid()) + "_" +
         std::to_string(std::time(nullptr));
}

// 删除所有 tombstone（启动恢复必做）
void PurgeTombstones(const std::string& target_dir) {
  auto tombs = ListTombstonePaths(target_dir);
  for (const auto& t : tombs) {
    LOG(WARNING) << "[AtomicDirSwap] removing leftover tombstone: " << t.string();
    RemoveDirIfExists(t.string());
  }
}

// v1.2: 用 OpenSSL EVP API 计算文件 SHA256，返回 64 字符小写 hex；失败返回空串。
// 用流式 8KB 缓冲读，避免大 pbstream 一次性占内存。
std::string ComputeFileSha256(const std::string& path) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    LOG(WARNING) << "[AtomicDirSwap] sha256: cannot open file: " << path;
    return "";
  }
  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  if (ctx == nullptr) {
    LOG(ERROR) << "[AtomicDirSwap] sha256: EVP_MD_CTX_new failed";
    return "";
  }
  if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1) {
    LOG(ERROR) << "[AtomicDirSwap] sha256: EVP_DigestInit_ex failed";
    EVP_MD_CTX_free(ctx);
    return "";
  }
  constexpr size_t kBuf = 8192;
  char buf[kBuf];
  while (ifs.good()) {
    ifs.read(buf, kBuf);
    std::streamsize got = ifs.gcount();
    if (got > 0) {
      if (EVP_DigestUpdate(ctx, buf, static_cast<size_t>(got)) != 1) {
        LOG(ERROR) << "[AtomicDirSwap] sha256: EVP_DigestUpdate failed: " << path;
        EVP_MD_CTX_free(ctx);
        return "";
      }
    }
  }
  if (ifs.bad()) {
    LOG(WARNING) << "[AtomicDirSwap] sha256: read error on " << path;
    EVP_MD_CTX_free(ctx);
    return "";
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_len = 0;
  if (EVP_DigestFinal_ex(ctx, digest, &digest_len) != 1) {
    LOG(ERROR) << "[AtomicDirSwap] sha256: EVP_DigestFinal_ex failed";
    EVP_MD_CTX_free(ctx);
    return "";
  }
  EVP_MD_CTX_free(ctx);
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.resize(digest_len * 2);
  for (unsigned int i = 0; i < digest_len; ++i) {
    out[i * 2]     = kHex[(digest[i] >> 4) & 0x0F];
    out[i * 2 + 1] = kHex[digest[i] & 0x0F];
  }
  return out;
}

// 解析 dir/.complete 中所有 "sha256_<file>=<hex>" 行；返回 file -> hex 的 map。
// 不存在 / 解析失败时返回空 map（调用方需另判 sentinel 是否存在）。
std::map<std::string, std::string> ReadSentinelChecksums(
    const std::string& sentinel_path) {
  std::map<std::string, std::string> result;
  std::ifstream ifs(sentinel_path);
  if (!ifs) return result;
  std::string line;
  const std::string prefix = "sha256_";
  while (std::getline(ifs, line)) {
    if (line.rfind(prefix, 0) != 0) continue;
    auto eq = line.find('=');
    if (eq == std::string::npos || eq <= prefix.size()) continue;
    std::string fname = line.substr(prefix.size(), eq - prefix.size());
    std::string hex = line.substr(eq + 1);
    if (!fname.empty() && !hex.empty()) {
      result[fname] = hex;
    }
  }
  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// AtomicDirSwap public methods
// ---------------------------------------------------------------------------

AtomicDirSwap::AtomicDirSwap(std::string target_dir, int max_backups,
                             bool enable_checksum)
    : target_dir_(std::move(target_dir)),
      max_backups_(max_backups < 0 ? 0 : max_backups),
      enable_checksum_(enable_checksum) {}

std::string AtomicDirSwap::BeginStage() {
  const std::string staging = StagingPath();
  // 清掉残留 staging（可能是上次中断留下的）
  if (!RemoveDirIfExists(staging)) {
    LOG(ERROR) << "[AtomicDirSwap] BeginStage: failed to remove old staging: "
               << staging;
    return "";
  }
  if (!CreateDir(staging)) {
    LOG(ERROR) << "[AtomicDirSwap] BeginStage: failed to create staging: "
               << staging;
    return "";
  }
  LOG(INFO) << "[AtomicDirSwap] BeginStage: staging=" << staging;
  return staging;
}

bool AtomicDirSwap::CommitStage(
    const std::vector<std::string>& expected_files) {
  const std::string staging = StagingPath();
  if (!fs::exists(staging)) {
    LOG(ERROR) << "[AtomicDirSwap] CommitStage: staging missing: " << staging;
    return false;
  }
  // 1. 验证 + fsync 每个文件
  for (const auto& f : expected_files) {
    const std::string full = staging + "/" + f;
    std::error_code ec;
    if (!fs::exists(full, ec) || fs::file_size(full, ec) == 0) {
      LOG(ERROR) << "[AtomicDirSwap] CommitStage: expected file missing or empty: "
                 << full;
      return false;
    }
    if (!FsyncPath(full)) {
      LOG(ERROR) << "[AtomicDirSwap] CommitStage: fsync failed: " << full;
      return false;
    }
  }
  // 1.5 v1.2: 计算 SHA256（fsync 之后、写 sentinel 之前；保证读到的是落盘内容）
  std::vector<std::pair<std::string, std::string>> checksums;
  if (enable_checksum_) {
    checksums.reserve(expected_files.size());
    for (const auto& f : expected_files) {
      const std::string full = staging + "/" + f;
      std::string hex = ComputeFileSha256(full);
      if (hex.empty()) {
        LOG(ERROR) << "[AtomicDirSwap] CommitStage: sha256 failed for: " << full;
        return false;
      }
      checksums.emplace_back(f, std::move(hex));
    }
  }
  // 2. 写 sentinel
  const std::string sentinel = staging + "/" + kSentinelName;
  const std::time_t now_unix = std::time(nullptr);
  {
    std::ofstream ofs(sentinel);
    if (!ofs) {
      LOG(ERROR) << "[AtomicDirSwap] CommitStage: cannot open sentinel: "
                 << sentinel;
      return false;
    }
    ofs << "version=1\n";
    ofs << "timestamp=" << FormatIsoTimestamp(now_unix) << "\n";
    ofs << "timestamp_unix=" << now_unix << "\n";
    ofs << "saved_by=cartographer_ros::AtomicDirSwap\n";
    ofs << "files=";
    for (size_t i = 0; i < expected_files.size(); ++i) {
      if (i) ofs << ",";
      ofs << expected_files[i];
    }
    ofs << "\n";
    // v1.2: 写 sha256_<file>=<hex> 行（仅 enable_checksum=true 时）
    for (const auto& kv : checksums) {
      ofs << "sha256_" << kv.first << "=" << kv.second << "\n";
    }
    ofs.flush();
    if (!ofs) {
      LOG(ERROR) << "[AtomicDirSwap] CommitStage: write sentinel failed";
      return false;
    }
  }
  // 3. fsync sentinel + staging 目录
  if (!FsyncPath(sentinel)) return false;
  if (!FsyncPath(staging)) return false;
  LOG(INFO) << "[AtomicDirSwap] CommitStage: " << expected_files.size()
            << " files committed"
            << (enable_checksum_ ? " (checksum)" : "")
            << ", sentinel ts=" << now_unix;
  return true;
}

bool AtomicDirSwap::ActivateAndCleanup() {
  const std::string staging = StagingPath();
  const std::string swap_backup = SwapBackupPath();
  const std::string target = target_dir_;
  const std::string sentinel = staging + "/" + kSentinelName;

  // 0. 前置检查：staging 必须已 commit
  if (!fs::exists(sentinel)) {
    LOG(ERROR) << "[AtomicDirSwap] ActivateAndCleanup: staging not committed (no sentinel): "
               << sentinel;
    return false;
  }
  // 读 timestamp 用于命名历史 backup
  const std::time_t commit_unix = ReadSentinelTimestamp(sentinel);
  const std::string ts_str = FormatBackupTimestamp(commit_unix);
  const std::string history_backup = target + ".backup." + ts_str;
  const std::string parent = fs::path(target).parent_path().string();

  std::error_code ec;

  // 防御：如果上次因为某些原因 swap_backup 还残留，决策表应已清掉；
  // 兜底再清一次（避免 rename(target, swap_backup) 因 swap_backup 已存在而失败）
  if (fs::exists(swap_backup)) {
    LOG(WARNING) << "[AtomicDirSwap] ActivateAndCleanup: leftover swap_backup, removing: "
                 << swap_backup;
    if (!RemoveDirIfExists(swap_backup)) return false;
  }
  // 兜底：history_backup 同名冲突（同秒两次保存）→ 退一秒重试
  std::string final_history = history_backup;
  for (int retry = 0; retry < 5 && fs::exists(final_history); ++retry) {
    final_history = target + ".backup." + ts_str + "-" + std::to_string(retry + 1);
  }

  // step 3: rename target → swap_backup（同卷原子）
  if (fs::exists(target)) {
    fs::rename(target, swap_backup, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] step 3 rename(target -> swap_backup) failed: "
                 << ec.message();
      return false;
    }
  } else {
    LOG(WARNING) << "[AtomicDirSwap] step 3: target does not exist, first-time save?";
  }

  // step 4: rename staging → target（同卷原子）
  fs::rename(staging, target, ec);
  if (ec) {
    LOG(ERROR) << "[AtomicDirSwap] step 4 rename(staging -> target) failed: "
               << ec.message();
    // 紧急回滚：把 swap_backup 改回 target
    if (fs::exists(swap_backup)) {
      std::error_code ec2;
      fs::rename(swap_backup, target, ec2);
      if (ec2) {
        LOG(FATAL) << "[AtomicDirSwap] EMERGENCY: rollback failed: " << ec2.message();
      }
    }
    return false;
  }
  FsyncPath(parent);

  // step 5a: rename swap_backup → backup.<ts>（同卷原子；归档到历史）
  if (fs::exists(swap_backup)) {
    fs::rename(swap_backup, final_history, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] step 5a rename(swap_backup -> backup.<ts>) failed: "
                 << ec.message() << "; new target is still valid";
      // step 4 已成功，target 有效；返回 true 但记录错误
      // 下次启动恢复表会识别 swap_backup 残留并续做
      return true;
    }
    FsyncPath(parent);
    LOG(INFO) << "[AtomicDirSwap] step 5a: archived swap_backup -> "
              << final_history;
  }

  // step 5b: 轮换历史 backup，保留最近 max_backups 个
  if (max_backups_ >= 0) {  // 0 也合法（不保留任何历史）
    auto backups = ListBackupPaths(target_dir_);
    // 按字典序升序 = 时间升序；最旧在前
    const int total = static_cast<int>(backups.size());
    const int to_remove = total - max_backups_;
    if (to_remove > 0) {
      LOG(INFO) << "[AtomicDirSwap] step 5b: " << total << " backups, max=" << max_backups_
                << ", rotating out " << to_remove;
      for (int i = 0; i < to_remove; ++i) {
        const std::string tombstone = NextTombstonePath(target_dir_);
        std::error_code ec_rn;
        fs::rename(backups[i], tombstone, ec_rn);
        if (ec_rn) {
          LOG(WARNING) << "[AtomicDirSwap] step 5b rename to tombstone failed: "
                       << backups[i].string() << " -> " << tombstone
                       << ": " << ec_rn.message();
          continue;
        }
        RemoveDirIfExists(tombstone);
      }
    }
  }

  LOG(INFO) << "[AtomicDirSwap] ActivateAndCleanup completed, target activated, "
            << "history archived as " << final_history;
  return true;
}

bool AtomicDirSwap::ValidateSentinel(const std::string& dir) {
  const std::string sentinel = dir + "/" + kSentinelName;
  if (!fs::exists(sentinel)) {
    LOG(WARNING) << "[AtomicDirSwap] ValidateSentinel: sentinel missing: "
                 << sentinel;
    return false;
  }
  auto expected = ReadSentinelChecksums(sentinel);
  if (expected.empty()) {
    // 旧版本 sentinel（或 enable_checksum=false 写入的）无 sha256_ 字段 → 兼容
    LOG(INFO) << "[AtomicDirSwap] ValidateSentinel: no sha256_* fields, "
              << "skip checksum (legacy / disabled): " << dir;
    return true;
  }
  bool all_ok = true;
  int mismatch = 0;
  for (const auto& kv : expected) {
    const std::string& fname = kv.first;
    const std::string& want = kv.second;
    const std::string full = dir + "/" + fname;
    if (!fs::exists(full)) {
      LOG(WARNING) << "[AtomicDirSwap] ValidateSentinel: file missing: " << full;
      all_ok = false;
      ++mismatch;
      continue;
    }
    std::string got = ComputeFileSha256(full);
    if (got.empty()) {
      LOG(WARNING) << "[AtomicDirSwap] ValidateSentinel: cannot hash: " << full;
      all_ok = false;
      ++mismatch;
      continue;
    }
    if (got != want) {
      LOG(WARNING) << "[AtomicDirSwap] ValidateSentinel: sha256 mismatch on "
                   << fname << " (expected=" << want.substr(0, 12)
                   << "... got=" << got.substr(0, 12) << "...)";
      all_ok = false;
      ++mismatch;
    }
  }
  if (all_ok) {
    LOG(INFO) << "[AtomicDirSwap] ValidateSentinel: " << expected.size()
              << " file(s) sha256 matched: " << dir;
  } else {
    LOG(WARNING) << "[AtomicDirSwap] ValidateSentinel: " << mismatch << "/"
                 << expected.size() << " file(s) failed in " << dir;
  }
  return all_ok;
}

bool AtomicDirSwap::RecoverIfInterrupted(const std::string& target_dir) {
  const std::string staging = target_dir + ".staging";
  const std::string swap_backup = target_dir + ".swap_backup";
  const std::string sentinel = staging + "/" + kSentinelName;

  // 第一件事：清掉所有 tombstone 残留
  PurgeTombstones(target_dir);

  const bool has_target = fs::exists(target_dir);
  const bool has_staging = fs::exists(staging);
  const bool has_swap_backup = fs::exists(swap_backup);
  const bool has_staging_sentinel = fs::exists(sentinel);

  LOG(INFO) << "[AtomicDirSwap] RecoverIfInterrupted: target=" << target_dir
            << " has_target=" << has_target
            << " has_staging=" << has_staging
            << " has_staging_sentinel=" << has_staging_sentinel
            << " has_swap_backup=" << has_swap_backup;

  std::error_code ec;
  const std::string parent = fs::path(target_dir).parent_path().string();

  // 状态 1: 仅 target 存在 → 正常
  if (has_target && !has_staging && !has_swap_backup) {
    LOG(INFO) << "[AtomicDirSwap] state: normal, target ready";
    return true;
  }

  // 状态 2: target + staging（无 sentinel） → step 1 中断 → 清 staging
  if (has_target && has_staging && !has_staging_sentinel && !has_swap_backup) {
    LOG(WARNING) << "[AtomicDirSwap] state: step 1 interrupted, removing staging";
    RemoveDirIfExists(staging);
    return true;
  }

  // 状态 3: target + staging+sentinel → step 2 完成、step 3 未开始 → 续做 3-5a
  if (has_target && has_staging && has_staging_sentinel && !has_swap_backup) {
    // v1.2: 先校验 staging 完整性（若 sentinel 含 sha256_）
    if (!ValidateSentinel(staging)) {
      LOG(WARNING) << "[AtomicDirSwap] state 3: staging checksum failed, "
                   << "treating as incomplete; removing staging, keeping old target";
      RemoveDirIfExists(staging);
      return true;  // 旧 target 仍然有效
    }
    LOG(WARNING) << "[AtomicDirSwap] state: step 2 done, completing activation";
    // 不传 max_backups（用默认 5）；如有特殊需求外部应在 Recover 之后立即触发一次正常保存
    // 这里只完成 step 3-5a（不做 5b 轮换），下次正常保存时再轮换
    const std::time_t commit_unix = ReadSentinelTimestamp(sentinel);
    const std::string ts_str = FormatBackupTimestamp(commit_unix);
    std::string history_backup = target_dir + ".backup." + ts_str;
    for (int r = 0; r < 5 && fs::exists(history_backup); ++r) {
      history_backup = target_dir + ".backup." + ts_str + "-" + std::to_string(r + 1);
    }
    // 3: target → swap_backup
    fs::rename(target_dir, swap_backup, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] recover step 3 failed: " << ec.message();
      return false;
    }
    // 4: staging → target
    fs::rename(staging, target_dir, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] recover step 4 failed: " << ec.message();
      // 紧急回滚
      std::error_code ec2;
      fs::rename(swap_backup, target_dir, ec2);
      return false;
    }
    FsyncPath(parent);
    // 5a: swap_backup → backup.<ts>
    fs::rename(swap_backup, history_backup, ec);
    if (ec) {
      LOG(WARNING) << "[AtomicDirSwap] recover step 5a failed (target still valid): "
                   << ec.message();
    } else {
      FsyncPath(parent);
    }
    return true;
  }

  // 状态 4: swap_backup + staging+sentinel（无 target）→ step 3 完成、step 4 未开始
  if (!has_target && has_staging && has_staging_sentinel && has_swap_backup) {
    // v1.2: 先校验 staging 完整性。失败时：staging 不可信、target 不在，
    // 必须把 swap_backup 还原为 target 才能继续提供服务。
    if (!ValidateSentinel(staging)) {
      LOG(WARNING) << "[AtomicDirSwap] state 4: staging checksum failed, "
                   << "rolling back: drop staging, restore swap_backup as target";
      RemoveDirIfExists(staging);
      std::error_code ec_rb;
      fs::rename(swap_backup, target_dir, ec_rb);
      if (ec_rb) {
        LOG(ERROR) << "[AtomicDirSwap] state 4 rollback rename failed: "
                   << ec_rb.message();
        return false;
      }
      FsyncPath(parent);
      return true;
    }
    LOG(WARNING) << "[AtomicDirSwap] state: step 3 done, completing step 4-5a";
    // 4: staging → target
    fs::rename(staging, target_dir, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] recover step 4 (from state 4) failed: " << ec.message();
      // 紧急回滚 swap_backup
      std::error_code ec2;
      fs::rename(swap_backup, target_dir, ec2);
      return !ec2;
    }
    FsyncPath(parent);
    // 5a: swap_backup → backup.<ts>
    const std::time_t commit_unix = ReadSentinelTimestamp(target_dir + "/" + kSentinelName);
    const std::string ts_str = FormatBackupTimestamp(commit_unix);
    std::string history_backup = target_dir + ".backup." + ts_str;
    for (int r = 0; r < 5 && fs::exists(history_backup); ++r) {
      history_backup = target_dir + ".backup." + ts_str + "-" + std::to_string(r + 1);
    }
    std::error_code ec3;
    fs::rename(swap_backup, history_backup, ec3);
    if (ec3) {
      LOG(WARNING) << "[AtomicDirSwap] recover step 5a (from state 4) failed: " << ec3.message();
    } else {
      FsyncPath(parent);
    }
    return true;
  }

  // 状态 5: target + swap_backup（无 staging）→ step 4 完成、step 5a 未开始 → 归档 swap_backup
  if (has_target && !has_staging && has_swap_backup) {
    LOG(WARNING) << "[AtomicDirSwap] state: step 4 done, archiving swap_backup";
    const std::time_t commit_unix = ReadSentinelTimestamp(target_dir + "/" + kSentinelName);
    const std::string ts_str = FormatBackupTimestamp(commit_unix);
    std::string history_backup = target_dir + ".backup." + ts_str;
    for (int r = 0; r < 5 && fs::exists(history_backup); ++r) {
      history_backup = target_dir + ".backup." + ts_str + "-" + std::to_string(r + 1);
    }
    fs::rename(swap_backup, history_backup, ec);
    if (ec) {
      LOG(WARNING) << "[AtomicDirSwap] recover step 5a (from state 5) failed: " << ec.message();
      // target 已有效，返回 true
    } else {
      FsyncPath(parent);
    }
    return true;
  }

  // 状态 6: 仅 swap_backup → 极端：staging 已清、target 不存在 → 还原 swap_backup
  if (!has_target && !has_staging && has_swap_backup) {
    LOG(WARNING) << "[AtomicDirSwap] state: only swap_backup remains, restoring as target";
    fs::rename(swap_backup, target_dir, ec);
    return !ec;
  }

  // 状态 7: target + staging（无 sentinel）+ swap_backup → 异常组合 → 保留 target，清 staging + swap_backup
  if (has_target && has_staging && !has_staging_sentinel && has_swap_backup) {
    LOG(WARNING) << "[AtomicDirSwap] state: unusual (target+staging-incomplete+swap_backup), "
                 << "keeping target, removing extras";
    RemoveDirIfExists(staging);
    RemoveDirIfExists(swap_backup);
    return true;
  }

  // 状态 8: 全空 → 首次启动 / 无图可加载
  if (!has_target && !has_staging && !has_swap_backup) {
    LOG(WARNING) << "[AtomicDirSwap] state: empty (first start or no map available)";
    return false;
  }

  // 兜底
  LOG(WARNING) << "[AtomicDirSwap] state: unhandled combination, has_target=" << has_target;
  return has_target;
}

std::vector<std::string> AtomicDirSwap::ListHistoricalBackups(
    const std::string& target_dir) {
  auto paths = ListBackupPaths(target_dir);
  std::vector<std::string> result;
  result.reserve(paths.size());
  for (auto it = paths.rbegin(); it != paths.rend(); ++it) {  // 降序：最新在前
    result.push_back(it->string());
  }
  return result;
}

}  // namespace cartographer_ros
