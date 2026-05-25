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
#include <sys/mman.h>
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

// v3: 列出 <target>/backups/backup.* 内所有历史 backup（target 内部容器）
std::vector<fs::path> ListBackupPaths(const std::string& target_dir) {
  std::vector<fs::path> result;
  const fs::path container = target_dir + "/backups";
  std::error_code ec;
  if (!fs::exists(container, ec)) return result;
  for (auto const& entry : fs::directory_iterator(container, ec)) {
    if (ec) break;
    if (!entry.is_directory()) continue;
    const std::string fname = entry.path().filename().string();
    if (fname.rfind("backup.", 0) == 0) {
      result.push_back(entry.path());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

// v3: 列出 <target>/backups/tombstone.*
std::vector<fs::path> ListTombstonePaths(const std::string& target_dir) {
  std::vector<fs::path> result;
  const fs::path container = target_dir + "/backups";
  std::error_code ec;
  if (!fs::exists(container, ec)) return result;
  for (auto const& entry : fs::directory_iterator(container, ec)) {
    if (ec) break;
    if (!entry.is_directory()) continue;
    const std::string fname = entry.path().filename().string();
    if (fname.rfind("tombstone.", 0) == 0) {
      result.push_back(entry.path());
    }
  }
  return result;
}

// v3: tombstone 路径在 <target>/backups/ 内
std::string NextTombstonePath(const std::string& target_dir) {
  const std::string container = target_dir + "/backups";
  for (int i = 0; i < 10000; ++i) {
    std::string path = container + "/tombstone." + std::to_string(i);
    if (!fs::exists(path)) return path;
  }
  return container + "/tombstone." + std::to_string(::getpid()) + "_" +
         std::to_string(std::time(nullptr));
}

// v3 helper: 启动恢复路径中"抽出 swap_backup 内 backups 子目录 + 归档 swap_backup"的统一逻辑。
//   - 若 swap_backup/backups 存在 → rename 到 target/backups
//   - rename swap_backup → target/backups/backup.<ts>
//   - 同秒冲突追加 -retry 后缀
// 调用前 target 必须已存在；swap_backup 也必须存在；本函数容错处理同名冲突和不存在情况。
static bool ArchiveSwapBackupToTarget(const std::string& target_dir,
                                      const std::string& swap_backup,
                                      const std::string& ts_str) {
  std::error_code ec;
  if (!fs::exists(swap_backup, ec)) return true;  // 幂等：swap_backup 不存在视为已归档

  // 5a: 抽出 swap_backup 内的 backups 子目录（如果存在）到新 target
  const std::string swap_backups_subdir = swap_backup + "/backups";
  const std::string target_backups_subdir = target_dir + "/backups";
  if (fs::exists(swap_backups_subdir, ec)) {
    if (fs::exists(target_backups_subdir, ec)) {
      LOG(WARNING) << "[AtomicDirSwap] ArchiveSwapBackupToTarget: target/backups already exists; "
                   << "removing before extracting from swap_backup";
      RemoveDirIfExists(target_backups_subdir);
    }
    fs::rename(swap_backups_subdir, target_backups_subdir, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] ArchiveSwapBackupToTarget: extract backups failed: "
                 << ec.message();
      return false;
    }
    FsyncPath(target_dir);
  } else {
    // swap_backup 内无 backups（首次保存）→ 在 target 内创建空容器供 step 5b 用
    std::error_code ec_mk;
    fs::create_directories(target_backups_subdir, ec_mk);
    if (ec_mk) {
      LOG(ERROR) << "[AtomicDirSwap] ArchiveSwapBackupToTarget: cannot create target/backups: "
                 << ec_mk.message();
      return false;
    }
    FsyncPath(target_dir);
  }

  // 5b: rename swap_backup → target/backups/backup.<ts>，同秒冲突追加 retry
  std::string history_backup = target_backups_subdir + "/backup." + ts_str;
  for (int r = 0; r < 5 && fs::exists(history_backup); ++r) {
    history_backup = target_backups_subdir + "/backup." + ts_str + "-" + std::to_string(r + 1);
  }
  std::error_code ec_arc;
  fs::rename(swap_backup, history_backup, ec_arc);
  if (ec_arc) {
    LOG(WARNING) << "[AtomicDirSwap] ArchiveSwapBackupToTarget: archive failed (target still valid): "
                 << ec_arc.message();
    return false;
  }
  FsyncPath(target_backups_subdir);
  LOG(INFO) << "[AtomicDirSwap] ArchiveSwapBackupToTarget: archived to " << history_backup;
  return true;
}

// v3: 确保 <target>/backups/ 容器目录存在（幂等）
// 注意：target 本身必须先存在；本函数只负责 mkdir backups 子目录
bool EnsureBackupsContainer(const std::string& target_dir) {
  const std::string container = target_dir + "/backups";
  std::error_code ec;
  if (fs::exists(container, ec)) return true;
  if (!fs::exists(target_dir, ec)) {
    LOG(WARNING) << "[AtomicDirSwap] EnsureBackupsContainer: target_dir not exists yet: "
                 << target_dir;
    return false;
  }
  fs::create_directories(container, ec);
  if (ec) {
    LOG(ERROR) << "[AtomicDirSwap] EnsureBackupsContainer failed: "
               << container << " " << ec.message();
    return false;
  }
  // fsync target 目录（因为 backups 是其新子目录）
  FsyncPath(target_dir);
  return true;
}

// 删除所有 tombstone（启动恢复必做）
void PurgeTombstones(const std::string& target_dir) {
  auto tombs = ListTombstonePaths(target_dir);
  for (const auto& t : tombs) {
    LOG(WARNING) << "[AtomicDirSwap] removing leftover tombstone: " << t.string();
    RemoveDirIfExists(t.string());
  }
}

// 把 32 字节 SHA256 digest 转 64 hex 字符
std::string DigestToHex(const unsigned char* digest, unsigned int len) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.resize(len * 2);
  for (unsigned int i = 0; i < len; ++i) {
    out[i * 2]     = kHex[(digest[i] >> 4) & 0x0F];
    out[i * 2 + 1] = kHex[digest[i] & 0x0F];
  }
  return out;
}

// 8KB 流式 SHA256 — 作为 mmap 路径的 fallback（小文件 / mmap 失败时使用）
std::string ComputeFileSha256Stream(const std::string& path) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    LOG(WARNING) << "[AtomicDirSwap] sha256(stream): cannot open file: " << path;
    return "";
  }
  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  if (ctx == nullptr) return "";
  if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1) {
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
        EVP_MD_CTX_free(ctx);
        return "";
      }
    }
  }
  if (ifs.bad()) {
    EVP_MD_CTX_free(ctx);
    return "";
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_len = 0;
  if (EVP_DigestFinal_ex(ctx, digest, &digest_len) != 1) {
    EVP_MD_CTX_free(ctx);
    return "";
  }
  EVP_MD_CTX_free(ctx);
  return DigestToHex(digest, digest_len);
}

// v1.2 + v2: 用 OpenSSL EVP API 计算文件 SHA256，返回 64 字符小写 hex；失败返回空串。
// 优先用 mmap（单次 EVP_DigestUpdate 避免 syscall 开销，对 ≥1MB 文件提速 2-5×）；
// 小文件 / mmap 失败 / 空文件 → fallback 到 8KB 流式实现。
std::string ComputeFileSha256(const std::string& path) {
  constexpr size_t kMmapThreshold = 64 * 1024;  // < 64KB 直接流式（mmap 开销不值）

  int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG(WARNING) << "[AtomicDirSwap] sha256: cannot open file: " << path
                 << " errno=" << errno;
    return "";
  }
  struct stat st;
  if (::fstat(fd, &st) != 0) {
    ::close(fd);
    return ComputeFileSha256Stream(path);
  }
  const size_t file_size = static_cast<size_t>(st.st_size);
  if (file_size == 0) {
    ::close(fd);
    return ComputeFileSha256Stream(path);  // 让流式路径产生 empty-hash
  }
  if (file_size < kMmapThreshold) {
    ::close(fd);
    return ComputeFileSha256Stream(path);
  }

  void* addr = ::mmap(nullptr, file_size, PROT_READ, MAP_PRIVATE, fd, 0);
  if (addr == MAP_FAILED) {
    LOG(WARNING) << "[AtomicDirSwap] sha256: mmap failed for " << path
                 << " size=" << file_size << " errno=" << errno
                 << "; falling back to stream";
    ::close(fd);
    return ComputeFileSha256Stream(path);
  }
  // 提示内核：顺序访问 + 提前预读
  ::madvise(addr, file_size, MADV_SEQUENTIAL);

  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  if (ctx == nullptr) {
    ::munmap(addr, file_size);
    ::close(fd);
    return "";
  }
  if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(ctx);
    ::munmap(addr, file_size);
    ::close(fd);
    return "";
  }
  if (EVP_DigestUpdate(ctx, addr, file_size) != 1) {
    LOG(ERROR) << "[AtomicDirSwap] sha256: EVP_DigestUpdate(mmap) failed: " << path;
    EVP_MD_CTX_free(ctx);
    ::munmap(addr, file_size);
    ::close(fd);
    return "";
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_len = 0;
  if (EVP_DigestFinal_ex(ctx, digest, &digest_len) != 1) {
    EVP_MD_CTX_free(ctx);
    ::munmap(addr, file_size);
    ::close(fd);
    return "";
  }
  EVP_MD_CTX_free(ctx);
  ::munmap(addr, file_size);
  ::close(fd);
  return DigestToHex(digest, digest_len);
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
  // v3: 历史 backup 路径 = <target>/backups/backup.<ts>
  const std::string container = target + "/backups";
  const std::string history_backup = container + "/backup." + ts_str;
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
    final_history = container + "/backup." + ts_str + "-" + std::to_string(retry + 1);
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

  // step 5a: 把 swap_backup 内的 backups 子目录抽出来放回新 target（v3 关键步骤）
  //          step 3 rename target → swap_backup 时，整个 target 含 backups 子目录被带走；
  //          这里需要把旧的 backups 历史"救回"新 target 内。
  //          首次保存时 swap_backup 内没有 backups（跳过即可）。
  const std::string swap_backups_subdir = swap_backup + "/backups";
  const std::string target_backups_subdir = target + "/backups";
  if (fs::exists(swap_backup) && fs::exists(swap_backups_subdir)) {
    // 此时 staging 写入的新 target 还没有 backups 子目录（staging 内不带 backups），
    // 所以直接 rename 即可；若旧版本有 race condition 导致已存在，先清空
    if (fs::exists(target_backups_subdir)) {
      LOG(WARNING) << "[AtomicDirSwap] step 5a: new target unexpectedly has backups; removing";
      RemoveDirIfExists(target_backups_subdir);
    }
    fs::rename(swap_backups_subdir, target_backups_subdir, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] step 5a rename(swap_backup/backups -> target/backups) failed: "
                 << ec.message();
      // step 4 已成功；返回 true 让启动恢复续做（避免破坏 target）
      return true;
    }
    FsyncPath(target);
    LOG(INFO) << "[AtomicDirSwap] step 5a: extracted backups from swap_backup back to new target";
  } else {
    // 首次保存：旧 target 无 backups → 仅确保新 target 内有空 backups/ 子目录供 step 5b 用
    if (!EnsureBackupsContainer(target)) {
      LOG(ERROR) << "[AtomicDirSwap] step 5a: cannot create backups container in target";
      return true;  // target 有效，仅历史归档失败
    }
  }

  // step 5b: 归档旧 target 三件套为新历史 backup
  if (fs::exists(swap_backup)) {
    fs::rename(swap_backup, final_history, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] step 5b rename(swap_backup -> backup.<ts>) failed: "
                 << ec.message() << "; new target is still valid";
      return true;
    }
    FsyncPath(container);
    LOG(INFO) << "[AtomicDirSwap] step 5b: archived swap_backup -> " << final_history;
  }

  // step 5c: 轮换历史 backup，保留最近 max_backups 个
  if (max_backups_ >= 0) {  // 0 也合法（不保留任何历史）
    auto backups = ListBackupPaths(target_dir_);
    // 按字典序升序 = 时间升序；最旧在前
    const int total = static_cast<int>(backups.size());
    const int to_remove = total - max_backups_;
    if (to_remove > 0) {
      LOG(INFO) << "[AtomicDirSwap] step 5c: " << total << " backups, max=" << max_backups_
                << ", rotating out " << to_remove;
      for (int i = 0; i < to_remove; ++i) {
        const std::string tombstone = NextTombstonePath(target_dir_);
        std::error_code ec_rn;
        fs::rename(backups[i], tombstone, ec_rn);
        if (ec_rn) {
          LOG(WARNING) << "[AtomicDirSwap] step 5c rename to tombstone failed: "
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

  // 第一步 — 检测并重做未完成的 rollback（marker 协议）
  //   rollback 路径不留 staging/swap_backup，先处理 marker 再走 save-path 决策表
  RecoverRollbackIfInterrupted(target_dir);

  // 清掉所有 tombstone 残留
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
    const std::time_t commit_unix = ReadSentinelTimestamp(sentinel);
    const std::string ts_str = FormatBackupTimestamp(commit_unix);
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
    // v3 step 5a+5b: 抽出 swap_backup/backups → target/backups，然后归档 swap_backup
    ArchiveSwapBackupToTarget(target_dir, swap_backup, ts_str);
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
    LOG(WARNING) << "[AtomicDirSwap] state: step 3 done, completing step 4-5b";
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
    // v3 step 5a+5b: 抽出 + 归档
    const std::time_t commit_unix = ReadSentinelTimestamp(target_dir + "/" + kSentinelName);
    const std::string ts_str = FormatBackupTimestamp(commit_unix);
    ArchiveSwapBackupToTarget(target_dir, swap_backup, ts_str);
    return true;
  }

  // 状态 5: target + swap_backup（无 staging）→ step 4 完成、step 5a/5b 未开始 → 抽出+归档
  if (has_target && !has_staging && has_swap_backup) {
    LOG(WARNING) << "[AtomicDirSwap] state: step 4 done, extracting+archiving swap_backup";
    const std::time_t commit_unix = ReadSentinelTimestamp(target_dir + "/" + kSentinelName);
    const std::string ts_str = FormatBackupTimestamp(commit_unix);
    if (ArchiveSwapBackupToTarget(target_dir, swap_backup, ts_str)) {
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

// ---------------------------------------------------------------------------
// v3.1: rollback = 纯三件套替换（不碰 backups），marker + 重做保证崩溃安全
// ---------------------------------------------------------------------------

namespace {

// rollback 替换的文件清单（三件套 + sentinel）
const char* kRollbackFiles[] = {"map.pbstream", "map.png", "map.yaml", ".complete"};

// 清理 target 内残留的 *.rbtmp 临时文件
void CleanupRollbackTmp(const std::string& target_dir) {
  std::error_code ec;
  if (!fs::exists(target_dir, ec)) return;
  for (auto const& entry : fs::directory_iterator(target_dir, ec)) {
    if (ec) break;
    if (!entry.is_regular_file()) continue;
    const std::string fname = entry.path().filename().string();
    if (fname.size() >= 6 && fname.compare(fname.size() - 6, 6, ".rbtmp") == 0) {
      fs::remove(entry.path(), ec);
    }
  }
}

// 核心：用 source backup 的三件套替换 target 三件套（幂等，可重复调用做崩溃重做）。
//   phase 1: 全部 cp 到 <target>/<f>.rbtmp + fsync（不动现有三件套）
//   phase 2: 连续 rename(.rbtmp → 正式名)（单文件 POSIX 原子，窗口极小）
//   完全不碰 <target>/backups/
bool ReplaceTripletFromBackup(const std::string& target_dir,
                              const std::string& source_backup_dir) {
  std::error_code ec;
  if (!fs::exists(source_backup_dir, ec)) {
    LOG(ERROR) << "[AtomicDirSwap] ReplaceTriplet: source missing: " << source_backup_dir;
    return false;
  }
  // phase 1: cp 全部到 .rbtmp
  for (const char* f : kRollbackFiles) {
    const std::string src = source_backup_dir + "/" + f;
    if (!fs::exists(src, ec)) {
      // .complete 可能不存在于老 backup；三件套必须存在
      if (std::string(f) == ".complete") continue;
      LOG(ERROR) << "[AtomicDirSwap] ReplaceTriplet: source file missing: " << src;
      return false;
    }
    const std::string tmp = target_dir + "/" + f + ".rbtmp";
    fs::copy_file(src, tmp, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] ReplaceTriplet: copy " << src << " -> " << tmp
                 << " failed: " << ec.message();
      CleanupRollbackTmp(target_dir);
      return false;
    }
    FsyncPath(tmp);
  }
  FsyncPath(target_dir);

  // phase 2: 连续 rename .rbtmp → 正式名（窗口极小；崩溃靠 marker 重做）
  for (const char* f : kRollbackFiles) {
    const std::string tmp = target_dir + "/" + f + ".rbtmp";
    if (!fs::exists(tmp, ec)) continue;  // .complete 可能没 cp
    fs::rename(tmp, target_dir + "/" + f, ec);
    if (ec) {
      LOG(ERROR) << "[AtomicDirSwap] ReplaceTriplet: rename " << tmp
                 << " failed: " << ec.message();
      return false;
    }
  }
  FsyncPath(target_dir);
  LOG(INFO) << "[AtomicDirSwap] ReplaceTriplet: target triplet replaced from "
            << source_backup_dir;
  return true;
}

}  // namespace

bool AtomicDirSwap::WriteRollbackMarker(const std::string& target_dir,
                                       const std::string& chosen_backup) {
  const std::string marker = target_dir + kRollbackMarkerSuffix;
  int fd = ::open(marker.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0644);
  if (fd < 0) {
    LOG(ERROR) << "[AtomicDirSwap] WriteRollbackMarker: open failed: " << marker
               << " errno=" << errno;
    return false;
  }
  std::ostringstream oss;
  oss << "chosen_backup=" << chosen_backup << "\n"
      << "started_unix=" << std::time(nullptr) << "\n";
  const std::string content = oss.str();
  const ssize_t got = ::write(fd, content.data(), content.size());
  if (got != static_cast<ssize_t>(content.size())) {
    LOG(ERROR) << "[AtomicDirSwap] WriteRollbackMarker: short write";
    ::close(fd);
    return false;
  }
  if (::fsync(fd) != 0) {
    LOG(WARNING) << "[AtomicDirSwap] WriteRollbackMarker: fsync failed";
  }
  ::close(fd);
  const std::string parent = fs::path(target_dir).parent_path().string();
  FsyncPath(parent.empty() ? "." : parent);
  LOG(INFO) << "[AtomicDirSwap] rollback marker written: chosen=" << chosen_backup;
  return true;
}

bool AtomicDirSwap::RemoveRollbackMarker(const std::string& target_dir) {
  const std::string marker = target_dir + kRollbackMarkerSuffix;
  std::error_code ec;
  if (!fs::exists(marker, ec)) return true;  // 幂等
  fs::remove(marker, ec);
  if (ec) {
    LOG(WARNING) << "[AtomicDirSwap] RemoveRollbackMarker: failed: " << ec.message();
    return false;
  }
  const std::string parent = fs::path(target_dir).parent_path().string();
  FsyncPath(parent.empty() ? "." : parent);
  return true;
}

bool AtomicDirSwap::ReplaceCurrentMapFrom(const std::string& source_backup_dir) {
  std::error_code ec;
  if (!fs::exists(source_backup_dir, ec)) {
    LOG(ERROR) << "[AtomicDirSwap] ReplaceCurrentMapFrom: source missing: " << source_backup_dir;
    return false;
  }
  // 1. 写 marker（崩溃后重启可从 chosen 重做）
  if (!WriteRollbackMarker(target_dir_, source_backup_dir)) {
    LOG(ERROR) << "[AtomicDirSwap] ReplaceCurrentMapFrom: cannot write marker";
    return false;
  }
  // 2. 替换三件套（不碰 backups）
  const bool ok = ReplaceTripletFromBackup(target_dir_, source_backup_dir);
  // 3. 清 marker
  RemoveRollbackMarker(target_dir_);
  return ok;
}

bool AtomicDirSwap::RecoverRollbackIfInterrupted(const std::string& target_dir) {
  const std::string marker = target_dir + kRollbackMarkerSuffix;
  std::error_code ec;
  // 无论有无 marker，先清理可能的 .rbtmp 残留
  if (!fs::exists(marker, ec)) {
    CleanupRollbackTmp(target_dir);
    return true;
  }

  // 解析 marker 的 chosen_backup
  std::string chosen_backup;
  {
    std::ifstream ifs(marker);
    if (ifs) {
      std::string line;
      const std::string key = "chosen_backup=";
      while (std::getline(ifs, line)) {
        if (line.rfind(key, 0) == 0) {
          chosen_backup = line.substr(key.size());
          break;
        }
      }
    }
  }

  LOG(WARNING) << "[AtomicDirSwap] RecoverRollback: unfinished rollback detected, chosen="
               << chosen_backup << "; redoing triplet replace";

  if (!chosen_backup.empty() && fs::exists(chosen_backup, ec)) {
    // 重做三件套替换（幂等；chosen 始终完整保留在 backups 内）
    ReplaceTripletFromBackup(target_dir, chosen_backup);
  } else {
    LOG(WARNING) << "[AtomicDirSwap] RecoverRollback: chosen backup gone, cannot redo; "
                 << "cleaning up tmp (target may be partial — re-run rollback manually)";
  }

  CleanupRollbackTmp(target_dir);
  fs::remove(marker, ec);
  return true;
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
