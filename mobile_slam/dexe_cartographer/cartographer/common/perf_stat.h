#ifndef CARTOGRAPHER_COMMON_PERF_STAT_H_
#define CARTOGRAPHER_COMMON_PERF_STAT_H_

// 轻量运行时性能统计：累积各计时项的样本，按需输出
// n / min / mean / p50 / p99 / max（毫秒）表格 + 计数/频率。
//
// 用途：local map 更新热路径（BuildLocalSubmap / AssembleLocalSubmap /
// RebuildIkdTrees+Finish / LoadOnPose）的耗时统计，配合 chunk on/off 做 A/B 对比。
//
// 用法：
//   void Foo() {
//     CARTO_PERF_SCOPE("Foo");        // 作用域计时，析构时记录一次
//     ...
//   }
//   PerfRegistry::Get().AddScalar("rebuild_count", 1);   // 计数累加
//   PerfRegistry::Get().Dump("final");                   // 打印 [PERF] 表格
//
// 输出统一带 [PERF] 前缀，便于回放后 `grep '\[PERF\]'` 汇总。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "glog/logging.h"

namespace cartographer {
namespace common {

// 单个计时项：线程安全累积所有样本（毫秒）。
class PerfTiming {
 public:
  void Add(double ms) {
    std::lock_guard<std::mutex> lk(mu_);
    samples_.push_back(ms);
  }

  // 输出一行：name n min mean p50 p99 max
  std::string Report(const std::string& name) const {
    std::vector<double> s;
    {
      std::lock_guard<std::mutex> lk(mu_);
      s = samples_;
    }
    std::ostringstream os;
    os << std::left << std::setw(30) << name << std::right;
    if (s.empty()) {
      os << "   (no samples)";
      return os.str();
    }
    std::sort(s.begin(), s.end());
    const double sum = std::accumulate(s.begin(), s.end(), 0.0);
    const double mean = sum / static_cast<double>(s.size());
    auto pct = [&s](double p) {
      const size_t i = static_cast<size_t>(p * (s.size() - 1));
      return s[i];
    };
    os << std::setw(8) << s.size() << std::fixed << std::setprecision(3)
       << std::setw(10) << s.front() << std::setw(10) << mean
       << std::setw(10) << pct(0.50) << std::setw(10) << pct(0.99)
       << std::setw(10) << s.back();
    return os.str();
  }

 private:
  mutable std::mutex mu_;
  std::vector<double> samples_;
};

// 全局注册表：按名管理计时项与标量计数；Dump 打印汇总表。
class PerfRegistry {
 public:
  static PerfRegistry& Get() {
    static PerfRegistry inst;
    return inst;
  }

  PerfTiming& Timing(const std::string& name) {
    std::lock_guard<std::mutex> lk(mu_);
    auto& p = timings_[name];
    if (!p) p = std::make_unique<PerfTiming>();
    return *p;
  }

  void AddScalar(const std::string& name, double delta) {
    std::lock_guard<std::mutex> lk(mu_);
    scalars_[name] += delta;
  }

  double GetScalar(const std::string& name) {
    std::lock_guard<std::mutex> lk(mu_);
    const auto it = scalars_.find(name);
    return it == scalars_.end() ? 0.0 : it->second;
  }

  void Dump(const std::string& title) {
    std::lock_guard<std::mutex> lk(mu_);
    std::ostringstream os;
    os << "\n[PERF] ============ 运行时间统计 [" << title
       << "] ============\n";
    os << "[PERF] " << std::left << std::setw(30) << "功能" << std::right
       << std::setw(8) << "n" << std::setw(10) << "min" << std::setw(10)
       << "mean" << std::setw(10) << "p50" << std::setw(10) << "p99"
       << std::setw(10) << "max" << "   (ms)\n";
    for (const auto& kv : timings_) {
      os << "[PERF] " << kv.second->Report(kv.first) << "\n";
    }
    if (!scalars_.empty()) {
      os << "[PERF] ------------ 计数 / 频率 ------------\n";
      for (const auto& kv : scalars_) {
        os << "[PERF] " << std::left << std::setw(30) << kv.first << std::right
           << std::fixed << std::setprecision(3) << kv.second << "\n";
      }
      // 重建频率（次/米、次/秒），需要 rebuild_count + robot_distance_m / run_seconds。
      const auto fc = scalars_.find("rebuild_count");
      const auto fd = scalars_.find("robot_distance_m");
      if (fc != scalars_.end() && fd != scalars_.end() && fd->second > 1e-6) {
        os << "[PERF] " << std::left << std::setw(30) << "rebuild_freq(次/m)"
           << std::right << std::fixed << std::setprecision(4)
           << (fc->second / fd->second) << "\n";
      }
    }
    os << "[PERF] ==================================================\n";
    LOG(INFO) << os.str();
    // 同时落文件（覆盖写）：累积统计取最新即最完整，进程被硬杀（Ctrl+C / SIGKILL，
    // 析构 Dump 不触发）也能从该文件拿到最后一次完整表。与监控 csv 同目录，便于一起收集。
    if (const char* home = std::getenv("HOME")) {
      const std::string dir =
          std::string(home) + "/.dexe/localization/localization_assessment";
      std::error_code ec;
      std::filesystem::create_directories(dir, ec);
      std::ofstream f(dir + "/perf_stat.txt", std::ios::trunc);
      if (f) f << os.str();
    }
  }

 private:
  std::mutex mu_;
  std::map<std::string, std::unique_ptr<PerfTiming>> timings_;
  std::map<std::string, double> scalars_;
};

// RAII 作用域计时：构造记起点，析构记录耗时到指定计时项。
class ScopedPerf {
 public:
  explicit ScopedPerf(const std::string& name)
      : name_(name), t0_(std::chrono::steady_clock::now()) {}
  ~ScopedPerf() {
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0_)
                          .count();
    PerfRegistry::Get().Timing(name_).Add(ms);
  }

 private:
  std::string name_;
  std::chrono::steady_clock::time_point t0_;
};

#define CARTO_PERF_CONCAT2(a, b) a##b
#define CARTO_PERF_CONCAT(a, b) CARTO_PERF_CONCAT2(a, b)
#define CARTO_PERF_SCOPE(name) \
  ::cartographer::common::ScopedPerf CARTO_PERF_CONCAT(_carto_perf_, __LINE__)(name)

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PERF_STAT_H_
