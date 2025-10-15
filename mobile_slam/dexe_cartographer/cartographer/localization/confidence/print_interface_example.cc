/*
 * 置信度评估打印接口使用示例
 * 
 * 本文件演示如何使用各个评估器的打印功能
 */

#include "cartographer/localization/confidence/confidence_estimator.h"
#include "cartographer/localization/confidence/scan_match_confidence.h"
#include "cartographer/localization/confidence/motion_consistency_checker.h"
#include "cartographer/localization/confidence/map_coherence_evaluator.h"
#include "cartographer/localization/confidence/degeneracy_detector.h"
#include "glog/logging.h"

namespace cartographer {
namespace localization {
namespace confidence {

// 示例1: 打印主置信度评估结果
void ExamplePrintMainMetrics() {
  LOG(INFO) << "===== Example 1: Main Confidence Metrics =====";
  
  // 模拟评估结果
  ConfidenceEstimator::Metrics metrics;
  metrics.total_confidence = 0.7234;
  metrics.level = ConfidenceLevel::GOOD;
  
  // Layer 1
  metrics.scan_match_score = 0.7856;
  metrics.ceres_residual_score = 0.7645;
  metrics.match_ratio = 0.8123;
  metrics.covariance_score = 0.7423;
  
  // Layer 2
  metrics.motion_consistency = 0.6723;
  metrics.motion_delta_score = 0.7123;
  metrics.velocity_smooth_score = 0.6323;
  
  // Layer 3
  metrics.map_coherence = 0.7134;
  metrics.map_entropy = 0.6845;
  metrics.submap_match_score = 0.7423;
  
  // Layer 4
  metrics.degeneracy_factor = 0.8956;
  metrics.is_corridor_detected = false;
  metrics.feature_distribution_score = 0.8956;
  
  // 方式1: 直接打印
  metrics.Print();
  
  // 方式2: 获取字符串
  std::string report = metrics.ToString();
  LOG(INFO) << "Detailed report:\n" << report;
}

// 示例2: 条件打印 - 仅在置信度较低时
void ExampleConditionalPrint() {
  LOG(INFO) << "\n===== Example 2: Conditional Printing =====";
  
  ConfidenceEstimator::Metrics metrics;
  metrics.total_confidence = 0.35;  // 较低的置信度
  metrics.level = ConfidenceLevel::POOR;
  metrics.scan_match_score = 0.4;
  metrics.motion_consistency = 0.3;
  metrics.map_coherence = 0.35;
  metrics.degeneracy_factor = 0.4;
  
  // 仅在置信度低于阈值时打印详细信息
  if (metrics.level <= ConfidenceLevel::POOR) {
    LOG(WARNING) << "Low confidence detected! Detailed analysis:\n"
                 << metrics.ToString();
  }
  
  // 也可以根据总置信度判断
  if (metrics.total_confidence < 0.5) {
    LOG(WARNING) << "Total confidence below threshold: " 
                 << metrics.total_confidence;
  }
}

// 示例3: 打印各层评估器的详细结果
void ExamplePrintLayerResults() {
  LOG(INFO) << "\n===== Example 3: Individual Layer Results =====";
  
  // Layer 1: Scan Match
  {
    ScanMatchConfidence::Result result;
    result.total_confidence = 0.7856;
    result.ceres_confidence = 0.7645;
    result.match_ratio = 0.8123;
    result.covariance_confidence = 0.7423;
    result.angle_coverage_entropy = 0.7234;
    
    LOG(INFO) << "--- Layer 1: Scan Match ---";
    result.Print();
  }
  
  // Layer 2: Motion Consistency
  {
    MotionConsistencyChecker::Result result;
    result.total_confidence = 0.6723;
    result.prediction_observation_score = 0.7123;
    result.velocity_smoothness_score = 0.6323;
    result.delta_distance = 0.0234;  // meters
    result.delta_angle = 0.0603;     // radians (~3.45 degrees)
    
    LOG(INFO) << "\n--- Layer 2: Motion Consistency ---";
    result.Print();
  }
  
  // Layer 3: Map Coherence
  {
    MapCoherenceEvaluator::Result result;
    result.total_confidence = 0.7134;
    result.submap_match_score = 0.7423;
    result.map_entropy = 0.6845;
    
    LOG(INFO) << "\n--- Layer 3: Map Coherence ---";
    result.Print();
  }
  
  // Layer 4: Degeneracy Detection
  {
    DegeneracyDetector::Result result;
    result.degeneracy_factor = 0.8956;
    result.is_corridor = false;
    result.num_parallel_walls = 0;
    result.feature_distribution = 0.8956;
    result.hessian_condition_num = 12.3456;
    
    LOG(INFO) << "\n--- Layer 4: Degeneracy Detection ---";
    result.Print();
  }
}

// 示例4: 走廊检测场景
void ExampleCorridorDetection() {
  LOG(INFO) << "\n===== Example 4: Corridor Detection Scenario =====";
  
  ConfidenceEstimator::Metrics metrics;
  metrics.total_confidence = 0.45;
  metrics.level = ConfidenceLevel::MEDIUM;
  metrics.scan_match_score = 0.65;
  metrics.motion_consistency = 0.55;
  metrics.map_coherence = 0.60;
  metrics.degeneracy_factor = 0.30;  // 低退化因子
  metrics.is_corridor_detected = true;  // 检测到走廊
  metrics.feature_distribution_score = 0.30;
  
  LOG(WARNING) << "Corridor environment detected!\n" << metrics.ToString();
  
  // 建议采取的措施
  if (metrics.is_corridor_detected) {
    LOG(INFO) << "Recommendation: Reduce speed or switch to alternative localization";
  }
}

// 示例5: 实时监控和趋势分析
void ExampleRealTimeMonitoring() {
  LOG(INFO) << "\n===== Example 5: Real-time Monitoring =====";
  
  // 模拟多个时间点的置信度
  std::vector<double> confidence_history = {0.85, 0.82, 0.78, 0.65, 0.55, 0.48};
  
  LOG(INFO) << "Confidence trend over time:";
  for (size_t i = 0; i < confidence_history.size(); ++i) {
    LOG(INFO) << "  t" << i << ": " << confidence_history[i];
  }
  
  // 检测置信度下降趋势
  if (confidence_history.back() < confidence_history.front() - 0.2) {
    LOG(WARNING) << "Significant confidence drop detected! "
                 << "From " << confidence_history.front() 
                 << " to " << confidence_history.back();
    
    // 打印当前详细状态
    ConfidenceEstimator::Metrics current;
    current.total_confidence = confidence_history.back();
    current.level = ConfidenceLevel::MEDIUM;
    current.scan_match_score = 0.50;
    current.motion_consistency = 0.45;
    current.map_coherence = 0.48;
    current.degeneracy_factor = 0.50;
    
    LOG(WARNING) << "Current state:\n" << current.ToString();
  }
}

// 示例6: 集成到实际定位流程中
void ExampleIntegrationInScanMatch(
    const ConfidenceEstimator::Metrics& metrics) {
  
  // 在实际的ScanMatch函数中使用
  
  // 1. 正常情况：使用VLOG避免日志泛滥
  VLOG(2) << "Scan match confidence:\n" << metrics.ToString();
  
  // 2. 置信度警告
  if (metrics.level <= ConfidenceLevel::MEDIUM) {
    LOG(WARNING) << "Moderate confidence detected (score=" 
                 << metrics.total_confidence << ")";
    VLOG(1) << "Detailed metrics:\n" << metrics.ToString();
  }
  
  // 3. 置信度严重异常
  if (metrics.level <= ConfidenceLevel::POOR) {
    LOG(ERROR) << "Poor confidence detected!\n" << metrics.ToString();
    // 采取恢复措施
  }
  
  // 4. 定位失败
  if (metrics.level == ConfidenceLevel::FAILED) {
    LOG(ERROR) << "Localization FAILED!\n" << metrics.ToString();
    // 触发重定位
  }
  
  // 5. 分层分析问题
  if (metrics.total_confidence < 0.5) {
    if (metrics.scan_match_score < 0.5) {
      LOG(WARNING) << "Scan matching quality issue";
    }
    if (metrics.motion_consistency < 0.5) {
      LOG(WARNING) << "Motion consistency issue";
    }
    if (metrics.degeneracy_factor < 0.5) {
      LOG(WARNING) << "Environment degeneracy detected";
    }
  }
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

// 主函数 - 运行所有示例
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;  // 输出到stderr
  FLAGS_colorlogtostderr = 1;  // 彩色输出
  
  using namespace cartographer::localization::confidence;
  
  LOG(INFO) << "\n\n========================================";
  LOG(INFO) << "Confidence Evaluation Print Interface Examples";
  LOG(INFO) << "========================================\n";
  
  // 运行各个示例
  ExamplePrintMainMetrics();
  ExampleConditionalPrint();
  ExamplePrintLayerResults();
  ExampleCorridorDetection();
  ExampleRealTimeMonitoring();
  
  LOG(INFO) << "\n========================================";
  LOG(INFO) << "All examples completed!";
  LOG(INFO) << "========================================\n";
  
  return 0;
}
