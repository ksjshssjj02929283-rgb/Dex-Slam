#ifndef AMCL_SCAN_MATCH_SCAN_MATCHING_OPTIONS_H_
#define AMCL_SCAN_MATCH_SCAN_MATCHING_OPTIONS_H_

#include "common/configuration_file_resolver.h"
#include "scan_match/fast_correlative_scan_matcher_2d.h"
#include "scan_match/ceres_scan_matcher_2d.h"
#include "scan_match/real_time_correlative_scan_matcher.h"
#include "scan_match/real_time_correlative_scan_matcher_2d.h"

namespace cartographer
{
namespace mapping
{
namespace scan_matching
{

struct ScanMatchingOptions
{
  double global_localization_min_score;
  bool use_online_correlative_scan_matching;
  bool use_global_matching;
  double voxel_filter_size;

  ::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
  fast_correlative_scan_matcher_options;

  ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
  ceres_scan_matcher_options;

  ::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions
  real_time_correlative_scan_matcher_options;
};

ScanMatchingOptions CreateScanMatchingOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

// std::tuple<ScanMatchingOptions, TrajectoryOptions> ScanMatchingOptions(
//     const std::string& configuration_directory,
//     const std::string& configuration_basename);


} // namespace scan_matching

} // namespace mapping

} // namespace cartographer



#endif // AMCL_SCAN_MATCH_SCAN_MATCHING_OPTIONS_H_