#include "scan_match/scan_matching_options.h"

namespace cartographer
{

namespace mapping
{

namespace scan_matching
{

ScanMatchingOptions CreateScanMatchingOptions(
  ::cartographer::common::LuaParameterDictionary* const
  lua_parameter_dictionary)
{
  ScanMatchingOptions options;
  options.global_localization_min_score =
    lua_parameter_dictionary->GetDouble("global_localization_min_score");

  options.use_global_matching =
    lua_parameter_dictionary->GetBool("use_global_matching");

  options.fast_correlative_scan_matcher_options =
    ::cartographer::mapping::scan_matching::CreateFastCorrelativeScanMatcherOptions2D(
      lua_parameter_dictionary->GetDictionary("fast_correlative_scan_matcher").get());

  options.ceres_scan_matcher_options =
    ::cartographer::mapping::scan_matching::CreateCeresScanMatcherOptions2D(
      lua_parameter_dictionary->GetDictionary("ceres_scan_matcher").get());

  options.use_online_correlative_scan_matching =
    lua_parameter_dictionary->GetBool("use_online_correlative_scan_matching");

  options.real_time_correlative_scan_matcher_options =
    ::cartographer::mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
      lua_parameter_dictionary->GetDictionary("real_time_correlative_scan_matcher").get());

  options.voxel_filter_size =
    lua_parameter_dictionary->GetDouble("voxel_filter_size");

  return options;
}

} // namespace scan_matching

} // namespace mapping

} // namespace cartographer
