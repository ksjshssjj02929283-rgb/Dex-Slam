// Copyright (c) 2026 Dexforce Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "perception_2d/sensor/sync_timed_point_cloud.hpp"
#include <vector>
namespace perception_2d
{
namespace sensor
{

RangeData TransformSyncRangeData(
  const std::vector<SyncRangeData> & sync_range_data,
  const transform::Rigid3f & transform)
{
  RangeData reset;
  reset.origin = transform::Project2D(transform).ToVector();
  for (int i = 0; i < static_cast<int>(sync_range_data.size()); i++) {
    PointCloud hits = TransformPointCloud(sync_range_data[i].hits, transform);
    PointCloud misses = TransformPointCloud(sync_range_data[i].misses, transform);
    reset.returns.insert(reset.returns.end(), hits.begin(), hits.end());
    reset.misses.insert(reset.misses.end(), misses.begin(), misses.end());
  }
  return reset;
}

RangeDataNormal TransformSyncRangeDataNormal(
  const std::vector<SyncRangeData> & sync_range_data,
  const std::vector<Normals> & normals,
  const transform::Rigid3f & transform)
{
  CHECK_EQ(sync_range_data.size(), normals.size());
  RangeDataNormal reset;
  reset.range_data.origin = transform::Project2D(transform).ToVector();
  for (int i = 0; i < static_cast<int>(sync_range_data.size()); i++) {
    CHECK_EQ(sync_range_data[i].hits.size(), normals[i].size());
    PointCloud hits = TransformPointCloud(sync_range_data[i].hits, transform);
    PointCloud misses = TransformPointCloud(sync_range_data[i].misses, transform);
    reset.range_data.returns.insert(reset.range_data.returns.end(), hits.begin(), hits.end());
    reset.range_data.misses.insert(reset.range_data.misses.end(), misses.begin(), misses.end());
    Normals normals_temp = TransformNormals(normals[i], transform);
    reset.normals.insert(
      reset.normals.end(),
      normals_temp.begin(), normals_temp.end());
  }
  return reset;
}

}  // namespace sensor
}  // namespace perception_2d
