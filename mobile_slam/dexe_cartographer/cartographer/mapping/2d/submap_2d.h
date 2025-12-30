/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/proto/submaps_options_2d.pb.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/feature_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/ikd_tree/ikd_tree.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

class Submap2D : public Submap {
 public:
  Submap2D(const Eigen::Vector2f& origin,
           std::unique_ptr<Grid2D> grid,
           ValueConversionTables* conversion_tables,
           std::unique_ptr<sensor::PointCloud> cloud,
           std::unique_ptr<sensor::PointCloud> highlight_cloud,
           int latest_line_id,
           std::map<int, MapLine2D> active_line_data);

  explicit Submap2D(const proto::Submap2D& proto,
                    ValueConversionTables* conversion_tables);

  ~Submap2D() override;

  proto::Submap ToProto(bool include_grid_data) const override;
  void UpdateFromProto(const proto::Submap& proto) override;

  void ToResponseProto(const transform::Rigid3d& global_submap_pose,
                       proto::SubmapQuery::Response* response) const override;

  const Grid2D* grid() const { return grid_.get(); }
  const sensor::PointCloud* cloud() const {
    if (is_dirty_cloud_) {
      FreshCloudData();
    }
    return cloud_.get();
  }
  const sensor::PointCloud* highlight_cloud() const { return highlight_cloud_.get(); }

  ikd_tree::iKdTree<pcl::PointXYZ>* cloud_ikdtree() const { return cloud_ikdtree_.get(); }
  ikd_tree::iKdTree<pcl::PointXYZ>* highlight_cloud_ikdtree() const { return highlight_cloud_ikdtree_.get(); }

  const LineMap2DList * line_map_2d_list() const {return line_map_2d_list_.get();}

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const RangeDataInserterInterface* range_data_inserter);

  void InsertCloudData(const sensor::PointCloud & cloud_data,
                       const sensor::PointCloud & highlight_cloud_in_local);

  // void InsertNormalData(const sensor::NormalPointCloud & normals,
  //                       const RangeDataInserterInterface * range_data_inserter);

  void InsertLineData(const std::map<int, std::vector<sensor::LineData>> & line_data);


  void InsertReflectorData(
          const std::vector<sensor::LandmarkPointData> & reflector_data);
  // void InsertHighLightCloudData(const sensor::PointCloudWithIntensity & high_light_cloud);

  void Finish();

  bool SaveSubmapCloud(const std::string& folder,
                       const std::string& filename);

  void SetCloudCullingProb(const double prob) { cloud_culling_prob_ = prob; }
  void SetLineCullingParam(const double& min_line_length,
                           const double& max_line_length,
                           const int& culling_obs);
  void SetCullingInterval(const int& interval) { culling_interval_ = interval; }

  void FreshCloudData() const;
 private:
  void CloudDataCulling();
  // void GenerateCloudKdTree();
  void PointCloudToPCL(const sensor::PointCloud & cloud_data,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);

 private:
  std::unique_ptr<Grid2D> grid_;
  mutable std::unique_ptr<sensor::PointCloud> cloud_;
  mutable std::unique_ptr<sensor::PointCloud> highlight_cloud_;
  std::unique_ptr<ikd_tree::iKdTree<pcl::PointXYZ>> cloud_ikdtree_;
  std::unique_ptr<ikd_tree::iKdTree<pcl::PointXYZ>> highlight_cloud_ikdtree_;

  std::unique_ptr<LineMap2DList> line_map_2d_list_;
  std::vector<sensor::LandmarkPointData> reflector_map_;
  ValueConversionTables* conversion_tables_;
  double cloud_culling_prob_ = 0.5;

  int line_culling_count_ = 0;
  int cloud_culling_count_ = 0;
  int culling_interval_ = 5;

  // TODO: linfeng.guo: 这个标志位是否需要推及到所有的地图类型
  mutable bool is_dirty_cloud_ = false;
};

// The first active submap will be created on the insertion of the first range
// data. Except during this initialization when no or only one single submap
// exists, there are always two submaps into which range data is inserted: an
// old submap that is used for matching, and a new one, which will be used for
// matching next, that is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
class ActiveSubmaps2D {
 public:
  explicit ActiveSubmaps2D(const proto::SubmapsOptions2D& options);

  ~ActiveSubmaps2D() = default;

  ActiveSubmaps2D(const ActiveSubmaps2D&) = delete;
  ActiveSubmaps2D& operator=(const ActiveSubmaps2D&) = delete;

  // Inserts 'range_data' into the Submap collection.
  // Only responsible for inserting range data, does not handle submap creation/finishing.
  void InsertRangeData(const std::vector<sensor::RangeData>& range_data_vector);

  // Gets the insertion submaps and handles submap creation/finishing logic.
  std::vector<std::shared_ptr<const Submap2D>> GetInsertionSubmaps(
      const Eigen::Vector2f& origin);

  // void InsertNormalData(const sensor::NormalPointCloud & normals);

  void InsertCloudData(const sensor::PointCloud & cloud_data,
                       const sensor::PointCloud & highlight_cloud_data);

  void InsertLineData(
    const std::map<int, std::vector<sensor::LineData>> & line_data);

  // void InsertHighLightCloudData(const sensor::PointCloudWithIntensity & high_light_cloud);

  void InsertReflectorData(const std::vector<sensor::LandmarkPointData> & reflector_data);

  void clear() { submaps_.clear(); }

  std::vector<std::shared_ptr<const Submap2D>> submaps() const;

  const LineMap2DList * line_map_2d_list() {return submaps().front()->line_map_2d_list();}

 private:
  std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
  std::unique_ptr<GridInterface> CreateGrid(const Eigen::Vector2f& origin);
  void FinishSubmap();
  void AddSubmap(const Eigen::Vector2f& origin);

  const proto::SubmapsOptions2D options_;
  std::vector<std::shared_ptr<Submap2D>> submaps_;
  std::unique_ptr<RangeDataInserterInterface> range_data_inserter_;
  ValueConversionTables conversion_tables_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
