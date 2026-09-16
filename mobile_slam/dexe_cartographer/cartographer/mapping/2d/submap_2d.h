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
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
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

namespace scan_matching {
class PrecomputationGridStack2D;  // 前向声明
}  // namespace scan_matching

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

  // 栅格覆盖区域的几何中心（世界系）。
  //
  // ⚠️ 任何需要「地图在哪」的场合一律用本函数，绝不可用 local_pose()。
  //
  // local_pose 的契约（submaps.h:73）是「该 submap 在 local map 系中的位姿」
  // 这一刚体变换，而非位置：
  //   * 对注册进 pose_graph 的滑动窗口 submap，它是真实位姿，含义是
  //     submap 自身系 -> local map 系 的变换；
  //   * 对 global_static_submap，它从不被当作变换使用（map_builder.cc:634
  //     只赋给独立成员，无 AddSubmap 路径；且数据插入前已变换到 global 系，
  //     见 map_builder.cc:514-522），因此是 Submap2D 构造函数的副产物，
  //     不承载位姿信息。
  //
  // 生产侧把它赋成「节点包围盒 + lidar_range_pad(30m)」的右上角
  // （map_transform_common.cc:378-387，bbox_max 经 SnapMaxToFirstNodeLattice
  // 是为栅格相位对齐，见同文件 :281-288 注释），与位姿语义无关。
  //
  // 该值可被 set_local_pose() 改写（submaps.h:76-79），
  // 任何代码都不得假设它与栅格几何位置有固定关系。
  Eigen::Vector2d GridCenterXY() const;

  // 返回全局定位地图缓存——缓存存在且层数与 options.branch_and_bound_depth
  // 一致则直接返回，否则从 grid 现场构建并缓存。matcher 创建时调用以避免重复构建。
  std::shared_ptr<scan_matching::PrecomputationGridStack2D>
  GetOrBuildPrecomputationGridStack(
      const scan_matching::proto::FastCorrelativeScanMatcherOptions2D& options);
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
  // 从源 submap 继承线特征 culling 判据。
  // 深拷贝构造（origin/grid/cloud/line_data 那个 ctor）不会带上判据，新 submap
  // 会退回 LineMap2DList 的类内默认值；若源 submap 的判据来自建图期 lua 或
  // pbstream 内嵌 options，两套判据的差集就会在首次 culling 时被静默剔除。
  void InheritLineCullingParamFrom(const Submap2D& source);
  void SetCullingInterval(const int& interval) { culling_interval_ = interval; }

  void FreshCloudData() const;
  
  // 重建ikd-tree (从已有的点云数据)
  void RebuildIkdTrees();

  // 对已重建的 ikd-tree 调用 Finish()，使其切换为 PCL 只读后端（内存更小，查询更快）
  void FinishIkdTrees();

  // 销毁 ikd-tree，释放内存（active submap 离开活跃集合时调用）
  void DestroyIkdTrees();
  
 private:
  void CloudDataCulling();
  // void GenerateCloudKdTree();
  void PointCloudToPCL(const sensor::PointCloud & cloud_data,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);

 private:
  std::unique_ptr<Grid2D> grid_;
  // 全局定位地图缓存（pbstream 载入或首次定位构建填入），随 grid 走。
  mutable std::shared_ptr<scan_matching::PrecomputationGridStack2D>
      precomputation_grid_stack_;
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

  // Inserts all data types into the Submap collection.
  // This method handles submap creation and returns the current submaps.
  // Empty data will be skipped automatically.
  std::vector<std::shared_ptr<const Submap2D>> InsertData(
      const std::vector<sensor::RangeData>& range_data_vector,
      const sensor::PointCloud& cloud_data,
      const sensor::PointCloud& highlight_cloud_data,
      const std::map<int, std::vector<sensor::LineData>>& line_data,
      const std::vector<sensor::LandmarkPointData>& reflector_data);

  // Inserts 'range_data' into the Submap collection.
  // This method now only inserts data without returning submaps.
  void InsertRangeData(
      const std::vector<sensor::RangeData>& range_data_vector);

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
