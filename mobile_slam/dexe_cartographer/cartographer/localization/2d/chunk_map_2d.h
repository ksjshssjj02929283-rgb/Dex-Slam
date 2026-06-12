#ifndef CARTOGRAPHER_LOCALIZATION_2D_CHUNK_MAP_2D_H_
#define CARTOGRAPHER_LOCALIZATION_2D_CHUNK_MAP_2D_H_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Core"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/internal/box_filter.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace localization {

// 把单张全局地图按固定网格切分成内存中的 chunk，
// 用"覆盖匹配窗口的 chunk"拼装 local submap，替代对全局点云的 O(N) 全量 BoxFilter。
//
// 每个 chunk = 一块完整的小 Submap2D（grid 子块 + 点云 + line 子集，自包含），
// 与 pbstream 的 ChunkSubmap2D.submap=Submap2D 统一，序列化/匹配/调试落盘全复用。
//   - Pos2Grid 网格哈希寻址（借鉴 lightning-lm tiled_map）
//   - BuildFromSubmap：把全局地图按 chunk 切分，每 chunk 造完整 Submap2D
//       · 点云：按 Pos2Grid 归属 chunk
//       · grid：全图 grid 已知 cell 按所在 chunk 切到子 grid
//       · line：按线段中点 Pos2Grid 归属 chunk
//   - LoadOnPose：load_r/unload_r 曼哈顿滞回，维护活跃 chunk 集（骨架）
//   - AssembleLocalSubmap：从覆盖 box 的 chunk Submap2D 拼装 local submap
//
// 线程安全：BuildFromSubmap 一次性建立 chunk 后数据不再变动；
// AssembleLocalSubmap 纯读、自包含（只读 chunk Submap2D），可在后台线程无锁调用，
// 不会与写路径（ApplyDynamicUpdate 持 submap_mutex_）竞争。
class ChunkMap2D {
 public:
  struct Options {
    double chunk_length = 20.0;  // 单 chunk 边长（米）
    // 加载/卸载半径均为**切比雪夫（方块）半径**，单位 chunk：
    // R=load_radius → local map = (2R+1)×(2R+1) 方块（R=1 即 3×3=9，含中心）。
    int load_radius = 1;       // 加载半径（方块；1 → 3×3=9）
    int unload_radius = 2;     // 卸载半径（> load_radius 形成滞回）
  };

  ChunkMap2D(const Options& options,
             mapping::ValueConversionTables* conversion_tables);

  // 把全局 submap 按 Pos2Grid 切分为内存 chunk，每 chunk 造完整 Submap2D
  // （grid 子块 + 点云 + line 子集）。之后只读、不随写路径变化。
  // 用于 pbstream 不含 chunk（或 chunk_length 不符）时从整图重建。
  void BuildFromSubmap(const std::shared_ptr<mapping::Submap2D>& global_submap);

  // 直接装入 pbstream 已序列化的 chunk Submap2D（不切分），用于复用 map.pbstream 携带的 chunk。
  // 每项 = (gx, gy, 该 chunk 的完整 Submap2D)。
  void LoadFromChunkSubmaps(
      std::vector<std::tuple<int, int, std::shared_ptr<mapping::Submap2D>>>
          chunk_submaps);

  bool empty() const { return chunks_.empty(); }
  int NumChunks() const { return static_cast<int>(chunks_.size()); }
  int NumActiveChunks() const { return static_cast<int>(active_.size()); }

  // 可视化用访问器：chunks_ 建好后只读、线程安全；ActiveChunkIds 仅额外读 last_gx_/gy_/last_grid_set_
  // 这几个标量（benign race，最多差一帧），不触碰可变容器 active_。
  double chunk_length() const { return options_.chunk_length; }
  int load_radius() const { return options_.load_radius; }
  std::vector<std::pair<int, int>> ChunkIds() const;        // 全部 chunk 的 (gx,gy)
  std::vector<std::pair<int, int>> ActiveChunkIds() const;  // 当前 load_radius 方块内已存在的 chunk

  // 产出 chunk map 的序列化 proto（ToProto 形式，不直接碰 writer，风格对齐 Submap2D::ToProto）：
  //   ToInfoProto   = ChunkMapInfo（chunk_length + num_chunks）
  //   ToChunkProtos = 各 chunk 的 ChunkSubmap2D（gx/gy + 自包含 Submap2D，含 grid+点云+line）
  // 由 io::SerializeChunkMap 在 WritePbStream 末尾包装成 SerializedData 写入（chunk 最后写）。
  mapping::proto::ChunkMapInfo ToInfoProto() const;
  std::vector<mapping::proto::ChunkSubmap2D> ToChunkProtos() const;

  // 定位侧持久化（map_persistence_manager 原子换目录用）：把 chunk map 落盘到 chunks_dir/。
  // 统一 pbstream 格式：每 chunk 一个 chunk_<gx>_<gy>.pbstream（Header + SerializedData{chunk_submap}）
  // + chunk_index.pbstream（ChunkMapInfo）。返回 true 表示全部写成功。
  bool SaveToDir(const std::string& chunks_dir) const;

  // 调试落盘：每 chunk 输出 chunk_<gx>_<gy>.pcd（点云）+ chunk_<gx>_<gy>.png（grid 子块）到 debug_dir/。
  // active_only=false 输出全部 chunk；true 只输出当前活跃（已加载）chunk（active_ 集）。
  // 返回 true 表示有写成功；chunk 为空返回 false。
  bool SaveDebugDir(const std::string& debug_dir, bool active_only = false) const;

  // 依据位姿用滞回策略更新活跃 chunk 集；活跃集变化返回 true。
  bool LoadOnPose(const transform::Rigid2d& pose);

  // 把 origin 所在 chunk 的 load_radius（切比雪夫方块）内所有 chunk **整块**组合成 submap：
  // 拷贝这些 chunk 的 grid 已知 cell + 点云 + line，建 ikd-tree。
  // finish_submap=true（默认）：Finish 成只读匹配 submap（截取式静态用法）。
  // finish_submap=false：不 Finish，保持可插入——即 **live submap**（DEC-023/024：active 区直接更新）。
  std::shared_ptr<mapping::Submap2D> AssembleLocalSubmap(
      const transform::Rigid2d& origin, bool finish_submap = true) const;

  // DEC-023：拼装 live submap（= AssembleLocalSubmap 不 Finish，保持可插入）。
  std::shared_ptr<mapping::Submap2D> AssembleLiveSubmap(
      const transform::Rigid2d& origin) const {
    return AssembleLocalSubmap(origin, /*finish_submap=*/false);
  }

  // DEC-023/024：把 live submap 的累积更新切回它覆盖的 chunk（每个被覆盖 chunk **新建** grid 从 live
  // 已知 cell 填入 + SetGrid 换入，cloud 按 Pos2Grid 分回 + VoxelFilter；新建空 grid → SetProbability
  // 合法，规避 bug-592）。若 global_grid 非空，同时把这些 chunk 的 grid 用 SetProbabilityOverwrite
  // **整合进 global_static**（保持全局图新鲜，供重定位 + 保存）。
  void FlushLiveSubmap(const mapping::Submap2D& live,
                       mapping::ProbabilityGrid* global_grid);

 private:
  // 每 chunk = 一块完整的小 Submap2D（grid 子块 + 点云 + line 子集）。
  struct Chunk {
    int gx = 0;
    int gy = 0;
    std::shared_ptr<mapping::Submap2D> submap;
  };

  // 世界坐标 → 网格索引（+0.5 让 chunk 以原点为中心对齐，同 lightning Pos2Grid）。
  void Pos2Grid(double x, double y, int* gx, int* gy) const;
  static int64_t Key(int gx, int gy) {
    return (static_cast<int64_t>(gx) << 32) ^
           (static_cast<int64_t>(static_cast<uint32_t>(gy)));
  }

  Options options_;
  double inv_chunk_length_;
  mapping::ValueConversionTables* conversion_tables_;

  std::unordered_map<int64_t, Chunk> chunks_;
  std::unordered_set<int64_t> active_;
  int last_gx_ = 0;
  int last_gy_ = 0;
  bool last_grid_set_ = false;
  // DEC-023：chunk 路径全同步（更新/跨格/拼装都在匹配线程），无并发，故无需 chunks_mutex_。
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_2D_CHUNK_MAP_2D_H_
