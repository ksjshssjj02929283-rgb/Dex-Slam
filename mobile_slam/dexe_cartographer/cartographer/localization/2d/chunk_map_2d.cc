#include "cartographer/localization/2d/chunk_map_2d.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <tuple>
#include <utility>

#include "cartographer/io/internal/mapping_state_serialization.h"  // kMappingStateSerializationFormatVersion
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace localization {

ChunkMap2D::ChunkMap2D(const Options& options,
                       mapping::ValueConversionTables* conversion_tables)
    : options_(options),
      inv_chunk_length_(1.0 / options.chunk_length),
      conversion_tables_(conversion_tables) {
  CHECK_GT(options_.chunk_length, 0.0);
  CHECK_GT(options_.unload_radius, options_.load_radius)
      << "unload_radius 必须大于 load_radius 才能形成滞回缓冲带";
}

void ChunkMap2D::Pos2Grid(double x, double y, int* gx, int* gy) const {
  *gx = static_cast<int>(std::floor(x * inv_chunk_length_ + 0.5));
  *gy = static_cast<int>(std::floor(y * inv_chunk_length_ + 0.5));
}

void ChunkMap2D::BuildFromSubmap(
    const std::shared_ptr<mapping::Submap2D>& global_submap) {
  chunks_.clear();
  active_.clear();
  last_grid_set_ = false;

  if (!global_submap) {
    LOG(ERROR) << "ChunkMap2D::BuildFromSubmap: global_submap is null.";
    return;
  }
  const auto* prob_grid =
      dynamic_cast<const mapping::ProbabilityGrid*>(global_submap->grid());
  if (!prob_grid) {
    LOG(ERROR) << "ChunkMap2D::BuildFromSubmap: global grid is not "
                  "ProbabilityGrid.";
    return;
  }

  const double res = prob_grid->limits().resolution();
  const int n = static_cast<int>(std::lround(options_.chunk_length / res));
  if (n <= 0) {
    LOG(ERROR) << "ChunkMap2D::BuildFromSubmap: chunk grid size n<=0 (chunk_length="
               << options_.chunk_length << ", res=" << res << ").";
    return;
  }
  const double half = 0.5 * options_.chunk_length;

  // 临时按 chunk 网格分箱（pair=(gx,gy)，避免 int64 key 解码）。
  using GridKey = std::pair<int, int>;
  struct Bin {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    std::map<int, mapping::MapLine2D> lines;
    std::unique_ptr<mapping::ProbabilityGrid> grid;
  };
  std::map<GridKey, Bin> bins;

  auto ensure_grid = [&](const GridKey& g) -> mapping::ProbabilityGrid* {
    Bin& b = bins[g];
    if (!b.grid) {
      const double min_x = g.first * options_.chunk_length - half;
      const double min_y = g.second * options_.chunk_length - half;
      mapping::MapLimits limits(
          res, Eigen::Vector2d(min_x + n * res, min_y + n * res),
          mapping::CellLimits(n, n));
      b.grid =
          std::make_unique<mapping::ProbabilityGrid>(limits, conversion_tables_);
    }
    return b.grid.get();
  };

  // 1) grid：遍历全图已知 cell，按所在 chunk 切到子 grid。
  const auto& gl = prob_grid->limits();
  const int num_x = gl.cell_limits().num_x_cells;
  const int num_y = gl.cell_limits().num_y_cells;
  size_t grid_known = 0;    // 全图已知 cell 总数
  size_t grid_set = 0;      // 成功落入某 chunk 子 grid 的 cell 数
  for (int y = 0; y < num_y; ++y) {
    for (int x = 0; x < num_x; ++x) {
      // cartographer cell index = (x_index, y_index)，x∈[0,num_x)、y∈[0,num_y)
      // （与 XYIndexRangeIterator 产出的约定一致：index.x()∈[0,num_x), index.y()∈[0,num_y)）。
      const Eigen::Array2i gi(x, y);
      if (!prob_grid->IsKnown(gi)) continue;
      ++grid_known;
      const Eigen::Vector2f center = gl.GetCellCenter(gi);
      int gx, gy;
      Pos2Grid(center.x(), center.y(), &gx, &gy);
      auto* sub = ensure_grid({gx, gy});
      const Eigen::Array2i li = sub->limits().GetCellIndex(center);
      if (sub->limits().Contains(li)) {
        sub->SetProbability(li, prob_grid->GetProbability(gi));
        ++grid_set;
      }
    }
  }
  if (grid_set != grid_known) {
    LOG(WARNING) << "[chunk 校验] 栅格 cell 落格不全：已知 " << grid_known
                 << "，落入 chunk " << grid_set << "，丢失 "
                 << (grid_known - grid_set)
                 << "（子 grid GetCellIndex 越界，边界对齐问题）";
  }

  // 2) 点云：按 Pos2Grid 归属 chunk。
  size_t num_points = 0;
  if (const sensor::PointCloud* cloud = global_submap->cloud()) {
    const bool has_intensities = !cloud->intensities().empty();
    const auto& points = cloud->points();
    num_points = points.size();
    for (size_t i = 0; i < points.size(); ++i) {
      int gx, gy;
      Pos2Grid(points[i].position.x(), points[i].position.y(), &gx, &gy);
      Bin& b = bins[{gx, gy}];
      b.points.push_back(points[i]);
      if (has_intensities) b.intensities.push_back(cloud->intensities()[i]);
    }
  }

  // 3) line：按线段中点 Pos2Grid 归属 chunk。
  int latest_line_id = 0;
  size_t num_lines = 0;
  if (const auto* lm = global_submap->line_map_2d_list()) {
    latest_line_id = lm->latest_local_id();
    const auto& line_data = lm->GetLineMapData2DList();
    num_lines = line_data.size();
    for (const auto& kv : line_data) {
      const auto& ld = kv.second.line_data();
      const double mx = 0.5 * (ld.begin().x() + ld.end().x());
      const double my = 0.5 * (ld.begin().y() + ld.end().y());
      int gx, gy;
      Pos2Grid(mx, my, &gx, &gy);
      bins[{gx, gy}].lines.insert_or_assign(kv.first, kv.second);
    }
  }

  // 4) 每个 chunk 造完整 Submap2D（grid 子块 + 点云 + line 子集）。
  for (auto& kv : bins) {
    const int gx = kv.first.first;
    const int gy = kv.first.second;
    Bin& b = kv.second;

    // grid：若该 chunk 无已知 cell，建空子 grid（Submap2D 要求 grid 非空）。
    std::unique_ptr<mapping::ProbabilityGrid> grid =
        b.grid ? std::move(b.grid) : nullptr;
    if (!grid) {
      const double min_x = gx * options_.chunk_length - half;
      const double min_y = gy * options_.chunk_length - half;
      mapping::MapLimits limits(
          res, Eigen::Vector2d(min_x + n * res, min_y + n * res),
          mapping::CellLimits(n, n));
      grid =
          std::make_unique<mapping::ProbabilityGrid>(limits, conversion_tables_);
    }

    std::unique_ptr<sensor::PointCloud> cloud_sub;
    if (!b.intensities.empty() &&
        b.intensities.size() == b.points.size()) {
      cloud_sub = std::make_unique<sensor::PointCloud>(
          std::move(b.points), std::move(b.intensities));
    } else {
      cloud_sub = std::make_unique<sensor::PointCloud>(std::move(b.points));
    }

    auto submap = std::make_shared<mapping::Submap2D>(
        Eigen::Vector2f(static_cast<float>(gx * options_.chunk_length),
                        static_cast<float>(gy * options_.chunk_length)),
        std::move(grid), conversion_tables_, std::move(cloud_sub),
        std::make_unique<sensor::PointCloud>(), latest_line_id,
        std::move(b.lines));
    // chunk 是静态只读地图，cloud_ 已是构造时直填的最终点云。用 FinishStatic（只裁剪 grid +
    // 标记 finished）而非 Finish()——后者以 ikd-tree 为 cloud_ 权威源，会用空树把直填的 cloud_
    // 刷空，且需建树/剔除/同步/销毁的无谓开销。AssembleLocalSubmap 只读 cloud()，无需 chunk 常驻 ikd-tree。
    submap->FinishStatic();

    Chunk c;
    c.gx = gx;
    c.gy = gy;
    c.submap = std::move(submap);
    chunks_[Key(gx, gy)] = std::move(c);
  }

  LOG(INFO) << "ChunkMap2D: built " << chunks_.size() << " chunks (size="
            << options_.chunk_length << "m) from " << num_points << " points, "
            << num_lines << " lines.";

  // === 校验：按地图边界推算应覆盖的 chunk id 范围/数量，与实际生成对比 ===
  // chunk id 来历：Pos2Grid(x,y)=(floor(x/L+0.5), floor(y/L+0.5))，即把世界坐标按边长 L
  // 的网格量化；chunk(gx,gy) 覆盖世界 [gx*L-L/2, gx*L+L/2) × [gy*L-L/2, gy*L+L/2)。
  {
    const auto& lim = prob_grid->limits();
    const auto& cl = lim.cell_limits();
    // 全图 4 角世界坐标（取 cell 中心的对角两角即可定 bbox）。
    const Eigen::Vector2f cA = lim.GetCellCenter(Eigen::Array2i(0, 0));
    const Eigen::Vector2f cB =
        lim.GetCellCenter(Eigen::Array2i(cl.num_x_cells - 1, cl.num_y_cells - 1));
    const double wx_lo = std::min(cA.x(), cB.x());
    const double wx_hi = std::max(cA.x(), cB.x());
    const double wy_lo = std::min(cA.y(), cB.y());
    const double wy_hi = std::max(cA.y(), cB.y());
    int gx_lo, gy_lo, gx_hi, gy_hi;
    Pos2Grid(wx_lo, wy_lo, &gx_lo, &gy_lo);
    Pos2Grid(wx_hi, wy_hi, &gx_hi, &gy_hi);
    if (gx_lo > gx_hi) std::swap(gx_lo, gx_hi);
    if (gy_lo > gy_hi) std::swap(gy_lo, gy_hi);
    const int span =
        (gx_hi - gx_lo + 1) * (gy_hi - gy_lo + 1);  // 满铺（含全空 chunk）上限
    // 实际 chunk id 范围。
    int agx_lo = 0, agx_hi = 0, agy_lo = 0, agy_hi = 0;
    bool first = true;
    for (const auto& kv : chunks_) {
      const Chunk& c = kv.second;
      if (first) {
        agx_lo = agx_hi = c.gx;
        agy_lo = agy_hi = c.gy;
        first = false;
      } else {
        agx_lo = std::min(agx_lo, c.gx);
        agx_hi = std::max(agx_hi, c.gx);
        agy_lo = std::min(agy_lo, c.gy);
        agy_hi = std::max(agy_hi, c.gy);
      }
    }
    LOG(INFO) << "[chunk 校验] 地图世界范围 x[" << wx_lo << "," << wx_hi << "] y["
              << wy_lo << "," << wy_hi << "]，分辨率 " << lim.resolution()
              << "，cell " << cl.num_x_cells << "x" << cl.num_y_cells;
    LOG(INFO) << "[chunk 校验] 边界推算 chunk id gx[" << gx_lo << "," << gx_hi
              << "] gy[" << gy_lo << "," << gy_hi << "] → 满铺上限 " << span
              << " 个；实际生成 " << chunks_.size() << " 个，id gx[" << agx_lo
              << "," << agx_hi << "] gy[" << agy_lo << "," << agy_hi << "]";
    if (agx_lo > gx_lo || agx_hi < gx_hi || agy_lo > gy_lo || agy_hi < gy_hi) {
      LOG(WARNING) << "[chunk 校验] 实际 chunk id 范围未铺满边界推算范围 → 边界处可能漏 chunk";
    }
  }
}

void ChunkMap2D::LoadFromChunkSubmaps(
    std::vector<std::tuple<int, int, std::shared_ptr<mapping::Submap2D>>>
        chunk_submaps) {
  chunks_.clear();
  active_.clear();
  last_grid_set_ = false;
  for (auto& t : chunk_submaps) {
    const int gx = std::get<0>(t);
    const int gy = std::get<1>(t);
    auto submap = std::get<2>(t);
    if (!submap) continue;
    Chunk c;
    c.gx = gx;
    c.gy = gy;
    c.submap = std::move(submap);
    chunks_[Key(gx, gy)] = std::move(c);
  }
  LOG(INFO) << "ChunkMap2D: loaded " << chunks_.size()
            << " chunks directly from pbstream chunk submaps.";
}

bool ChunkMap2D::LoadOnPose(const transform::Rigid2d& pose) {
  int gx, gy;
  Pos2Grid(pose.translation().x(), pose.translation().y(), &gx, &gy);

  // 未跨格则不更新（频率控制，同 lightning LoadOnPose）。
  if (last_grid_set_ && gx == last_gx_ && gy == last_gy_) {
    return false;
  }
  last_gx_ = gx;
  last_gy_ = gy;
  last_grid_set_ = true;

  // 重算活跃集：load_radius 内全部激活；曾激活且仍在 unload_radius 内的保留（滞回）。
  // 用切比雪夫（方块）距离 → load_radius=R 即 (2R+1)×(2R+1) 方块（R=1 → 3×3=9，含中心），
  // 与 AssembleLocalSubmap 的方块选择一致。
  std::unordered_set<int64_t> new_active;
  for (const auto& kv : chunks_) {
    const Chunk& c = kv.second;
    const int cheb = std::max(std::abs(c.gx - gx), std::abs(c.gy - gy));
    if (cheb <= options_.load_radius) {
      new_active.insert(kv.first);
    } else if (cheb <= options_.unload_radius && active_.count(kv.first)) {
      new_active.insert(kv.first);
    }
  }
  active_.swap(new_active);
  return true;
}

std::vector<std::pair<int, int>> ChunkMap2D::ChunkIds() const {
  std::vector<std::pair<int, int>> ids;
  ids.reserve(chunks_.size());
  for (const auto& kv : chunks_) {
    ids.emplace_back(kv.second.gx, kv.second.gy);
  }
  return ids;
}

std::vector<std::pair<int, int>> ChunkMap2D::ActiveChunkIds() const {
  // 用当前所在 chunk（last_gx_/last_gy_，LoadOnPose 维护）为中心、load_radius 方块内已存在的 chunk。
  // 与 AssembleLocalSubmap 的选择一致；只读静态 chunks_ + 标量，避免读可变 active_ 的数据竞争。
  std::vector<std::pair<int, int>> ids;
  if (!last_grid_set_) return ids;
  const int cgx = last_gx_, cgy = last_gy_;
  const int r = options_.load_radius;
  for (int dx = -r; dx <= r; ++dx) {
    for (int dy = -r; dy <= r; ++dy) {
      const auto it = chunks_.find(Key(cgx + dx, cgy + dy));
      if (it != chunks_.end()) {
        ids.emplace_back(it->second.gx, it->second.gy);
      }
    }
  }
  return ids;
}

std::shared_ptr<mapping::Submap2D> ChunkMap2D::AssembleLocalSubmap(
    const transform::Rigid2d& origin) const {
  if (chunks_.empty()) {
    LOG(ERROR) << "ChunkMap2D::AssembleLocalSubmap: no chunks built.";
    return nullptr;
  }
  const double res =
      chunks_.begin()->second.submap->grid()->limits().resolution();
  const double L = options_.chunk_length;
  const double ox = origin.translation().x();
  const double oy = origin.translation().y();

  // ===== 1. 选出 origin 所在 chunk 的 load_radius（切比雪夫方块）内已存在的 chunk =====
  // R=load_radius → (2R+1)×(2R+1) 方块；R=1 即 3×3=9 个（含中心）。
  int cgx, cgy;
  Pos2Grid(ox, oy, &cgx, &cgy);
  const int r = options_.load_radius;
  std::vector<const Chunk*> sel;
  int gx_lo = 0, gx_hi = 0, gy_lo = 0, gy_hi = 0;
  bool first = true;
  for (int dx = -r; dx <= r; ++dx) {
    for (int dy = -r; dy <= r; ++dy) {
      // 方块：|dx|≤r 且 |dy|≤r（dx/dy 循环已限定），不做曼哈顿裁剪。
      const auto it = chunks_.find(Key(cgx + dx, cgy + dy));
      if (it == chunks_.end()) continue;
      sel.push_back(&it->second);
      const int gx = it->second.gx, gy = it->second.gy;
      if (first) {
        gx_lo = gx_hi = gx;
        gy_lo = gy_hi = gy;
        first = false;
      } else {
        gx_lo = std::min(gx_lo, gx);
        gx_hi = std::max(gx_hi, gx);
        gy_lo = std::min(gy_lo, gy);
        gy_hi = std::max(gy_hi, gy);
      }
    }
  }
  if (sel.empty()) {
    LOG(WARNING) << "ChunkMap2D::AssembleLocalSubmap: load_radius=" << r
                 << " 内无 chunk（pose 在地图外？）at (" << ox << "," << oy << ").";
    return nullptr;
  }

  // ===== 2. merged grid：覆盖所选 chunk 的 bbox，逐 chunk 拷贝已知 cell（无 box 二次裁剪）=====
  const int n = static_cast<int>(std::lround(L / res));
  const int nx = (gx_hi - gx_lo + 1) * n;
  const int ny = (gy_hi - gy_lo + 1) * n;
  const double half = 0.5 * L;
  mapping::MapLimits limits(
      res, Eigen::Vector2d(gx_hi * L + half, gy_hi * L + half),
      mapping::CellLimits(nx, ny));
  auto local_grid =
      std::make_unique<mapping::ProbabilityGrid>(limits, conversion_tables_);

  std::vector<sensor::RangefinderPoint> local_points;
  std::vector<float> local_intensities;
  bool keep_intensities = true;
  int latest_line_id = 0;
  std::map<int, mapping::MapLine2D> line_data;

  for (const Chunk* c : sel) {
    const auto& sm = c->submap;
    // grid：拷贝该 chunk 全部已知 cell（按世界坐标定位到 merged grid）。
    const auto* cg =
        dynamic_cast<const mapping::ProbabilityGrid*>(sm->grid());
    if (cg) {
      for (const Eigen::Array2i& li :
           mapping::XYIndexRangeIterator(cg->limits().cell_limits())) {
        if (!cg->IsKnown(li)) continue;
        const Eigen::Vector2f world = cg->limits().GetCellCenter(li);
        const Eigen::Array2i ti = limits.GetCellIndex(world);
        if (limits.Contains(ti)) {
          local_grid->SetProbability(ti, cg->GetProbability(li));
        }
      }
    }
    // 点云：整块纳入（不再按 box 裁剪）。
    if (const auto* cl = sm->cloud()) {
      const bool has_int = !cl->intensities().empty();
      const auto& pts = cl->points();
      for (size_t i = 0; i < pts.size(); ++i) {
        local_points.push_back(pts[i]);
        if (has_int) {
          local_intensities.push_back(cl->intensities()[i]);
        } else {
          keep_intensities = false;
        }
      }
    }
    // line：合并各 chunk 子集。
    if (const auto* lm = sm->line_map_2d_list()) {
      latest_line_id = std::max(latest_line_id, lm->latest_local_id());
      for (const auto& kv : lm->GetLineMapData2DList()) {
        line_data.insert_or_assign(kv.first, kv.second);
      }
    }
  }

  std::unique_ptr<sensor::PointCloud> local_cloud;
  if (keep_intensities && local_intensities.size() == local_points.size() &&
      !local_points.empty()) {
    local_cloud = std::make_unique<sensor::PointCloud>(
        std::move(local_points), std::move(local_intensities));
  } else {
    local_cloud =
        std::make_unique<sensor::PointCloud>(std::move(local_points));
  }

  // ===== 3. 组装 local submap 并重建 ikd-tree（用于匹配）=====
  auto new_submap = std::make_shared<mapping::Submap2D>(
      Eigen::Vector2f(static_cast<float>(ox), static_cast<float>(oy)),
      std::move(local_grid),
      nullptr,  // local submap 只用于匹配，不需要 conversion_tables
      std::move(local_cloud), std::make_unique<sensor::PointCloud>(),
      latest_line_id, std::move(line_data));
  new_submap->RebuildIkdTrees();
  new_submap->Finish();
  return new_submap;
}

bool ChunkMap2D::SaveToDir(const std::string& chunks_dir) const {
  namespace fs = std::filesystem;
  if (empty()) {
    LOG(WARNING) << "ChunkMap2D::SaveToDir: chunk map empty, skip.";
    return false;
  }
  std::error_code ec;
  fs::create_directories(chunks_dir, ec);
  if (ec) {
    LOG(ERROR) << "ChunkMap2D::SaveToDir: cannot create dir " << chunks_dir
               << ": " << ec.message();
    return false;
  }

  mapping::proto::SerializationHeader header;
  header.set_format_version(io::kMappingStateSerializationFormatVersion);

  // 每 chunk 一个 pbstream：直接序列化 chunk 自包含的 Submap2D（grid+点云+line）。
  int written = 0;
  double res = 0.0;
  for (const auto& kv : chunks_) {
    const Chunk& c = kv.second;
    if (!c.submap) continue;
    auto submap_proto = c.submap->ToProto(/*include_grid_data=*/true);
    if (c.submap->grid()) {
      res = c.submap->grid()->limits().resolution();
    }

    mapping::proto::SerializedData data;
    auto* chunk_proto = data.mutable_chunk_submap();
    chunk_proto->set_gx(c.gx);
    chunk_proto->set_gy(c.gy);
    chunk_proto->mutable_submap()->CopyFrom(submap_proto.submap_2d());

    const std::string file = chunks_dir + "/chunk_" + std::to_string(c.gx) +
                             "_" + std::to_string(c.gy) + ".pbstream";
    io::ProtoStreamWriter writer(file);
    writer.WriteProto(header);
    writer.WriteProto(data);
    if (!writer.Close()) {
      LOG(ERROR) << "ChunkMap2D::SaveToDir: failed writing " << file;
      return false;
    }
    ++written;
  }

  // chunk_index.pbstream（ChunkMapInfo：仅 chunk_length + num_chunks）。
  {
    (void)res;
    mapping::proto::SerializedData info_data;
    auto* info = info_data.mutable_chunk_map_info();
    info->set_chunk_length(static_cast<float>(options_.chunk_length));
    info->set_num_chunks(written);
    const std::string index_file = chunks_dir + "/chunk_index.pbstream";
    io::ProtoStreamWriter writer(index_file);
    writer.WriteProto(header);
    writer.WriteProto(info_data);
    if (!writer.Close()) {
      LOG(ERROR) << "ChunkMap2D::SaveToDir: failed writing " << index_file;
      return false;
    }
  }

  LOG(INFO) << "ChunkMap2D::SaveToDir: saved " << written
            << " chunk pbstreams to " << chunks_dir;
  return true;
}

mapping::proto::ChunkMapInfo ChunkMap2D::ToInfoProto() const {
  mapping::proto::ChunkMapInfo info;
  info.set_chunk_length(static_cast<float>(options_.chunk_length));
  info.set_num_chunks(static_cast<int>(chunks_.size()));
  return info;
}

std::vector<mapping::proto::ChunkSubmap2D> ChunkMap2D::ToChunkProtos() const {
  std::vector<mapping::proto::ChunkSubmap2D> protos;
  protos.reserve(chunks_.size());
  for (const auto& kv : chunks_) {
    const Chunk& c = kv.second;
    if (!c.submap) continue;
    // chunk 自包含 Submap2D，直接 ToProto（含 grid+点云+line）。
    auto submap_proto = c.submap->ToProto(/*include_grid_data=*/true);
    mapping::proto::ChunkSubmap2D chunk_proto;
    chunk_proto.set_gx(c.gx);
    chunk_proto.set_gy(c.gy);
    chunk_proto.mutable_submap()->CopyFrom(submap_proto.submap_2d());
    protos.push_back(std::move(chunk_proto));
  }
  return protos;
}

bool ChunkMap2D::SaveDebugDir(const std::string& debug_dir,
                             bool active_only) const {
  namespace fs = std::filesystem;
  if (empty()) {
    LOG(WARNING) << "ChunkMap2D::SaveDebugDir: chunk map empty, skip.";
    return false;
  }
  std::error_code ec;
  fs::create_directories(debug_dir, ec);
  if (ec) {
    LOG(ERROR) << "ChunkMap2D::SaveDebugDir: cannot create dir " << debug_dir
               << ": " << ec.message();
    return false;
  }

  // 每 chunk 直接用其自包含 Submap2D 输出 PCD（点云）+ PNG（grid 可视化）。
  // active_only 时只输出当前活跃（已加载）chunk（key 命中 active_ 集）。
  int written = 0;
  for (const auto& kv : chunks_) {
    if (active_only && active_.find(kv.first) == active_.end()) continue;
    const Chunk& c = kv.second;
    if (!c.submap) continue;
    const std::string stem =
        "chunk_" + std::to_string(c.gx) + "_" + std::to_string(c.gy);
    c.submap->SaveSubmapCloud(debug_dir, stem + ".pcd");
    if (c.submap->grid()) {
      // 在完整 chunk footprint 画布上渲染（每张图都是固定 chunk 大小的 tile，空/稀疏 chunk
      // 也是完整灰底 tile），而非 DrawAsImage 的已知 cell 包围盒（会得到空/可变小图）。
      const double res = c.submap->grid()->limits().resolution();
      const int n = static_cast<int>(std::lround(options_.chunk_length / res));
      const double half = 0.5 * options_.chunk_length;
      const double min_x = c.gx * options_.chunk_length - half;
      const double min_y = c.gy * options_.chunk_length - half;
      const mapping::MapLimits canvas(
          res, Eigen::Vector2d(min_x + n * res, min_y + n * res),
          mapping::CellLimits(n, n));
      c.submap->grid()->DrawAsImageOnCanvas(debug_dir, stem + ".png", canvas);
    }
    ++written;
  }

  // 全量模式额外输出"所有 chunk 合并的完整地图图"merged.png：把每个 chunk 的已知栅格按世界
  // 坐标拼到一张覆盖 chunk-id bbox 的大图，直接验证 chunk 能否还原整图（免手动拼 tile + 旋转约定）。
  if (!active_only && !chunks_.empty()) {
    const double res = chunks_.begin()->second.submap->grid()->limits().resolution();
    const double L = options_.chunk_length;
    int gx_lo, gx_hi, gy_lo, gy_hi;
    bool first = true;
    for (const auto& kv : chunks_) {
      const Chunk& c = kv.second;
      if (first) {
        gx_lo = gx_hi = c.gx;
        gy_lo = gy_hi = c.gy;
        first = false;
      } else {
        gx_lo = std::min(gx_lo, c.gx);
        gx_hi = std::max(gx_hi, c.gx);
        gy_lo = std::min(gy_lo, c.gy);
        gy_hi = std::max(gy_hi, c.gy);
      }
    }
    const int n = static_cast<int>(std::lround(L / res));
    const int nx = (gx_hi - gx_lo + 1) * n;
    const int ny = (gy_hi - gy_lo + 1) * n;
    const double half = 0.5 * L;
    const mapping::MapLimits big(res,
                                 Eigen::Vector2d(gx_hi * L + half, gy_hi * L + half),
                                 mapping::CellLimits(nx, ny));
    auto big_grid =
        std::make_unique<mapping::ProbabilityGrid>(big, conversion_tables_);
    for (const Eigen::Array2i& li :
         mapping::XYIndexRangeIterator(big.cell_limits())) {
      const Eigen::Vector2f world = big.GetCellCenter(li);
      int gx, gy;
      Pos2Grid(world.x(), world.y(), &gx, &gy);
      const auto it = chunks_.find(Key(gx, gy));
      if (it == chunks_.end()) continue;
      const auto* cg = dynamic_cast<const mapping::ProbabilityGrid*>(
          it->second.submap->grid());
      if (!cg) continue;
      const Eigen::Array2i gi = cg->limits().GetCellIndex(world);
      if (cg->limits().Contains(gi) && cg->IsKnown(gi)) {
        big_grid->SetProbability(li, cg->GetProbability(gi));
      }
    }
    // 用与各 chunk tile 相同的"北朝上"朝向渲染，保证 merged 与 tile 拼接结果一致。
    big_grid->DrawAsImageOnCanvas(debug_dir, "merged.png", big);
    LOG(INFO) << "[chunk 校验] 合并全图 merged.png：" << nx << "x" << ny
              << " cells，chunk id gx[" << gx_lo << "," << gx_hi << "] gy["
              << gy_lo << "," << gy_hi << "] → " << debug_dir << "merged.png";
  }

  LOG(INFO) << "ChunkMap2D::SaveDebugDir: saved " << written << " "
            << (active_only ? "active " : "") << "chunks (PCD+PNG) to "
            << debug_dir;
  return true;
}

}  // namespace localization
}  // namespace cartographer
