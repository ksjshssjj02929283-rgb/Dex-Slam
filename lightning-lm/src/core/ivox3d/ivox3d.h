//
// Created by xiang on 2021/9/16.
//
#pragma once

#include <glog/logging.h>
#include <execution>
#include <list>
#include <thread>
#include <unordered_map>

#include "common/eigen_types.h"
#include "core/ivox3d/ivox3d_node.hpp"
#include "core/lightning_math.hpp"

namespace lightning {

enum class IVoxNodeType {
    DEFAULT,  // linear ivox
    PHC,      // phc ivox
};

/// traits for NodeType
template <IVoxNodeType node_type, typename PointT, int dim>
struct IVoxNodeTypeTraits {};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
    using NodeType = IVoxNode<PointT, dim>;
};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::PHC, PointT, dim> {
    using NodeType = IVoxNodePhc<PointT, dim>;
};

template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT, typename PointType = pcl::PointXYZ>
class IVox {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;
    using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using DistPoint = typename NodeType::DistPoint;

    enum class NearbyType {
        CENTER,  // center only
        NEARBY6,
        NEARBY18,
        NEARBY26,
    };

    struct Options {
        float resolution_ = 0.2;                        // ivox resolution
        float inv_resolution_ = 10.0;                   // inverse resolution
        NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
        std::size_t capacity_ = 1000000;                // capacity
    };

    /**
     * constructor
     * @param options  ivox options
     */
    explicit IVox(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0 / options_.resolution_;
        GenerateNearbyGrids();
    }

    /**
     * add points
     * @param points_to_add
     */
    void AddPoints(const PointVector& points_to_add);

    /// get nn
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt);

    /// get nn with condition
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_range = 5.0);

    /// get nn in cloud
    bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

    /// get number of points
    size_t NumPoints() const;

    /// get number of valid grids
    size_t NumValidGrids() const;

    /// get statistics of the points
    std::vector<float> StatGridPoints() const;

    // -------------------------------------------------------------------------
    //  Grid 操作接口
    // -------------------------------------------------------------------------

    /// 查询某世界坐标点所在 grid 是否存在
    bool HasGrid(const PtType& pt) const {
        KeyType key = Pos2Grid(math::ToEigen<float, dim>(pt));
        return grids_map_.find(key) != grids_map_.end();
    }

    /// 按点删除对应 grid（若不存在则返回 false）
    bool DeleteGrid(const PtType& pt) {
        KeyType key = Pos2Grid(math::ToEigen<float, dim>(pt));
        auto it = grids_map_.find(key);
        if (it == grids_map_.end()) return false;
        grids_cache_.erase(it->second);
        grids_map_.erase(it);
        return true;
    }

    /// 按 key 删除 grid（若不存在则返回 false）
    bool DeleteGrid(const KeyType& key) {
        auto it = grids_map_.find(key);
        if (it == grids_map_.end()) return false;
        grids_cache_.erase(it->second);
        grids_map_.erase(it);
        return true;
    }

    /**
     * @brief 删除距 center 超过 max_distance 的所有 grid
     * @param center     参考中心坐标（与点云同坐标系，float 3D）
     * @param max_distance  最大保留距离（米）
     * @return 删除的 grid 数量
     */
    size_t DeleteGridsFarFrom(const Eigen::Vector3f& center, float max_distance) {
        const float max_dist_sq = max_distance * max_distance;
        size_t removed = 0;
        for (auto it = grids_cache_.begin(); it != grids_cache_.end(); ) {
            // grid 中心坐标 = key * resolution
            Eigen::Vector3f grid_center = it->first.template cast<float>() * options_.resolution_;
            if ((grid_center - center).squaredNorm() > max_dist_sq) {
                grids_map_.erase(it->first);
                it = grids_cache_.erase(it);
                ++removed;
            } else {
                ++it;
            }
        }
        return removed;
    }

    /**
     * @brief 删除 x/y 落在指定 2D AABB 内的所有 grid（z 不限，适用于 chunk 级卸载）
     * @param min_xy  2D 包围盒最小角
     * @param max_xy  2D 包围盒最大角
     * @return 删除的 grid 数量
     */
    size_t DeleteGridsInAABB(const Eigen::Vector2f& min_xy, const Eigen::Vector2f& max_xy) {
        size_t removed = 0;
        for (auto it = grids_cache_.begin(); it != grids_cache_.end(); ) {
            Eigen::Vector3f gc = it->first.template cast<float>() * options_.resolution_;
            if (gc.x() >= min_xy.x() && gc.x() < max_xy.x() &&
                gc.y() >= min_xy.y() && gc.y() < max_xy.y()) {
                grids_map_.erase(it->first);
                it = grids_cache_.erase(it);
                ++removed;
            } else {
                ++it;
            }
        }
        return removed;
    }

   private:
    /// generate the nearby grids according to the given options
    void GenerateNearbyGrids();

    /// position to grid
    KeyType Pos2Grid(const PtType& pt) const;

    Options options_;
    std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator, math::hash_vec<dim>>
        grids_map_;                                        // voxel hash map
    std::list<std::pair<KeyType, NodeType>> grids_cache_;  // voxel cache
    std::vector<KeyType> nearby_grids_;                    // nearbys
};

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    std::vector<DistPoint> candidates;
    auto key = Pos2Grid(math::ToEigen<float, dim>(pt));
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            bool found = iter->second->second.NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        }
    });

    if (candidates.empty()) {
        return false;
    }

    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                                      double max_range) {
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    auto key = Pos2Grid(math::ToEigen<float, dim>(pt));

// #define INNER_TIMER
#ifdef INNER_TIMER
    static std::unordered_map<std::string, std::vector<int64_t>> stats;
    if (stats.empty()) {
        stats["knn"] = std::vector<int64_t>();
        stats["nth"] = std::vector<int64_t>();
    }
#endif

    for (const KeyType& delta : nearby_grids_) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
        }
    }

    if (candidates.empty()) {
        return false;
    }

#ifdef INNER_TIMER
    auto t1 = std::chrono::high_resolution_clock::now();
#endif

    if (candidates.size() <= max_num) {
    } else {
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

#ifdef INNER_TIMER
    auto t2 = std::chrono::high_resolution_clock::now();
    auto nth = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    stats["nth"].emplace_back(nth);

    constexpr int STAT_PERIOD = 100000;
    if (!stats["nth"].empty() && stats["nth"].size() % STAT_PERIOD == 0) {
        for (auto& it : stats) {
            const std::string& key = it.first;
            std::vector<int64_t>& stat = it.second;
            int64_t sum_ = std::accumulate(stat.begin(), stat.end(), 0);
            int64_t num_ = stat.size();
            stat.clear();
            std::cout << "inner_" << key << "(ns): sum=" << sum_ << " num=" << num_ << " ave=" << 1.0 * sum_ / num_
                      << " ave*n=" << 1.0 * sum_ / STAT_PERIOD << std::endl;
        }
    }
#endif

    closest_pt.clear();
    for (auto& it : candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
    return grids_map_.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumPoints() const {
    int ret = 0;
    for (auto& g : grids_map_) {
        ret += g.second->second.Size();
    }
    return ret;
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else {
        LOG(ERROR) << "Unknown nearby_type!";
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud) {
    std::vector<size_t> index(cloud.size());
    for (int i = 0; i < cloud.size(); ++i) {
        index[i] = i;
    }
    closest_cloud.resize(cloud.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&cloud, &closest_cloud, this](size_t idx) {
        PointType pt;
        if (GetClosestPoint(cloud[idx], pt)) {
            closest_cloud[idx] = pt;
        } else {
            closest_cloud[idx] = PointType();
        }
    });
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add) {
    if (points_to_add.empty()) return;

    // ------------------------------------------------------------------
    // Phase 1 (单线程): hash 查找 + 新体素创建 + LRU 更新 + capacity 淘汰
    //
    //   grids_map_ 和 grids_cache_ 是共享结构，必须串行访问。
    //   同时建立平坦映射表 node_pt_pairs：(NodeType*, PointType*)，
    //   按 NodeType* 分组供 Phase 2 并行插入。
    // ------------------------------------------------------------------

    // 平坦映射表：每个点对应它归属的 NodeType*（同 key 的点 NodeType* 相同）
    std::vector<std::pair<NodeType*, const PointType*>> node_pt_pairs;
    node_pt_pairs.reserve(points_to_add.size());

    for (const auto& pt : points_to_add) {
        auto key = Pos2Grid(math::ToEigen<float, dim>(pt));

        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) {
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;

            grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
            grids_map_.insert({key, grids_cache_.begin()});

            if (grids_map_.size() >= options_.capacity_) {
                grids_map_.erase(grids_cache_.back().first);
                grids_cache_.pop_back();
            }

            node_pt_pairs.emplace_back(&grids_cache_.front().second, &pt);
        } else {
            // LRU: 将已有体素移到 cache 头部
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map_[key] = grids_cache_.begin();

            node_pt_pairs.emplace_back(&iter->second->second, &pt);
        }
    }

    // ------------------------------------------------------------------
    // Phase 2 (并行): 按 NodeType* 分组，对每组并行调用 InsertPoint
    //
    //   先按 NodeType* 排序，使同一体素的点连续（避免 unordered_map 分配）；
    //   再用 par_unseq 并行处理各分组——不同体素的 NodeType 完全独立，
    //   InsertPoint 只写 this->points_，无竞争。
    // ------------------------------------------------------------------

    // 按 NodeType* 排序，使同一体素的点连续
    std::sort(node_pt_pairs.begin(), node_pt_pairs.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    // 收集各组的起始位置
    std::vector<size_t> group_starts;
    group_starts.reserve(32);
    group_starts.push_back(0);
    for (size_t i = 1; i < node_pt_pairs.size(); ++i) {
        if (node_pt_pairs[i].first != node_pt_pairs[i - 1].first) {
            group_starts.push_back(i);
        }
    }

    // 并行：每个分组内串行插入（同体素多点通常极少，串行可接受）
    std::for_each(std::execution::par_unseq, group_starts.begin(), group_starts.end(),
                  [&](const size_t start) {
                      NodeType* node = node_pt_pairs[start].first;
                      size_t end = start;
                      while (end < node_pt_pairs.size() && node_pt_pairs[end].first == node) {
                          node->InsertPoint(*node_pt_pairs[end].second);
                          ++end;
                      }
                  });
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().round().template cast<int>();
}

template <int dim, IVoxNodeType node_type, typename PointType>
std::vector<float> IVox<dim, node_type, PointType>::StatGridPoints() const {
    int num = grids_cache_.size(), valid_num = 0, max = 0, min = 100000000;
    int sum = 0, sum_square = 0;
    for (auto& it : grids_cache_) {
        int s = it.second.Size();
        valid_num += s > 0;
        max = s > max ? s : max;
        min = s < min ? s : min;
        sum += s;
        sum_square += s * s;
    }
    float ave = float(sum) / num;
    float stddev = num > 1 ? sqrt((float(sum_square) - num * ave * ave) / (num - 1)) : 0;
    return std::vector<float>{float(valid_num), ave, float(max), float(min), stddev};
}

}  // namespace lightning
