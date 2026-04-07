//
// Created by xiang on 25-6-24.
//

#ifndef LIGHTNING_G2P5_SUBGRID_H
#define LIGHTNING_G2P5_SUBGRID_H

#include "common/std_types.h"
#include "core/maps/g2p5/g2p5_grid_data.h"

namespace lightning::g2p5 {

/// 子网格定义，坐标系与MapToGrid类似
class SubGrid {
   public:
    static inline constexpr int SUB_GRID_SIZE = 4;

    // Log-Odds 更新参数
    static constexpr float kLogOddsHit  =  0.85f;   // P(occ|hit)  ≈ 0.70
    static constexpr float kLogOddsMiss = -0.20f;   // P(occ|miss) ≈ 0.45（弱化，减少 miss 射线消除已确认墙面）
    static constexpr float kLogOddsMax  =  5.0f;    // 防止过度确定
    static constexpr float kLogOddsMin  = -5.0f;
    static constexpr float kLogOddsOcc  =  0.4f;    // 占据判定阈值（约 P=0.60，需 ≥1 次命中）
    static constexpr float kLogOddsFree = -0.4f;    // 自由空间判定阈值（需 ≥2 次 miss 才能变白）

    SubGrid(int num_x = 0, int num_y = 0);

    ~SubGrid();

    SubGrid(const SubGrid &other);
    SubGrid &operator=(const SubGrid &other);

    /// 设置某个网格的占据或者非占据（Log-Odds 更新）
    void SetGridHitPoint(bool hit, int sub_xi, int sub_yi, float height) {
        UL lock(data_mutex_);
        if (grid_data_ == nullptr) {
            MallocGrid();
        }

        int index = sub_xi + sub_yi * width_;
        GridData &cell = grid_data_[index];

        if (hit) {
            // 障碍物命中：增加 log_odds，更新最低高度
            cell.log_odds_ = std::min(kLogOddsMax, cell.log_odds_ + kLogOddsHit);
            if (height < cell.height_) {
                cell.height_ = height;
            }
        } else {
            // 自由空间：仅当射线从障碍上方穿过（含同高度），或格子尚未确定为占据时才减少 log_odds
            // 使用 <= 而非 <，使同高度 miss 射线也能参与更新（修复同高度命中不统计的问题）
            if (cell.log_odds_ < kLogOddsOcc || height <= cell.height_) {
                cell.log_odds_ = std::max(kLogOddsMin, cell.log_odds_ + kLogOddsMiss);
            }
        }
    }

    void RemoveCarNoise(int sub_xi, int sub_yi) {
        UL lock(data_mutex_);
        if (grid_data_ == nullptr) {
            MallocGrid();
        }
        int index = sub_xi + sub_yi * width_;
        grid_data_[index].log_odds_ = kLogOddsMin;
    }

    /// 判定数据是否为空
    bool IsEmpty() {
        UL lock(data_mutex_);
        return grid_data_ == nullptr;
    }

    /// 获取 log_odds 和障碍高度
    void GetLogOdds(int sx, int sy, float &log_odds, float &height) {
        UL lock(data_mutex_);
        GridData &d = grid_data_[sx + sy * width_];
        log_odds = d.log_odds_;
        height = d.height_;
    }

   private:
    // 分配这个grid的内存
    void MallocGrid();

    /// 子网格的大小
    static inline constexpr int width_ = (1 << SUB_GRID_SIZE);
    static inline constexpr int width_2_ = (1 << SUB_GRID_SIZE) * (1 << SUB_GRID_SIZE);

    std::mutex data_mutex_;
    GridData *grid_data_ = nullptr;
};

}  // namespace lightning::g2p5

#endif  // LIGHTNING_G2P5_SUBGRID_H
