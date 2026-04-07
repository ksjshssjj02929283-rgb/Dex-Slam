//
// Created by xiang on 25-6-24.
//

#ifndef LIGHTNING_G2P5_GRID_DATA_H
#define LIGHTNING_G2P5_GRID_DATA_H

namespace lightning::g2p5 {

/// 2.5D 子地图定义
/// 子网格中的数据内容（Log-Odds 概率模型）
struct GridData {
    explicit GridData() : log_odds_(0.0f), height_(10000.0f) {}

    float log_odds_ = 0.0f;    // Log-Odds 概率值（0=未知，正=占据，负=自由）
    float height_ = 10000.0f;  // 相对地面的最低障碍高度，射线从上方穿过才能标记为自由
};

}  // namespace lightning::g2p5

#endif  // LIGHTNING_G2P5_GRID_DATA_H
