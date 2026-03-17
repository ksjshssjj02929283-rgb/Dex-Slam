//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_LOOP_CANDIDATE_H
#define LIGHTNING_LOOP_CANDIDATE_H

#include "common/eigen_types.h"

namespace lightning {

/**
 * 回环检测候选帧
 */
struct LoopCandidate {
    LoopCandidate() {}
    LoopCandidate(uint64_t id1, uint64_t id2) : idx1_(id1), idx2_(id2) {}

    uint64_t idx1_ = 0;
    uint64_t idx2_ = 0;
    SE3 Tij_;

    double ndt_score_ = 0.0;
    double icp_fitness_ = 1.0;  // GICP getFitnessScore()，越小越好，供 AddLoopFactor 构建噪声模型
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CANDIDATE_H
