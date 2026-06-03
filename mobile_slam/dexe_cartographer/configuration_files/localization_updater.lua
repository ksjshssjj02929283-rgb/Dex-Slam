-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "pose_graph.lua"

LOCALIZATION_UPDATER = {
  num_background_threads = 4,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
  -- chunk 化参数（定位侧据此建 ChunkMap2D::Options）。就地定义，不再共享
  -- chunk_map_options.lua（消除跨文件 include 与二次 include FATAL 风险）。
  chunk_map_options = {
    -- use_chunk_map = false,   -- 是否启用 chunk 化（保存整合 + 定位运行时拼装）
    use_chunk_map = true,   -- 是否启用 chunk 化（保存整合 + 定位运行时拼装）
    chunk_length = 20.,     -- 单 chunk 边长（米）
    -- 加载/卸载半径为切比雪夫（方块）半径：R → (2R+1)×(2R+1) 方块。
    load_radius = 1,        -- 加载半径（方块；1 → local map = 3×3 = 9 个 chunk，含中心）
    unload_radius = 2,      -- 卸载半径（> load_radius 形成滞回）
  },
}
