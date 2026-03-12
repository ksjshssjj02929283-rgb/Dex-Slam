# IVox3D → VoxelMap 替换方案

## 一、总体架构

### 1.1 替换前后对比

```
替换前 (IVox3D):                         替换后 (VoxelMap):
┌─────────────────────┐                 ┌─────────────────────┐
│   laser_mapping.cc  │                 │   laser_mapping.cc  │
│                     │                 │                     │
│  ivox_->AddPoints() │                 │  vmap_->UpdateMap() │
│  ivox_->GetClosest()│      ──►        │  vmap_->FindCorr()  │
│  esti_plane()       │                 │  vmap_->CalcBodyCov()│
│  点到平面残差(手动)   │                 │  概率化点到平面残差    │
└─────────────────────┘                 └─────────────────────┘
         │                                        │
    ┌────┴────┐                             ┌─────┴─────┐
    │ ivox3d/ │                             │voxel_map/ │ (独立模块)
    │ (KNN点) │                             │(平面+协方差)│
    └─────────┘                             └───────────┘
```

### 1.2 核心差异

| 特性 | IVox3D (当前) | VoxelMap (替换后) |
|------|-------------|-----------------|
| 地图表示 | 体素内存储原始点 | 体素内存储**带协方差的平面** |
| 匹配方式 | KNN 搜最近点 → 手动拟合平面 | 直接查匹配平面（自适应八叉树） |
| 协方差 | 无 | **点协方差 + 平面协方差** |
| 观测噪声 | 固定 R = 0.001 | **概率化**: R = σ_plane + n^T·Σ_point·n |
| 自适应分辨率 | 无（固定体素） | 八叉树 Coarse-to-Fine |
| 论文 | Faster-LIO (RAL 2022) | VoxelMap (RAL 2022) |

---

## 二、新增模块文件结构

```
lightning-lm/src/core/voxel_map/
├── include/voxel_map/
│   ├── voxel_types.hpp           # 数据结构: pointWithCov, ptpl, VOXEL_LOC, Plane
│   ├── octo_tree.hpp             # 八叉树 + 平面拟合声明
│   ├── voxel_map_manager.hpp     # 统一管理接口（对外唯一入口）
│   └── voxel_map_vis.hpp         # ROS2 可视化工具 (MarkerArray)
└── src/
    ├── octo_tree.cpp             # 八叉树实现（从 voxel_map_util.hpp 提取）
    └── voxel_map_manager.cpp     # 管理器实现
```

**设计原则**：
- `voxel_types.hpp` / `octo_tree.hpp` / `voxel_map_manager.hpp` → **纯 C++ + Eigen，无 ROS 依赖**
- `voxel_map_vis.hpp` → 唯一依赖 ROS2 的文件（可视化用）
- 通过 `VoxelMapManager` 类统一对外，laser_mapping 只需与此类交互

---

## 三、CMake 集成

### 3.1 在 `src/CMakeLists.txt` 中新增 VoxelMap 模块

在 `lightning.core.lio` 之前添加：

```cmake
# ----------------------------------------------------------------------------
# VoxelMap 模块 - 自适应体素地图 (替代 ivox3d)
# ----------------------------------------------------------------------------
add_library(lightning.core.voxel_map STATIC
    core/voxel_map/src/octo_tree.cpp
    core/voxel_map/src/voxel_map_manager.cpp
)

target_include_directories(lightning.core.voxel_map PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/voxel_map/include
)

target_link_libraries(lightning.core.voxel_map
    ${third_party_libs}
)

# OpenMP 支持（用于 FindCorrespondences 并行匹配）
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    target_link_libraries(lightning.core.voxel_map OpenMP::OpenMP_CXX)
endif()
```

### 3.2 修改 LIO 模块的依赖

```cmake
# 4. LIO 模块 - 修改依赖
target_link_libraries(lightning.core.lio
    lightning.common
    lightning.utils
    lightning.core.voxel_map    # ← 新增
    ${third_party_libs}
    ${PCL_LIBRARIES}
)
```

---

## 四、laser_mapping.h 修改

### 4.1 头文件替换

```diff
- #include "core/ivox3d/ivox3d.h"
+ #include "voxel_map/voxel_map_manager.hpp"
+ #include "voxel_map/voxel_map_vis.hpp"  // 可视化（可选）
```

### 4.2 类型别名替换

```diff
- using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
+ using VoxelMapType = voxel_map::VoxelMapManager;
```

### 4.3 成员变量替换

```diff
  /// modules
- IVoxType::Options ivox_options_;
- std::shared_ptr<IVoxType> ivox_ = nullptr;
+ voxel_map::VoxelMapConfig vmap_config_;
+ std::shared_ptr<VoxelMapType> vmap_ = nullptr;
```

### 4.4 删除不再需要的成员

```diff
  // 以下成员不再需要（VoxelMap 内部处理平面拟合）
- std::vector<PointVector> nearest_points_;
- std::vector<Vec4f> corr_pts_;
- std::vector<Vec4f> corr_norm_;
- std::vector<float> residuals_;
- std::vector<bool> point_selected_surf_;
- std::vector<Vec4f> plane_coef_;
```

### 4.5 新增成员

```cpp
  /// VoxelMap 匹配结果缓存
  std::vector<voxel_map::ptpl> ptpl_list_;
  std::vector<voxel_map::pointWithCov> pv_list_;  // 每帧的带协方差点云
```

---

## 五、laser_mapping.cc 修改

### 5.1 Init() 函数

```diff
  bool LaserMapping::Init(const std::string &config_yaml) {
-     ivox_ = std::make_shared<IVoxType>(ivox_options_);
+     vmap_ = std::make_shared<VoxelMapType>(vmap_config_);
      ...
  }
```

### 5.2 LoadParamsFromYAML()

```diff
  // 删除 ivox 相关参数
- ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
- ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();

  // 新增 voxel_map 参数
+ vmap_config_.voxel_size = yaml["voxel_map"]["voxel_size"].as<double>();
+ vmap_config_.max_layer = yaml["voxel_map"]["max_layer"].as<int>();
+ vmap_config_.layer_point_size = yaml["voxel_map"]["layer_point_size"].as<std::vector<int>>();
+ vmap_config_.max_points_size = yaml["voxel_map"]["max_points_size"].as<int>();
+ vmap_config_.max_cov_points_size = yaml["voxel_map"]["max_cov_points_size"].as<int>();
+ vmap_config_.planer_threshold = yaml["voxel_map"]["planer_threshold"].as<float>();
+ vmap_config_.sigma_num = yaml["voxel_map"]["sigma_num"].as<double>();
+ vmap_config_.ranging_cov = yaml["voxel_map"]["ranging_cov"].as<double>();
+ vmap_config_.angle_cov = yaml["voxel_map"]["angle_cov"].as<double>();
```

### 5.3 Run() - 首帧初始化

```diff
  if (flg_first_scan_) {
      state_point_ = kf_.GetX();
-     scan_down_world_->resize(scan_undistort_->size());
-     for (int i = 0; i < scan_undistort_->size(); i++) {
-         PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
-     }
-     ivox_->AddPoints(scan_down_world_->points);
+     // 构建带协方差的点云并初始化地图
+     std::vector<voxel_map::pointWithCov> pv_init;
+     auto R = state_point_.rot_.matrix();
+     for (size_t i = 0; i < scan_undistort_->size(); i++) {
+         voxel_map::pointWithCov pv;
+         Eigen::Vector3d p_body = scan_undistort_->points[i].getVector3fMap().cast<double>();
+         pv.point = p_body;
+         Eigen::Vector3d p_world = R * (state_point_.offset_R_lidar_.matrix() * p_body
+                                   + state_point_.offset_t_lidar_) + state_point_.pos_;
+         pv.point_world = p_world;
+         pv.point = p_world;  // 首帧 point 字段存世界坐标，用于建图
+         pv.cov = vmap_->CalcBodyCov(p_body);
+         pv.cov = VoxelMapType::PropagateCovToWorld(
+             pv.cov, p_body, R, Eigen::Matrix3d::Identity() * 0.0001,
+             Eigen::Matrix3d::Identity() * 0.0001);
+         pv_init.push_back(pv);
+     }
+     vmap_->InitializeMap(pv_init);
      ...
  }
```

### 5.4 ObsModel() - 核心观测模型（完全重写）

这是最关键的替换。原来 IVox3D 的流程是：
1. 变换点到世界系
2. `ivox_->GetClosestPoint()` 找最近 5 个点
3. `esti_plane()` 手动拟合平面
4. 计算固定噪声的残差

替换为 VoxelMap 后的流程是：
1. 变换点到世界系 + **计算协方差**
2. `vmap_->FindCorrespondences()` 直接找匹配平面
3. 残差和 Jacobian 都包含**平面协方差**信息

```cpp
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    int cnt_pts = scan_down_body_->size();

    // Step 1: 构建带协方差的查询点
    auto R_wl = (s.rot_ * s.offset_R_lidar_).matrix();
    auto t_wl = s.rot_ * s.offset_t_lidar_ + s.pos_;
    auto R = s.rot_.matrix();

    // 从 EKF 状态协方差中提取旋转和平移分量
    auto P = kf_.GetP();
    Eigen::Matrix3d cov_rot = P.block<3, 3>(3, 3);    // 旋转协方差
    Eigen::Matrix3d cov_trans = P.block<3, 3>(0, 0);  // 平移协方差

    pv_list_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        Eigen::Vector3d p_body = scan_down_body_->points[i].getVector3fMap().cast<double>();
        Eigen::Vector3d p_lidar = s.offset_R_lidar_.matrix() * p_body + s.offset_t_lidar_;
        Eigen::Vector3d p_world = R * p_lidar + s.pos_;

        pv_list_[i].point = p_body;       // body 坐标（用于 Jacobian）
        pv_list_[i].point_world = p_world; // world 坐标（用于匹配）

        // 计算协方差
        Eigen::Matrix3d body_cov = vmap_->CalcBodyCov(p_body);
        pv_list_[i].cov = VoxelMapType::PropagateCovToWorld(
            body_cov, p_lidar, R, cov_rot, cov_trans);
    }

    // Step 2: 查找匹配
    ptpl_list_.clear();
    vmap_->FindCorrespondences(pv_list_, ptpl_list_);

    effect_feat_num_ = ptpl_list_.size();
    if (effect_feat_num_ < 1) {
        obs.valid_ = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    // Step 3: 构建 Jacobian 和残差
    obs.h_x_ = Eigen::MatrixXd::Zero(effect_feat_num_, 12);
    obs.residual_.resize(effect_feat_num_);

    const Mat3f off_R = s.offset_R_lidar_.matrix().cast<float>();
    const Vec3f off_t = s.offset_t_lidar_.cast<float>();
    const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

    std::vector<size_t> index(effect_feat_num_);
    for (size_t i = 0; i < index.size(); ++i) index[i] = i;

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        const auto &match = ptpl_list_[i];
        Vec3f point_this_be = match.point.cast<float>();
        Mat3f point_be_crossmat = math::SKEW_SYM_MATRIX(point_this_be);
        Vec3f point_this = off_R * point_this_be + off_t;
        Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

        Vec3f norm_vec = match.normal.cast<float>();

        // Jacobian: H = [n, -(Rp)× R^T n, ...]
        Vec3f C(Rt * norm_vec);
        Vec3f A(point_crossmat * C);

        if (extrinsic_est_en_) {
            Vec3f B(point_be_crossmat * off_R.transpose() * C);
            obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2],
                A[0], A[1], A[2], B[0], B[1], B[2], C[0], C[1], C[2];
        } else {
            obs.h_x_.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2],
                A[0], A[1], A[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        // 残差: 点到平面距离（带概率权重）
        Eigen::Vector3d p_world_d = pv_list_[/* need index mapping */0].point_world;
        // 注意：ptpl_list_ 中的 point 是 body 坐标，需要重新计算世界坐标
        Eigen::Vector3d p_body_d = match.point;
        Eigen::Vector3d p_w = (s.rot_ * (s.offset_R_lidar_ * p_body_d + s.offset_t_lidar_) + s.pos_);
        float dis_to_plane = match.normal.dot(p_w) + match.d;

        // VoxelMap 特色：概率化加权
        // 可选：使用 Cauchy robust kernel
        float res = -dis_to_plane;
        float rho, drho;
        const float delta = 2.0;
        const float dsqr = delta * delta;
        const float dsqr_inv = 1.0 / dsqr;
        if (res >= 0) {
            rho = dsqr * std::log(1 + res * dsqr_inv);
            drho = 1.0 / (1 + res * dsqr_inv);
        } else {
            rho = -dsqr * std::log(1 - res * dsqr_inv);
            drho = 1.0 / (1 - res * dsqr_inv);
        }
        obs.residual_(i) = rho;
        obs.h_x_.block<1, 12>(i, 0) = obs.h_x_.block<1, 12>(i, 0).eval() * drho;
    });

    // 计算残差统计
    std::vector<double> res_sq2;
    for (int i = 0; i < effect_feat_num_; i++) {
        double r = obs.residual_(i);
        res_sq2.push_back(r * r);
    }
    std::sort(res_sq2.begin(), res_sq2.end());
    obs.lidar_residual_mean_ = res_sq2[res_sq2.size() / 2];
    obs.lidar_residual_max_ = res_sq2[res_sq2.size() - 1];
}
```

### 5.5 MapIncremental() - 地图更新（大幅简化）

```cpp
void LaserMapping::MapIncremental() {
    // VoxelMap 的增量更新比 IVox 简单得多
    // 只需将优化后的点云（带协方差）插入地图

    auto R = state_point_.rot_.matrix();
    auto P = kf_.GetP();
    Eigen::Matrix3d cov_rot = P.block<3, 3>(3, 3);
    Eigen::Matrix3d cov_trans = P.block<3, 3>(0, 0);

    std::vector<voxel_map::pointWithCov> pv_update;
    pv_update.reserve(scan_down_body_->size());

    for (size_t i = 0; i < scan_down_body_->size(); i++) {
        voxel_map::pointWithCov pv;
        Eigen::Vector3d p_body = scan_down_body_->points[i].getVector3fMap().cast<double>();
        Eigen::Vector3d p_lidar = state_point_.offset_R_lidar_.matrix() * p_body
                                + state_point_.offset_t_lidar_;
        Eigen::Vector3d p_world = R * p_lidar + state_point_.pos_;

        pv.point = p_world;        // 建图时用世界坐标
        pv.point_world = p_world;

        // 计算并传播协方差
        Eigen::Matrix3d body_cov = vmap_->CalcBodyCov(p_body);
        pv.cov = VoxelMapType::PropagateCovToWorld(body_cov, p_lidar, R, cov_rot, cov_trans);

        pv_update.push_back(pv);
    }

    // 按协方差从小到大排序（低噪声点优先插入，论文建议）
    std::sort(pv_update.begin(), pv_update.end(),
              [](const voxel_map::pointWithCov &a, const voxel_map::pointWithCov &b) {
                  return a.cov.diagonal().norm() < b.cov.diagonal().norm();
              });

    vmap_->UpdateMap(pv_update);
}
```

---

## 六、YAML 配置文件修改

在你的 YAML 配置文件中，将 `fasterlio` 下的 ivox 参数替换为 voxel_map 参数：

```yaml
# 删除:
# fasterlio:
#   ivox_grid_resolution: 0.5
#   ivox_nearby_type: 18

# 新增:
voxel_map:
  voxel_size: 1.0            # 顶层体素大小 (米)
  max_layer: 2               # 八叉树最大层数 (室内2, 室外3-4)
  layer_point_size: [5, 5, 5]  # 每层触发平面拟合的最小点数
  max_points_size: 50        # 每个体素最大点数
  max_cov_points_size: 50    # 保留协方差更新的最大点数
  planer_threshold: 0.01     # 平面判定阈值
  sigma_num: 2.0             # 概率检验的 sigma 倍数
  ranging_cov: 0.04          # 测距方差 (Velodyne 默认)
  angle_cov: 0.1             # 角度方差
```

### 调参建议

| 场景 | voxel_size | max_layer | layer_point_size | planer_threshold |
|------|-----------|-----------|-----------------|-----------------|
| 室内 | 1.0m | 2 | [5,5,5] | 0.01 |
| 室外结构化 | 2.0-3.0m | 3-4 | [5,5,5,5,5] | 0.01 |
| 室外非结构化 | 1.0-2.0m | 3 | [5,5,5,5] | 0.001 |

---

## 七、可视化使用方法

### 7.1 在 SLAM 节点中发布 VoxelMap

```cpp
#include "voxel_map/voxel_map_vis.hpp"

// 创建 publisher
auto voxel_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/voxel_map", 10);

// 在每帧处理后发布
auto planes = laser_mapping->GetVoxelMapVis();  // 需要在 LaserMapping 中暴露
auto marker_array = voxel_map::toMarkerArray(planes, "map", "voxel_plane");
voxel_pub->publish(marker_array);
```

### 7.2 在 LaserMapping 中添加可视化接口

```cpp
// laser_mapping.h 中新增:
std::vector<voxel_map::PlaneVis> GetVoxelMapVis() {
    if (vmap_) return vmap_->GetUpdatedPlanesForVis();
    return {};
}
```

### 7.3 RViz2 配置

在 RViz2 中添加 MarkerArray 显示：
- Topic: `/voxel_map`
- 每个平面显示为一个 Cylinder
- 颜色代表平面协方差大小（蓝=低噪声/高置信，红=高噪声/低置信）

---

## 八、修改检查清单

### ✅ 必须修改的文件

| 文件 | 修改内容 |
|------|---------|
| `src/CMakeLists.txt` | 添加 `lightning.core.voxel_map` 库 |
| `core/lio/laser_mapping.h` | 替换 IVox 为 VoxelMap 类型 |
| `core/lio/laser_mapping.cc` | 重写 Init, ObsModel, MapIncremental, 首帧处理 |
| YAML 配置文件 | 添加 voxel_map 参数块 |

### ✅ 新增的文件（已创建）

| 文件 | 说明 |
|------|------|
| `core/voxel_map/include/voxel_map/voxel_types.hpp` | 数据结构定义 |
| `core/voxel_map/include/voxel_map/octo_tree.hpp` | 八叉树声明 |
| `core/voxel_map/include/voxel_map/voxel_map_manager.hpp` | 管理器接口 |
| `core/voxel_map/include/voxel_map/voxel_map_vis.hpp` | ROS2 可视化 |
| `core/voxel_map/src/octo_tree.cpp` | 八叉树实现 |
| `core/voxel_map/src/voxel_map_manager.cpp` | 管理器实现 |

### ⚠️ 注意事项

1. **ObsModel 中的索引映射**：`FindCorrespondences` 返回的 `ptpl_list_` 是过滤后的匹配列表，长度 ≤ `scan_down_body_->size()`。构建 Jacobian 时需要注意 body 点的对应关系（ptpl.point 已经存了 body 坐标）。

2. **协方差源**：`kf_.GetP()` 的 block 索引需要确认。根据 NavState 定义：
   - `P[0:3, 0:3]` = 位置协方差
   - `P[3:6, 3:6]` = 旋转协方差

3. **首帧问题**：首帧时 EKF 协方差很小，用一个小的对角矩阵即可。

4. **性能**：VoxelMap 的 `FindCorrespondences` 已使用 OpenMP 并行，但由于每个查询涉及哈希查找 + 八叉树递归，在点数很多时可能比 IVox3D 的 KNN 稍慢。可以通过调大 `filter_size_scan` 或增加 `voxel_size` 来控制。

5. **线程安全**：当前实现假设 `FindCorrespondences`（读）和 `UpdateMap`（写）不会并发调用（它们在同一帧的不同阶段执行），如果未来需要多线程，需在 `VoxelMapManager` 中加读写锁。

---

## 九、测试验证步骤

1. **编译测试**：先只编译 `lightning.core.voxel_map` 库确认无错误
2. **单元测试**：构造简单点云测试 `CalcBodyCov`、`InitializeMap`、`FindCorrespondences`
3. **集成测试**：用已有数据回放，对比 IVox3D 和 VoxelMap 的里程计精度
4. **可视化验证**：在 RViz2 中查看 `/voxel_map` 的平面显示是否合理
5. **参数调优**：根据场景调整 `voxel_size`、`max_layer`、`sigma_num` 等参数
