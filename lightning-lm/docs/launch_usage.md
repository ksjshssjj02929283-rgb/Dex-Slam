# Lightning-LM Launch 文件使用说明

## 快速开始

### 1. 在线建图 (SLAM)

```bash
# 编译并 source 工作空间
cd ~/dexforce/0_slam/add_slam_3d/dexe_mobile_slam
colcon build --packages-select lightning
source install/setup.bash

# 启动 SLAM + RViz (使用默认配置)
ros2 launch lightning slam_online.launch.py

# 使用自定义配置文件
ros2 launch lightning slam_online.launch.py \
    config_file:=/path/to/your/config.yaml

# 只启动 SLAM,不启动 RViz
ros2 launch lightning slam_online.launch.py use_rviz:=false

# 使用自定义 RViz 配置
ros2 launch lightning slam_online.launch.py \
    rviz_config:=/path/to/your.rviz
```

### 2. 在线定位 (Localization)

```bash
# 启动定位 + RViz (使用默认配置)
ros2 launch lightning loc_online.launch.py

# 使用自定义配置文件
ros2 launch lightning loc_online.launch.py \
    config_file:=/path/to/your/config.yaml

# 只启动定位,不启动 RViz
ros2 launch lightning loc_online.launch.py use_rviz:=false

# 使用自定义 RViz 配置
ros2 launch lightning loc_online.launch.py \
    rviz_config:=/path/to/your.rviz
```

## 配置文件位置说明

安装后的配置文件位置：
```
install/lightning/share/lightning/
├── config/
│   ├── default_livox.yaml   # Livox 激光雷达配置（默认）
│   ├── default_nclt.yaml    # NCLT 数据集配置
│   ├── default_utbm.yaml    # UTBM 数据集配置
│   ├── default_vbr.yaml     # VBR 数据集配置
│   └── default.yaml         # 通用配置
├── rviz/
│   └── lightning.rviz       # RViz 可视化配置
└── launch/
    ├── slam_online.launch.py
    └── loc_online.launch.py
```

**注意**: Launch 文件会自动从 `install/lightning/share/lightning/` 目录读取配置，无需手动指定完整路径。

## Launch 参数说明

### slam_online.launch.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_file` | `config/default_livox.yaml` | SLAM 配置文件的完整路径 |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `rviz_config` | `config/lightning.rviz` | RViz 配置文件的完整路径 |

### loc_online.launch.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_file` | `config/default_livox.yaml` | 定位配置文件的完整路径 |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `rviz_config` | `config/lightning.rviz` | RViz 配置文件的完整路径 |

## 查看可用的 Launch 文件

```bash
# 列出所有 lightning 包的 launch 文件
ros2 launch lightning -s

# 查看 launch 文件的参数说明
ros2 launch lightning slam_online.launch.py -s
ros2 launch lightning loc_online.launch.py -s
```

## 故障排查

### 问题: launch 文件找不到

```bash
# 确保已经 source 工作空间
cd ~/dexforce/0_slam/add_slam_3d/dexe_mobile_slam
source install/setup.bash

# 检查 launch 文件是否安装
ls install/lightning/share/lightning/launch/
```

### 问题: 配置文件路径错误

```bash
# 使用绝对路径
ros2 launch lightning slam_online.launch.py \
    config_file:=/home/fengbro/dexforce/0_slam/add_slam_3d/dexe_mobile_slam/lightning-lm/config/default_livox.yaml
```

### 问题: RViz 启动失败

```bash
# 单独测试 RViz
rviz2 -d ~/dexforce/0_slam/add_slam_3d/dexe_mobile_slam/lightning-lm/config/lightning.rviz

# 或者禁用 RViz 自动启动
ros2 launch lightning slam_online.launch.py use_rviz:=false
```

## 与原命令的对比

### 原始命令

```bash
# 建图
ros2 run lightning run_slam_online --config '/home/fengbro/dexforce/0_slam/add_slam_3d/dexe_mobile_slam/lightning-lm/config/default_livox.yaml'

# 定位
ros2 run lightning run_loc_online --config '/home/fengbro/dexforce/0_slam/add_slam_3d/dexe_mobile_slam/lightning-lm/config/default_livox.yaml'
```

### 使用 Launch (推荐)

```bash
# 建图 (自动启动 RViz)
ros2 launch lightning slam_online.launch.py

# 定位 (自动启动 RViz)
ros2 launch lightning loc_online.launch.py
```

## 优势

使用 Launch 文件的优势:
1. **一键启动**: 自动启动 SLAM/定位 + RViz
2. **参数管理**: 统一管理配置文件路径
3. **易于扩展**: 可以轻松添加其他节点
4. **ROS2 标准**: 符合 ROS2 最佳实践
5. **灵活配置**: 通过参数控制启动行为

## 注意事项

1. 确保在正确的工作空间目录下 source
2. 配置文件路径使用绝对路径更可靠
3. RViz 配置文件需要与话题名称匹配
4. 首次使用建议先测试默认配置
