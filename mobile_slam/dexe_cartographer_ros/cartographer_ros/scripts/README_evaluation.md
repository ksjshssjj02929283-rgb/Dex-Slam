# Static Localization Evaluation Script

## 功能说明

该脚本用于评估静态定位结果的质量，通过对比原始Cartographer定位位姿和robot_localization滤波后的位姿。

## 主要功能

1. **数据收集**
   - 订阅原始定位结果 (`/cartographer/loc_pose`)
   - 订阅滤波后的定位结果 (`/odometry/filtered`)
   - 时间同步两个topic的数据（容差0.1秒）
   - 收集指定数量的同步位姿样本（默认100个）

2. **评估指标**

   ### 位置波动指标
   - **标准差 (STD)**: 计算 x, y, θ 的标准差 (σ_x, σ_y, σ_θ)
   - **均方根误差 (RMSE)**: 相对于均值位置的波动
   - **最大偏移量**: 最大的位置偏差

   ### 稳定性评分
   - **抖动频率**: 统计位姿变化超过阈值的比例
   - **3σ原则**: 计算落在 ±3σ 范围外的异常点比例

3. **可视化**
   - 散点图显示所有位姿
   - 黑色箭头表示朝向（自动调整长度）
   - 红色叉号标记均值位置
   - 1σ和3σ置信椭圆
   - 图上显示主要评估指标

## 使用方法

### 1. 基本使用

```bash
# 确保脚本有执行权限
chmod +x evaluate_static_localization.py

# 运行脚本
ros2 run mobile_slam evaluate_static_localization.py
```

### 2. 自定义参数

```bash
ros2 run mobile_slam evaluate_static_localization.py \
    --ros-args \
    -p loc_pose_topic:=/cartographer/loc_pose \
    -p filtered_odom_topic:=/odometry/filtered \
    -p buffer_size:=100 \
    -p position_threshold:=0.01 \
    -p orientation_threshold:=0.05
```

### 3. 参数说明

- `loc_pose_topic`: 原始定位位姿topic（默认: `/cartographer/loc_pose`）
- `filtered_odom_topic`: 滤波后的里程计topic（默认: `/odometry/filtered`）
- `buffer_size`: 采集的样本数量（默认: 100）
- `position_threshold`: 位置抖动阈值，单位米（默认: 0.01）
- `orientation_threshold`: 姿态抖动阈值，单位弧度（默认: 0.05）

## 使用场景

### 典型工作流程

1. **启动定位系统**
   ```bash
   # 启动Cartographer定位
   ros2 launch mobile_slam localization.launch.py
   
   # 启动robot_localization
   ros2 launch robot_localization ekf.launch.py
   ```

2. **确保机器人静止**
   - 将机器人放置在固定位置
   - 确保不要移动机器人

3. **运行评估脚本**
   ```bash
   ros2 run mobile_slam evaluate_static_localization.py
   ```

4. **等待数据收集**
   - 脚本会自动收集100个同步的位姿样本
   - 进度会实时显示在终端

5. **查看结果**
   - 终端显示详细的评估指标
   - 自动生成三个文件，保存在 `~/.dexe/localization/localization_assessment/`:
     - `metrics_YYYYMMDD_HHMMSS.txt`: 文本格式评估报告
     - `localization_evaluation_YYYYMMDD_HHMMSS.png`: 可视化对比图
     - `pose_data_YYYYMMDD_HHMMSS.csv`: 原始位姿数据

## 📈 Output

脚本会将所有评估结果保存到 `~/.dexe/localization/localization_assessment/` 目录中：

1. **评估指标文件** (`metrics_YYYYMMDD_HHMMSS.txt`): 
   - 详细的文本格式评估报告
   - 包含两组位姿的所有统计指标

2. **可视化图像** (`localization_evaluation_YYYYMMDD_HHMMSS.png`): 
   - 高清PNG图像（150 DPI）
   - 左右对比原始位姿和滤波后位姿

3. **原始数据CSV** (`pose_data_YYYYMMDD_HHMMSS.csv`):
   - 时间戳对齐的原始位姿数据
   - 包含所有收集的样本
   - 可用于进一步分析或绘图

### 输出示例

#### 目录结构
```
~/.dexe/localization/localization_assessment/
├── metrics_20251126_143022.txt
├── localization_evaluation_20251126_143022.png
├── pose_data_20251126_143022.csv
├── metrics_20251126_150315.txt
├── localization_evaluation_20251126_150315.png
└── pose_data_20251126_150315.csv
```

#### 终端输出

```
============================================================
Evaluation Results for Raw Cartographer Localization
============================================================

1. Position Fluctuation Indicators:
   Standard Deviation (STD):
     σ_x     = 0.002341 m
     σ_y     = 0.001876 m
     σ_θ     = 0.003142 rad (0.1800°)

   Root Mean Square Error (RMSE):
     RMSE_x  = 0.002341 m
     RMSE_y  = 0.001876 m
     RMSE_θ  = 0.003142 rad (0.1800°)

   Maximum Deviation:
     Max_x   = 0.007123 m
     Max_y   = 0.005642 m
     Max_θ   = 0.009876 rad (0.5658°)

2. Stability Scoring:
   Jitter Frequency (change rate exceeding threshold):
     Position:    12.34%
     Orientation: 8.76%

   3-Sigma Outliers:
     X:     2.00%
     Y:     1.00%
     Theta: 3.00%
============================================================

[INFO] [localization_evaluator]: Output directory: /home/fengbro/.dexe/localization/localization_assessment
[INFO] [localization_evaluator]: Saved metrics to: /home/fengbro/.dexe/localization/localization_assessment/metrics_20251126_143022.txt
[INFO] [localization_evaluator]: Saved visualization to: /home/fengbro/.dexe/localization/localization_assessment/localization_evaluation_20251126_143022.png
[INFO] [localization_evaluator]: Saved raw data to: /home/fengbro/.dexe/localization/localization_assessment/pose_data_20251126_143022.csv
```

### CSV文件示例 (`pose_data_YYYYMMDD_HHMMSS.csv`)

```csv
timestamp,loc_x,loc_y,loc_theta,filtered_x,filtered_y,filtered_theta
1732614022.123,10.234,5.678,1.234,10.235,5.677,1.233
1732614022.223,10.235,5.678,1.235,10.236,5.678,1.234
1732614022.323,10.234,5.679,1.234,10.235,5.679,1.234
...
```

### 可视化输出

生成的图像包含：
- 左图: 原始Cartographer定位结果
- 右图: robot_localization滤波后结果
- 每个图包含：
  - 蓝色/绿色散点: 位姿点
  - 黑色箭头: 朝向
  - 红叉: 均值位置
  - 半透明椭圆: 1σ和3σ置信区间
  - 文本框: 主要评估指标

所有文件保存在 `~/.dexe/localization/localization_assessment/` 目录中，文件名包含时间戳以便追踪多次评估。

## 依赖项

```bash
# ROS 2依赖
sudo apt install ros-humble-message-filters

# Python依赖
pip3 install numpy matplotlib
```

## 注意事项

1. **时间同步**: 脚本使用近似时间同步，容差为0.1秒。如果两个topic的发布频率差异很大，可能需要调整。

2. **静止状态**: 评估期间确保机器人完全静止，任何移动都会影响评估结果。

3. **样本数量**: 默认收集100个样本。可以根据需要调整`buffer_size`参数，但建议不少于50个样本以保证统计有效性。

4. **阈值设置**: 
   - `position_threshold`: 建议设置为定位系统精度的1/5到1/10
   - `orientation_threshold`: 建议设置为0.05弧度（约3度）左右

## 故障排除

### 问题1: 没有收到数据

**检查**:
```bash
# 检查topic是否存在
ros2 topic list | grep -E "(loc_pose|filtered)"

# 检查topic发布频率
ros2 topic hz /cartographer/loc_pose
ros2 topic hz /odometry/filtered
```

### 问题2: 时间同步失败

**解决**: 增加时间同步容差，修改脚本中的`slop`参数：
```python
self.sync = ApproximateTimeSynchronizer(
    [self.loc_pose_sub, self.filtered_odom_sub],
    queue_size=10,
    slop=0.2  # 增加到0.2秒
)
```

### 问题3: 图像无法保存

**检查**: 确保有写入权限，或指定输出路径：
```python
plt.savefig('/path/to/output/static_localization_evaluation.png', dpi=150)
```

## 扩展功能

### 查看历史评估结果

所有评估结果都保存在统一目录中，可以方便地查看历史记录：

```bash
# 查看所有评估记录
ls -lh ~/.dexe/localization/localization_assessment/

# 查看最新的评估报告
cat $(ls -t ~/.dexe/localization/localization_assessment/metrics_*.txt | head -1)

# 对比多次评估的图像
eog ~/.dexe/localization/localization_assessment/localization_evaluation_*.png
```

### 批量分析多次评估

可以编写脚本分析多个CSV文件：

```python
import pandas as pd
from pathlib import Path
import glob

# 读取所有评估数据
data_dir = Path.home() / '.dexe' / 'localization' / 'localization_assessment'
csv_files = glob.glob(str(data_dir / 'pose_data_*.csv'))

for csv_file in csv_files:
    df = pd.read_csv(csv_file)
    print(f"\nFile: {Path(csv_file).name}")
    print(f"Samples: {len(df)}")
    print(f"Duration: {df['timestamp'].max() - df['timestamp'].min():.2f}s")
```

### 清理旧的评估结果

```bash
# 删除30天前的评估结果
find ~/.dexe/localization/localization_assessment/ -name "*.png" -mtime +30 -delete
find ~/.dexe/localization/localization_assessment/ -name "*.txt" -mtime +30 -delete
find ~/.dexe/localization/localization_assessment/ -name "*.csv" -mtime +30 -delete
```

## 作者

Created for evaluating static localization performance in the dexe_mobile_slam project.
