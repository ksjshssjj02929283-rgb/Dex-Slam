# Cartographer Node Resource Monitoring

这个工具用于监控和可视化 cartographer_node 的 CPU 和内存资源使用情况。

## 功能特点

- **实时监控**：每 30 秒记录一次 CPU 和内存使用率
- **数据持久化**：将监控数据保存到 CSV 文件
- **可视化分析**：生成详细的图表展示资源使用趋势
- **统计分析**：计算平均值、中位数、标准差等统计信息

## 工具组成

### 1. monitor_cartographer_resources.sh
监控脚本，用于实时记录 cartographer_node 的资源使用情况。

**功能**：
- 自动查找 cartographer_node 进程
- 每 30 秒采集一次数据
- 实时在终端显示监控信息
- 将数据保存到 CSV 文件

**数据保存位置**：
```
~/.dexe/localization/localization_assessment/cartographer_resources_YYYYMMDD_HHMMSS.csv
```

### 2. visualize_cartographer_resources.py
可视化脚本，用于生成资源使用图表。

**功能**：
- 读取监控数据 CSV 文件
- 生成 CPU 使用率时序图
- 生成内存使用率时序图
- 生成 RSS 内存时序图
- 生成 CPU 和内存组合视图
- 计算并显示统计信息

**输出文件**：
- `resource_usage.png` - 详细的三合一图表（CPU%, MEM%, RSS MB）
- `resource_usage_combined.png` - CPU 和内存组合视图（双 Y 轴）
- `resource_usage_report.txt` - 统计报告文本文件

## 使用方法

### 1. 启动监控

在 cartographer_node 运行时，打开新终端运行：

```bash
# 方式 1：直接运行脚本
./scripts/monitor_cartographer_resources.sh

# 方式 2：如果已安装，使用 ros2 run
ros2 run cartographer_ros monitor_cartographer_resources.sh
```

**监控输出示例**：
```
Found cartographer_node process with PID: 12345
Recording interval: 30s
Output file: /home/user/.dexe/localization/localization_assessment/cartographer_resources_20260106_143022.csv
Press Ctrl+C to stop monitoring

Timestamp            Time         CPU(%)     RSS(MB)      MEM(%)      
--------------------------------------------------------------------------------
1704534622          14:30:22     45.20      1234.56      5.67       
1704534652          14:30:52     48.30      1245.78      5.72       
1704534682          14:31:22     46.10      1250.12      5.74       
...
```

**停止监控**：
按 `Ctrl+C` 停止监控。数据会自动保存。

### 2. 生成可视化

监控结束后，运行可视化脚本：

```bash
# 方式 1：自动使用最新的监控数据
./scripts/visualize_cartographer_resources.py

# 方式 2：指定特定的 CSV 文件
./scripts/visualize_cartographer_resources.py /path/to/cartographer_resources_20260106_143022.csv

# 方式 3：如果已安装，使用 ros2 run
ros2 run cartographer_ros visualize_cartographer_resources.py

# 查看所有可用的监控数据文件
./scripts/visualize_cartographer_resources.py --list

# 指定输出目录
./scripts/visualize_cartographer_resources.py --output-dir /path/to/output
```

**可视化输出示例**：
```
Loading resource data from: /home/user/.dexe/localization/localization_assessment/cartographer_resources_20260106_143022.csv
Loaded 120 data points

======================================================================
CARTOGRAPHER NODE RESOURCE USAGE REPORT
======================================================================

--- CPU Usage Statistics ---
  Mean:   46.82%
  Median: 46.50%
  Std:    2.34%
  Min:    42.10%
  Max:    52.30%

--- Memory Usage Statistics (%) ---
  Mean:   5.71%
  Median: 5.70%
  Std:    0.05%
  Min:    5.67%
  Max:    5.80%

--- RSS Memory Statistics (MB) ---
  Mean:   1245.67 MB
  Median: 1244.50 MB
  Std:    8.23 MB
  Min:    1234.56 MB
  Max:    1265.43 MB

======================================================================

Generating visualizations...
Saved resource usage plot to: /home/user/.dexe/localization/localization_assessment/resource_usage.png
Saved combined resource plot to: /home/user/.dexe/localization/localization_assessment/resource_usage_combined.png
Saved report to: /home/user/.dexe/localization/localization_assessment/resource_usage_report.txt

Visualization complete!
Results saved to: /home/user/.dexe/localization/localization_assessment
```

## 数据格式

### CSV 文件格式
```csv
timestamp,time,cpu_percent,rss_mb,mem_percent
1704534622,14:30:22,45.20,1234.56,5.67
1704534652,14:30:52,48.30,1245.78,5.72
...
```

**字段说明**：
- `timestamp`: Unix 时间戳（秒）
- `time`: 时间（HH:MM:SS 格式）
- `cpu_percent`: CPU 使用率（%）
- `rss_mb`: RSS 内存（MB）
- `mem_percent`: 内存使用率（%）

## 配置选项

### 修改监控间隔

编辑 `monitor_cartographer_resources.sh` 中的 `INTERVAL` 变量：

```bash
INTERVAL=30  # 修改为所需的秒数
```

### 修改输出目录

编辑 `monitor_cartographer_resources.sh` 中的 `OUTPUT_DIR` 变量：

```bash
OUTPUT_DIR="$HOME/.dexe/localization/localization_assessment"
```

## 典型使用场景

### 场景 1：性能基准测试
在不同配置下运行 cartographer，比较资源使用情况：

```bash
# 测试配置 A
monitor_cartographer_resources.sh
# ... 运行测试 ...
# Ctrl+C 停止

# 测试配置 B
monitor_cartographer_resources.sh
# ... 运行测试 ...
# Ctrl+C 停止

# 分别生成可视化进行比较
visualize_cartographer_resources.py config_a_data.csv --output-dir ./results_config_a
visualize_cartographer_resources.py config_b_data.csv --output-dir ./results_config_b
```

### 场景 2：长时间稳定性测试
监控 cartographer 在长时间运行时的资源使用稳定性：

```bash
# 启动长时间监控
monitor_cartographer_resources.sh
# 让系统运行数小时...
# Ctrl+C 停止

# 生成可视化，查看是否有内存泄漏或性能下降
visualize_cartographer_resources.py
```

### 场景 3：问题诊断
当 cartographer 出现性能问题时，监控资源使用：

```bash
# 启动监控
monitor_cartographer_resources.sh
# 重现问题场景...
# 观察 CPU 和内存是否异常飙升
```

## 依赖要求

### monitor_cartographer_resources.sh
- Bash shell
- `ps` 命令
- `pgrep` 命令
- `awk` 命令

### visualize_cartographer_resources.py
- Python 3
- numpy
- matplotlib
- pandas

**安装 Python 依赖**：
```bash
pip3 install numpy matplotlib pandas
```

## 故障排除

### 问题 1：找不到 cartographer_node 进程
**错误信息**：
```
Error: No process found matching pattern: /cartographer_node
```

**解决方法**：
- 确认 cartographer_node 正在运行
- 使用 `ps aux | grep cartographer_node` 确认进程名称
- 如果进程名称不同，修改脚本中的 `PATTERN` 变量

### 问题 2：权限不足
**错误信息**：
```
Permission denied
```

**解决方法**：
```bash
chmod +x scripts/monitor_cartographer_resources.sh
chmod +x scripts/visualize_cartographer_resources.py
```

### 问题 3：Python 模块未找到
**错误信息**：
```
ModuleNotFoundError: No module named 'pandas'
```

**解决方法**：
```bash
pip3 install numpy matplotlib pandas
```

### 问题 4：进程 PID 在监控期间改变
监控脚本会自动检测进程退出并尝试重新查找新的进程。如果你重启了 cartographer_node，监控会自动切换到新进程。

## 注意事项

1. **监控间隔**：默认 30 秒。如果需要更高频率的监控，请修改 `INTERVAL` 变量，但要注意这会增加数据量。

2. **磁盘空间**：长时间监控会产生较大的 CSV 文件。请定期清理旧的监控数据。

3. **性能影响**：监控脚本本身的资源消耗很低，不会显著影响 cartographer 性能。

4. **多实例**：如果运行多个 cartographer_node 实例，脚本会监控最新启动的实例（PID 最大）。

5. **时区**：时间戳使用系统本地时区。

## 文件位置

- **脚本源码**：`cartographer_ros/scripts/`
- **安装位置**：`install/cartographer_ros/lib/cartographer_ros/`
- **数据保存**：`~/.dexe/localization/localization_assessment/`

## 相关工具

- `collect_trajectory_data.py` - 收集轨迹数据
- `evaluate_trajectory.py` - 评估轨迹精度
- `evaluate_static_localization.py` - 评估静态定位精度

## 更新日志

- **v1.0** (2026-01-06): 初始版本
  - 实现基本的资源监控功能
  - 支持 CPU 和内存使用率监控
  - 生成可视化图表和统计报告
