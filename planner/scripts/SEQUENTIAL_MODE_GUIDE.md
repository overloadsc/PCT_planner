# 串联路径执行模式使用指南

## 🎯 功能说明

串联路径执行模式允许机器人连续走完多条路径，自动规划路径之间的连接段。

## 🔄 执行流程

```
起点 → 路径1 → 连接段1 → 路径2 → 连接段2 → 路径3 → ... → 终点
```

### 示例（Oldtown 场景）：

```
起点: [-32.80, 30.07]
  ↓ route_1 (8个航点)
终点1: [-33.05, 14.82]
  ↓ 自动规划连接段 (约15.3m)
起点2: [7.95, -3.43]
  ↓ route_2 (8个航点)
终点: [-1.8, -19.18]
```

## ⚙️ 配置方法

在 `scene_configs.yaml` 中添加 `execution_mode` 字段：

```yaml
  Oldtown:
    tomo_file: OldTown
    description: "老城区场景"
    execution_mode: "sequential"  # ← 串联执行模式
    routes:
      - name: "route_1"
        start_pos: [-32.80, 30.07]
        end_pos: [-33.05, 14.82]
        num_waypoints: 8
      - name: "route_2"
        start_pos: [7.95, -3.43]
        end_pos: [-1.8, -19.18]
        num_waypoints: 8
```

### 执行模式选项：

| 值 | 说明 | 行为 |
|---|------|------|
| `sequential` | 串联执行 | 规划完整连续路径，发布到 `/pct_planner` |
| `independent` | 独立执行（默认） | 每条路径独立发布到 `/pct_path_route_X` |

## 🚀 运行方法

```bash
cd ~/PCT_planner/planner/scripts

# 运行串联路径规划
python3 multi_route_planner.py --scene Oldtown
```

## 📊 输出信息

### 控制台输出示例：

```
============================================================
场景: Oldtown
描述: 老城区场景
Tomogram: OldTown
路径类型: 多路径
路径数量: 2
执行模式: sequential
------------------------------------------------------------

=== 串联路径规划模式 ===

🚀 全局起点: [-32.80, 30.07]

--- 路径 1/2: route_1 ---
  规划主路径: 10 个航点
    ✓ 段 1/9: 245 点
    ✓ 段 2/9: 189 点
    ...
  ✓ 路径 route_1 完成

--- 路径 2/2: route_2 ---
📍 规划连接段: [-33.05, 14.82] → [7.95, -3.43] (距离: 45.32m)
  ✓ 连接段完成: 512 点
  规划主路径: 10 个航点
    ✓ 段 1/9: 198 点
    ...
  ✓ 路径 route_2 完成

🎉 串联规划完成！
   总航点数: 20
   总轨迹点数: 3456
   路径总长: 123.45m
```

## 📡 ROS2 话题

### 串联模式发布：

| 话题名 | 类型 | 说明 |
|-------|------|------|
| `/pct_planner` | `nav_msgs/Path` | 完整串联轨迹 |
| `/pct_waypoints_sequential` | `visualization_msgs/MarkerArray` | 所有航点可视化 |

### 独立模式发布（原有行为）：

| 话题名 | 类型 | 说明 |
|-------|------|------|
| `/pct_path_route_1` | `nav_msgs/Path` | 路径1 |
| `/pct_path_route_2` | `nav_msgs/Path` | 路径2 |
| `/pct_waypoints_route_1` | `visualization_msgs/MarkerArray` | 路径1航点 |
| `/pct_waypoints_route_2` | `visualization_msgs/MarkerArray` | 路径2航点 |

## 🎨 可视化

在 RViz 中订阅：
- `/pct_planner` - 查看完整串联路径（紫色线）
- `/pct_waypoints_sequential` - 查看所有航点：
  - 🟢 起点（绿色）
  - 🔴 终点（红色）
  - 🔵 中间航点（蓝色）

## 🔧 连接段规划

当路径之间有间隙时（route_1终点 ≠ route_2起点），系统会：

1. **自动检测间隙**
   - 计算两点间直线距离
   - 判断是否需要连接段（阈值 0.1m）

2. **规划连接路径**
   - 使用 A* + 轨迹优化
   - 保证可通行性
   - 平滑连接

3. **失败处理**
   - 如果连接段规划失败，报错并跳过该路径
   - 继续规划后续路径

## 💡 最佳实践

### 1. 路径设计
- 尽量让相邻路径的终点和起点接近，减少连接段长度
- 使用 `interactive_waypoint_selector.py` 可视化选择路径

### 2. 航点数量
- 短路径（<20m）：4-6 个航点
- 中等路径（20-50m）：6-10 个航点
- 长路径（>50m）：10-15 个航点

### 3. 调试
```bash
# 先测试独立模式，确保每条路径都能规划成功
execution_mode: "independent"

# 确认无误后，切换为串联模式
execution_mode: "sequential"
```

## ⚠️ 注意事项

1. **内存消耗**
   - 串联模式会存储完整轨迹，长路径可能消耗较多内存
   - 建议总轨迹点数 < 10000

2. **规划时间**
   - 串联模式需要规划所有连接段
   - 路径越多，规划时间越长

3. **连接段失败**
   - 如果连接段不可达（被障碍物阻挡），该路径会被跳过
   - 建议使用 `interactive_waypoint_selector.py` 确认连通性

## 🔗 相关文件

- 配置文件：`scene_configs.yaml`
- 规划脚本：`multi_route_planner.py`
- 路径选择：`interactive_waypoint_selector.py`
- 自动化流水线：`../../run_pct_pipeline.py`

## 📞 完整工作流

```bash
# 1. 启动 UE4
cd /home/user/Unreal_Projects_UE4
python3 run_ue_random_group_map.py ~/项目.uproject

# 2. 生成 tomogram
cd /home/user/PCT_planner
python3 run_pct_pipeline.py

# 3. 选择路径（可选）
cd planner/scripts
python3 interactive_waypoint_selector.py
# 交互选择多条路径后按 's' 保存

# 4. 编辑配置文件
# 在 scene_configs.yaml 中添加 execution_mode: "sequential"

# 5. 运行串联规划
python3 multi_route_planner.py --scene Oldtown

# 6. 在 RViz 中查看 /pct_path 话题
```

## 🎉 成功！

现在你的机器人可以连续走完多条路径了！

