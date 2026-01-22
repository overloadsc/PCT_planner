# 交互式路径点选择工具使用指南

## 📖 简介

`interactive_waypoint_selector.py` 是一个可视化工具，用于：
- **交互式选择**多条路径的起点和终点
- **自动生成** YAML 配置文件格式
- **可视化分析**地图的连通区域和可通行性

## 🚀 快速开始

### 基本用法

```bash
cd /home/user/PCT_planner/planner/scripts

# 使用预定义场景（交互式）
python3 interactive_waypoint_selector.py --scene Plaza --cost_threshold 20.0

# 使用自定义tomogram文件（交互式）
python3 interactive_waypoint_selector.py --tomo_file OldTown_all_new1 --cost_threshold 20.0

# 静态可视化模式（原debug.py功能）
python3 interactive_waypoint_selector.py --scene Plaza --static
```

## 🎮 交互式操作指南

### 启动工具

```bash
python3 interactive_waypoint_selector.py --tomo_file OldTown_all_new1 --cost_threshold 20.0
```

启动后会看到：
- **绿色区域**：可通行的最大连通区域
- **灰色背景**：cost地图
- **网格线**：帮助定位

### 鼠标操作

| 操作 | 功能 | 效果 |
|------|------|------|
| **左键点击** | 选择起点 | 显示绿色圆圈 + S1/S2/S3... 标签 |
| **右键点击** | 选择终点 | 显示红色圆圈 + E1/E2/E3... 标签 |

### 键盘操作

| 按键 | 功能 | 说明 |
|------|------|------|
| **n** | 保存当前路径并开始新路径 | 开始选择 route_2, route_3... |
| **s** | 保存并打印YAML配置 | 输出可直接复制的配置 |
| **c** | 清除当前路径的点 | 重新选择当前路径 |
| **q** | 退出程序 | 关闭窗口 |

### ⚠️ 注意事项

- ✅ 只能在**绿色可通行区域**选择点
- ✅ 点击非可通行区域会显示警告
- ✅ 终端会实时显示所选坐标

## 📝 完整工作流程

### 为 OldTown 场景添加多条路径

#### 步骤 1: 启动工具

```bash
python3 interactive_waypoint_selector.py --tomo_file OldTown_all_new1 --cost_threshold 20.0
```

#### 步骤 2: 选择第一条路径

1. **左键点击**地图选择起点（例如：左下角）
   - 终端输出：`✓ Start point selected: [-22.61, -20.24]`
   
2. **右键点击**地图选择终点（例如：右上角）
   - 终端输出：`✓ End point selected: [6.39, -10.24]`

3. **按 'n'** 保存这条路径，开始下一条
   - 终端输出：`✓ Route 1 saved!`
   - 窗口标题变为：`Route 2`

#### 步骤 3: 选择第二条路径

重复步骤2，选择第二条路径的起点和终点，再按 'n'

#### 步骤 4: 选择第三条路径

重复步骤2，选择第三条路径的起点和终点

#### 步骤 5: 生成配置

按 **'s'** 键，终端会输出：

```yaml
======================================================================
  YAML Configuration (copy to scene_configs.yaml)
======================================================================

  Plaza:
    tomo_file: OldTown_all_new1
    description: "广场场景"
    execution_mode: "independent"
    routes:
      # Route 1
      - name: "route_1"
        start_pos: [-22.61, -20.24]
        end_pos: [6.39, -10.24]
        num_waypoints: 6  # Adjust as needed

      # Route 2
      - name: "route_2"
        start_pos: [10.50, 5.30]
        end_pos: [-15.20, -8.40]
        num_waypoints: 6  # Adjust as needed

      # Route 3
      - name: "route_3"
        start_pos: [-5.00, 12.00]
        end_pos: [18.50, -18.00]
        num_waypoints: 6  # Adjust as needed

======================================================================

📊 Route Summary:
----------------------------------------------------------------------
  Route 1: [-22.61, -20.24] → [  6.39, -10.24]  (dist:  30.89m)
  Route 2: [ 10.50,   5.30] → [-15.20,  -8.40]  (dist:  28.16m)
  Route 3: [ -5.00,  12.00] → [ 18.50, -18.00]  (dist:  37.74m)
======================================================================
```

#### 步骤 6: 复制配置到文件

将上面的 YAML 配置复制粘贴到 `scene_configs.yaml` 文件中

#### 步骤 7: 运行多路径规划

```bash
python3 multi_route_planner.py --scene Plaza
```

## 🎯 使用场景

### 场景 1: 为已有场景添加新路径

如果你已经有 `Plaza` 场景配置，想添加新路径：

```bash
# 1. 查看已有路径起终点
cat scene_configs.yaml | grep -A 10 "Plaza"

# 2. 启动工具
python3 interactive_waypoint_selector.py --tomo_file OldTown_all_new1

# 3. 只选择第三条路径的起点和终点
# 4. 按 's' 生成配置，复制 route_3 部分到配置文件
```

### 场景 2: 创建全新的多路径场景

```bash
# 1. 启动工具
python3 interactive_waypoint_selector.py --tomo_file your_scene_name

# 2. 依次选择多条路径（按 'n' 切换到下一条）
# 3. 按 's' 生成完整配置
# 4. 复制到 scene_configs.yaml
```

### 场景 3: 静态分析地图（调试用）

```bash
# 查看地图的连通区域分析，不进行交互选择
python3 interactive_waypoint_selector.py --scene Plaza --static --cost_threshold 15.0
```

会显示6个子图：
1. 原始 cost 地图
2. 低 cost 区域（二值）
3. 所有连通区域（不同颜色）
4. 最大连通区域
5. 最大区域 + 起终点
6. 距离变换图（到边界的距离）

## 🔧 命令行参数

```bash
python3 interactive_waypoint_selector.py [OPTIONS]
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--scene` | str | Plaza | 预定义场景名称 (Spiral/Building/Plaza) |
| `--tomo_file` | str | None | 直接指定tomogram文件名（覆盖--scene） |
| `--cost_threshold` | float | 20.0 | cost阈值，小于此值认为可通行 |
| `--static` | flag | False | 启用静态可视化模式（不可交互） |

### 示例

```bash
# 使用预定义场景
python3 interactive_waypoint_selector.py --scene Plaza

# 使用自定义文件
python3 interactive_waypoint_selector.py --tomo_file OldTown_all_new1

# 调整cost阈值（更宽松）
python3 interactive_waypoint_selector.py --scene Plaza --cost_threshold 25.0

# 静态可视化
python3 interactive_waypoint_selector.py --scene Plaza --static
```

## 💡 技巧与建议

### 选择好的路径点

1. **远离边界**：选择绿色区域中心，避免靠近障碍物边界
2. **均匀分布**：多条路径应覆盖整个区域，不要集中在一处
3. **合理距离**：起点到终点距离建议 20-50 米
4. **避免狭窄通道**：优先选择开阔区域

### 调整 cost_threshold

- **默认值 20.0**：适用于大多数场景
- **增大（如 25.0）**：放松约束，可通行区域更大
- **减小（如 15.0）**：更严格，只选择最安全区域

### 快速迭代

如果对选择的点不满意：
1. 按 **'c'** 清除当前路径
2. 重新选择
3. 或直接按 **'q'** 退出重新运行

## 📂 相关文件

```
planner/scripts/
├── interactive_waypoint_selector.py  # 本工具
├── scene_configs.yaml               # 配置文件（手动编辑）
├── multi_route_planner.py           # 多路径规划器
├── multi_waypoint_planner.py        # 单路径规划器
└── WAYPOINT_SELECTOR_GUIDE.md       # 本文档
```

## ❓ 常见问题

### Q1: 点击地图没有反应？

**A**: 检查：
- 是否点击在绿色可通行区域内？
- 终端是否显示警告信息？
- 窗口是否处于激活状态？

### Q2: 如何选择多条路径？

**A**: 每选完一条路径（起点+终点），按 **'n'** 键保存并开始下一条

### Q3: 选错了怎么办？

**A**: 
- 清除当前路径：按 **'c'**
- 退出重来：按 **'q'**

### Q4: 生成的坐标如何使用？

**A**: 
1. 按 **'s'** 生成 YAML 配置
2. 复制输出到 `scene_configs.yaml`
3. 运行 `multi_route_planner.py`

### Q5: 可以编辑已有路径吗？

**A**: 本工具只能新建路径。如需修改已有路径：
1. 用本工具重新选择
2. 或直接编辑 `scene_configs.yaml` 文件中的坐标

### Q6: cost_threshold 如何选择？

**A**: 
- 先用默认值 20.0 尝试
- 如果可通行区域太小，增大阈值
- 如果想要更严格的安全区域，减小阈值
- 可以用 `--static` 模式预览不同阈值的效果

## 🔗 下一步

选择完路径点后：

1. **单路径规划**：
   ```bash
   python3 multi_waypoint_planner.py --scene YourScene
   ```

2. **多路径规划**：
   ```bash
   python3 multi_route_planner.py --scene YourScene
   ```

3. **可视化结果**：
   在 RViz 中查看：
   - `/pct_path_route_1`
   - `/pct_path_route_2`
   - `/pct_path_route_3`
   - `/pct_waypoints_route_1`
   - ...

## 📚 更多文档

- [场景配置指南](SCENE_CONFIG_GUIDE.md)
- [多路径规划说明](MULTI_ROUTE_README.md)
- [快速开始](QUICK_START.md)

---

**作者**: PCT Planner Team  
**更新日期**: 2025-12-17  
**版本**: v1.0

