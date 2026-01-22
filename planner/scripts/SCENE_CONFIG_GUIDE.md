# 场景配置说明

## 概述

起点和终点已经从代码中外部化到 `scene_configs.yaml` 配置文件中，方便管理和修改。

## 配置文件结构

`scene_configs.yaml` 文件定义了所有场景的参数：

```yaml
scenes:
  Spiral:
    tomo_file: spiral0.3_2
    start_pos: [-16.0, -6.0]
    end_pos: [-26.0, -5.0]
    description: "螺旋场景"
  
  Building:
    tomo_file: AI_vol5_02_all
    start_pos: [1.0, 0.0]
    end_pos: [-1.0, 0.5]
    description: "建筑场景"
  
  Plaza:
    tomo_file: OldTown_all_new1
    start_pos: [-22.61, -20.24]
    end_pos: [6.39, -10.24]
    description: "广场场景"
```

## 使用方法

### 方法1: 使用默认配置文件

```bash
python3 multi_waypoint_planner.py --scene Spiral
python3 multi_waypoint_planner.py --scene Building
python3 multi_waypoint_planner.py --scene Plaza
```

### 方法2: 指定自定义配置文件

```bash
python3 multi_waypoint_planner.py --scene Spiral --config my_scenes.yaml
```

## 添加新场景

只需要编辑 `scene_configs.yaml` 文件，添加新的场景配置：

```yaml
scenes:
  MyNewScene:
    tomo_file: my_new_scene_tomogram
    start_pos: [x1, y1]
    end_pos: [x2, y2]
    description: "我的新场景"
```

然后使用：

```bash
python3 multi_waypoint_planner.py --scene MyNewScene
```

## 如何确定起点和终点

如果你不知道环境的具体坐标，可以使用以下方法：

### 方法1: 使用交互式路径点选择器 ⭐️ 推荐

```bash
python3 interactive_waypoint_selector.py --tomo_file YourScene --cost_threshold 20.0
```

这是最便捷的方法！在可视化地图上**直接点击选择**起点和终点，自动生成 YAML 配置。

**详细使用方法**：查看 [WAYPOINT_SELECTOR_GUIDE.md](WAYPOINT_SELECTOR_GUIDE.md) 或 [QUICK_REFERENCE.md](QUICK_REFERENCE.md)

### 方法2: 使用静态可视化工具

```bash
python3 interactive_waypoint_selector.py --scene YourScene --static --cost_threshold 20.0
```

这会生成可视化图像，显示可通行区域，你可以在图上估算合适的起点和终点坐标。

### 方法3: 在RViz中查看

1. 运行单段规划器查看地图：
   ```bash
   python3 plan.py --scene YourScene
   ```

2. 在RViz中，查看 `/pct_cost_map` 或点云，使用 `Publish Point` 工具点击地图获取坐标

3. 将获取的坐标添加到 `scene_configs.yaml`

### 方法4: 检查点云坐标范围

```bash
# 查看点云的坐标范围
python3 check_pcd_coords.py --pcd_file rsc/pcd/your_scene.pcd
```

根据输出的坐标范围，选择合适的起点和终点。

## 多轨迹场景的建议

如果需要规划多条轨迹，建议：

1. **为每个任务创建独立的配置文件**：
   ```
   scene_configs_task1.yaml
   scene_configs_task2.yaml
   ...
   ```

2. **或者在同一配置文件中定义多组起点终点**：
   ```yaml
   scenes:
     Plaza_Route1:
       tomo_file: OldTown_all_new1
       start_pos: [-22.61, -20.24]
       end_pos: [6.39, -10.24]
     
     Plaza_Route2:
       tomo_file: OldTown_all_new1
       start_pos: [10.0, -15.0]
       end_pos: [-5.0, -25.0]
   ```

3. **使用脚本批量运行**：
   ```bash
   for scene in Plaza_Route1 Plaza_Route2 Plaza_Route3; do
       python3 multi_waypoint_planner.py --scene $scene --num_waypoints 5
   done
   ```

## 完整示例

```bash
# 螺旋场景，2个中间路径点
python3 multi_waypoint_planner.py --scene Spiral --num_waypoints 2

# 广场场景，10个中间路径点，放松cost阈值
python3 multi_waypoint_planner.py --scene Plaza --num_waypoints 10 --cost_threshold 20.0

# 使用自定义配置文件
python3 multi_waypoint_planner.py --scene MyScene --config custom_scenes.yaml --num_waypoints 5
```

