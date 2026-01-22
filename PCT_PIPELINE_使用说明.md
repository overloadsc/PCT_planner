# PCT 自动化流水线使用说明

## 📖 简介

`run_pct_pipeline.py` 是一个自动化包装脚本，用于根据 UE4 随机选择的场景自动运行 PCT 的完整流水线。

它会：
1. 读取 UE4 生成的场景状态文件 (`/tmp/ue_scene_state.json`)
2. 自动映射场景名称
3. 依次运行 tomography 和 multi-route planner

## 🚀 快速开始

### 步骤 1: 启动 UE4 并生成场景状态

```bash
cd /home/user/Unreal_Projects_UE4
python3 run_ue_random_group_map.py ~/你的项目.uproject
```

这会随机选择一个场景并将信息保存到 `/tmp/ue_scene_state.json`

### 步骤 2: 运行 PCT 流水线

```bash
cd /home/user/PCT_planner
python3 run_pct_pipeline.py
```

就这么简单！脚本会自动：
- ✅ 读取 UE4 选择的场景
- ✅ 运行 tomography 生成地图
- ✅ 运行 planner 规划路径

## 📋 命令行参数

### 基本用法

```bash
# 运行完整流水线（tomography + planner）
python3 run_pct_pipeline.py

# 显示帮助信息
python3 run_pct_pipeline.py --help
```

### 高级选项

```bash
# 只运行 planner（假设 tomogram 已生成）
python3 run_pct_pipeline.py --planner-only

# 跳过 tomography（等同于 --planner-only）
python3 run_pct_pipeline.py --skip-tomography

# 自定义状态文件路径
python3 run_pct_pipeline.py --state-file /path/to/custom_state.json

# 调整 planner 参数
python3 run_pct_pipeline.py \
    --cost-threshold 20.0 \
    --min-obstacle-dist 2.0 \
    --min-spacing 10.0
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--state-file` | 字符串 | `/tmp/ue_scene_state.json` | UE4 场景状态文件路径 |
| `--skip-tomography` | 开关 | False | 跳过 tomography 步骤 |
| `--planner-only` | 开关 | False | 只运行 planner |
| `--cost-threshold` | 浮点数 | 15.0 | 最大可通行 cost 值 |
| `--min-obstacle-dist` | 浮点数 | 1.5 | 到障碍物最小距离(米) |
| `--min-spacing` | 浮点数 | 8.0 | 航点间最小间距(米) |

## 🗺️ 场景映射表

脚本会自动将 UE4 的场景组名映射到对应的 PCT 场景配置：

| UE4 场景组 | Tomography 场景 | Planner 场景 |
|-----------|-----------------|--------------|
| OldTown | Oldtown | Oldtown |
| Spiral | Spiral | Spiral |
| Plaza | Plaza | Plaza |
| Building | Building | Building |

## 📁 文件结构

```
/home/user/PCT_planner/
├── run_pct_pipeline.py          ← 主脚本（新创建）
├── tomography/
│   └── scripts/
│       └── tomography.py        ← 步骤 1
├── planner/
│   └── scripts/
│       └── multi_route_planner.py  ← 步骤 2
└── rsc/
    └── tomogram/                ← 生成的 tomogram 文件

/tmp/
└── ue_scene_state.json          ← UE4 生成的场景状态
```

## 💡 使用场景

### 场景 1: 日常开发测试

每次修改代码后快速测试：

```bash
# 一条命令搞定
cd /home/user/PCT_planner && python3 run_pct_pipeline.py
```

### 场景 2: 多次规划（地图不变）

如果只是调整规划参数，不需要重新生成 tomogram：

```bash
# 第一次：完整流程
python3 run_pct_pipeline.py

# 后续：只运行 planner
python3 run_pct_pipeline.py --planner-only --min-spacing 12.0
python3 run_pct_pipeline.py --planner-only --cost-threshold 18.0
```

### 场景 3: 批量处理

结合 UE4 脚本的随机种子功能，进行可重复的测试：

```bash
# 使用固定种子启动 UE4
cd /home/user/Unreal_Projects_UE4
python3 run_ue_random_group_map.py ~/项目.uproject --seed=42

# 运行 PCT 流水线
cd /home/user/PCT_planner
python3 run_pct_pipeline.py
```

## 🔧 添加新场景

如果你有新的 UE4 场景（比如 "CityBlock"），只需编辑 `run_pct_pipeline.py` 第 13-18 行：

```python
SCENE_MAPPING = {
    "OldTown": ("Oldtown", "Oldtown"),
    "Spiral": ("Spiral", "Spiral"),
    "Plaza": ("Plaza", "Plaza"),
    "Building": ("Building", "Building"),
    "CityBlock": ("Cityblock", "Cityblock"),  # 新增
}
```

然后在相应位置创建配置：
1. `tomography/config/scene_cityblock.py` - 创建 `SceneCityblock` 类
2. `planner/scripts/scene_configs.yaml` - 添加 `Cityblock` 配置（使用统一的 routes 格式）

## ❓ 故障排查

### 错误：场景状态文件不存在

```
❌ 错误: 场景状态文件不存在: /tmp/ue_scene_state.json
💡 提示: 请先运行 UE4 随机场景启动脚本
```

**解决方法**：先运行 UE4 启动脚本：
```bash
cd /home/user/Unreal_Projects_UE4
python3 run_ue_random_group_map.py ~/你的项目.uproject
```

### 错误：未知的场景组

```
❌ 错误: 未知的场景组 'NewScene'
💡 可用场景组: ['OldTown', 'Spiral', 'Plaza', 'Building']
```

**解决方法**：在 `SCENE_MAPPING` 中添加新场景的映射关系（见上方"添加新场景"）

### Tomography 或 Planner 失败

检查日志输出，常见原因：
- 缺少依赖包
- PCD 文件不存在
- ROS2 环境未配置
- 配置文件有误

## 📞 获取帮助

遇到问题？可以：
1. 查看详细日志输出（脚本会打印所有执行步骤）
2. 使用 `--help` 查看所有可用参数
3. 检查 `/tmp/ue_scene_state.json` 文件内容
4. 单独运行 tomography.py 或 multi_route_planner.py 进行调试

## 📝 版本历史

- **v1.0** (2025-12-30): 初始版本
  - 支持从 UE4 状态文件自动读取场景
  - 支持 4 个基础场景映射
  - 支持自定义 planner 参数

