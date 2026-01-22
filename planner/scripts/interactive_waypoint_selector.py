#!/usr/bin/env python3
"""
可视化最大连通区域分析结果 + 交互式路径点选择
独立脚本，使用matplotlib绘图，不依赖RViz
"""
import sys
import argparse
import numpy as np
import pickle
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
import scipy.ndimage as ndimage
from pathlib import Path
from ruamel.yaml import YAML


sys.path.append('../')


def normalize_scene_name(scene_name: str) -> str:
    """统一场景名称格式：首字母大写，其余小写
    
    避免大小写不一致导致的问题
    Examples: "OldTown" → "Oldtown", "BLOCKS" → "Blocks"
    """
    return scene_name.capitalize()


# UE4 场景组映射（与 run_pct_pipeline.py 保持一致）
# 格式: UE4场景组 -> (tomogram文件名, 配置场景名)
# 注：场景名会自动规范化为首字母大写格式（OldTown→Oldtown, BLOCKS→Blocks）
#
# 约定命名规则（推荐）：
#   - 规范化后的 scene_name = tomo_file = scene_name
#   - 文件命名: {scene_name}.pickle
#
# 仅文件名与场景名不同的历史场景需要在此映射
SCENE_TO_TOMO_MAPPING = {
    "Spiral": ("spiral0.3_2", "Spiral"),        # 文件名小写+版本号
    "Plaza": ("OldTown_all_new1", "Plaza"),     # 使用了其他场景的文件
    "Building": ("AI_vol5_02_all", "Building"), # AI项目命名规则
    # 新场景（如 Blocks, Oldtown）无需添加，大小写自动规范化
}


def read_ue_scene_state(state_file='/tmp/ue_scene_state.json'):
    """读取 UE4 场景状态 JSON 文件"""
    import json
    try:
        with open(state_file, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"❌ 错误: 场景状态文件不存在: {state_file}")
        print("💡 提示: 请先运行 UE4 随机场景启动脚本")
        return None
    except json.JSONDecodeError as e:
        print(f"❌ 错误: JSON 解析失败: {e}")
        return None


def append_route_to_config(scene_name, routes, config_file='scene_configs.yaml'):
    """将路径追加到 scene_configs.yaml 文件中（保留格式和注释）"""
    config_path = Path(__file__).parent / config_file
    
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.indent(mapping=2, sequence=2, offset=0)
    
    # 读取现有配置
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.load(f)
    except FileNotFoundError:
        print(f"❌ 配置文件不存在: {config_path}")
        return False
    except Exception as e:
        print(f"❌ 读取配置文件失败: {e}")
        return False
    
    if config is None or 'scenes' not in config:
        print("❌ 配置文件格式错误")
        return False
    
    # 统一格式：直接使用场景名，不添加 _MultiRoute 后缀
    scene_key = scene_name
    
    # 如果场景不存在，创建新场景
    if scene_key not in config['scenes']:
        # 从映射中查找对应的 tomo_file
        tomo_file = scene_name  # 默认使用 scene_name
        for ue_scene, (tomo, config_scene) in SCENE_TO_TOMO_MAPPING.items():
            if config_scene == scene_name:
                tomo_file = tomo
                break
        
        config['scenes'][scene_key] = {
            'tomo_file': tomo_file,
            'description': f"{scene_name}场景",
            'execution_mode': 'independent',
            'routes': []
        }
        print(f"✓ 创建新场景配置: {scene_key}")
    
    # 获取现有路径数量，用于命名新路径
    existing_routes = config['scenes'][scene_key].get('routes', [])
    if existing_routes is None:
        existing_routes = []
        config['scenes'][scene_key]['routes'] = []
    
    next_route_num = len(existing_routes) + 1
    
    # 追加新路径（保持坐标格式一致：两位小数，单行数组）
    for i, route in enumerate(routes):
        # 格式化坐标为两位小数的列表
        start_pos_formatted = [
            round(float(route['start_pos'][0]), 2),
            round(float(route['start_pos'][1]), 2)
        ]
        end_pos_formatted = [
            round(float(route['end_pos'][0]), 2),
            round(float(route['end_pos'][1]), 2)
        ]
        
        new_route = {
            'name': f"route_{next_route_num + i}",
            'start_pos': start_pos_formatted,
            'end_pos': end_pos_formatted,
            'num_waypoints': 6  # 默认值，可以后续手动调整
        }
        
        # 使用 flow style 保持单行格式 [x, y]
        from ruamel.yaml.comments import CommentedSeq
        new_route['start_pos'] = CommentedSeq(start_pos_formatted)
        new_route['start_pos'].fa.set_flow_style()
        new_route['end_pos'] = CommentedSeq(end_pos_formatted)
        new_route['end_pos'].fa.set_flow_style()
        
        config['scenes'][scene_key]['routes'].append(new_route)
        print(f"✓ 添加路径: route_{next_route_num + i}")
    
    # 写回文件（保留原有格式和注释）
    try:
        with open(config_path, 'w', encoding='utf-8') as f:
            yaml.dump(config, f)
        print(f"\n✅ 配置已保存到: {config_path}")
        return True
    except Exception as e:
        print(f"❌ 保存配置失败: {e}")
        return False


def load_tomogram(tomo_file):
    """加载断层图数据"""
    tomo_dir = '../../rsc/tomogram/'
    tomo_path = tomo_dir + tomo_file + '.pickle'
    
    with open(tomo_path, 'rb') as handle:
        data_dict = pickle.load(handle)
        tomogram = np.asarray(data_dict['data'], dtype=np.float32)
        resolution = float(data_dict['resolution'])
        center = np.asarray(data_dict['center'], dtype=np.float64)
        
        cost_map = tomogram[0, 0, :, :]
        
    print(f"Map size: {cost_map.shape}")
    print(f"Resolution: {resolution}m")
    print(f"Cost range: [{np.nanmin(cost_map):.1f}, {np.nanmax(cost_map):.1f}]")
    
    return cost_map, resolution, center


def analyze_connected_regions(cost_map, cost_threshold):
    """分析连通区域"""
    # 低cost区域掩码
    low_cost_mask = (cost_map < cost_threshold) & (~np.isnan(cost_map))
    num_low_cost = np.sum(low_cost_mask)
    print(f"\nLow-cost cells (cost < {cost_threshold}): {num_low_cost}")
    
    # 连通域分析
    labeled_array, num_features = ndimage.label(low_cost_mask)
    print(f"Found {num_features} connected regions")
    
    # 统计每个区域的大小
    region_sizes = []
    for i in range(1, num_features + 1):
        size = np.sum(labeled_array == i)
        region_sizes.append((i, size))
    
    region_sizes.sort(key=lambda x: x[1], reverse=True)
    
    # 打印前5个最大区域
    print("\nTop regions:")
    for i, (label, size) in enumerate(region_sizes[:5]):
        print(f"  Region {i+1}: {size} cells ({size/num_low_cost*100:.1f}%)")
    
    # 最大区域
    largest_label = region_sizes[0][0]
    largest_region_mask = (labeled_array == largest_label)
    
    return low_cost_mask, labeled_array, largest_region_mask, num_features


class InteractiveWaypointSelector:
    """交互式路径点选择器"""
    
    def __init__(self, cost_map, largest_region_mask, resolution, center, scene_name):
        self.cost_map = cost_map
        self.largest_region_mask = largest_region_mask
        self.resolution = resolution
        self.center = center
        self.scene_name = scene_name
        
        # 存储所有路径
        self.routes = []
        # 当前路径的起点和终点
        self.current_start = None
        self.current_end = None
        self.current_route_num = 1
        
        # 图形元素
        self.start_marker = None
        self.end_marker = None
        self.start_text = None
        self.end_text = None
        
        # 创建图形
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.setup_plot()
        
        # 绑定事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        self.show_instructions()
    
    def setup_plot(self):
        """设置绘图"""
        # 显示cost地图 + 最大连通区域
        self.ax.imshow(self.cost_map.T, origin='lower', cmap='gray', alpha=0.4)
        self.ax.imshow(self.largest_region_mask.T, origin='lower', cmap='Greens', alpha=0.6)
        
        self.ax.set_title(f'Interactive Waypoint Selector - Route {self.current_route_num}\n'
                         f'Left Click: Start | Right Click: End | Keys: n/s/c/q',
                         fontsize=12, fontweight='bold')
        self.ax.set_xlabel('X (grid)')
        self.ax.set_ylabel('Y (grid)')
        self.ax.grid(True, alpha=0.3)
    
    def show_instructions(self):
        """显示使用说明"""
        instructions = (
            "\n" + "="*60 + "\n"
            "       Interactive Waypoint Selector\n"
            "="*60 + "\n"
            "Mouse:\n"
            "  Left Click:  Select Start Point (Green)\n"
            "  Right Click: Select End Point (Red)\n"
            "\n"
            "Keyboard:\n"
            "  'n': Save current route and start new route\n"
            "  's': Save & append all routes to scene_configs.yaml\n"
            "  'c': Clear current route points\n"
            "  'q': Quit\n"
            "="*60 + "\n"
        )
        print(instructions)
    
    def grid_to_world(self, grid_x, grid_y):
        """栅格坐标转世界坐标"""
        offset = np.array([self.cost_map.shape[0] // 2, self.cost_map.shape[1] // 2])
        grid_pos = np.array([grid_x, grid_y]) - offset
        world_pos = grid_pos * self.resolution + self.center
        return world_pos
    
    def on_click(self, event):
        """鼠标点击事件"""
        if event.inaxes != self.ax:
            return
        
        grid_x, grid_y = int(round(event.xdata)), int(round(event.ydata))
        
        # 检查是否在有效区域内
        if not (0 <= grid_x < self.cost_map.shape[0] and 
                0 <= grid_y < self.cost_map.shape[1]):
            print("⚠️  Point outside map bounds!")
            return
        
        if not self.largest_region_mask[grid_x, grid_y]:
            print("⚠️  Point not in valid region!")
            return
        
        world_pos = self.grid_to_world(grid_x, grid_y)
        
        # 左键：起点
        if event.button == 1:
            self.current_start = world_pos
            print(f"✓ Start point selected: [{world_pos[0]:.2f}, {world_pos[1]:.2f}]")
            self.update_markers()
        
        # 右键：终点
        elif event.button == 3:
            self.current_end = world_pos
            print(f"✓ End point selected: [{world_pos[0]:.2f}, {world_pos[1]:.2f}]")
            self.update_markers()
        
        self.fig.canvas.draw()
    
    def update_markers(self):
        """更新图上的标记"""
        offset = np.array([self.cost_map.shape[0] // 2, self.cost_map.shape[1] // 2])
        
        # 清除旧标记
        if self.start_marker:
            self.start_marker.remove()
            self.start_text.remove()
        if self.end_marker:
            self.end_marker.remove()
            self.end_text.remove()
        
        # 绘制起点
        if self.current_start is not None:
            start_grid = np.round((self.current_start - self.center) / self.resolution).astype(int) + offset
            self.start_marker = self.ax.plot(start_grid[0], start_grid[1], 'go', 
                                            markersize=15, markeredgecolor='black', 
                                            markeredgewidth=2, label='Start')[0]
            self.start_text = self.ax.text(start_grid[0], start_grid[1] + 20, 
                                          f'S{self.current_route_num}', 
                                          color='green', fontsize=12, fontweight='bold',
                                          ha='center')
        
        # 绘制终点
        if self.current_end is not None:
            end_grid = np.round((self.current_end - self.center) / self.resolution).astype(int) + offset
            self.end_marker = self.ax.plot(end_grid[0], end_grid[1], 'ro', 
                                          markersize=15, markeredgecolor='black', 
                                          markeredgewidth=2, label='End')[0]
            self.end_text = self.ax.text(end_grid[0], end_grid[1] + 20, 
                                        f'E{self.current_route_num}', 
                                        color='red', fontsize=12, fontweight='bold',
                                        ha='center')
    
    def on_key(self, event):
        """键盘事件"""
        # 'n': 保存当前路径，开始新路径
        if event.key == 'n':
            if self.current_start is not None and self.current_end is not None:
                self.save_current_route()
                self.current_route_num += 1
                self.current_start = None
                self.current_end = None
                self.update_markers()
                self.ax.set_title(f'Interactive Waypoint Selector - Route {self.current_route_num}\n'
                                 f'Left Click: Start | Right Click: End | Keys: n/s/c/q',
                                 fontsize=12, fontweight='bold')
                self.fig.canvas.draw()
                print(f"\n--- Starting Route {self.current_route_num} ---")
            else:
                print("⚠️  Please select both start and end points first!")
        
        # 's': 保存并打印所有路径
        elif event.key == 's':
            if self.current_start is not None and self.current_end is not None:
                self.save_current_route()
            self.print_yaml_config()
        
        # 'c': 清除当前路径
        elif event.key == 'c':
            self.current_start = None
            self.current_end = None
            self.update_markers()
            self.fig.canvas.draw()
            print("✓ Current route cleared")
        
        # 'q': 退出
        elif event.key == 'q':
            plt.close(self.fig)
    
    def save_current_route(self):
        """保存当前路径"""
        route = {
            'name': f'route_{self.current_route_num}',
            'start_pos': [float(self.current_start[0]), float(self.current_start[1])],
            'end_pos': [float(self.current_end[0]), float(self.current_end[1])]
        }
        self.routes.append(route)
        print(f"✓ Route {self.current_route_num} saved!")
    
    def print_yaml_config(self):
        """打印YAML配置并保存到文件"""
        if not self.routes:
            print("⚠️  No routes to save!")
            return
        
        print("\n" + "="*70)
        print("  YAML Configuration Preview")
        print("="*70)
        print(f"\n  {self.scene_name}:")
        print(f"    tomo_file: {self.scene_name}")
        print(f'    description: "{self.scene_name}场景"')
        print(f"    execution_mode: independent")
        print("    routes:")
        
        for route in self.routes:
            print(f"      # Route {route['name'].split('_')[1]}")
            print(f"      - name: \"{route['name']}\"")
            print(f"        start_pos: [{route['start_pos'][0]:.2f}, {route['start_pos'][1]:.2f}]")
            print(f"        end_pos: [{route['end_pos'][0]:.2f}, {route['end_pos'][1]:.2f}]")
            print(f"        num_waypoints: 6  # Adjust as needed")
            print()
        
        print("="*70)
        print("\n📊 Route Summary:")
        print("-"*70)
        for i, route in enumerate(self.routes):
            dist = np.linalg.norm(np.array(route['end_pos']) - np.array(route['start_pos']))
            print(f"  Route {i+1}: "
                  f"[{route['start_pos'][0]:7.2f}, {route['start_pos'][1]:7.2f}] → "
                  f"[{route['end_pos'][0]:7.2f}, {route['end_pos'][1]:7.2f}]  "
                  f"(dist: {dist:6.2f}m)")
        print("="*70)
        
        # 保存到配置文件
        print("\n💾 正在保存到 scene_configs.yaml...")
        if append_route_to_config(self.scene_name, self.routes):
            print("✅ 路径已成功追加到配置文件！")
            print(f"📝 场景配置: {self.scene_name}")
            print("💡 提示: 你可以手动编辑 scene_configs.yaml 来调整 num_waypoints 和 execution_mode 参数")
        else:
            print("❌ 保存失败，请检查配置文件")
        print()
    
    def show(self):
        """显示交互界面"""
        plt.show()


def visualize_regions(cost_map, low_cost_mask, labeled_array, largest_region_mask, 
                      num_features, cost_threshold, resolution, center, 
                      start_pos=None, end_pos=None):
    """可视化连通区域分析结果（静态版本）"""
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle(f'Connected Region Analysis (cost_threshold={cost_threshold})', fontsize=16)
    
    # 1. 原始cost地图
    ax = axes[0, 0]
    im1 = ax.imshow(cost_map.T, origin='lower', cmap='viridis', vmin=0, vmax=50)
    ax.set_title('Original Cost Map')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    plt.colorbar(im1, ax=ax, label='Cost')
    
    # 2. 低cost区域（二值）
    ax = axes[0, 1]
    ax.imshow(low_cost_mask.T, origin='lower', cmap='RdYlGn', alpha=0.7)
    ax.set_title(f'Low Cost Region (cost < {cost_threshold})')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    
    # 3. 所有连通区域（不同颜色）
    ax = axes[0, 2]
    cmap = plt.cm.get_cmap('tab20', num_features)
    im3 = ax.imshow(labeled_array.T, origin='lower', cmap=cmap, vmin=0, vmax=num_features)
    ax.set_title(f'All Connected Regions ({num_features} regions)')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    plt.colorbar(im3, ax=ax, label='Region ID')
    
    # 4. 最大连通区域（高亮）
    ax = axes[1, 0]
    ax.imshow(cost_map.T, origin='lower', cmap='gray', alpha=0.3)
    ax.imshow(largest_region_mask.T, origin='lower', cmap='Greens', alpha=0.7)
    ax.set_title('Largest Connected Region (Green)')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    
    # 5. 最大区域 + 起终点
    ax = axes[1, 1]
    ax.imshow(cost_map.T, origin='lower', cmap='gray', alpha=0.3)
    ax.imshow(largest_region_mask.T, origin='lower', cmap='Greens', alpha=0.7)
    
    if start_pos is not None and end_pos is not None:
        offset = np.array([cost_map.shape[0] // 2, cost_map.shape[1] // 2])
        start_grid = np.round((start_pos - center) / resolution).astype(int) + offset
        end_grid = np.round((end_pos - center) / resolution).astype(int) + offset
        
        ax.plot(start_grid[0], start_grid[1], 'go', markersize=15, label='Start', 
                markeredgecolor='black', markeredgewidth=2)
        ax.plot(end_grid[0], end_grid[1], 'ro', markersize=15, label='End', 
                markeredgecolor='black', markeredgewidth=2)
        ax.legend()
    
    ax.set_title('Largest Region + Start/End Points')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    
    # 6. 距离变换
    ax = axes[1, 2]
    distance_map = ndimage.distance_transform_edt(largest_region_mask) * resolution
    im6 = ax.imshow(distance_map.T, origin='lower', cmap='hot')
    ax.set_title('Distance to Boundary (meters)')
    ax.set_xlabel('X (grid)')
    ax.set_ylabel('Y (grid)')
    plt.colorbar(im6, ax=ax, label='Distance (m)')
    
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize and select waypoints interactively',
        epilog='默认行为: 自动检测 UE4 状态文件(/tmp/ue_scene_state.json)，如不存在则使用预定义场景'
    )
    parser.add_argument('--scene', type=str, default='Plaza',
                        help='预定义场景名称: Spiral, Building, Plaza (仅当 UE 状态不可用时使用)')
    parser.add_argument('--tomo_file', type=str, default=None,
                        help='直接指定 tomogram 文件名 (最高优先级，覆盖所有其他选项)')
    parser.add_argument('--from-ue', action='store_true',
                        help='强制从 UE4 场景状态文件加载 (默认会自动检测)')
    parser.add_argument('--ue-state-file', type=str, default='/tmp/ue_scene_state.json',
                        help='UE4 场景状态文件路径 (默认: /tmp/ue_scene_state.json)')
    parser.add_argument('--cost_threshold', type=float, default=20.0,
                        help='Cost threshold for traversable region')
    parser.add_argument('--static', action='store_true',
                        help='Show static analysis view instead of interactive')
    args = parser.parse_args()
    
    # 确定场景文件
    # 优先级：--tomo_file > 自动检测 UE 状态 > --scene
    if args.tomo_file:
        # 直接指定 tomogram 文件（最高优先级）
        tomo_file = args.tomo_file
        scene_name = args.tomo_file
        start_pos = None
        end_pos = None
        print(f"\n📁 使用指定的 tomogram 文件: {tomo_file}")
        
    else:
        # 自动检测 UE 状态文件
        ue_state_path = Path(args.ue_state_file)
        ue_state_exists = ue_state_path.exists()
        use_ue = args.from_ue or ue_state_exists
        
        if use_ue:
            # 从 UE4 状态文件读取
            print(f"\n📂 从 UE4 状态文件读取场景: {args.ue_state_file}")
            state = read_ue_scene_state(args.ue_state_file)
            if state is None:
                print("⚠️  UE 状态文件读取失败，回退到默认场景...")
                use_ue = False
            else:
                scene_group = state.get('scene_group')
                if not scene_group:
                    print("❌ 错误: scene_group 字段缺失")
                    print("⚠️  回退到默认场景...")
                    use_ue = False
                else:
                    print(f"🎮 UE4 场景组: {scene_group}")
                    
                    # 规范化场景名（首字母大写）
                    normalized_name = normalize_scene_name(scene_group)
                    if scene_group != normalized_name:
                        print(f"📝 场景名规范化: {scene_group} → {normalized_name}")
                    
                    print(f"🗺️  UE4 地图: {state.get('ue_map_ref', 'N/A')}")
                    print(f"🕐 时间戳: {state.get('timestamp', 'N/A')}")
                    
                    # 智能映射：映射表优先（处理文件名不同），然后约定命名
                    tomo_dir = Path(__file__).parent.parent.parent / "rsc" / "tomogram"
                    
                    if normalized_name in SCENE_TO_TOMO_MAPPING:
                        # 映射表：兼容文件名不同的场景
                        tomo_file, scene_name = SCENE_TO_TOMO_MAPPING[normalized_name]
                        print(f"✓ 使用映射表: {normalized_name} → {tomo_file}.pickle")
                    else:
                        # 约定命名：检查文件是否存在
                        expected_tomo_file = tomo_dir / f"{normalized_name}.pickle"
                        
                        if expected_tomo_file.exists():
                            tomo_file = normalized_name
                            scene_name = normalized_name
                            print(f"✓ 使用约定命名: {normalized_name} → {tomo_file}.pickle")
                        else:
                            # 文件不存在
                            print(f"❌ 错误: Tomogram 文件不存在: {expected_tomo_file}")
                            print(f"💡 请先运行 tomography 生成 {normalized_name}.pickle")
                            print(f"💡 或在映射表中添加场景: {list(SCENE_TO_TOMO_MAPPING.keys())}")
                            print("⚠️  回退到默认场景...")
                            use_ue = False
                    
                    if use_ue:
                        start_pos = None
                        end_pos = None
        
        # 如果 UE 状态不可用，使用预定义场景
        if not use_ue:
            scene_configs = {
                'Spiral': ('spiral0.3_2', [-16.0, -6.0], [-26.0, -5.0]),
                'Building': ('AI_vol5_02_all', [1.0, 0.0], [-1.0, 0.5]),
                'Plaza': ('OldTown_all_new1', [-22.61, -20.24], [6.39, -10.24])
            }
            
            if args.scene in scene_configs:
                tomo_file, start, end = scene_configs[args.scene]
                scene_name = args.scene
                start_pos = np.array(start, dtype=np.float32)
                end_pos = np.array(end, dtype=np.float32)
                print(f"\n📍 使用预定义场景: {scene_name}")
            else:
                tomo_file = args.scene
                scene_name = args.scene
                start_pos = None
                end_pos = None
                print(f"\n📍 使用自定义场景: {scene_name}")
    
    print(f"\n{'='*60}")
    print(f"  Scene: {scene_name}")
    print(f"  Tomogram: {tomo_file}")
    print(f"  Cost Threshold: {args.cost_threshold}")
    print(f"{'='*60}")
    
    # 加载数据
    cost_map, resolution, center = load_tomogram(tomo_file)
    
    # 分析连通区域
    low_cost_mask, labeled_array, largest_region_mask, num_features = \
        analyze_connected_regions(cost_map, args.cost_threshold)
    
    # 交互式选择 vs 静态可视化
    if args.static:
        print("\nGenerating static visualization...")
        visualize_regions(cost_map, low_cost_mask, labeled_array, largest_region_mask,
                         num_features, args.cost_threshold, resolution, center,
                         start_pos if 'start_pos' in locals() else None,
                         end_pos if 'end_pos' in locals() else None)
    else:
        print("\n🎯 Starting interactive waypoint selector...")
        selector = InteractiveWaypointSelector(cost_map, largest_region_mask, 
                                              resolution, center, scene_name)
        selector.show()
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
