#!/usr/bin/env python3
"""
Planner 自动化脚本：从 Tomogram 生成路径规划
输入: /tmp/ue_scene_state.json + rsc/tomogram/{scene}.pickle + scene_configs.yaml
输出: ROS2 Topics (/pct_path, /pct_path_route_X, etc.)
"""

import sys
import json
import subprocess
from pathlib import Path
from typing import Optional
import os

# 场景组映射表：用于 planner 场景名映射
# 注：场景名会自动规范化为首字母大写格式
PLANNER_SCENE_MAPPING = {
    "Spiral": "Spiral",
    "Plaza": "Plaza",
    "Building": "Building",
    # 新场景自动使用约定命名
}

DEFAULT_STATE_FILE = "/tmp/ue_scene_state.json"
PLANNER_SCRIPT = Path(__file__).parent / "planner/scripts/multi_route_planner.py"


def normalize_scene_name(scene_name: str) -> str:
    """统一场景名称格式：首字母大写，其余小写"""
    return scene_name.capitalize()


def read_ue_scene_state(state_file: str = DEFAULT_STATE_FILE) -> Optional[dict]:
    """读取 UE4 场景状态 JSON 文件"""
    try:
        with open(state_file, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"❌ 错误: 场景状态文件不存在: {state_file}")
        return None
    except json.JSONDecodeError as e:
        print(f"❌ 错误: JSON 解析失败: {e}")
        return None


def map_scene_to_planner(scene_group: str) -> str:
    """将场景组映射为 planner 场景名"""
    normalized_name = normalize_scene_name(scene_group)
    if scene_group != normalized_name:
        print(f"📝 场景名规范化: {scene_group} → {normalized_name}")
    
    # 检查映射表
    if normalized_name in PLANNER_SCENE_MAPPING:
        planner_scene = PLANNER_SCENE_MAPPING[normalized_name]
        if planner_scene != normalized_name:
            print(f"✓ 使用映射: {normalized_name} → {planner_scene}")
        return planner_scene
    
    # 使用约定命名
    return normalized_name


def run_planner(scene_name: str, extra_args: list = None) -> bool:
    """运行多路径规划脚本"""
    print("\n" + "="*60)
    print(f"🛤️  运行路径规划 (场景: {scene_name})")
    print("="*60)
    
    cmd = ["python3", str(PLANNER_SCRIPT), "--scene", scene_name]
    if extra_args:
        cmd.extend(extra_args)
    
    print(f"🔧 执行命令: {' '.join(cmd)}")
    print()
    
    work_dir = PLANNER_SCRIPT.parent
    
    # 设置库路径
    env = os.environ.copy()
    planner_root = PLANNER_SCRIPT.parent.parent
    gtsam_lib = planner_root / "lib/3rdparty/gtsam-4.1.1/install/lib"
    smoothing_lib = planner_root / "lib/build/src/common/smoothing"
    lib_path = planner_root / "lib"
    
    ld_path = env.get('LD_LIBRARY_PATH', '')
    new_paths = [str(gtsam_lib), str(smoothing_lib)]
    env['LD_LIBRARY_PATH'] = ':'.join(new_paths + ([ld_path] if ld_path else []))
    
    python_path = env.get('PYTHONPATH', '')
    env['PYTHONPATH'] = f"{lib_path}:{python_path}" if python_path else str(lib_path)
    
    print(f"🔧 设置库路径:")
    print(f"   LD_LIBRARY_PATH: {gtsam_lib}")
    print(f"   PYTHONPATH: {lib_path}")
    print()
    
    try:
        result = subprocess.run(cmd, check=True, cwd=str(work_dir), env=env)
        print(f"\n✅ Planner 完成")
        return True
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Planner 失败 (退出码: {e.returncode})")
        return False
    except FileNotFoundError:
        print(f"\n❌ 找不到脚本: {PLANNER_SCRIPT}")
        return False


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="路径规划自动化：从 Tomogram 生成 ROS2 路径话题"
    )
    parser.add_argument(
        '--state-file',
        type=str,
        default=DEFAULT_STATE_FILE,
        help=f'UE4 场景状态文件路径 (默认: {DEFAULT_STATE_FILE})'
    )
    parser.add_argument(
        '--scene',
        type=str,
        default=None,
        help='直接指定场景名（覆盖状态文件）'
    )
    parser.add_argument(
        '--cost-threshold',
        type=float,
        default=None,
        help='最大可通行 cost 值'
    )
    parser.add_argument(
        '--min-obstacle-dist',
        type=float,
        default=None,
        help='航点距障碍物最小距离（米）'
    )
    parser.add_argument(
        '--min-spacing',
        type=float,
        default=None,
        help='航点间最小间距（米）'
    )
    parser.add_argument(
        '--max-elevation-diff',
        type=float,
        default=None,
        help='高度过滤阈值（米），用于过滤人行道'
    )
    
    args = parser.parse_args()
    
    print("="*60)
    print("🛤️  PCT 路径规划")
    print("="*60)
    
    # 确定场景名
    if args.scene:
        scene_group = args.scene
        print(f"📝 使用指定场景: {scene_group}")
    else:
        print(f"📂 读取场景状态文件: {args.state_file}")
        state = read_ue_scene_state(args.state_file)
        if state is None:
            return 1
        
        scene_group = state.get('scene_group')
        if not scene_group:
            print("❌ 错误: scene_group 字段缺失")
            return 1
        
        print(f"🎮 UE4 场景组: {scene_group}")
    
    # 映射场景名
    planner_scene = map_scene_to_planner(scene_group)
    print(f"🔗 Planner 场景名: {planner_scene}")
    
    # 构建额外参数
    extra_args = []
    if args.cost_threshold is not None:
        extra_args.extend(['--cost_threshold', str(args.cost_threshold)])
    if args.min_obstacle_dist is not None:
        extra_args.extend(['--min_obstacle_dist', str(args.min_obstacle_dist)])
    if args.min_spacing is not None:
        extra_args.extend(['--min_spacing', str(args.min_spacing)])
    if args.max_elevation_diff is not None:
        extra_args.extend(['--max_elevation_diff', str(args.max_elevation_diff)])
    
    # 运行 planner
    if not run_planner(planner_scene, extra_args):
        print("\n❌ 路径规划失败")
        return 1
    
    print("\n" + "="*60)
    print("✅ 路径规划完成！")
    print("="*60)
    print("📡 ROS2 Topics:")
    print("   - /pct_path (主路径)")
    print("   - /pct_path_route_X (单独路径)")
    print("   - /pct_path_connections (衔接段)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

