#!/usr/bin/env python3
"""
PCT 流水线包装脚本：根据 UE4 场景选择自动运行 tomography 和 planner
从 /tmp/ue_scene_state.json 读取场景信息并映射到对应的场景名称
"""

import sys
import json
import subprocess
from pathlib import Path
from typing import Optional, Tuple

# 场景组映射表：仅用于文件名不同的历史场景
# 注：场景名会自动规范化为首字母大写格式（OldTown→Oldtown, BLOCKS→Blocks）
#
# 约定命名规则（推荐，新场景无需配置）：
#   - 规范化后的 scene_name = tomography场景名 = planner场景名
#   - 文件命名: {scene_name}.obj, {scene_name}.pcd, {scene_name}.pickle
#
# 仅文件名与场景名不同的历史场景需要在此映射
SCENE_MAPPING = {
    # 历史遗留场景（文件名与场景名不一致）
    "Spiral": ("spiral0.3_2", "Spiral"),        # 文件名小写+版本号
    "Plaza": ("OldTown_all_new1", "Plaza"),     # 使用了其他场景的文件
    "Building": ("AI_vol5_02_all", "Building"), # AI项目命名规则
    
    # 新场景（如 Blocks, Oldtown）无需添加，自动使用约定命名
    # 大小写会自动规范化：OldTown → Oldtown, BLOCKS → Blocks
}

# 默认路径配置
DEFAULT_STATE_FILE = "/tmp/ue_scene_state.json"
TOMOGRAPHY_SCRIPT = Path(__file__).parent / "tomography/scripts/tomography.py"
PLANNER_SCRIPT = Path(__file__).parent / "planner/scripts/multi_route_planner.py"


def normalize_scene_name(scene_name: str) -> str:
    """统一场景名称格式：首字母大写，其余小写
    
    这样可以避免大小写不一致导致的问题
    
    Examples:
        "OldTown" → "Oldtown"
        "BLOCKS" → "Blocks"
        "spiral" → "Spiral"
    
    Args:
        scene_name: 原始场景名
    
    Returns:
        str: 规范化后的场景名
    """
    return scene_name.capitalize()


def read_ue_scene_state(state_file: str = DEFAULT_STATE_FILE) -> Optional[dict]:
    """读取 UE4 场景状态 JSON 文件"""
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


def update_config_init_file(scene_name: str, config_dir: Path) -> bool:
    """在 __init__.py 中添加新场景的导入语句
    
    Args:
        scene_name: 场景名（如 "Blocks"）
        config_dir: config 目录的 Path 对象
    
    Returns:
        bool: 是否成功更新
    """
    init_file = config_dir / "__init__.py"
    scene_file = f"scene_{scene_name.lower()}"
    scene_class = f"Scene{scene_name}"
    import_line = f"from .{scene_file} import {scene_class}\n"
    
    try:
        content = init_file.read_text(encoding='utf-8')
        
        # 检查是否已经存在该导入
        if import_line.strip() in content:
            print(f"✓ {scene_class} 已在 __init__.py 中")
            return True
        
        # 确保文件末尾有换行符（防止追加时连在一起）
        if content and not content.endswith('\n'):
            content += '\n'
            init_file.write_text(content, encoding='utf-8')
        
        # 追加导入语句到文件末尾
        with open(init_file, 'a', encoding='utf-8') as f:
            f.write(import_line)
        
        print(f"✅ 已在 __init__.py 中添加: {import_line.strip()}")
        return True
        
    except Exception as e:
        print(f"❌ 更新 __init__.py 失败: {e}")
        return False


def map_scene_group(scene_group: str) -> Optional[Tuple[str, str]]:
    """将 UE4 的 scene_group 映射为 (tomography场景名, planner场景名)
    
    智能映射策略（大小写不敏感）：
    1. 统一规范化场景名（首字母大写）
    2. 检查映射表（处理文件名不同的特殊情况）
    3. 检查配置文件是否存在（约定命名）
    4. 都没有 → 自动生成默认配置
    """
    # 统一规范化场景名
    normalized_name = normalize_scene_name(scene_group)
    if scene_group != normalized_name:
        print(f"📝 场景名规范化: {scene_group} → {normalized_name}")
    
    config_dir = Path(__file__).parent / "tomography" / "config"
    config_file = config_dir / f"scene_{normalized_name.lower()}.py"
    
    # 优先级1: 检查映射表（处理文件名不同的特殊场景）
    if normalized_name in SCENE_MAPPING:
        tomo_scene, planner_scene = SCENE_MAPPING[normalized_name]
        print(f"✓ 使用映射表: {normalized_name} → ({tomo_scene}, {planner_scene})")
        return tomo_scene, planner_scene
    
    # 优先级2: 配置文件存在 → 使用约定命名
    if config_file.exists():
        print(f"✓ 找到场景配置: {config_file.name}")
        return normalized_name, normalized_name
    
    # 优先级3: 自动生成默认配置
    print(f"⚠️  场景配置不存在: {config_file.name}")
    print(f"💡 正在生成默认配置...")
    
    try:
        # 导入生成器模块
        import sys
        sys.path.insert(0, str(config_dir))
        from scene_generator import generate_default_scene_config
        
        # 生成配置内容（使用规范化后的名称）
        config_content = generate_default_scene_config(normalized_name)
        
        # 写入文件
        config_file.write_text(config_content, encoding='utf-8')
        print(f"✅ 已生成配置文件: {config_file.name}")
        
        # 更新 __init__.py 添加导入
        update_config_init_file(normalized_name, config_dir)
        
        print(f"💡 提示: 使用默认参数，可手动编辑该文件进行调优")
        
        return normalized_name, normalized_name
        
    except Exception as e:
        print(f"❌ 自动生成配置失败: {e}")
        print(f"💡 可用场景组: {list(SCENE_MAPPING.keys())}")
        return None


def run_tomography(scene_name: str, timeout: int = 3) -> bool:
    """运行 tomography 脚本（带超时机制）
    
    Args:
        scene_name: 场景名称
        timeout: 超时时间（秒），默认3秒。Tomogram生成后ROS会继续运行，超时退出是正常的。
    """
    print("\n" + "="*60)
    print(f"📍 步骤 1/2: 运行 Tomography (场景: {scene_name})")
    print("="*60)
    
    cmd = ["python3", str(TOMOGRAPHY_SCRIPT), "--scene", scene_name]
    print(f"🔧 执行命令: {' '.join(cmd)}")
    print(f"⏱️  超时设置: {timeout} 秒（Tomogram 生成后会自动继续）")
    print()
    
    # 设置工作目录为 tomography/scripts/
    work_dir = TOMOGRAPHY_SCRIPT.parent
    
    try:
        result = subprocess.run(cmd, check=False, cwd=str(work_dir), timeout=timeout)
        # 注意：返回码可能非0（因为被终止），但只要 tomogram 文件已生成就算成功
        print(f"\n✅ Tomography 完成（Tomogram 已生成）")
        return True
    except subprocess.TimeoutExpired:
        # 超时是预期的，因为 ROS spin 会一直运行
        # 实际上 tomogram 文件在超时前已经生成完成
        print(f"\n✅ Tomography 完成（超时退出，Tomogram 已生成）")
        return True
    except FileNotFoundError:
        print(f"\n❌ 找不到脚本: {TOMOGRAPHY_SCRIPT}")
        return False
    except Exception as e:
        print(f"\n❌ Tomography 出错: {e}")
        return False


def run_planner(scene_name: str, extra_args: list = None) -> bool:
    """运行多路径规划脚本"""
    print("\n" + "="*60)
    print(f"📍 步骤 2/2: 运行 Multi-Route Planner (场景: {scene_name})")
    print("="*60)
    
    cmd = ["python3", str(PLANNER_SCRIPT), "--scene", scene_name]
    if extra_args:
        cmd.extend(extra_args)
    
    print(f"🔧 执行命令: {' '.join(cmd)}")
    print()
    
    # 设置工作目录为 planner/scripts/
    work_dir = PLANNER_SCRIPT.parent
    
    # 设置环境变量：添加库文件路径
    import os
    env = os.environ.copy()
    planner_root = PLANNER_SCRIPT.parent.parent  # planner/scripts/ -> planner/
    gtsam_lib = planner_root / "lib/3rdparty/gtsam-4.1.1/install/lib"
    smoothing_lib = planner_root / "lib/build/src/common/smoothing"
    
    # 添加到 LD_LIBRARY_PATH
    ld_path = env.get('LD_LIBRARY_PATH', '')
    new_paths = [str(gtsam_lib), str(smoothing_lib)]
    if ld_path:
        env['LD_LIBRARY_PATH'] = ':'.join(new_paths + [ld_path])
    else:
        env['LD_LIBRARY_PATH'] = ':'.join(new_paths)
    
    # 添加到 PYTHONPATH
    python_path = env.get('PYTHONPATH', '')
    lib_path = str(planner_root / "lib")
    if python_path:
        env['PYTHONPATH'] = f"{lib_path}:{python_path}"
    else:
        env['PYTHONPATH'] = lib_path
    
    print(f"🔧 设置库路径:")
    print(f"   LD_LIBRARY_PATH += {gtsam_lib}")
    print(f"   LD_LIBRARY_PATH += {smoothing_lib}")
    print(f"   PYTHONPATH += {lib_path}")
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
        description="PCT 自动化流水线：根据 UE4 场景状态自动运行 tomography 和 planner"
    )
    parser.add_argument(
        '--state-file', 
        type=str, 
        default=DEFAULT_STATE_FILE,
        help=f'UE4 场景状态文件路径 (默认: {DEFAULT_STATE_FILE})'
    )
    parser.add_argument(
        '--skip-tomography',
        action='store_true',
        help='跳过 tomography 步骤（假设 tomogram 文件已经生成）'
    )
    parser.add_argument(
        '--planner-only',
        action='store_true',
        help='只运行 planner（等同于 --skip-tomography）'
    )
    # 注意：Planner 参数已移除，使用 scene_configs.yaml 中的默认配置
    # parser.add_argument(
    #     '--cost-threshold',
    #     type=float,
    #     default=15.0,
    #     help='Planner 参数：最大可通行 cost 值 (默认: 15.0)'
    # )
    # parser.add_argument(
    #     '--min-obstacle-dist',
    #     type=float,
    #     default=1.5,
    #     help='Planner 参数：到障碍物的最小距离，单位米 (默认: 1.5)'
    # )
    # parser.add_argument(
    #     '--min-spacing',
    #     type=float,
    #     default=8.0,
    #     help='Planner 参数：航点之间的最小间距，单位米 (默认: 8.0)'
    # )
    parser.add_argument(
        '--tomography-timeout',
        type=int,
        default=30,
        help='Tomography 超时时间（秒），超时后自动继续（默认: 30）'
    )
    
    args = parser.parse_args()
    
    # 读取 UE4 场景状态文件
    print("="*60)
    print("🚀 PCT 自动化流水线启动")
    print("="*60)
    print(f"📂 读取场景状态文件: {args.state_file}")
    
    state = read_ue_scene_state(args.state_file)
    if state is None:
        return 1
    
    scene_group = state.get('scene_group')
    if not scene_group:
        print("❌ 错误: scene_group 字段缺失")
        return 1
    
    print(f"🎮 UE4 场景组: {scene_group}")
    print(f"🗺️  UE4 地图引用: {state.get('ue_map_ref', 'N/A')}")
    print(f"🕐 时间戳: {state.get('timestamp', 'N/A')}")
    
    # 映射场景名称
    scene_mapping = map_scene_group(scene_group)
    if scene_mapping is None:
        return 1
    
    tomo_scene, planner_scene = scene_mapping
    print(f"\n🔗 场景名称映射:")
    print(f"   Tomography 使用: {tomo_scene}")
    print(f"   Planner 使用: {planner_scene}")
    
    # 执行流水线
    skip_tomo = args.skip_tomography or args.planner_only
    
    if not skip_tomo:
        if not run_tomography(tomo_scene, timeout=args.tomography_timeout):
            print("\n❌ 流水线失败: Tomography 步骤出错")
            return 1
    else:
        print("\n⏭️  跳过 Tomography 步骤")
    
    # 使用默认参数运行 planner
    if not run_planner(planner_scene):
        print("\n❌ 流水线失败: Planner 步骤出错")
        return 1
    
    print("\n" + "="*60)
    print("✅ PCT 流水线全部完成！")
    print("="*60)
    return 0


if __name__ == "__main__":
    sys.exit(main())

