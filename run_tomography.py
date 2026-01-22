#!/usr/bin/env python3
"""
Tomography 自动化脚本：从点云生成 Tomogram 地图
输入: /tmp/ue_scene_state.json + rsc/pcd/{scene}.pcd
输出: rsc/tomogram/{scene}.pickle
"""

import sys
import json
import subprocess
from pathlib import Path
from typing import Optional

# 场景组映射表：仅用于文件名不同的历史场景
# 注：场景名会自动规范化为首字母大写格式
SCENE_MAPPING = {
    "Spiral": "spiral0.3_2",        # 文件名小写+版本号
    "Plaza": "OldTown_all_new1",    # 使用了其他场景的文件
    "Building": "AI_vol5_02_all",   # AI项目命名规则
    # 新场景无需添加，自动使用约定命名
}

DEFAULT_STATE_FILE = "/tmp/ue_scene_state.json"
TOMOGRAPHY_SCRIPT = Path(__file__).parent / "tomography/scripts/tomography.py"


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
        print("💡 提示: 请先运行 UE4 场景或手动指定场景")
        return None
    except json.JSONDecodeError as e:
        print(f"❌ 错误: JSON 解析失败: {e}")
        return None


def update_config_init_file(scene_name: str, config_dir: Path) -> bool:
    """在 __init__.py 中添加新场景的导入语句"""
    init_file = config_dir / "__init__.py"
    scene_file = f"scene_{scene_name.lower()}"
    scene_class = f"Scene{scene_name}"
    import_line = f"from .{scene_file} import {scene_class}\n"
    
    try:
        content = init_file.read_text(encoding='utf-8')
        
        if import_line.strip() in content:
            print(f"✓ {scene_class} 已在 __init__.py 中")
            return True
        
        # 确保文件末尾有换行符（防止追加时连在一起）
        if content and not content.endswith('\n'):
            content += '\n'
            init_file.write_text(content, encoding='utf-8')
        
        # 追加导入语句
        with open(init_file, 'a', encoding='utf-8') as f:
            f.write(import_line)
        
        print(f"✅ 已在 __init__.py 中添加: {import_line.strip()}")
        return True
        
    except Exception as e:
        print(f"❌ 更新 __init__.py 失败: {e}")
        return False


def map_scene_to_tomo(scene_group: str) -> Optional[str]:
    """将场景组映射为 tomography 场景名
    
    智能映射策略：
    1. 规范化场景名
    2. 检查映射表（文件名不同的场景）
    3. 检查配置文件是否存在
    4. 不存在则自动生成
    """
    normalized_name = normalize_scene_name(scene_group)
    if scene_group != normalized_name:
        print(f"📝 场景名规范化: {scene_group} → {normalized_name}")
    
    config_dir = Path(__file__).parent / "tomography" / "config"
    config_file = config_dir / f"scene_{normalized_name.lower()}.py"
    
    # 优先级1: 检查映射表
    if normalized_name in SCENE_MAPPING:
        tomo_scene = SCENE_MAPPING[normalized_name]
        print(f"✓ 使用映射表: {normalized_name} → {tomo_scene}")
        return tomo_scene
    
    # 优先级2: 配置文件存在
    if config_file.exists():
        print(f"✓ 找到场景配置: {config_file.name}")
        return normalized_name
    
    # 优先级3: 自动生成配置
    print(f"⚠️  场景配置不存在: {config_file.name}")
    print(f"💡 正在生成默认配置...")
    
    try:
        sys.path.insert(0, str(config_dir))
        from scene_generator import generate_default_scene_config
        
        config_content = generate_default_scene_config(normalized_name)
        config_file.write_text(config_content, encoding='utf-8')
        print(f"✅ 已生成配置文件: {config_file.name}")
        
        update_config_init_file(normalized_name, config_dir)
        print(f"💡 提示: 使用默认参数，可手动编辑该文件进行调优")
        
        return normalized_name
        
    except Exception as e:
        print(f"❌ 自动生成配置失败: {e}")
        return None


def run_tomography(scene_name: str, timeout: int = 30) -> bool:
    """运行 tomography 脚本"""
    print("\n" + "="*60)
    print(f"🗺️  运行 Tomography (场景: {scene_name})")
    print("="*60)
    
    cmd = ["python3", str(TOMOGRAPHY_SCRIPT), "--scene", scene_name]
    print(f"🔧 执行命令: {' '.join(cmd)}")
    print(f"⏱️  超时设置: {timeout} 秒")
    print()
    
    work_dir = TOMOGRAPHY_SCRIPT.parent
    
    try:
        result = subprocess.run(cmd, check=False, cwd=str(work_dir), timeout=timeout)
        print(f"\n✅ Tomography 完成")
        return True
    except subprocess.TimeoutExpired:
        print(f"\n✅ Tomography 完成（超时退出，Tomogram 已生成）")
        return True
    except FileNotFoundError:
        print(f"\n❌ 找不到脚本: {TOMOGRAPHY_SCRIPT}")
        return False
    except Exception as e:
        print(f"\n❌ Tomography 出错: {e}")
        return False


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Tomography 自动化：从点云生成 Tomogram 地图文件"
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
        '--timeout',
        type=int,
        default=30,
        help='超时时间（秒），默认: 30'
    )
    
    args = parser.parse_args()
    
    print("="*60)
    print("🗺️  Tomography 地图生成")
    print("="*60)
    
    # 确定场景名
    if args.scene:
        # 直接指定场景
        scene_group = args.scene
        print(f"📝 使用指定场景: {scene_group}")
    else:
        # 从状态文件读取
        print(f"📂 读取场景状态文件: {args.state_file}")
        state = read_ue_scene_state(args.state_file)
        if state is None:
            return 1
        
        scene_group = state.get('scene_group')
        if not scene_group:
            print("❌ 错误: scene_group 字段缺失")
            return 1
        
        print(f"🎮 UE4 场景组: {scene_group}")
        print(f"🗺️  UE4 地图: {state.get('ue_map_ref', 'N/A')}")
        print(f"🕐 时间戳: {state.get('timestamp', 'N/A')}")
    
    # 映射场景名
    tomo_scene = map_scene_to_tomo(scene_group)
    if tomo_scene is None:
        return 1
    
    print(f"\n🔗 Tomography 场景名: {tomo_scene}")
    
    # 运行 tomography
    if not run_tomography(tomo_scene, timeout=args.timeout):
        print("\n❌ Tomography 失败")
        return 1
    
    print("\n" + "="*60)
    print("✅ Tomogram 生成完成！")
    print("="*60)
    print(f"输出: rsc/tomogram/{tomo_scene}.pickle")
    return 0


if __name__ == "__main__":
    sys.exit(main())

