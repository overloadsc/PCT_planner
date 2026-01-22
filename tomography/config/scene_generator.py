"""
自动生成场景配置文件的工具模块
用于新场景的快速配置生成
"""


def generate_default_scene_config(scene_name, pcd_file_name=None):
    """自动生成默认的场景配置文件
    
    Args:
        scene_name: 场景名（如 "Blocks"）
        pcd_file_name: PCD文件名（默认为 {scene_name}.pcd）
    
    Returns:
        str: 配置文件内容
    """
    if pcd_file_name is None:
        pcd_file_name = f"{scene_name}.pcd"
    
    # 使用通用的默认参数（基于 OldTown 的经验值）
    template = f'''from .scene import ScenePCD, SceneMap, SceneTrav


class Scene{scene_name}():
    """自动生成的场景配置（可手动调整参数）"""
    
    pcd = ScenePCD()
    pcd.file_name = '{pcd_file_name}'

    map = SceneMap()
    map.resolution = 0.25      # 地图分辨率（米）
    map.ground_h = -5.0        # 地面高度（米）
    map.slice_dh = 0.3         # 层高切片（米）

    trav = SceneTrav()
    trav.kernel_size = 7       # 卷积核大小
    trav.interval_min = 2.0    # 最小通行高度（米）
    trav.interval_free = 2.5   # 自由空间高度（米）
    trav.slope_max = 0.60      # 最大坡度
    trav.step_max = 0.25       # 最大台阶高度（米）
    trav.standable_ratio = 0.10  # 可站立比例
    trav.cost_barrier = 50.0   # Cost上限
    trav.safe_margin = 1.5     # 安全边距（米）
    trav.inflation = 1.0       # 障碍物膨胀（米）
'''
    return template


def create_scene_config_file(scene_name, output_dir, pcd_file_name=None):
    """创建场景配置文件
    
    Args:
        scene_name: 场景名（如 "Blocks"）
        output_dir: 输出目录（Path对象或字符串）
        pcd_file_name: PCD文件名（可选）
    
    Returns:
        bool: 是否成功创建
    """
    from pathlib import Path
    
    output_dir = Path(output_dir)
    output_file = output_dir / f"scene_{scene_name.lower()}.py"
    
    if output_file.exists():
        print(f"⚠️  配置文件已存在: {output_file.name}")
        return False
    
    # 生成配置内容
    config_content = generate_default_scene_config(scene_name, pcd_file_name)
    
    # 写入文件
    try:
        output_file.write_text(config_content, encoding='utf-8')
        print(f"✅ 已生成配置文件: {output_file.name}")
        print(f"💡 提示: 可手动编辑该文件调整场景参数")
        return True
    except Exception as e:
        print(f"❌ 创建配置文件失败: {e}")
        return False

