import open3d as o3d
import numpy as np
import json
import os
from pathlib import Path
import yaml
from datetime import datetime


def mesh_to_pcd(obj_path, pcd_path, num_points=200000, scale_factor=0.01, 
                y_up_to_z_up=True):
    """
    将 OBJ 网格模型转成 PCD 点云
    
    :param obj_path: 输入 OBJ 文件路径
    :param pcd_path: 输出 PCD 文件路径
    :param num_points: 采样点数量
    :param scale_factor: 缩放因子 (例如 0.01 表示 cm→m)
    :param y_up_to_z_up: 是否将Y-up坐标系转换为Z-up坐标系 (绕X轴旋转90°)
    :return: (pcd, gazebo_offset) - 点云对象和Gazebo偏移量
    """

    # 读取 mesh
    print(f"正在加载 mesh: {obj_path}")
    mesh = o3d.io.read_triangle_mesh(obj_path)

    # 1. 先中心化 mesh（在缩放前），记录偏移量
    mesh_center = mesh.get_center()
    mesh.translate(-mesh_center)
    print(f"Mesh中心化偏移 (原始单位): ({mesh_center[0]:.2f}, {mesh_center[1]:.2f}, {mesh_center[2]:.2f})")

    # 2. 缩放 mesh（以原点为中心）
    print(f"正在缩放 mesh (缩放因子: {scale_factor})...")
    mesh.scale(scale_factor, center=[0, 0, 0])
    mesh.compute_vertex_normals()
    
    # 3. 计算并打印Gazebo偏移量（缩放后，坐标变换后）
    gazebo_offset_raw = mesh_center * scale_factor
    if y_up_to_z_up:
        # 坐标变换: (x, y, z) → (x, -z, y)
        gazebo_offset = np.array([gazebo_offset_raw[0], -gazebo_offset_raw[2], gazebo_offset_raw[1]])
    else:
        gazebo_offset = gazebo_offset_raw
    print(f"💡 Gazebo偏移量 (米): ({gazebo_offset[0]:.2f}, {gazebo_offset[1]:.2f}, {gazebo_offset[2]:.2f})")

    # 4. 采样点云
    print(f"正在采样点云（{num_points:,} 点）...")
    pcd = mesh.sample_points_uniformly(number_of_points=num_points)

    # 5. 坐标系转换: Y-up → Z-up (绕X轴旋转90°)
    if y_up_to_z_up:
        print("坐标变换: Y-up → Z-up (绕X轴旋转90°)")
        points = np.asarray(pcd.points)
        # (x, y, z) → (x, -z, y)
        points = points[:, [0, 2, 1]]  # 交换 Y 和 Z
        points[:, 1] = -points[:, 1]   # 翻转新 Y 轴
        pcd.points = o3d.utility.Vector3dVector(points)

    # 6. 打印结果
    points = np.asarray(pcd.points)
    print(f"\n📐 点云信息:")
    for i, axis in enumerate(['X', 'Y', 'Z']):
        min_val, max_val = points[:, i].min(), points[:, i].max()
        span = max_val - min_val
        print(f"   {axis}: [{min_val:.2f}, {max_val:.2f}] 跨度 {span:.2f}m")
    
    # 点云密度
    mesh_area = mesh.get_surface_area()
    density = len(points) / mesh_area if mesh_area > 0 else 0
    print(f"   表面密度: {density:,.0f} 点/m² (≈ {np.sqrt(1/density)*100:.2f} cm 点间距)")

    # 保存
    o3d.io.write_point_cloud(pcd_path, pcd)
    print(f"\n✅ 已保存: {pcd_path}")
    
    return pcd, gazebo_offset


def load_scene_offsets(offsets_file='rsc/scene_offsets.yaml'):
    """加载场景偏移量数据库
    
    Returns:
        dict: 偏移量数据字典，格式 {'scenes': {scene_name: {...}}}
    """
    offsets_path = Path(offsets_file)
    if offsets_path.exists():
        try:
            with open(offsets_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                return data if data else {'scenes': {}}
        except Exception as e:
            print(f"⚠️  读取偏移量数据库失败: {e}")
            return {'scenes': {}}
    return {'scenes': {}}


def save_scene_offset(scene_name, offset, offsets_file='rsc/scene_offsets.yaml'):
    """保存场景偏移量到数据库
    
    Args:
        scene_name: 场景名
        offset: numpy数组 [x, y, z]
        offsets_file: 数据库文件路径
    """
    offsets_path = Path(offsets_file)
    
    # 加载现有数据
    data = load_scene_offsets(offsets_file)
    
    # 更新场景偏移量
    data['scenes'][scene_name] = {
        'pct_to_gazebo_offset': {
            'x': float(offset[0]),
            'y': float(offset[1]),
            'z': float(offset[2])
        },
        'last_updated': datetime.now().isoformat()
    }
    
    # 保存
    try:
        offsets_path.parent.mkdir(parents=True, exist_ok=True)
        with open(offsets_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
        print(f"💾 偏移量已保存到: {offsets_file}")
        return True
    except Exception as e:
        print(f"❌ 保存偏移量失败: {e}")
        return False


def read_ue_scene_state(state_file="/tmp/ue_scene_state.json"):
    """读取 UE4 场景状态文件"""
    try:
        with open(state_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        scene_group = data.get('scene_group', None)
        if scene_group:
            print(f"📖 读取场景: {scene_group}")
            return data, scene_group
        else:
            print("❌ 错误: scene_group 字段不存在")
            return None, None
    except FileNotFoundError:
        print(f"❌ 错误: 文件不存在 {state_file}")
        return None, None
    except json.JSONDecodeError as e:
        print(f"❌ 错误: JSON 解析失败 - {e}")
        return None, None


def write_gazebo_offset(state_file, gazebo_offset):
    """将 Gazebo 偏移量写入场景状态文件"""
    try:
        # 读取现有数据
        with open(state_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        # 添加/更新 pct_to_gazebo_offset 字段
        data['pct_to_gazebo_offset'] = {
            'x': float(gazebo_offset[0]),
            'y': float(gazebo_offset[1]),
            'z': float(gazebo_offset[2])
        }
        
        # 写回文件
        with open(state_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"✅ Gazebo偏移量已写入: {state_file}")
        print(f"   pct_to_gazebo_offset: {data['pct_to_gazebo_offset']}")
        return True
        
    except Exception as e:
        print(f"❌ 写入失败: {e}")
        return False


def main():
    """主函数：手动指定场景名进行转换"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Mesh 到点云转换：手动指定场景名",
        epilog="示例: python3 mesh2pcd.py Blocks"
    )
    parser.add_argument(
        'scene',
        type=str,
        nargs='?',
        default=None,
        help='场景名（如 Blocks, Oldtown, Spiral）'
    )
    parser.add_argument(
        '--scene',
        type=str,
        dest='scene_opt',
        default=None,
        help='场景名（命名参数方式）'
    )
    parser.add_argument(
        '--num-points',
        type=int,
        default=20000000,
        help='采样点数（默认：20000000）'
    )
    parser.add_argument(
        '--no-cache',
        action='store_true',
        help='强制重新转换，忽略缓存'
    )
    
    args = parser.parse_args()
    
    # 确定场景名（位置参数优先）
    scene_group = args.scene or args.scene_opt
    
    if not scene_group:
        print("❌ 错误: 请指定场景名")
        print("💡 使用方法: python3 mesh2pcd.py Blocks")
        print("💡 或者: python3 mesh2pcd.py --scene Blocks")
        return
    
    # 配置固定路径
    RSC_DIR = Path("/home/user/PCT_planner/rsc")
    MESH_DIR = RSC_DIR / "mesh"
    PCD_DIR = RSC_DIR / "pcd"
    OFFSETS_FILE = RSC_DIR / "scene_offsets.yaml"
    STATE_FILE = "/tmp/ue_scene_state.json"
    
    PCD_DIR.mkdir(parents=True, exist_ok=True)
    
    print("="*60)
    print(f"🎨 Mesh 转点云: {scene_group}")
    print("="*60)
    
    # 构建文件路径
    obj_path = MESH_DIR / f"{scene_group}.obj"
    pcd_path = PCD_DIR / f"{scene_group}.pcd"
    
    print(f"📂 文件路径:")
    print(f"   OBJ 输入: {obj_path}")
    print(f"   PCD 输出: {pcd_path}")
    
    # 检查 OBJ 文件
    if not obj_path.exists():
        print(f"\n❌ 错误: OBJ 文件不存在: {obj_path}")
        print(f"💡 可用场景:")
        for f in sorted(MESH_DIR.glob("*.obj")):
            print(f"   - {f.stem}")
        return
    
    # 检查缓存
    if not args.no_cache:
        print(f"\n🔍 检查偏移量缓存...")
        offsets_db = load_scene_offsets(OFFSETS_FILE)
        
        if scene_group in offsets_db.get('scenes', {}):
            existing_data = offsets_db['scenes'][scene_group]
            existing_offset = existing_data['pct_to_gazebo_offset']
            gazebo_offset = np.array([existing_offset['x'], existing_offset['y'], existing_offset['z']])
            
            print(f"✓ 找到缓存偏移量: ({gazebo_offset[0]:.2f}, {gazebo_offset[1]:.2f}, {gazebo_offset[2]:.2f})")
            print(f"   上次更新: {existing_data.get('last_updated', 'N/A')}")
            
            if pcd_path.exists():
                print(f"✓ PCD 文件已存在")
                print(f"⏭️  跳过转换（使用缓存）")
                
                print("\n" + "="*60)
                print("✅ 完成（使用缓存）⚡")
                print("="*60)
                print(f"场景: {scene_group}")
                print(f"点云: {pcd_path}")
                print(f"偏移: ({gazebo_offset[0]:.2f}, {gazebo_offset[1]:.2f}, {gazebo_offset[2]:.2f})")
                return
            else:
                print(f"⚠️  PCD 不存在，需要重新生成")
        else:
            print(f"ℹ️  场景 {scene_group} 首次处理")
    else:
        print(f"\n🔄 强制重新转换模式")
    
    # 执行转换
    print(f"\n" + "="*60)
    print("🔄 Mesh 转换为点云")
    print("="*60)
    pcd, gazebo_offset = mesh_to_pcd(
        obj_path=str(obj_path),
        pcd_path=str(pcd_path),
        num_points=args.num_points,
        scale_factor=0.01,
        y_up_to_z_up=True
    )
    
    # 保存偏移量到数据库
    print(f"\n" + "="*60)
    print("💾 保存偏移量到数据库")
    print("="*60)
    save_scene_offset(scene_group, gazebo_offset, OFFSETS_FILE)
    
    print("\n" + "="*60)
    print("✅ 转换完成！")
    print("="*60)
    print(f"场景: {scene_group}")
    print(f"点云: {pcd_path}")
    print(f"偏移: ({gazebo_offset[0]:.2f}, {gazebo_offset[1]:.2f}, {gazebo_offset[2]:.2f})")
    print(f"数据库: {OFFSETS_FILE}")


if __name__ == "__main__":
    main()
