from .scene import ScenePCD, SceneMap, SceneTrav


class SceneBlocks():
    """自动生成的场景配置（可手动调整参数）"""
    
    pcd = ScenePCD()
    pcd.file_name = 'Blocks.pcd'

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
