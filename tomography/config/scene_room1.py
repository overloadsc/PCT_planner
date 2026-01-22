from .scene import ScenePCD, SceneMap, SceneTrav


class SceneRoom1():
    """室内场景配置"""
    
    pcd = ScenePCD()
    pcd.file_name = 'Room1.pcd'

    map = SceneMap()
    map.resolution = 0.10      # 室内需要更高精度
    map.ground_h = -1.0        # 根据实际点云调整
    map.slice_dh = 1.5         # 室内天花板高度

    trav = SceneTrav()
    trav.kernel_size = 5       # 更精细的检测
    trav.interval_min = 0.35   # 机器人通行高度（~0.5m机器人）
    trav.interval_free = 0.50  # 自由空间要求
    trav.slope_max = 0.50      # 室内坡度限制
    trav.step_max = 0.20       # 台阶高度
    trav.standable_ratio = 0.15
    trav.cost_barrier = 50.0
    trav.safe_margin = 0.25    # 室内空间紧凑，安全边距小
    trav.inflation = 0.15      # 障碍物膨胀小