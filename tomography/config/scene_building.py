from .scene import ScenePCD, SceneMap, SceneTrav


class SceneBuilding():
    pcd = ScenePCD()
    pcd.file_name = 'AI_vol5_02_all.pcd'

    map = SceneMap()
    map.resolution = 0.10
    map.ground_h = -1.33
    map.slice_dh = 1.5

    trav = SceneTrav()
    trav.kernel_size = 5
    trav.interval_min = 0.35
    trav.interval_free = 0.50
    trav.slope_max = 0.50
    trav.step_max = 0.20
    trav.standable_ratio = 0.15
    trav.cost_barrier = 50.0
    trav.safe_margin = 0.25
    trav.inflation = 0.15

