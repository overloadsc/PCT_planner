from .scene import ScenePCD, SceneMap, SceneTrav


class SceneOldtown():
    pcd = ScenePCD()
    pcd.file_name = 'Oldtown.pcd'

    map = SceneMap()
    map.resolution = 0.25
    map.ground_h = -5.0 #-4.87
    map.slice_dh = 0.3  #层高切片

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 2.0
    trav.interval_free = 2.5
    trav.slope_max = 0.60
    trav.step_max = 0.25
    trav.standable_ratio = 0.10
    trav.cost_barrier = 50.0
    trav.safe_margin = 1.5
    trav.inflation = 1.0 #膨胀

