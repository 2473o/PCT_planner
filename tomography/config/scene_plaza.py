from .scene import Scene


class ScenePlaza(Scene):
    def __init__(self):
        super().__init__()
        self.pcd.file_name = 'plaza3_10.pcd'

        self.map.resolution = 0.10
        self.map.ground_h = 0.0
        self.map.slice_dh = 0.5

        self.trav.kernel_size = 7
        self.trav.interval_min = 0.50
        self.trav.interval_free = 0.65
        self.trav.slope_max = 0.36
        self.trav.step_max = 0.17
        self.trav.standable_ratio = 0.2
        self.trav.cost_barrier = 50.0
        self.trav.safe_margin = 0.4
        self.trav.inflation = 0.2

