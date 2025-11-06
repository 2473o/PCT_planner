from .scene import Scene


class SceneSpiral(Scene):
    def __init__(self) -> None:
        super().__init__()

        self.pcd.file_name = 'spiral0.3_2.pcd'

        self.map.resolution = 0.20
        self.map.ground_h = 0.0
        self.map.slice_dh = 0.5

        self.trav.kernel_size = 7
        self.trav.interval_min = 0.50
        self.trav.interval_free = 0.65
        self.trav.slope_max = 0.40
        self.trav.step_max = 0.30
        self.trav.standable_ratio = 0.40
        self.trav.cost_barrier = 50.0
        self.trav.safe_margin = 1.2
        self.trav.inflation = 0.2

