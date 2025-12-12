import pathlib
import sys
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Path

from utils import traj2ros
from planner_wrapper import TomogramPlanner

# sys.path.append('../')
parent_dir = pathlib.Path(__file__).resolve().parent
if str(parent_dir) not in sys.path:
    sys.path.append(str(parent_dir))

from config import Config

class PCTPlanner(Node):
    def __init__(self, cfg: Config, tomo_file, start_pos, end_pos):
        super().__init__('pct_planner')
        self.cfg = cfg
        self.tomo_file = tomo_file
        self.start_pos = start_pos
        self.end_pos = end_pos

        self.declare_parameter("rsg_root", None)

        rsg_root_param = self.get_parameter("rsg_root")

        if rsg_root_param.value is None:
            raise ValueError("Missing required parameter: rsg_root")
        
        self.rsg_root = rsg_root_param.get_parameter_value().string_value
        
    
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.path_pub = self.create_publisher(Path, "/pct_path", qos)
        self.planner = TomogramPlanner(cfg, self.rsg_root)

        self.pct_plan()

    def pct_plan(self):
        self.planner.loadTomogram(self.tomo_file)

        traj_3d = self.planner.plan(self.start_pos, self.end_pos)
        if traj_3d is not None:
            self.path_pub.publish(traj2ros(traj_3d))
            self.get_logger().info("Trajectory published")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Plaza', help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
    args = parser.parse_args()

    cfg = Config()

    match args.scene:
        case 'Spiral':
            tomo_file = 'spiral0.3_2'
            start_pos = np.array([-16.0, -6.0], dtype=np.float32)
            end_pos = np.array([-26.0, -5.0], dtype=np.float32)
        case 'Building':
            tomo_file = 'building2_9'
            start_pos = np.array([5.0, 5.0], dtype=np.float32)
            end_pos = np.array([-6.0, -1.0], dtype=np.float32)
        case 'Plaza':
            tomo_file = 'plaza3_10'
            start_pos = np.array([0.0, 0.0], dtype=np.float32)
            end_pos = np.array([23.0, 10.0], dtype=np.float32)
        case _:
            raise ValueError('Invalid scene name')

    rclpy.init(args=None)

    node = PCTPlanner(cfg, tomo_file, start_pos, end_pos)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()