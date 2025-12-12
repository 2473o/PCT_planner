import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Path

from pct_planner.utils import traj2ros
from pct_planner.planner_wrapper import TomogramPlanner
from pct_planner.config import Config

class PCTPlanner(Node):
    def __init__(self):
        super().__init__('pct_planner')
        
        # Declare parameters
        self.declare_parameter("rsg_root", rclpy.Parameter.Type.STRING)
        self.declare_parameter("scene_name", "Plaza")
        
        # Get parameters
        rsg_root_param = self.get_parameter("rsg_root")
        if rsg_root_param.value is None:
            self.get_logger().error("Missing required parameter: rsg_root")
            raise ValueError("Missing required parameter: rsg_root")
        self.rsg_root = rsg_root_param.get_parameter_value().string_value
        
        scene_name = self.get_parameter("scene_name").get_parameter_value().string_value
        self.get_logger().info(f"Using scene: {scene_name}")
        
        # Scene configuration
        self.configure_scene(scene_name)
        
        self.cfg = Config()
        
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.path_pub = self.create_publisher(Path, "/pct_path", qos)
        self.planner = TomogramPlanner(self.cfg, self.rsg_root)

        # Plan immediately
        self.pct_plan()

    def configure_scene(self, scene_name):
        if scene_name == 'Spiral':
            self.tomo_file = 'spiral0.3_2'
            self.start_pos = np.array([-16.0, -6.0], dtype=np.float32)
            self.end_pos = np.array([-26.0, -5.0], dtype=np.float32)
        elif scene_name == 'Building':
            self.tomo_file = 'building2_9'
            self.start_pos = np.array([5.0, 5.0], dtype=np.float32)
            self.end_pos = np.array([-6.0, -1.0], dtype=np.float32)
        elif scene_name == 'Plaza':
            self.tomo_file = 'plaza3_10'
            self.start_pos = np.array([0.0, 0.0], dtype=np.float32)
            self.end_pos = np.array([23.0, 10.0], dtype=np.float32)
        else:
            self.get_logger().error(f"Invalid scene name: {scene_name}")
            raise ValueError(f"Invalid scene name: {scene_name}")

    def pct_plan(self):
        self.get_logger().info(f"Loading tomogram: {self.tomo_file}")
        self.planner.loadTomogram(self.tomo_file)

        self.get_logger().info(f"Planning from {self.start_pos} to {self.end_pos}")
        traj_3d = self.planner.plan(self.start_pos, self.end_pos)
        
        if traj_3d is not None:
            self.path_pub.publish(traj2ros(traj_3d))
            self.get_logger().info("Trajectory published")
        else:
            self.get_logger().warn("Failed to generate trajectory")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PCTPlanner()
        rclpy.spin(node)
    except ValueError as e:
        print(f"Error initializing node: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
