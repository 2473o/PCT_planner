from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
from rclpy.time import Time

def traj2ros(traj_3d):
    path = Path()
    path.header.stamp = Clock().now().to_msg()
    path.header.frame_id = "map"
    for i in range(traj_3d.shape[0]):
        pose = PoseStamped()
        pose.header.stamp = path.header.stamp
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = traj_3d[i, 0]
        pose.pose.position.y = traj_3d[i, 1]
        pose.pose.position.z = traj_3d[i, 2]
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path
