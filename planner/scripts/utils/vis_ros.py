from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def traj2ros(traj, clock=None):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    if clock is not None:
        path_msg.header.stamp = clock.now().to_msg()

    for waypoint in traj:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        if clock is not None:
            pose.header.stamp = clock.now().to_msg()
        pose.pose.position.x = float(waypoint[0])
        pose.pose.position.y = float(waypoint[1])
        pose.pose.position.z = float(waypoint[2])
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    return path_msg