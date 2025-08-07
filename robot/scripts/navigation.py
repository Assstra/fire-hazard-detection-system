import math
import actionlib
import rospy
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseGoal
from typing import List, Optional, Tuple
import global_vars


def load_waypoints_from_file(filename: str) -> List[Pose]:
    """
    Load waypoints from a file.
    Each line should be in the format: "waypoint_name: x, y".
    :param filename: The path to the file containing waypoints.
    :return: A list of Pose objects representing the waypoints.
    """
    waypoints = []
    try:
        with open(filename, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" not in line or "," not in line:
                    rospy.logwarn(f"Skipping invalid waypoint line: {line}")
                    continue
                _, coords_part = line.split(":", 1)
                coords = coords_part.split(",")
                if len(coords) < 2:
                    rospy.logwarn(f"Skipping invalid waypoint line: {line}")
                    continue
                try:
                    x = float(coords[0].strip())
                    y = float(coords[1].strip())
                except ValueError:
                    rospy.logwarn(f"Skipping invalid waypoint coordinates: {coords}")
                    continue
                waypoints.append(make_waypoint(x, y, 0.0, 0.0, 0.0, 0.0, 1.0))
    except Exception as e:
        rospy.logerr(f"Failed to load waypoints from {filename}: {e}")
    return waypoints


def make_waypoint(
    x: float, y: float, z: float, ox: float, oy: float, oz: float, ow: float
) -> Pose:
    """
    Create a Pose object representing a waypoint.
    :param x: X coordinate of the waypoint.
    :param y: Y coordinate of the waypoint.
    :param z: Z coordinate of the waypoint (usually 0).
    :param ox: Orientation x component.
    :param oy: Orientation y component.
    :param oz: Orientation z component.
    :param ow: Orientation w component.
    :return: A Pose object representing the waypoint.
    """
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow
    return pose


def get_waypoint_path(current_idx: int, target_idx: int, total: int) -> List[int]:
    """
    Get the path of waypoints from the current index to the target index.
    :param current_idx: The current waypoint index.
    :param target_idx: The target waypoint index.
    :param total: Total number of waypoints.
    :return: A list of indices representing the path from current to target.
    """
    fwd_path = []
    idx = current_idx
    while idx != target_idx:
        fwd_path.append(idx)
        idx = (idx + 1) % total
    fwd_path.append(target_idx)
    fwd_len = len(fwd_path)

    bwd_path = []
    idx = current_idx
    while idx != target_idx:
        bwd_path.append(idx)
        idx = (idx - 1 + total) % total
    bwd_path.append(target_idx)
    bwd_len = len(bwd_path)

    if fwd_len <= bwd_len:
        return fwd_path
    else:
        return bwd_path


def get_nearest_waypoint(pose: Pose) -> Tuple[int, float]:
    """
    Find the nearest waypoint to a given pose.
    :param pose: The pose from which to find the nearest waypoint.
    :return: A tuple containing the index of the nearest waypoint and its distance from the pose.
    """
    min_dist = float("inf")
    min_idx = -1
    for idx, wp in enumerate(global_vars.waypoints):
        dx = wp.position.x - pose.position.x
        dy = wp.position.y - pose.position.y
        dist = math.hypot(dx, dy)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx
    return min_idx, min_dist


def send_patrol_goal(client: actionlib.SimpleActionClient, waypoint_idx: int) -> int:
    """
    Send a patrol goal to the move_base action server.
    :param client: The action client for move_base.
    :param waypoint_idx: The index of the waypoint to which the robot should patrol.
    :return: The index of the waypoint that was sent as a goal.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = global_vars.waypoints[waypoint_idx]
    rospy.loginfo(f"Patrolling to waypoint: {waypoint_idx} at {goal.target_pose.pose}")
    client.send_goal(goal)
    return waypoint_idx


def send_target_goal(client: actionlib.SimpleActionClient, target_pose: Pose) -> None:
    """
    Send a target goal to the move_base action server.
    :param client: The action client for move_base.
    :param target_pose: The position to which the robot should go.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = target_pose
    rospy.loginfo(f"Sending ALERT goal: {target_pose}")
    client.send_goal(goal)


def is_near_target(
    target_pose: Pose, current_pose: Optional[Pose], threshold: float = 0.75
) -> bool:
    """
    Check if the robot is near the target position.
    :param target_pose: The target position to check against.
    :param current_pose: The current position of the robot.
    :param threshold: The distance threshold to consider as "near".
    :return: True if the robot is near the target, False otherwise.
    """
    if current_pose is None:
        return False
    dx = target_pose.position.x - current_pose.position.x
    dy = target_pose.position.y - current_pose.position.y
    distance = math.hypot(dx, dy)
    return distance < threshold
