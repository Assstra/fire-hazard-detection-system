import math
import actionlib
import rospy
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseGoal
from typing import List, Optional, Tuple
import global_vars
import networkx as nx


def parse_from_file(filename: str) -> Tuple[List[Pose], nx.Graph]:
    """
    Load waypoints and build a weighted graph using networkx.
    Each line should be in the format: "waypoint_name: x, y".
    :param filename: The path to the file containing waypoints.
    :return: A list of Pose objects and a networkx Graph.
    """
    waypoints = []
    names = []
    edges = []
    with open(filename, 'r') as f:
        lines = f.readlines()

    edge_section = False
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            if line.startswith('# Edges'):
                edge_section = True
            continue
        if not edge_section:
            if ':' in line:
                name, coords = line.split(':')
                x, y = map(float, coords.split(','))
                waypoints.append(make_waypoint(x, y, 0.0, 0.0, 0.0, 0.0, 1.0))
                names.append(name.strip())
        else:
            if '-' in line:
                a, b = line.split('-')
                a, b = a.strip(), b.strip()
                edges.append((a, b))

    # Build graph with weights
    G = nx.Graph()
    for idx, name in enumerate(names):
        G.add_node(name, idx=idx)
    for a, b in edges:
        idx_a = names.index(a)
        idx_b = names.index(b)
        wp_a = waypoints[idx_a]
        wp_b = waypoints[idx_b]
        dist = math.hypot(wp_a.position.x - wp_b.position.x, wp_a.position.y - wp_b.position.y)
        G.add_edge(a, b, weight=dist)
    return waypoints, G


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


def dijkstra_path(edges: dict, start: str, goal: str) -> list:
    """
    Find shortest path using Dijkstra's algorithm.
    :param edges: Dict of adjacency list {node: [neighbors]}
    :param start: Start node name
    :param goal: Goal node name
    :return: List of node names representing the shortest path
    """
    import heapq
    queue = [(0, start, [start])]
    visited = set()
    while queue:
        cost, node, path = heapq.heappop(queue)
        if node == goal:
            return path
        if node in visited:
            continue
        visited.add(node)
        for neighbor in edges.get(node, []):
            if neighbor not in visited:
                heapq.heappush(queue, (cost + 1, neighbor, path + [neighbor]))
    return []

def get_waypoint_path(current_idx: int, target_idx: int, waypoints: list, graph: nx.Graph) -> list:
    """
    Get the shortest path of waypoints using networkx Dijkstra's algorithm.
    :param current_idx: The current waypoint index.
    :param target_idx: The target waypoint index.
    :param waypoints: List of Pose objects.
    :param graph: networkx Graph.
    :return: List of indices representing the path.
    """
    # Find node names by index
    start = None
    goal = None
    for node, data in graph.nodes(data=True):
        if data['idx'] == current_idx:
            start = node
        if data['idx'] == target_idx:
            goal = node
    if start is None or goal is None:
        return []
    path_names = nx.shortest_path(graph, start, goal, weight='weight')
    path_indices = [graph.nodes[name]['idx'] for name in path_names]
    return path_indices


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
