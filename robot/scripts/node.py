#!/usr/bin/env python

# Standard library imports
import math
import sys

# ROS imports
import rospy
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from enum import Enum
from typing import Optional, List, Tuple


class RobotState(Enum):
    PATROL = 1  # Normal patrol mode
    ALERT = 2  # Responding to alert
    SEARCH = 3  # Searching for hazard


current_state: RobotState = RobotState.PATROL  # Current robot state
alert_pose: Optional[Pose] = None  # Pose to respond to in ALERT state
current_goal: Optional[int] = None  # Current goal index or "ALERT"
current_position: Optional[Pose] = None  # Latest known robot position


# Debug flag to disable patrol mode
DEBUG: bool = False  # If True, disables patrol mode for testing


# Function to load waypoints from a txt file
def load_waypoints_from_file(filename: str) -> List[Pose]:
    """
    Loads waypoints from a text file. Each line should be in the format:
    name: x, y
    Returns a list of Pose objects.
    """
    waypoints = []
    try:
        with open(filename, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                # Expected format: `name: x, y`
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
                # Default values
                waypoints.append(make_waypoint(x, y, 0.0, 0.0, 0.0, 0.0, 1.0))
    except Exception as e:
        rospy.logerr(f"Failed to load waypoints from {filename}: {e}")
    return waypoints


def make_waypoint(
    x: float, y: float, z: float, ox: float, oy: float, oz: float, ow: float
) -> Pose:
    """
    Creates a Pose object from position and orientation values.
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


waypoints: List[Pose] = []  # List of loaded waypoints


def get_waypoint_path(current_idx: int, target_idx: int, total: int, current_pose: Optional[Pose] = None) -> List[int]:
    """
    Computes the shortest path (forward or backward) between two waypoints.
    If current_pose is provided, treats it as a virtual waypoint at index 'total'.
    Returns a list of waypoint indices (with 'VIRTUAL' for the current position if used).
    """
    # If current_pose is provided, treat it as a virtual waypoint at index 'total'
    use_virtual = current_pose is not None
    pool_size = total + 1 if use_virtual else total

    # Forward path
    fwd_path = []
    idx = current_idx
    while idx != target_idx:
        fwd_path.append(idx)
        idx = (idx + 1) % pool_size
    fwd_path.append(target_idx)
    fwd_len = len(fwd_path)

    # Backward path
    bwd_path = []
    idx = current_idx
    while idx != target_idx:
        bwd_path.append(idx)
        idx = (idx - 1 + pool_size) % pool_size
    bwd_path.append(target_idx)
    bwd_len = len(bwd_path)

    # If using virtual waypoint, replace its index with a string for clarity
    if use_virtual:
        fwd_path = ["VIRTUAL" if i == total else i for i in fwd_path]
        bwd_path = ["VIRTUAL" if i == total else i for i in bwd_path]

    # Choose shortest
    if fwd_len <= bwd_len:
        return fwd_path
    else:
        return bwd_path


def go_to_waypoint(client: actionlib.SimpleActionClient, target_waypoint: int) -> None:
    """
    Navigates the robot from its current position to the target waypoint,
    following the shortest path through waypoints.
    If the last waypoint in the path is farther than the target_waypoint, ignore it.
    """
    global current_position
    if current_position is None:
        rospy.logwarn("Current position unknown, cannot navigate.")
        return
    start_idx, _ = get_nearest_waypoint(current_position)
    total = len(waypoints)
    path = get_waypoint_path(start_idx, target_waypoint, total, current_position)

    # Remove last waypoint if it's farther than the target_waypoint
    if len(path) > 1:
        last = path[-1]
        prev = path[-2]
        # Only compare if both are valid indices (not 'VIRTUAL')
        if last != "VIRTUAL" and prev != "VIRTUAL":
            last_dist = math.hypot(
                waypoints[last].position.x - waypoints[target_waypoint].position.x,
                waypoints[last].position.y - waypoints[target_waypoint].position.y,
            )
            prev_dist = math.hypot(
                waypoints[prev].position.x - waypoints[target_waypoint].position.x,
                waypoints[prev].position.y - waypoints[target_waypoint].position.y,
            )
            if last_dist > prev_dist:
                rospy.loginfo(f"Ignoring last waypoint {last} as it is farther from target {target_waypoint} than previous waypoint {prev}.")
                path = path[:-1]

    rospy.loginfo(
        f"Navigating from waypoint {start_idx} to {target_waypoint} via path: {path}"
    )
    for idx, waypoint in enumerate(path):
        send_patrol_goal(client, waypoint)
        # Wait until goal is reached
        goal_pose = waypoints[waypoint]
        goal_active = True
        while goal_active and not rospy.is_shutdown():
            state = client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED or is_near_goal(
                goal_pose, current_position
            ):
                rospy.loginfo(f"Reached waypoint {waypoint}")
                goal_active = False
            elif state == actionlib.GoalStatus.ABORTED:
                rospy.logwarn(f"Goal aborted at waypoint {waypoint}, retrying...")
                send_patrol_goal(client, waypoint)
            rospy.sleep(0.2)
        # Turn towards next waypoint if not last
        if waypoint != target_waypoint and idx + 1 < len(path):
            next_wp = waypoints[path[idx + 1]]
            if current_position is not None:
                client.cancel_all_goals()
                turn_to_position(current_position, next_wp)


def turn_robot(angular_z: float, duration: float = 1.0) -> None:
    """
    Publishes Twist messages to /cmd_vel to turn the robot at a fixed angular velocity for a given duration.
    """
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = angular_z
    rate = rospy.Rate(10)  # 10 Hz
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        pub.publish(twist)
        rate.sleep()
    # Stop turning after duration
    twist.angular.z = 0.0
    pub.publish(twist)


def turn_to_position(current_position: Pose, next_position: Pose) -> None:
    """
    Turns the robot to face the direction of the next waypoint.
    """
    dx = next_position.position.x - current_position.position.x
    dy = next_position.position.y - current_position.position.y
    desired_yaw = math.atan2(dy, dx)
    import tf.transformations

    q = [
        current_position.orientation.x,
        current_position.orientation.y,
        current_position.orientation.z,
        current_position.orientation.w,
    ]
    _, _, current_yaw = tf.transformations.euler_from_quaternion(q)
    yaw_diff = desired_yaw - current_yaw
    # Normalize angle to [-pi, pi]
    while yaw_diff > math.pi:
        yaw_diff -= 2 * math.pi
    while yaw_diff < -math.pi:
        yaw_diff += 2 * math.pi
    # Always turn with fixed angular_z
    angular_z = 0.3 if yaw_diff >= 0 else -0.3
    turn_duration = abs(yaw_diff) / 0.3
    try:
        wp = waypoints.index(next_position)
        rospy.loginfo(
            f"Turning robot to waypoint={wp} with angular_z={angular_z} for {turn_duration:.2f}s"
        )
    except ValueError:
        rospy.loginfo(
            f"Turning robot to position={next_position} with angular_z={angular_z} for {turn_duration:.2f}s"
        )
    turn_robot(angular_z, duration=turn_duration)


def turn_degree(degrees: float) -> None:
    """
    Turns the robot by a specified number of degrees (positive for left, negative for right).
    """
    radians = math.radians(degrees)
    # Use sign of radians for direction, magnitude for speed (fixed at 0.3)
    angular_z = 0.3 if radians >= 0 else -0.3
    # Duration is proportional to angle
    turn_duration = abs(radians) / 0.3
    rospy.loginfo(
        f"Turning robot {degrees} degrees ({radians:.2f} rad) with angular_z={angular_z} for {turn_duration:.2f}s"
    )
    turn_robot(angular_z, duration=turn_duration)


def get_nearest_waypoint(pose: Pose) -> Tuple[int, float]:
    """
    Finds the nearest waypoint to the given pose.
    Returns (index, distance).
    """
    min_dist = float("inf")
    min_idx = -1
    for idx, wp in enumerate(waypoints):
        dx = wp.position.x - pose.position.x
        dy = wp.position.y - pose.position.y
        dist = math.hypot(dx, dy)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx
    return min_idx, min_dist


def alert_callback(msg: Pose) -> None:
    """
    ROS subscriber callback for alert messages.
    Sets robot state to ALERT and stores alert position.
    """
    global current_state, alert_pose
    rospy.loginfo(f"Alert received at position: x={msg.position.x}, y={msg.position.y}")
    alert_pose = msg
    current_state = RobotState.ALERT


def pose_callback(msg: PoseWithCovarianceStamped) -> None:
    """
    ROS subscriber callback for robot pose updates.
    Updates current_position.
    """
    global current_position
    current_position = msg.pose.pose
    return


def is_near_goal(
    goal_pose: Pose, current_pose: Optional[Pose], threshold: float = 0.75
) -> bool:
    """
    Checks if the robot is within a threshold distance of the goal pose.
    """
    if current_pose is None:
        return False
    dx = goal_pose.position.x - current_pose.position.x
    dy = goal_pose.position.y - current_pose.position.y
    distance = math.hypot(dx, dy)
    return distance < threshold


def send_patrol_goal(client: actionlib.SimpleActionClient, waypoint_idx: int) -> int:
    """
    Sends a MoveBaseGoal to the navigation stack for the given waypoint index.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = waypoints[waypoint_idx]
    rospy.loginfo(f"Patrolling to waypoint: {waypoint_idx} at {goal.target_pose.pose}")
    client.send_goal(goal)
    return waypoint_idx


def send_alert_goal(client: actionlib.SimpleActionClient, alert_pose: Pose) -> None:
    """
    Sends a MoveBaseGoal to the navigation stack for the alert position.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = alert_pose
    rospy.loginfo(f"Sending ALERT goal: {alert_pose}")
    client.send_goal(goal)


def handle_patrol(
    client: actionlib.SimpleActionClient,
    goal_active: bool,
    current_goal: Optional[int],
    waypoint_idx: int,
) -> Tuple[bool, Optional[int], int]:
    """
    Handles robot behavior in PATROL state.
    Sends patrol goals, checks for completion, and turns towards next waypoint.
    Returns updated (goal_active, current_goal, waypoint_idx).
    """
    global current_position
    goal_pose = waypoints[waypoint_idx]
    if not goal_active or current_goal != waypoint_idx:
        send_patrol_goal(client, waypoint_idx)
        goal_active = True
        current_goal = waypoint_idx

    state = client.get_state()
    if (
        state == actionlib.GoalStatus.SUCCEEDED
        or goal_active
        and is_near_goal(goal_pose, current_position)
    ):
        rospy.loginfo(f"Reached waypoint: {waypoint_idx}")
        waypoint_idx = (waypoint_idx + 1) % len(waypoints)
        goal_active = False
        current_goal = None
        # Turn towards the next waypoint
        next_wp = waypoints[waypoint_idx]
        if current_position is not None:
            client.cancel_all_goals()
            turn_to_position(current_position, next_wp)
    elif state == actionlib.GoalStatus.ABORTED:
        rospy.logwarn(f"Goal aborted, retrying waypoint: {waypoint_idx}")
        goal_active = False
        current_goal = None
    return goal_active, current_goal, waypoint_idx


def handle_alert(
    client: actionlib.SimpleActionClient,
    goal_active: bool,
    current_goal: Optional[str],
    alert_pose: Pose,
) -> Tuple[bool, Optional[str], bool]:
    """
    Handles robot behavior in ALERT state.
    Navigates to alert position, turns to face it, and returns status.
    Returns updated (goal_active, current_goal, alert_done).
    """
    global current_position
    if not goal_active or current_goal != "ALERT":
        client.cancel_all_goals()
        rospy.loginfo(f"Receiving ALERT goal: {alert_pose}")
        if alert_pose is not None and current_position is not None:
            target_idx, _ = get_nearest_waypoint(alert_pose)
            go_to_waypoint(client, target_idx)
            turn_to_position(current_position, alert_pose)
        send_alert_goal(client, alert_pose)
        goal_active = True
        current_goal = "ALERT"

    state = client.get_state()
    alert_done = False

    if (
        state == actionlib.GoalStatus.SUCCEEDED
        or goal_active
        and is_near_goal(alert_pose, current_position)
    ):
        rospy.loginfo("Reached alert position.")
        rospy.loginfo(f"{current_position}")
        goal_active = False
        current_goal = None
        alert_done = True
        client.cancel_all_goals()
    elif state == actionlib.GoalStatus.ABORTED:
        rospy.logwarn("Alert goal aborted, retrying.")
        goal_active = False
        current_goal = None
    return goal_active, current_goal, alert_done


def handle_search(client: actionlib.SimpleActionClient) -> bool:
    """
    Handles robot behavior in SEARCH state.
    Performs a search and returns True when done.
    """
    global current_position
    # TODO: implement a more complex search pattern
    turn_degree(45)
    turn_degree(-90)
    turn_degree(45)
    # After searching, return to patrol mode
    rospy.loginfo("Search complete, returning to patrol mode.")
    return True  # Indicate that search is done


def robot_statemachine() -> bool:
    """
    Main robot state machine loop.
    Handles transitions between PATROL, ALERT, and SEARCH states.
    """
    global current_state, alert_pose, current_goal, current_position, DEBUG

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    if not DEBUG:
        nearest_waypoint = get_nearest_waypoint(current_position)
        waypoint_idx = nearest_waypoint[0]
        rospy.loginfo(
            f"Starting at nearest waypoint: {waypoint_idx} of distance: {nearest_waypoint[1]}"
        )
    goal_active = False

    rate = rospy.Rate(2)  # 2 Hz loop

    while not rospy.is_shutdown():
        if current_state == RobotState.ALERT and alert_pose is not None:
            goal_active, current_goal, alert_done = handle_alert(
                client, goal_active, current_goal, alert_pose
            )
            if alert_done:
                current_state = RobotState.SEARCH
                alert_pose = None

        elif current_state == RobotState.PATROL and not DEBUG:
            goal_active, current_goal, waypoint_idx = handle_patrol(
                client, goal_active, current_goal, waypoint_idx
            )

        elif current_state == RobotState.SEARCH:
            rospy.loginfo("Searching for fire hazard...")

            search_done = handle_search(client)
            if search_done:
                current_state = RobotState.PATROL
                nearest_waypoint = get_nearest_waypoint(current_position)
                waypoint_idx = nearest_waypoint[0]

        rate.sleep()

    rospy.loginfo("Shutting down the robot statemachine.")
    client.cancel_all_goals()
    return False


if __name__ == "__main__":
    """
    Main entry point for the robot node.
    Initializes ROS node, loads waypoints, subscribes to topics, and starts state machine.
    """
    # Check for debug argument
    if "--debug" in sys.argv or "-d" in sys.argv:
        DEBUG = True
        rospy.loginfo("[DEBUG] Patrol mode will be disabled.")

    rospy.init_node("check_odometry")
    odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    alert_sub = rospy.Subscriber("/alert", Pose, alert_callback)

    # Parse waypoints filename argument
    waypoints_file = None
    if "--waypoints" in sys.argv:
        try:
            idx = sys.argv.index("--waypoints")
            waypoints_file = sys.argv[idx + 1]
        except (ValueError, IndexError):
            rospy.logerr("Usage: --waypoints <filename>")
            exit(1)
    if waypoints_file:
        waypoints = load_waypoints_from_file(waypoints_file)
        if not waypoints:
            rospy.logerr("No waypoints loaded. Exiting.")
            exit(1)
    else:
        rospy.logerr("No waypoints file provided. Use --waypoints <filename>.")
        exit(1)

    # Check for go_to_waypoint argument
    if "--goto" in sys.argv:
        try:
            idx = sys.argv.index("--goto")
            target_idx = int(sys.argv[idx + 1])
            client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            client.wait_for_server()
            go_to_waypoint(client, target_idx)
        except (ValueError, IndexError) as e:
            rospy.logerr(e)
            rospy.logerr("Usage: --goto <waypoint_index>")
            exit(1)
    try:
        result = robot_statemachine()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

    rospy.spin()
