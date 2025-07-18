#!/usr/bin/env python

import math
import rospy
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from enum import Enum
from typing import Optional, List, Tuple


class RobotState(Enum):
    PATROL = 1
    ALERT = 2

current_state: RobotState = RobotState.PATROL
alert_pose: Optional[Pose] = None
current_goal: Optional[int] = None  # can also be "ALERT" (str), but default is int
current_position: Optional[Pose] = None

# Debug flag to disable patrol mode
DEBUG: bool = False


def make_waypoint(
    x: float, y: float, z: float, ox: float, oy: float, oz: float, ow: float
) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow
    return pose


waypoint_SW: Pose = make_waypoint(86.0, 0.0, 0.0, 0, 0, 0, 1)
waypoint_S1: Pose = make_waypoint(87.0, 12.0, 0.0, 0, 0, 0.707, 0.707)
waypoint_S2: Pose = make_waypoint(85.0, 20.0, 0.0, 0, 0, 0.707, 0.707)
waypoint_SE: Pose = make_waypoint(84.5, 50.0, 0.0, 0, 0, 0.707, 0.707)
waypoint_E1: Pose = make_waypoint(57.5, 49.0, 0.0, 0, 0, -1, 0)
waypoint_E2: Pose = make_waypoint(55, 43.0, 0.0, 0, 0, -1, 0)
waypoint_E3: Pose = make_waypoint(43.0, 43.0, 0.0, 0, 0, -1, 0)
waypoint_E4: Pose = make_waypoint(35.0, 44.0, 0.0, 0, 0, -1, 0)
waypoint_E5: Pose = make_waypoint(15.0, 45.0, 0.0, 0, 0, -1, 0)
waypoint_E6: Pose = make_waypoint(15.0, 51.0, 0.0, 0, 0, -1, 0)
waypoint_E7: Pose = make_waypoint(-35.0, 52.5, 0.0, 0, 0, -1, 0)
waypoint_E8: Pose = make_waypoint(-35.0, 45.0, 0.0, 0, 0, -1, 0)
waypoint_E9: Pose = make_waypoint(-68.0, 47.5, 0.0, 0, 0, -1, 0)
waypoint_E10: Pose = make_waypoint(-68.0, 55, 0.0, 0, 0, -1, 0)
waypoint_NE: Pose = make_waypoint(-105.0, 52.5, 0.0, 0, 0, -1, 0)
waypoint_NW: Pose = make_waypoint(-105.0, 5.0, 0.0, 0, 0, -0.707, 0.707)
waypoint_W1: Pose = make_waypoint(-80.0, 4.0, 0.0, 0, 0, -0.707, 0.707)
waypoint_W2: Pose = make_waypoint(-50.0, 3.0, 0.0, 0, 0, -0.707, 0.707)
waypoint_W3: Pose = make_waypoint(-35.0, 2.0, 0.0, 0, 0, -0.707, 0.707)
waypoint_W4: Pose = make_waypoint(20.0 ,0.0, 0.0, 0, 0, -0.707, 0.707)

waypoints: List[Pose] = [
    waypoint_SW,
    waypoint_S1,
    waypoint_S2,
    waypoint_SE,
    waypoint_E1,
    waypoint_E2,
    waypoint_E3,
    waypoint_E4,
    waypoint_E5,
    waypoint_E6,
    waypoint_E7,
    waypoint_E8,
    waypoint_E9,
    waypoint_E10,
    waypoint_NE,
    waypoint_NW,
    waypoint_W1,
    waypoint_W2,
    waypoint_W3,
    waypoint_W4,
]


def get_nearest_waypoint(pose: Pose) -> Tuple[int, float]:
    """
    Returns the index and distance of the nearest waypoint to the given pose.
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
    global current_state, alert_pose
    rospy.loginfo(
        "Alert received at position: x={}, y={}".format(msg.position.x, msg.position.y)
    )
    alert_pose = msg
    current_state = RobotState.ALERT


def pose_callback(msg: PoseWithCovarianceStamped) -> None:
    global current_position
    current_position = msg.pose.pose
    return


def is_near_goal(
    goal_pose: Pose, current_pose: Optional[Pose], threshold: float = 0.75
) -> bool:
    if current_pose is None:
        return False
    dx = goal_pose.position.x - current_pose.position.x
    dy = goal_pose.position.y - current_pose.position.y
    distance = math.hypot(dx, dy)
    return distance < threshold


def send_patrol_goal(client: actionlib.SimpleActionClient, waypoint_idx: int) -> int:
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = waypoints[waypoint_idx]
    rospy.loginfo("Patrolling to waypoint: {} at {}".format(waypoint_idx, goal.target_pose.pose))
    client.send_goal(goal)
    return waypoint_idx


def send_alert_goal(client: actionlib.SimpleActionClient, alert_pose: Pose) -> None:
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = alert_pose
    rospy.loginfo("Sending ALERT goal: {}".format(alert_pose))
    client.send_goal(goal)


def handle_patrol(
    client: actionlib.SimpleActionClient,
    goal_active: bool,
    current_goal: Optional[int],
    waypoint_idx: int,
) -> Tuple[bool, Optional[int], int]:
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
        rospy.loginfo("Reached waypoint: {}".format(waypoint_idx))
        waypoint_idx = (waypoint_idx + 1) % len(waypoints)
        goal_active = False
        current_goal = None
    elif state == actionlib.GoalStatus.ABORTED:
        rospy.logwarn("Goal aborted, retrying waypoint: {}".format(waypoint_idx))
        goal_active = False
        current_goal = None
    return goal_active, current_goal, waypoint_idx


def handle_alert(
    client: actionlib.SimpleActionClient,
    goal_active: bool,
    current_goal: Optional[str],
    alert_pose: Pose,
) -> Tuple[bool, Optional[str], bool]:  
    global current_position
    if not goal_active or current_goal != "ALERT":
        client.cancel_all_goals()
        send_alert_goal(client, alert_pose)
        goal_active = True
        current_goal = "ALERT"

    state = client.get_state()
    alert_done = False
    # TODO: handle camera turn

    if (
        state == actionlib.GoalStatus.SUCCEEDED
        or goal_active
        and is_near_goal(alert_pose, current_position)
    ):
        rospy.loginfo("Reached alert position.")
        rospy.loginfo(format(current_position))
        goal_active = False
        current_goal = None
        alert_done = True
        client.cancel_all_goals()
    elif state == actionlib.GoalStatus.ABORTED:
        rospy.logwarn("Alert goal aborted, retrying.")
        goal_active = False
        current_goal = None
    return goal_active, current_goal, alert_done


def robot_statemachine() -> bool:
    global current_state, alert_pose, current_goal, current_position, DEBUG

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    if not DEBUG:
        nearest_waypoint = get_nearest_waypoint(current_position)
        waypoint_idx = nearest_waypoint[0]
        rospy.loginfo("Starting at nearest waypoint: {} of distance: {}".format(waypoint_idx, nearest_waypoint[1]))
    goal_active = False

    rate = rospy.Rate(2)  # 2 Hz loop

    while not rospy.is_shutdown():
        if current_state == RobotState.ALERT and alert_pose is not None:
            goal_active, current_goal, alert_done = handle_alert(
                client, goal_active, current_goal, alert_pose
            )
            if alert_done:
                current_state = RobotState.PATROL
                alert_pose = None

        elif current_state == RobotState.PATROL and not DEBUG:
            goal_active, current_goal, waypoint_idx = handle_patrol(
                client, goal_active, current_goal, waypoint_idx
            )

        rate.sleep()

    rospy.loginfo("Shutting down the robot statemachine.")
    client.cancel_all_goals()
    return False


if __name__ == "__main__":
    # Check for debug argument
    if "--debug" in sys.argv or "-d" in sys.argv:
        DEBUG = True
        rospy.loginfo("[DEBUG] Patrol mode will be disabled.")

    rospy.init_node("check_odometry")
    odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    alert_sub = rospy.Subscriber("/alert", Pose, alert_callback)

    try:
        result = robot_statemachine()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

    rospy.spin()
