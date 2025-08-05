import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from states import RobotState
from navigation import (
    get_nearest_waypoint,
    is_near_goal,
    send_patrol_goal,
    send_alert_goal,
)
from motion import turn_to_position, turn_degree, go_to_position
import global_vars


def handle_patrol(client, goal_active, current_goal, waypoint_idx):
    goal_pose = global_vars.waypoints[waypoint_idx]
    if not goal_active or current_goal != waypoint_idx:
        send_patrol_goal(client, waypoint_idx)
        goal_active = True
        current_goal = waypoint_idx
    state = client.get_state()
    if (
        state == actionlib.GoalStatus.SUCCEEDED
        or goal_active
        and is_near_goal(goal_pose, global_vars.current_position)
    ):
        rospy.loginfo(f"Reached waypoint: {waypoint_idx}")
        waypoint_idx = (waypoint_idx + 1) % len(global_vars.waypoints)
        goal_active = False
        current_goal = None
        next_wp = global_vars.waypoints[waypoint_idx]
        if global_vars.current_position is not None:
            client.cancel_all_goals()
            turn_to_position(global_vars.current_position, next_wp)
    elif state == actionlib.GoalStatus.ABORTED:
        rospy.logwarn(f"Goal aborted, retrying waypoint: {waypoint_idx}")
        goal_active = False
        current_goal = None
    return goal_active, current_goal, waypoint_idx


def handle_alert(client, goal_active, current_goal, alert_pose):
    if not goal_active or current_goal != "ALERT":
        client.cancel_all_goals()
        rospy.loginfo(f"Receiving ALERT goal: {alert_pose}")
        if alert_pose is not None and global_vars.current_position is not None:
            go_to_position(client, alert_pose)
            turn_to_position(global_vars.current_position, alert_pose)
        send_alert_goal(client, alert_pose)
        goal_active = True
        current_goal = "ALERT"
    state = client.get_state()
    alert_done = False
    if (
        state == actionlib.GoalStatus.SUCCEEDED
        or goal_active
        and is_near_goal(alert_pose, global_vars.current_position)
    ):
        rospy.loginfo("Reached alert position.")
        rospy.loginfo(f"{global_vars.current_position}")
        goal_active = False
        current_goal = None
        alert_done = True
        client.cancel_all_goals()
    elif state == actionlib.GoalStatus.ABORTED:
        rospy.logwarn("Alert goal aborted, retrying.")
        goal_active = False
        current_goal = None
    return goal_active, current_goal, alert_done


def handle_search(client):
    turn_degree(45)
    turn_degree(-90)
    turn_degree(45)
    rospy.loginfo("Search complete, returning to patrol mode.")
    return True


def robot_statemachine():
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    if not global_vars.debug:
        nearest_waypoint = get_nearest_waypoint(global_vars.current_position)
        waypoint_idx = nearest_waypoint[0]
        rospy.loginfo(
            f"Starting at nearest waypoint: {waypoint_idx} of distance: {nearest_waypoint[1]}"
        )
    goal_active = False
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if (
            global_vars.current_state == RobotState.ALERT
            and global_vars.alert_pose is not None
            and not global_vars.debug
        ):
            goal_active, global_vars.current_goal, alert_done = handle_alert(
                client, goal_active, global_vars.current_goal, global_vars.alert_pose
            )
            if alert_done:
                global_vars.current_state = RobotState.SEARCH
                global_vars.alert_pose = None
        elif global_vars.current_state == RobotState.PATROL and not global_vars.debug:
            goal_active, global_vars.current_goal, waypoint_idx = handle_patrol(
                client, goal_active, global_vars.current_goal, waypoint_idx
            )
        elif global_vars.current_state == RobotState.SEARCH and not global_vars.debug:
            rospy.loginfo("Searching for fire hazard...")
            search_done = handle_search(client)
            if search_done:
                global_vars.current_state = RobotState.PATROL
                nearest_waypoint = get_nearest_waypoint(global_vars.current_position)
                waypoint_idx = nearest_waypoint[0]
        rate.sleep()
    rospy.loginfo("Shutting down the robot statemachine.")
    client.cancel_all_goals()
    return False
