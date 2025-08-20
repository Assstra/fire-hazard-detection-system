from typing import Tuple
import rospy
import actionlib
import multiprocessing
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import Pose
from states import RobotState
from navigation import (
    get_nearest_waypoint,
    is_near_target,
    send_patrol_goal,
    send_target_goal,
)
from motion import turn_to_position, go_to_position, turn_degree
import global_vars
from search import search


def handle_patrol(
    client: actionlib.SimpleActionClient,
    goal_active: bool,
    current_goal: str,
    waypoint_idx: int,
) -> Tuple[bool, str, int]:
    """
    Handles the patrol state by sending the robot to the next waypoint.
    :param client: The action client for move_base.
    :param goal_active: Whether a goal is currently active.
    :param current_goal: The current goal being processed.
    :param waypoint_idx: The nearest waypoint to the goal.
    :return: Tuple of (goal_active, current_goal, waypoint_idx).
    """
    goal_pose = global_vars.waypoints[waypoint_idx]
    # Send the goal if no goal is active or if the current goal is not the same as the waypoint index
    if not goal_active or current_goal != waypoint_idx:
        send_patrol_goal(client, waypoint_idx)
        goal_active = True
        current_goal = waypoint_idx
    state = client.get_state()
    # Check if the goal has been reached or if the robot is near the goal
    # If yes, proceed to the next goal
    # If the goal is aborted, retry the current waypoint
    if (
        state == actionlib.GoalStatus.SUCCEEDED
        or goal_active
        and is_near_target(goal_pose, global_vars.current_position)
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


def handle_alert(
    client: actionlib.SimpleActionClient,
    goal_active: bool,
    current_goal: str,
    alert_pose: Pose,
) -> Tuple[bool, str, bool]:
    """
    Handles the alert state by sending the robot to the alert position.
    :param client: The action client for move_base.
    :param goal_active: Whether a goal is currently active.
    :param current_goal: The current goal being processed.
    :param alert_pose: The position to which the robot should go in case of an alert.
    :return: Tuple of (goal_active, current_goal, alert_done).
    """
    # Send the goal if no goal is active or if the current goal is not in the ALERT state
    if not goal_active or current_goal != "ALERT":
        client.cancel_all_goals()
        rospy.loginfo(f"Receiving ALERT goal: {alert_pose}")
        if alert_pose is not None and global_vars.current_position is not None:
            go_to_position(client, alert_pose)
            turn_to_position(global_vars.current_position, alert_pose)
        send_target_goal(client, alert_pose)
        goal_active = True
        current_goal = "ALERT"
    state = client.get_state()
    alert_done = False
    # Check if the goal has been reached or if the robot is near the goal
    # If yes, proceed to the next goal
    # If the goal is aborted, retry the current waypoint
    if (
        state == actionlib.GoalStatus.SUCCEEDED
        or goal_active
        and is_near_target(alert_pose, global_vars.current_position)
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


def handle_search() -> bool:
    """
    Handles the search for fire hazards by starting a separate process to
    communicate with the fire detection server.
    :return: True if the search was successful, False otherwise.
    """
    rospy.loginfo("Starting search for fire hazards...")
    if global_vars.host is None or global_vars.port is None:
        rospy.logwarn("Fire detection server host or port not set, cannot search.")
        return False

    parent_conn, child_conn = multiprocessing.Pipe()
    p = multiprocessing.Process(
        target=search, args=(global_vars.host, global_vars.port, child_conn)
    )
    p.start()

    rospy.loginfo("Started search process, waiting for events...")
    try:
        last_move = ""
        turn_angle = 15
        while p.is_alive():
            if parent_conn.poll(1):  # Wait up to 1 second for event
                event = parent_conn.recv()

                if event.get("type") == "rgb_detection":
                    if event.get("position") == "left":
                        if last_move != "" and last_move != "left":
                            turn_angle = turn_angle / 2
                        turn_degree(turn_angle)
                    elif event.get("position") == "right":
                        if last_move != "" and last_move != "right":
                            turn_angle = turn_angle / 2
                        turn_degree(-turn_angle)
                    elif event.get("position") == "center":
                        rospy.loginfo("Fire hazard detected in front, stopping search.")
                        # kill the connection process to the search server
                        p.kill()
                        break
                    else:
                        rospy.loginfo(
                            f"Unknown position detected: {event.get('position')}"
                        )
                    last_move = event.get("position")

                # Break on error
                if event.get("type") == "error":
                    rospy.logwarn(f"Search process error: {event.get('message')}")
                    # kill the connection process to the search server
                    p.kill()
                    return False
            else:
                rospy.loginfo("No events received, turning...")
                turn_degree(-45)
        # Drain any remaining events
        while parent_conn.poll():
            event = parent_conn.recv()
            rospy.loginfo(f"Search event: {event}")
    finally:
        p.join()
    rospy.loginfo("Search complete, returning to patrol mode.")
    return True


def robot_statemachine() -> bool:
    """
    Main function for the robot state machine.
    It handles the robot's states:
    ALERT, PATROL, and SEARCH.
    :return: False if the robot is shutting down, run continuously otherwise.
    """
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
        elif (
            global_vars.current_state == RobotState.PATROL
            and not global_vars.debug
            and not global_vars.alert_mode
        ):
            goal_active, global_vars.current_goal, waypoint_idx = handle_patrol(
                client, goal_active, global_vars.current_goal, waypoint_idx
            )
        elif global_vars.current_state == RobotState.SEARCH and not global_vars.debug:
            rospy.loginfo("Searching for fire hazard...")
            search_done = handle_search()
            if search_done:
                global_vars.current_state = RobotState.PATROL
                nearest_waypoint = get_nearest_waypoint(global_vars.current_position)
                waypoint_idx = nearest_waypoint[0]
        rate.sleep()
    rospy.loginfo("Shutting down the robot statemachine.")
    client.cancel_all_goals()
    return False
