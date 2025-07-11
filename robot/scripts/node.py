#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from enum import Enum


class RobotState(Enum):
    PATROL = 1
    ALERT = 2


current_state = RobotState.PATROL
alert_pose = None
current_goal = None


def make_waypoint(x, y, z, ox, oy, oz, ow):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow
    return pose


waypoint_A = make_waypoint(85.0, 0.0, 0.0, 0, 0, 0.7071, 0.7071)
waypoint_B = make_waypoint(85.0, 50.0, 0.0, 0, 0, 1, 0)
waypoint_C = make_waypoint(-105.0, 50.0, 0.0, 0, 0, 0.7071, -0.7071)
waypoint_D = make_waypoint(-105.0, 0.0, 0.0, 0, 0, 0, -1)

waypoints = [waypoint_A, waypoint_B, waypoint_C, waypoint_D]


def alert_callback(msg: Pose):
    global current_state, alert_pose
    rospy.loginfo(
        "Alert received at position: x={}, y={}".format(msg.position.x, msg.position.y)
    )
    alert_pose = msg
    current_state = RobotState.ALERT


def odom_callback(msg: Odometry):
    # rospy.loginfo(msg.pose.pose)
    return


def send_patrol_goal(client, waypoint_idx):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = waypoints[waypoint_idx]
    rospy.loginfo("Patrolling to waypoint: {}".format(waypoint_idx))
    client.send_goal(goal)
    return waypoint_idx


def send_alert_goal(client, alert_pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = alert_pose
    rospy.loginfo("Sending ALERT goal: {}".format(alert_pose))
    client.send_goal(goal)


def handle_patrol(client, goal_active, current_goal, waypoint_idx):
    if not goal_active or current_goal != waypoint_idx:
        send_patrol_goal(client, waypoint_idx)
        goal_active = True
        current_goal = waypoint_idx

    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Reached waypoint: {}".format(waypoint_idx))
        waypoint_idx = (waypoint_idx + 1) % len(waypoints)
        goal_active = False
        current_goal = None
    elif state == actionlib.GoalStatus.ABORTED:
        rospy.logwarn("Goal aborted, retrying waypoint: {}".format(waypoint_idx))
        goal_active = False
        current_goal = None
    return goal_active, current_goal, waypoint_idx


def handle_alert(client, goal_active, current_goal, alert_pose):
    if not goal_active or current_goal != "ALERT":
        client.cancel_all_goals()
        send_alert_goal(client, alert_pose)
        goal_active = True
        current_goal = "ALERT"

    state = client.get_state()
    alert_done = False
    # TODO: handle camera turn
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Reached alert position.")
        goal_active = False
        current_goal = None
        alert_done = True
    elif state == actionlib.GoalStatus.ABORTED:
        rospy.logwarn("Alert goal aborted, retrying.")
        goal_active = False
        current_goal = None
    return goal_active, current_goal, alert_done


def robot_statemachine():
    global current_state, alert_pose, current_goal

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    waypoint_idx = 0
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

        elif current_state == RobotState.PATROL:
            goal_active, current_goal, waypoint_idx = handle_patrol(
                client, goal_active, current_goal, waypoint_idx
            )

        rate.sleep()

    rospy.loginfo("Shutting down the robot statemachine.")
    client.cancel_all_goals()
    return False


if __name__ == "__main__":
    rospy.init_node("check_odometry")
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
    alert_sub = rospy.Subscriber("/alert", Pose, alert_callback)

    try:
        result = robot_statemachine()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

    rospy.spin()
