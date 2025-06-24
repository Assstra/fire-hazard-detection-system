#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose


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
    rospy.loginfo("Alert received at position: x={}, y={}".format(msg.position.x, msg.position.y))
    
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    client.cancel_all_goals()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = msg

    rospy.loginfo("Sending goal to alert waypoint: {}".format(msg))
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        return None
    else:
        rospy.loginfo("Reached alert position: {}".format(msg))

def odom_callback(msg: Odometry):
    #rospy.loginfo(msg.pose.pose)
    return

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        for waypoint in waypoints:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = waypoint

            rospy.loginfo("Sending goal to waypoint: {}".format(waypoint))
            client.send_goal(goal)
            wait = client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                return None
            else:
                rospy.loginfo("Reached waypoint: {}".format(waypoint))


if __name__ == '__main__':
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    alert_sub = rospy.Subscriber('/alert', Pose, alert_callback)

    try:
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

    rospy.spin()