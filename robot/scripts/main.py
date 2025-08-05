#!/usr/bin/env python

import global_vars
import rospy
import sys
import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from navigation import load_waypoints_from_file
from callbacks import alert_callback, pose_callback
from statemachine import robot_statemachine
from motion import go_to_position
from move_base_msgs.msg import MoveBaseAction


def main():
    global_vars.init_global_vars()
    rospy.init_node("check_odometry")
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber("/alert", Pose, alert_callback)

    if "--debug" in sys.argv or "-d" in sys.argv:
        global_vars.DEBUG = True
        rospy.loginfo("[DEBUG] Patrol mode will be disabled.")

    waypoints_file = None
    if "--waypoints" in sys.argv:
        try:
            idx = sys.argv.index("--waypoints")
            waypoints_file = sys.argv[idx + 1]
        except (ValueError, IndexError):
            rospy.logerr("Usage: --waypoints <filename>")
            exit(1)
    if waypoints_file:
        global_vars.waypoints = load_waypoints_from_file(waypoints_file)
        if not global_vars.waypoints:
            rospy.logerr("No waypoints loaded. Exiting.")
            exit(1)
    else:
        rospy.logerr("No waypoints file provided. Use --waypoints <filename>.")
        exit(1)

    if "--goto" in sys.argv:
        try:
            idx = sys.argv.index("--goto")
            target_idx = int(sys.argv[idx + 1])

            client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            client.wait_for_server()
            go_to_position(client, global_vars.waypoints[target_idx])
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


if __name__ == "__main__":
    main()
