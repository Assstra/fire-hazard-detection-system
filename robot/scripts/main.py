#!/usr/bin/env python

import multiprocessing
import global_vars
import rospy
import sys
import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from navigation import load_waypoints_from_file
from callbacks import alert_callback, pose_callback
from statemachine import robot_statemachine
from motion import go_to_position
from serial_listener import main as serial_main
from move_base_msgs.msg import MoveBaseAction


def main():
    """
    Main function to initialize the robot state machine, subscribers, and handle command line arguments.
    """
    # Initialize global variables
    global_vars.init_global_vars()
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber("/alert", Pose, alert_callback)

    if "--debug" in sys.argv or "-d" in sys.argv:
        global_vars.debug = True
        rospy.loginfo("[DEBUG MODE] Patrol mode will be disabled.")
        rospy.loginfo("[DEBUG MODE] Alert will only be displayed.")

    if "--alert" in sys.argv or "-a" in sys.argv:
        global_vars.alert_mode = True
        rospy.loginfo("[ALERT MODE] Patrol mode will be disabled.")
        rospy.loginfo("[ALERT MODE] Only alerts and search will be processed.")

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

    if "--host" not in sys.argv or "--port" not in sys.argv:
        rospy.logwarn("No host or port provided. Search mode will not be available.")
    else:
        try:
            host_idx = sys.argv.index("--host")
            port_idx = sys.argv.index("--port")
            global_vars.host = sys.argv[host_idx + 1]
            global_vars.port = int(sys.argv[port_idx + 1])
        except (ValueError, IndexError) as e:
            rospy.logerr(e)
            rospy.logerr("Usage: --host <hostname> --port <port>")
            exit(1)

    # Start serial_listener
    p = multiprocessing.Process(target=serial_main)
    try:
        p.start()
        rospy.init_node("robot_statemachine")
        result = robot_statemachine()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    rospy.spin()
    p.kill()


if __name__ == "__main__":
    main()
