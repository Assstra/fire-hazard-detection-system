import math
import actionlib
from navigation import is_near_goal, send_patrol_goal
import rospy
from geometry_msgs.msg import Pose, Twist
from navigation import get_nearest_waypoint, get_waypoint_path
import global_vars
import tf.transformations


def turn_robot(angular_z: float, duration: float = 1.0) -> None:
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = angular_z
    rate = rospy.Rate(10)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0.0
    pub.publish(twist)


def turn_to_position(current_position: Pose, next_position: Pose) -> None:
    dx = next_position.position.x - current_position.position.x
    dy = next_position.position.y - current_position.position.y
    desired_yaw = math.atan2(dy, dx)

    q = [
        current_position.orientation.x,
        current_position.orientation.y,
        current_position.orientation.z,
        current_position.orientation.w,
    ]
    _, _, current_yaw = tf.transformations.euler_from_quaternion(q)
    yaw_diff = desired_yaw - current_yaw
    while yaw_diff > math.pi:
        yaw_diff -= 2 * math.pi
    while yaw_diff < -math.pi:
        yaw_diff += 2 * math.pi
    angular_z = 0.3 if yaw_diff >= 0 else -0.3
    turn_duration = abs(yaw_diff) / 0.3
    turn_robot(angular_z, duration=turn_duration)


def turn_degree(degrees: float) -> None:
    radians = math.radians(degrees)
    angular_z = 0.3 if radians >= 0 else -0.3
    turn_duration = abs(radians) / 0.3
    rospy.loginfo(
        f"Turning robot {degrees} degrees ({radians:.2f} rad) with angular_z={angular_z} for {turn_duration:.2f}s"
    )
    turn_robot(angular_z, duration=turn_duration)


def go_to_position(client: actionlib.SimpleActionClient, target: Pose) -> None:
    if global_vars.current_position is None:
        rospy.logwarn("Current position unknown, cannot navigate.")
        return
    start_idx, _ = get_nearest_waypoint(global_vars.current_position)
    target_waypoint, _ = get_nearest_waypoint(target)
    total = len(global_vars.waypoints)
    path = get_waypoint_path(start_idx, target_waypoint, total)
    if len(path) >= 1:
        last_wp_idx = path[-1]
        last_wp_pose = global_vars.waypoints[last_wp_idx]
        dist_to_target = math.hypot(
            target.position.x - global_vars.current_position.position.x,
            target.position.y - global_vars.current_position.position.y,
        )
        dist_to_last_wp = math.hypot(
            last_wp_pose.position.x - global_vars.current_position.position.x,
            last_wp_pose.position.y - global_vars.current_position.position.y,
        )
        if dist_to_target < dist_to_last_wp:
            rospy.loginfo(
                f"Removing last waypoint {last_wp_idx} from path because direct target is closer."
            )
            path = path[:-1]

    rospy.loginfo(f"Navigating from waypoint {start_idx} to {target} via path: {path}")
    for idx, waypoint in enumerate(path):
        send_patrol_goal(client, waypoint)
        goal_pose = global_vars.waypoints[waypoint]
        goal_active = True
        while goal_active and not rospy.is_shutdown():
            state = client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED or is_near_goal(
                goal_pose, global_vars.current_position
            ):
                rospy.loginfo(f"Reached waypoint {waypoint}")
                goal_active = False
            elif state == actionlib.GoalStatus.ABORTED:
                rospy.logwarn(f"Goal aborted at waypoint {waypoint}, retrying...")
                send_patrol_goal(client, waypoint)
            rospy.sleep(0.2)
        if waypoint != target and idx + 1 < len(path):
            next_wp = global_vars.waypoints[path[idx + 1]]
            if global_vars.current_position is not None:
                client.cancel_all_goals()
                turn_to_position(global_vars.current_position, next_wp)
