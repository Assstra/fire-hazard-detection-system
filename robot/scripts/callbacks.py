import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from states import RobotState
import global_vars


def alert_callback(msg: Pose) -> None:
    """
    Callback function to handle alert messages.
    :param msg: The alert message containing the position.
    """
    rospy.loginfo(f"Alert received at position: x={msg.position.x}, y={msg.position.y}")
    global_vars.alert_pose = msg
    global_vars.current_state = RobotState.ALERT


def pose_callback(msg: PoseWithCovarianceStamped) -> None:
    """
    Callback function to update the robot's current position.
    :param msg: The message containing the robot's pose.
    """
    global_vars.current_position = msg.pose.pose
    return
