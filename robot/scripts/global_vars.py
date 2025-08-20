from states import RobotState


def init_global_vars():
    """
    Initialize global variables for the robot state machine.
    """
    global \
        debug, \
        alert_mode, \
        current_position, \
        current_state, \
        alert_pose, \
        current_goal, \
        waypoints, \
        host, \
        port
    debug = False  #: bool If True, disables patrol mode for testing
    alert_mode = False  #: bool If True, disables patrol mode and only processes alerts and search
    current_state = RobotState.PATROL  #: RobotState  Current robot state
    alert_pose = None  #: Optional[Pose] Pose to respond to in ALERT state
    current_goal = None  #: Optional[int] Current goal index or "ALERT"
    current_position = None  #: Optional[Pose] Latest known robot position
    waypoints = []  #: List[Pose] List of waypoints for patrolling
    host = None  #: str Host for the fire detection server
    port = None  #: int Port for the fire detection server
