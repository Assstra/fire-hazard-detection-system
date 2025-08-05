from states import RobotState


def init_global_vars():
    global debug, current_position, current_state, alert_pose, current_goal, waypoints
    debug = False  #: bool If True, disables patrol mode for testing
    current_state = RobotState.PATROL  #: RobotState  Current robot state
    alert_pose = None  #: Optional[Pose] Pose to respond to in ALERT state
    current_goal = None  #: Optional[int] Current goal index or "ALERT"
    current_position = None  #: Optional[Pose] Latest known robot position
    waypoints = []  #: List[Pose] List of waypoints for patrolling
