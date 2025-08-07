from enum import Enum


class RobotState(Enum):
    """
    Enumeration for the robot's states in the state machine.
    """
    PATROL = 1  # Normal patrol mode
    ALERT = 2  # Responding to alert
    SEARCH = 3  # Searching for hazard
