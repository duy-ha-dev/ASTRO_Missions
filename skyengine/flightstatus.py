from aenum import Enum


class FlightStatus(Enum):
    """
    One of 5 basic flight statuses.
    """
    NOT_READY = 0   # Drone is not ready for flight commands.
    LANDED = 1      # Drone is sitting on the ground.
    TAKING_OFF = 2  # Drone is taking off.
    LANDING = 3     # Drone is landing.
    FLYING = 4      # Drone is flying.
    PANICKED = 5    # Drone is panicked!
