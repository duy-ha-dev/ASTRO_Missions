"""
Place the most useful classes from skyengine at top-level module scope, permitting convenient
imports like `from skyengine import DroneController`.
"""
# HACK:
# Remove any references to the ROS-provided, globally-installed Python packages,
# since they might be loaded instead of the versions we specify in the virtualenv.
# (The version of pymavlink we specify is not affected by the mode switching bug,
# but the ROS one is.)
import sys
sys.path = filter(lambda p: not p.startswith("/opt/ros"), sys.path)
# 'NOQA' silences Flake8 warnings about unused imports.
from skyengine.drone import DroneController, FlightController  # NOQA
from skyengine.flightstatus import FlightStatus                # NOQA
