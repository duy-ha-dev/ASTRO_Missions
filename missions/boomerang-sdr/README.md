# Boomerang Mission

In this mission, the drone has as inputs a vector of altitudes, hover_duration, air_speed, a vector of waypoints and number of loops.
It will pass, for each altitude in order, all the waypoints the number of loops choosed as input, and will land in the first waypoint if loops > 0.
If loops = 0, it will, for each altitude, pass all the waypoints one time and land in the last one.

Starting at some location, the drone will go to some specified GPS coordinates at different altitudes if requested, and land in the last waypoint.

The landing location can then be compared to the specified location to determine the accuracy of the system.

## Required dependencies

GPS sensor.

## Usage

```bash
$ python mission.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```
