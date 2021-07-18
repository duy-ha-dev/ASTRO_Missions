# Boomerang for Channel Sounding

This mission takes as inputs a vector of altitudes, hover_duration, air_speed, a vector of waypoints and number of loops.

From the taking off point, the drone will fly to specified GPS coordinates at different altitudes if requested, and land in the last waypoint.

At the same time, the drone will take IRIS measurments.

## Required dependencies

* GPS sensor
* IRIS board (Rev.C)

## Usage
To download SDR dependencies use the following commands:

```
git submodule init
git submodule update
```

To run the mission:

```bash
$ python mission.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```
