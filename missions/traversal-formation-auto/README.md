# Formation Traversal Auto Mission

In this missions, the drones visit a series of waypoints while maintaining a fixed formation.
The formation of the drones is determined by a chosen preset formation, which they will automatically take on in the beginning of the mission.
The basic process of the mission is as follows:

1. While on the ground the master drone collects location of all other drones and finds the center coordinates
2. Master assigns each drone a position in the chosen formation that they will maintain throughout the mission
3. Master signals each drone to launch to a different altitude, after which they will assume formation with the center at the original cluster center
4. Each drone visits each waypoint, offset by the drone's original offset from the center
5. At each waypoint, the each drone will wait for all drones to report that they have arrived before moving on
6. After all waypoints have been visited, the drones land


## Required dependencies

GPS sensor.

## Usage
Run Skyserve (script can be run from any dir)
```bash
$ ../bin/skyserve.sh
```

Run The Mission (start in this dir)
```bash
$ ../bin/run-mission.sh
```

Send Curl (start in this dir)
```bash
$ ./send-curl.sh <drone-name>
```
(Edit send-curl.sh to change the parameters)

### Alternatively (without scripts)
Run The Mission
```bash
$ python mission.py --fc-addr <flight controller MAV proxy address>
```

Send Curl
```bash
$ curl 172.27.0.XX:4002/start-mission -d '{"region_name": "wiess-2", "min_alt": 8, "hover_duration": 5, "speed": 2}'
```

# Simulation
## Running SkySim with Nearby Starting Positions
Wiess-2 Drone 1
```bash
$ skysim --port 6002 --port 6003 --port 6004 --home-lat 29.715002 --home-lon -95.401979
```
Wiess-2 Drone 2
```bash
$ skysim --port 6002 --port 6003 --port 6004 --home-lat 29.715043 --home-lon -95.401875
```

Wiess-2 Drone 3
```bash
$ skysim --port 6002 --port 6003 --port 6004 --home-lat 29.714984 --home-lon -95.401906
```

# Run SkyServe
```bash
$ ../bin/simulation/skyserve-simulation.sh
```

# Run Mission (Same as realtime)
```bash
$ ../bin/run-mission.sh
```