# Traversal Mission

This mission is a simple autonomous mission, where one drone will be given the coordinates of 4 corners of a region and will need to calculate waypoints in order to sweep the region at intervals of about 15m while minimizing the number of turns it needs to make. The drone will then repeat the path of traversal at each inputted altitude, reversing the direction of travel every other altitude in order to maximize efficiency.

This mission will also keep a log identical to that of the mission Non-Autonomous.

## Usage

```bash
$ python mission.py --fc-addr <flight controller MAV proxy address>
```

```bash
$ curl 172.27.0.XX:4004/start-mission -d '{"corners":"stadium", "alt": [<altitude values>]}'
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to begin the traversal mission and log.|`corners`, `alt`|
