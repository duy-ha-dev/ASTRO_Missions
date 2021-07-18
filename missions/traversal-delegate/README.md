# Delegate Traversal Mission

This mission is similar to the Traversal mission, but works for more than one drone. This autonomous mission will be given the coordinates of 4 corners of a region and will split the region along the longest side by the number of drones. Each drone will then be given the new 4 coordinates of it's region to perform the Traversal mission on.

Since this mission is used only on regions specified at Rice, it is assumed the regions are all rectangular. The regions also may not be split optimally if there are more than 2 drones.

This mission will also keep a log identical to that of the mission Non-Autonomous for each drone.


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
$ curl 172.27.0.XX:4003/start-mission -d '{"corners":"stadium", "alt": [<altitude values>]}'
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Client-invoked endpoint to have first drone split main region and delegate the partitions to other drones.|`corners`, `alt`|
|`/delegation-region`|Internally invoked endpoint that instructs the drone to begin the traversal mission and log.|`corner-coords`, `alt`|
