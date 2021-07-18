# Wireless Target - Swarm and Track mission

This mission is the only "Swarm and Track" phase of the Wireless Target


## Usage
Run Skyserve (run from home)
```bash
$ ./missions/bin/skyserve.sh
```

Run The Mission (start in this dir)
```bash
$ ../bin/run-mission.sh
```

Send Curl (start in this dir on GCS)
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
$ curl 172.27.0.XX:4000/start-mission -d '{"region_id":"stadium", "alt": [<altitude values>], "airspeed": 2, "type":"hover", "hover_time":30 }'
```

### Simulation
(Run skysim and skyserve)

Run The Mission (start in this dir)
```bash
$ ../bin/simulation/run-mission-simulation.sh
```

Send Curl (start in this dir on GCS)
```bash
$ ./send-curl.sh docker
```

### Landing
```bash
$ curl 172.27.0.XX:4000/land
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Client-invoked endpoint to have first drone split main region and delegate the partitions to other drones.|`region_id`, `alt`, `airspeed`|
|`/neighbors`|Query the neighbor drones detected by the drone during initialization.| None |
|`/wireless`|Query the number of wireless samples collected by the sensor.| None |
|`/land`|Client-invoked to land the drone.| None |
