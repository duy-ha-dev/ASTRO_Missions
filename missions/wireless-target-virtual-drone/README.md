# Virtual Drone mission

This mission is the Virtual Drone mission, where one drone sense the signal of a transmitter and calculates its position 


## Usage
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
$ curl 172.27.0.XX:4000/start-mission -d '{"region_id":"stadium", "alt": 10, "airspeed": 2 }'
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

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Client-invoked endpoint to have first drone split main region and delegate the partitions to other drones.|`region_id`, `alt`, `airspeed`|
