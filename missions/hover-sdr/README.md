# Hover-SDR Mission

When activated, this mission will simply log the drone's location, speed, heading, and reading from SDR while the drone is hovering.

## Usage
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
$ curl 172.27.0.XX:4002/start-mission -d '{"alt": [<list of integer values>], "hover_time":<>}'
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to begin log.|`alt`, `hover_time`|
