# Hover Mission

A simple mission that makes the drone take off to a specified height and fly upward to another certain height. Must instruct first altitude and second altitude; alt1 is starting altitude and alt2 is the end altitude. Must also give position 1, 2, or 3: these positions are spaced 10 yards apart and are predetermined according to the Vertical Experiment Flight Plan slide. 

## Usage

```bash
$ python vertical.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to takeoff to the specified altitude1 and then to the specified altitude2 at position 1, 2, or 3.|`alt1`, `alt2`, `position`|