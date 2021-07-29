# Hover Mission

A simple mission that makes the drone take off to a specified height and fly upward to another certain height

## Usage

```bash
$ python vertical.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to takeoff to the specified altitude1 and then to the specified altitude2 at position 1, 2, or 3.|`alt1`, `alt2`, `position`|