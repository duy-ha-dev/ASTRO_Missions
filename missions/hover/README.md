# Hover Mission

A simple mission that makes the drone take off to a specified height, maintain the altitude for a specified amount of time, and land.

## Usage

```bash
$ python hover.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to takeoff to the specified altitude, hover for the specified time, and land.|`alt`, `hover_time`|