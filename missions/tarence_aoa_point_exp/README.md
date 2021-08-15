# Hover Mission

A simple mission that makes the drone take off to a specified height and fly to 6 specified points along an arc

## Usage

```bash
$ python aoa.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to takeoff to the specified altitude, move to 6 specified points along an arc, and land.|`alt`, `hover_time`|