# Hover Mission

A simple mission that makes the drone take off to a specified height and fly forward and upward a certain distance and altitude, then going back down to first height; repeating once more

## Usage

```bash
$ python hori_vert.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to takeoff to the specified altitude, move forward a certain distance, and land.|`alt1`, `alt2`|