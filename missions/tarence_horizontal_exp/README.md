# Hover Mission

A simple mission that makes the drone take off to a specified height and fly forward a certain distance

## Usage

```bash
$ python horizontal.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to takeoff to the specified altitude, move forward a certain distance, and land.|`alt`, `distance`|