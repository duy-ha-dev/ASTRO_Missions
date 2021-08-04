# Simple Mission

A simple test mission that makes the drone take off to a specified height and fly forward a small distance of 5m. The start location should be at the center of the field (in the middle of the 50 yard line on the giant Rice "R"). The drone should be set up to run the mission there, but if it the location is off, then ideally the drone should move toward the center, then it will move roughly 5 yards down the Rice statium. Remember to set the altitude to something very low, so we can land relatively safely.

## Usage

```bash
$ python hori_vert.py --fc-addr <flight controller MAV proxy address> --log-file <name of output log file>
```

## Endpoints

|Endpoint|Purpose|Required parameters|
|-|-|-|
|`/start-mission`|Instructs the drone to takeoff to the specified altitude, move forward a certain distance, and land.|`alt`|