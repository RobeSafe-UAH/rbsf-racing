# rbsf-racing
RobeSafe Racing Stack for RoboRacer Autonomous Racing

## Tracks

`src/base_system/rbsf_worlds` holds the tracks every simulator (and later the car) uses: a ROS map
(yaml + image) plus the reference lines to drive on. Both simulators take `track:=<name>`.

| Track | Length | Lines |
|---|---|---|
| `levine` | 61.8 m | centerline |
| `spielberg` | 338 m | centerline, raceline (with speed profile) |

See `src/base_system/rbsf_worlds/tracks/README.md` for the file formats and sources.
