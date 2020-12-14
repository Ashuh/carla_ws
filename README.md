# CARLA Workspace

## Mapping

```bash
./mapping.bash
```

To save the map generated:

```bash
rosrun map_server map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] map:=/your/costmap/topic
```
