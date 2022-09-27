# Static Centerline Optimizer

## Purpose

This package statically calcualtes the centerline with ego's footprints inside the drivable area.

On narrow-road driving, the default centerline calculated by the middle line between lanelet's right and left bounds often makes the ego's footprints outside the drivable area.
With the static centerline generation, we have the following advantages.

- We can know the centerline shape beforehand.
- We do not have to calculate a heavy path optimization since the footprints is already inside the drivable area.

## Usecases

There are two interfaces to communicate with the centerline optimization server.

### Vector Map Builder Interface

### Command Line Interface

The optimized centerline is generated from the command line interfac.

```sh
ros2 run static_path_smoother optimize_path.sh <map-path> <start-lanelet-id> <end-lanelet-id>
```

The output map with the optimized centerline locates `/tmp/lanelet2_map.osm`
If you want to change the output map path, you can remap the path

```sh
ros2 run static_path_smoother optimize_path.sh <map-path> <start-lanelet-id> <end-lanelet-id> --ros-args --remap output:
```
