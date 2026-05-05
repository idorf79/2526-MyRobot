# Running Lidar node

Create a parameter file e.g. `/tmp/launch_params_i3aeyej4` with the following info:
`
/**:
  ros__parameters:
    angle_compensate: true
    channel_type: serial
    frame_id: laser
    inverted: false
    serial_baudrate: 115200
    serial_port: /dev/rplidar
`

```bash
ros2 run sllidar_ros2 sllidar_node --ros-args -r __node:=sllidar_node --params-file /tmp/launch_params_i3aeyej4
```

When adding `scan_mode: Simple` to the paramater file, the application will tell which options are possible:

```text
[ERROR] [1777484188.970864538] [sllidar_node]: scan mode `Simple' is not supported by lidar, supported modes:
[ERROR] [1777484188.971151943] [sllidar_node]:  Standard: max_distance: 12.0 m, Point number: 2.0K
[ERROR] [1777484188.971255868] [sllidar_node]:  Express: max_distance: 12.0 m, Point number: 3.9K
[ERROR] [1777484188.971334904] [sllidar_node]:  Boost: max_distance: 12.0 m, Point number: 7.9K
[ERROR] [1777484188.971411811] [sllidar_node]:  Sensitivity: max_distance: 12.0 m, Point number: 7.9K
[ERROR] [1777484188.971488496] [sllidar_node]: Can not start scan: 80008001!
```

This tells 'Standard' will have "2000 points" in 1 scan?
