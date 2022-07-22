# washing_house

# 0.Build (colcon)

$ colcon build --symlink-install 

# 1.Data from ros2 bag

```console
source install/setup.bash
ros2 bag play -l bag/bag.bd3
ros2 launch rslidar_sdk start.py
```

# 2.Data from Lidar

```console
nano src/rslidar_sdk/config/config.yaml
```

Set common.msg_source to 1 (MSG_FROM_LIDAR)

```console
ros2 launch rslidar_sdk start.py
```

# 3.Data Output

PointCloud2 data at topic /rslidar_points
