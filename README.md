# washing_house

# 0.Environment & Build

```console
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev libpcap-dev
```

```console
source install/setup.bash
colcon build --symlink-install 
```

# 1.Data from ros2 bag

```console
ros2 launch rslidar_sdk start_offline.py
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

Rawdata from LIDAR can be published by:

```console
ros2 bag play -l bag/bag.db3
ros2 run bynav point_cloud_node
```

# 4.Open VPN

```console
expect ~/Downloads/vpn/vpn.exp
```