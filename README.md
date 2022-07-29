# washing_house

# 0.Environment & Build

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev libpcap-dev
```

```sh
colcon build --symlink-install 
source install/setup.bash
```

```sh
apt install pip
pip install open3d
pip install open3d
```

# 1.Data from ros2 bagSS

```sh
ros2 launch rslidar_sdk start_offline.py
```

# 2.Data from Lidar

```sh
nano src/rslidar_sdk/config/config.yaml
```

Set common.msg_source to 1 (MSG_FROM_LIDAR)

```sh
ros2 launch rslidar_sdk start.py
```

# 3.Data Output

PointCloud2 data at topic /rslidar_points

Rawdata from LIDAR can be published by:

```sh
ros2 bag play -l bag/bag.db3
ros2 run bynav point_cloud_node
```

# 4.Display On Windows

Download MobaXterm on windows devices on:

```sh
https://mobaxterm.mobatek.net/
```

export display by:

```sh
export DISPLAY=172.16.196.215:0.0
```