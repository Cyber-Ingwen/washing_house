# washing_house

# 0.Environment & Build

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev libpcap-dev libomp-dev can-utils  libasio-dev 
```

```sh
colcon build --symlink-install
source install/setup.bash
echo source install/setup.bash >> ~/.bashrc
```
* Install Ceres
    ```sh
    sudo apt-get update && \
    sudo apt-get install libgoogle-glog-dev libgflags-dev && \
    git clone https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    git checkout tags/2.1.0 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j8 && \
    sudo make install && \
    ```

* Install Open3D
    ```sh
    apt install pip
    pip install open3d
    ```
* Install boost
    ```sh
    wget https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz
    tar -xf boost_1_80_0.tar.gz
    cd boost_1_80_0
    bash ./bootstrap.sh
    bash ./b2 install
    cd ..
    rm -r boost_1_80_0 boost_1_80_0.tar.gz
    ```

* CAN-USB Setup
    ```sh
    bash src/hunter_ros2/ugv_sdk/scripts/setup_can2usb.bash
    ```

* Start Hunter Node 
    Control chassis by geometry_msgs/msg/Twist in topic /cmd_vel
    Hunter Node publish odom info by nav_msgs/msg/Odometry in topic odom
    ```sh
    bash src/hunter_ros2/ugv_sdk/scripts/bringup_can2usb_start_hunter_node.bash
    ```

* Fix PCL
    ```sh
    sudo apt update
    sudo apt-get install libpcl-dev ros-${ROS_DISTRO}-pcl-conversions 
    ```
# 1.Data from ros2 bag

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
ros2 bag play -l bag/rosbag2_2022_08_11-09_11_08_0.db3
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