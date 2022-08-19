# washing_house

# 0.Environment & Build

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev libpcap-dev libomp-dev can-utils  libasio-dev
```

```sh
colcon build --symlink-install --parallel-workers 8
source install/setup.bash
echo source install/setup.bash >> ~/.bashrc
```
* Install Ceres
    ```sh
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
    sudo ./bootstrap.sh
    sudo ./b2 install
    cd ..
    rm -r boost_1_80_0 boost_1_80_0.tar.gz
    ```

* CAN-USB First Setup
    ```sh
    bash src/chassis_node_cpp/scripts/setup_can2usb.bash
    ```
* CAN-USB Everytime Setup
    ```sh
    bash src/chassis_node_cpp/scripts/setup_can2usb.bash
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
<<<<<<< HEAD
export DISPLAY=172.16.196.107:0.0  
=======
export DISPLAY=172.16.196.215:0.0     
>>>>>>> 46abb26b70ef79b7b03e7065f1dc95574203b9d4
```