import math
from math import asinh, atan2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np


class FormatNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("format_imu_pointcloud_node节点已创建")
        
        self.sub_imu = self.create_subscription(Imu, "/imu", self.callback_imu, 10)
        self.pub_imu = self.create_publisher(Imu, "/imu_2", 10) 
        
        self.time_sec = 0
        self.time_nsec = 0
        self.last_time_sec = 0
        self.last_time_nsec = 0
        self.change_flag = 0
        
        self.imu_count = 0
        self.pc_count = 0
        self.count = 0
        
        self.a = [0, 0, 0]
        self.omega = [0, 0, 0]
        self.theta = [0, 0, 0]
        
        self.list = []
        
    def callback_imu(self, data):
        new = data
        
        new.linear_acceleration.x += 0.46
        new.linear_acceleration.y += 0.26
        
        new.angular_velocity.x *= np.pi / 180
        new.angular_velocity.y *= np.pi / 180
        new.angular_velocity.z *= np.pi / 180
        
        self.omega = [new.angular_velocity.x, new.angular_velocity.y, new.angular_velocity.z]
        self.a = [new.linear_acceleration.x, new.linear_acceleration.y, new.linear_acceleration.z]
        Quat_list = self.cul_pose()
        [new.orientation.x, new.orientation.y, new.orientation.z, new.orientation.w] = Quat_list
        
        new.linear_acceleration.z = 0.0
        
        new.header.stamp = self.get_clock().now().to_msg()
        new.header.frame_id = "map"

        self.pub_imu.publish(new)    
        
        self.list.append((self.omega[1]))
        self.count += 1
        if (self.count % 1000 == 0):
            mu = 0.085
            sigma = 0.025
            bins = 300
            
            fig, ax = plt.subplots(1, 1)
            
            weights = np.ones_like(self.list) / float(len(self.list))
            _, bins, _ = ax.hist(self.list, bins, weights = weights, density=1)
            
            y = ((1/(np.power(2*np.pi, 0.5)*sigma))*np.exp(-0.5*np.power((bins-mu)/sigma, 2)))
            ax.plot(bins, y, color="#7744FF", ls="--", lw=3)
            
            plt.show()
        
        print("\r", self.count, "--", self.omega[1], end="", flush = True)
        
    def cul_pose(self):
        temp_theta = [0, 0, 0]
        temp_theta[0] = -atan2(self.a[1], self.a[2])
        temp_theta[1] = -asinh(self.a[0] / 9.815)
        
        delta_t = 0.01
        k = 10
        
        # self.theta[0] = (1 - k * delta_t) * self.theta[0] + k * delta_t * temp_theta[0] + self.omega[0] * delta_t
        # self.theta[1] = (1 - k * delta_t) * self.theta[1] + k * delta_t * temp_theta[1] + self.omega[1] * delta_t
        
        self.theta[0] = self.theta[0] + self.omega[0] * delta_t
        self.theta[1] = self.theta[1] + self.omega[1] * delta_t
        self.theta[2] = self.theta[2] + self.omega[2] * delta_t
        
        #print("\r", temp_theta[0], "--", self.theta[1], end="", flush = True)
        # print("\r", self.omega[1], end="", flush = True)
        
        Quat_list = self.Eular2Quat(self.theta)
        
        return Quat_list
        
    def Eular2Quat(self, EularAngle_list):
        """欧拉角转四元数"""
        [Roll, Pitch, Azimuth] = EularAngle_list

        cy = math.cos(Azimuth * 0.5)
        sy = math.sin(Azimuth * 0.5)
        cp = math.cos(Pitch * 0.5)
        sp = math.sin(Pitch * 0.5)
        cr = math.cos(Roll * 0.5)
        sr = math.sin(Roll * 0.5)
    
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        Quat_list = [x, y, z, w]
        
        return Quat_list
        
        
def main(args=None):
    rclpy.init(args=args) 
    node = FormatNode("format_imu_pointcloud_node")  
    rclpy.spin(node) 
    rclpy.shutdown()