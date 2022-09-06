import time
import math
import struct
from turtle import clear, shape
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
#import open3d as o3d
import matplotlib.pyplot as plt

import sys,os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
#from kalman import KalmanFliter

from kalman_test import Kalman

class Node_kf(Node):
    def __init__(self):
        super().__init__('dataread')
        self.sub_point_cloud = self.create_subscription(Odometry, '/odom2', self.callback1, 100)
        self.imu_msg = self.create_subscription(Imu,'/imu_filter',self.callback2,10)
        self.pub_pose = self.create_publisher(Odometry,'/odom_kf',10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.callback3)
        #self.subscription
        #self.kalmanfliter = KalmanFliter()
        self.get_logger().info("节点已创建")

        self.v_x_0 = 0
        self.v_y_0 = 0
        self.v_z_0 = 0
        self.p_x_0 = 0
        self.p_y_0 = 0
        self.p_z_0 = 0
        self.last_vel_x = 0
        self.last_vel_y = 0
        self.last_vel_z = 0
        self.x_clo = 0
        self.y_clo = 0
        self.z_clo = 0
        self.flag = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.roll_g = 0
        self.pitch_g = 0
        self.yaw_g = 0
        self.list = []
        self.index = []
        self.diff = []
        self.count = 0
        self.p = 0
        self.M_rotate = np.zeros((3,3))
        #KalmanFliter(self.p_x_0, self.p_y_0, self.p_z_0, self.v_x_0, self.v_y_0, self.v_z_0,self.last_vel_x, self.last_vel_y, self.last_vel_z, self.x_clo, self.y_clo, self.z_clo)
        self.m = Kalman()
        #self.m.Kalman_Fliter(self.p_x_0, self.p_y_0, self.p_z_0, self.v_x_0, self.v_y_0, self.v_z_0,self.last_vel_x, self.last_vel_y, self.last_vel_z, self.x_clo, self.y_clo, self.z_clo)

    def callback1(self, data):
        """读取解析数据"""
        self.pc_msg = Odometry()
        self.x_clo = self.pc_msg.pose.pose.position.x
        self.y_clo = self.pc_msg.pose.pose.position.y
        self.z_clo = self.pc_msg.pose.pose.position.z
    
    def callback2(self, kk):
        data=kk
        x_linear = data.linear_acceleration.x + 0.35
        y_linear = data.linear_acceleration.y + 0.26
        z_linear = data.linear_acceleration.z
        x_angle = data.angular_velocity.x
        y_angle = data.angular_velocity.y
        z_angle = data.angular_velocity.z
        self.list.append(z_angle)
        self.diff.append(y_linear - self.p)
        self.p = y_linear
        self.count += 1
        self.index.append(self.count)
        # if self.count == 800:
        #     plt.hist(self.list,bins=80)
        #     #plt.plot(self.index,self.list)
        #     plt.show()
        #     self.count = 0
        #     self.list = []
        #     self.index = []
        #     self.diff = []
        # print("1")
        self.imu_position_volcity(x_linear,y_linear,z_linear,x_angle,y_angle,z_angle)
        print(self.p_x_0, self.p_y_0, self.p_z_0, self.v_x_0, self.v_y_0, self.v_z_0,self.last_vel_x, self.last_vel_y, self.last_vel_z, self.x_clo, self.y_clo, self.z_clo,self.roll,self.pitch,self.roll)
        self.m.Kalman_Fliter(self.p_x_0, self.p_y_0, self.p_z_0, self.v_x_0, self.v_y_0, self.v_z_0,self.last_vel_x, self.last_vel_y, self.last_vel_z, self.x_clo, self.y_clo, self.z_clo,self.roll,self.pitch,self.roll)


    def callback3(self):
        pub_p = Odometry()
        pub_p.header.frame_id = "map"
        tt = np.array(self.m.x)
        if tt.shape == (0,):
            pass
        else:
            print(tt)
            pub_p.pose.pose.position.x = tt[0][0]
            pub_p.pose.pose.position.y = tt[1][0]
            pub_p.pose.pose.position.z = tt[2][0]
        self.pub_pose.publish(pub_p)
        self.get_logger().info('odom send')

    def imu_position_volcity(self,x_linear,y_linear,z_linear,x_angle,y_angle,z_angle):
        self.last_vel_x = self.v_x_0
        self.last_vel_y = self.v_y_0
        self.last_vel_z = self.v_z_0
        #d = math.pi / 180
        M_gro_inv = np.zeros((3,3))
        M_gro_inv[0][0] = 1
        M_gro_inv[0][1] = math.sin(self.roll)*math.tan(self.pitch)
        M_gro_inv[0][2] = math.cos(self.roll)*math.tan(self.pitch)
        M_gro_inv[1][1] = math.cos(self.roll)
        M_gro_inv[1][2] = -math.sin(self.roll)
        M_gro_inv[2][1] = math.sin(self.roll)/math.cos(self.pitch)
        M_gro_inv[2][2] = math.cos(self.roll)/math.cos(self.pitch)
        M_rpy_g = np.zeros((3,1))
        M_g = np.zeros((3,1))
        M_g[0][0] = x_angle
        M_g[1][0] = y_angle
        M_g[2][0] = z_angle
        #print("-------------------------",x_angle,y_angle,z_angle)
        M_rpy_g = np.dot(M_gro_inv,M_g)
        #print(M_rpy_g)
        d_roll_g = M_rpy_g[0][0] 
        d_pitch_g = M_rpy_g[1][0] 
        d_yaw_g = M_rpy_g[2][0]
        self.roll_g += d_roll_g * 0.01 
        self.pitch_g += d_pitch_g * 0.01        
        self.yaw_g += d_yaw_g * 0.01

        # M_acc = np.zeros((3,1))
        # M_acc[0][0] = -z_angle * math.sin(self.yaw)
        # M_acc[1][0] = z_angle * math.sin(self.roll) * math.cos(self.yaw)
        # M_acc[2][0] = z_angle * math.cos(self.roll) * math.cos(self.yaw)
        self.roll = math.atan(y_linear/x_linear)
        self.pitch = -math.atan(x_linear/math.sqrt(y_linear * y_linear + z_linear * z_linear))
        self.roll = self.roll_g + (self.roll - self.roll_g) * 0.4
        self.pitch = self.pitch_g + (self.pitch - self.pitch_g) * 0.4
        self.yaw = self.yaw_g
        print(self.roll,self.pitch,self.yaw)

        # self.M_rotate[0][0] = math.cos(self.pitch) * math.cos(self.yaw)
        # self.M_rotate[0][1] = math.sin(self.yaw) * math.cos(self.pitch)
        # self.M_rotate[0][2] = - math.sin(self.pitch)
        # self.M_rotate[1][0] = math.sin(self.pitch) * math.sin(self.roll) * math.cos(self.yaw) - math.sin(self.yaw) * math.cos(self.roll)
        # self.M_rotate[1][1] = math.sin(self.pitch) * math.sin(self.roll) * math.sin(self.yaw) + math.cos(self.roll) * math.cos(self.yaw)
        # self.M_rotate[1][2] = math.sin(self.roll) * math.cos(self.pitch)
        # self.M_rotate[2][0] = math.sin(self.pitch) * math.cos(self.roll) * math.cos(self.yaw) - math.sin(self.yaw) * math.sin(self.roll)
        # self.M_rotate[2][1] = math.sin(self.pitch) * math.cos(self.roll) * math.cos(self.yaw) - math.cos(self.yaw) * math.sin(self.roll)
        # self.M_rotate[2][2] = math.cos(self.pitch) * math.cos(self.roll)

        return 1



def main(args = None):
    rclpy.init(args = args)
    node = Node_kf()
    #KalmanFliter(node.p_x_0, node.p_y_0, node.p_z_0, node.v_x_0, node.v_y_0, node.v_z_0,node.last_vel_x, node.last_vel_y, node.last_vel_z, node.x_clo, node.y_clo, node.z_clo)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()