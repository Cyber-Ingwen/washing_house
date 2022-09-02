import time
import math
import struct
from turtle import shape
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
#import open3d as o3d
#import matplotlib.pyplot as plt

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
        x_linear = data.linear_acceleration.x
        y_linear = data.linear_acceleration.y
        z_linear = data.linear_acceleration.z
        x_angle = data.angular_velocity.x
        y_angle = data.angular_velocity.y
        z_angle = data.angular_velocity.z
        #print("1")
        self.imu_position_volcity(x_linear,y_linear,z_linear,x_angle,y_angle,z_angle)
        self.m.Kalman_Fliter(self.p_x_0, self.p_y_0, self.p_z_0, self.v_x_0, self.v_y_0, self.v_z_0,self.last_vel_x, self.last_vel_y, self.last_vel_z, self.x_clo, self.y_clo, self.z_clo)
    
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
        d = math.pi / 180
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
        M_rpy_g = np.dot(M_gro_inv,M_g)
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
        return 1

class mahony():
    q0 = float(1.0)
    q1 = float(0.0)
    q2 = float(0.0)
    q3 = float(0.0)
    ki = 2.0*0.0 #2 * integral gain (Ki)
    kp = 2.0*5.0 #2 * proportional gain (Kp)
    integeral_x = 0.0 #integral error terms scaled by Ki
    integeral_y = 0.0
    integeral_z = 0.0
    R = np.zeros((3,3))
    
    def mahony_imu(self,x_g,y_g,z_g,x_a,y_a,z_a):
        #print("2")
        if not((x_a == 0.0) and (y_a == 0.0) and (y_a == 0.0)):
            #print("3")
            Normcenter = Solution.invSqrt(x_a * x_a + y_a * y_a + z_a * z_a) #Normalise
            #print(Normcenter)
            x_a *= Normcenter
            y_a *= Normcenter
            z_a *= Normcenter
            #重力分量
            v_x = self.q1 * self.q3 - self.q0 * self.q2
            v_y = self.q0 * self.q1 + self.q2 * self.q3
            v_z = self.q0 * self.q0 - 0.5 + self.q3 * self.q3
            #error
            e_x = (y_a * v_z - z_a * v_y)
            e_y = (z_a * v_x - x_a * v_z)
            e_z = (x_a * v_y - y_a * v_x)
            if(self.ki > 0): #对误差进行积分运算
                self.integeral_x += self.ki * e_x * (1.0 / 125)
                self.integeral_y += self.ki * e_y * (1.0 / 125)
                self.integeral_z += self.ki * e_z * (1.0 / 125)
                x_g += self.integeral_x
                y_g += self.integeral_y
                z_g += self.integeral_z
            else:
                self.integeral_x = 0.0
                self.integeral_y = 0.0
                self.integeral_z = 0.0
            x_g += self.kp * e_x #对误差进行比例运算
            y_g += self.kp * e_y
            z_g += self.kp * e_z

        #integrate rate of change of quaternion
        x_g *= 0.5 * (1.0 / 100)
        y_g *= 0.5 * (1.0 / 100)
        z_g *= 0.5 * (1.0 / 100)
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += (-qb * x_g - qc * y_g - self.q3 * z_g)
        self.q1 += (qa * x_g + qc * z_g - self.q3 * y_g)
        self.q2 += (qa * y_g - qb * z_g + self.q3 * x_g)
        self.q3 += (qa * z_g + qb * y_g - qc * x_g)
        #normalise quaternion
        Norm1 = Solution.invSqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= Norm1
        self.q1 *= Norm1
        self.q2 *= Norm1
        self.q3 *= Norm1
        #print(self.q0,"---",self.q1,"---",self.q2,"---",self.q3,"---")
        #print("\r%s,%s,%s",(x_g,y_g,z_g), end = "", flush=True)
        value = [[1 - 2 * self.q2 * self.q2 - 2 * self.q3 * self.q3, 2 * self.q1 * self.q2 - 2 * self.q3 * self.q0,2 * self.q1 * self.q3 + 2 * self.q2 * self.q0], 
        [2 * self.q1 * self.q2 + 2 * self.q3 * self.q0, 1 - 2 * self.q1 * self.q1 - 2 * self.q3 * self.q3, 2 * self.q2 * self.q3 - 2 * self.q1 * self.q0],
        [2 * self.q1 * self.q3 - 2 * self.q2 * self.q0,2 * self.q2 * self.q3 + 2 * self.q1 * self.q0,1-2 * self.q1 * self.q1 - 2 * self.q2 * self.q2]]
        self.R = np.array(value)
        #print(self.R)
        return 1

class Solution():
    def invSqrt(num):
        t = num
        t = math.sqrt(t)
        t = 1 / t
        return t  


def main(args = None):
    rclpy.init(args = args)
    node = Node_kf()
    #KalmanFliter(node.p_x_0, node.p_y_0, node.p_z_0, node.v_x_0, node.v_y_0, node.v_z_0,node.last_vel_x, node.last_vel_y, node.last_vel_z, node.x_clo, node.y_clo, node.z_clo)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()