import time
import math
import struct
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
#import open3d as o3d
#import matplotlib.pyplot as plt

import sys,os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
#from kalman import KalmanFliter

from kalman_test import Kalman
from LOAM import LOAM
from Cul_Curvature import Cul_Curvature

class Node_kf(Node):
    def __init__(self):
        super().__init__('dataread')
        self.sub_point_cloud = self.create_subscription(PointCloud2, '/rslidar_points', self.callback1, 10)
        self.imu_msg = self.create_subscription(Imu,'/imu',self.callback2,10)
        #self.subscription
        #self.kalmanfliter = KalmanFliter()
        self.get_logger().info("节点已创建")
        

        # """配置可视化"""
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window()
        # self.o3d_pcd = o3d.geometry.PointCloud()
        # self.o3d_pcd_curv = o3d.geometry.PointCloud()
        # self.ctr = self.vis.get_view_control()

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
        #KalmanFliter(self.p_x_0, self.p_y_0, self.p_z_0, self.v_x_0, self.v_y_0, self.v_z_0,self.last_vel_x, self.last_vel_y, self.last_vel_z, self.x_clo, self.y_clo, self.z_clo)
        self.m = Kalman()
        #self.m.Kalman_Fliter(self.p_x_0, self.p_y_0, self.p_z_0, self.v_x_0, self.v_y_0, self.v_z_0,self.last_vel_x, self.last_vel_y, self.last_vel_z, self.x_clo, self.y_clo, self.z_clo)
        self.Cul_Curv = ()
        self.loam = LOAM()

    def callback1(self, data):
        """读取解析数据"""
        assert isinstance(data, PointCloud2)
        self.pc_msg = PointCloud2()
        pcd_as_numpy_array = np.array(list(self.read_points(data)))
        self.pcn = self.label(pcd_as_numpy_array)
        # temp_1 = self.x_clo
        # temp_2 = self.y_clo
        # temp_3 = self.z_clo
        self.loam.input(self.pcn)
        self.curv_pcn = self.loam.output(self.pcn)
        print(self.curv_pcn)
        '''for i in range(self.pcn.shape[0]):
            self.x_clo = self.pcn[i][0]
            self.y_clo = self.pcn[i][1]
            self.z_clo = self.pcn[i][2]
            print(self.pc_msg.header)'''
        #print(self.pc_msg.header)

        # """可视化点云"""
        # self.vis.remove_geometry(self.o3d_pcd_curv)
        # self.vis.remove_geometry(self.o3d_pcd)
        # self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.pcn[:,:3]))
        # self.o3d_pcd_curv = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.curv_pcn[:,:3]))
        # self.o3d_pcd.paint_uniform_color([60/255, 80/255, 120/255])
        # self.o3d_pcd_curv.paint_uniform_color([255/255, 0/255, 0/255])
        # self.vis.add_geometry(self.o3d_pcd)
        # self.vis.add_geometry(self.o3d_pcd_curv)
        # #self.vis.run()
        # self.vis.update_renderer()
        # self.vis.poll_events()
    
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
    
    def imu_position_volcity(self,x_linear,y_linear,z_linear,x_angle,y_angle,z_angle):
        self.last_vel_x = self.v_x_0
        self.last_vel_y = self.v_y_0
        self.last_vel_z = self.v_z_0
        self.v_x_0 = self.v_x_0 + 0.01 * x_linear
        self.v_y_0 = self.v_y_0 + 0.01 * y_linear
        self.v_z_0 = self.v_z_0 + 0.01 * z_linear
        m = mahony()
        m.mahony_imu(x_linear,y_linear,z_linear,x_angle,y_angle,z_angle)
        x_1 = mahony.R[0][0] 
        x_2 = mahony.R[1][0] 
        x_3 = mahony.R[2][0] 
        y_1 = mahony.R[0][1] 
        y_2 = mahony.R[1][1] 
        y_3 = mahony.R[2][1] 
        z_1 = mahony.R[0][2] 
        z_2 = mahony.R[1][2] 
        z_3 = mahony.R[2][2] 
        self.p_x_0 = self.p_x_0 * x_1 + self.p_x_0 * x_2 + self.p_x_0 * x_3 + 0.015 * x_linear - 0.005 * self.last_vel_x
        self.p_y_0 = self.p_y_0 * y_1 + self.p_y_0 * y_2 + self.p_y_0 * y_3 + 0.015 * y_linear - 0.005 * self.last_vel_y
        self.p_z_0 = self.p_z_0 * z_1 + self.p_z_0 * z_2 + self.p_z_0 * z_3 + 0.015 * z_linear - 0.005 * self.last_vel_z
        return 1

    def label(self, pcn):
        """给点云标注角度和线"""
        scan_mat = np.zeros(pcn.shape[0])
        degree_mat = np.zeros(pcn.shape[0])
        
        if pcn.shape[0] == 28800:
            for i in range(pcn.shape[0]):
                scan_mat[i] = (i % 16) if (i % 16) < 9 else 24 - (i % 16)
                degree_mat[i] = i % 1800
        else:
            pass # 可改为计算
            
        scan_mat = np.resize(scan_mat, (pcn.shape[0], 1))
        degree_mat = np.resize(degree_mat, (pcn.shape[0], 1))
        pcn = np.concatenate((pcn, scan_mat, degree_mat), axis = 1)
        
        return pcn

    def read_points(self, cloud):
        """读取点云数据"""
        assert isinstance(cloud, PointCloud2)
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        unpack_from = struct.Struct(fmt).unpack_from

        for v in range(height):
            offset = row_step * v
            for u in range(width):
                yield unpack_from(data, offset)
                offset += point_step

    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        """获取数据格式"""
        fmt = '>' if is_bigendian else '<'

        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset)):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            else:
                datatype_fmt = 'f'
                datatype_length = 4
                fmt += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt

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
        # t = num
        # t = 0x5f3759df - (t/2.0)
        # while not ((t * t <= num) and ((t+1) * (t+1) > num)):
        #     t = ((num / t) + t) / 2.0
        # return t
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