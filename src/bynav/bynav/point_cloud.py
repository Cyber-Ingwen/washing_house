import time
import math
import struct
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d


import sys,os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

from Cul_Curvature import Cul_Curvature
from LOAM import LOAM


class Node_PC(Node):
    """
    创建point_cloud节点
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("point_cloud节点已创建")

        """创建并初始化接收"""
        self.sub_point_cloud = self.create_subscription(PointCloud2, "/rslidar_points", self.callback, 10)

        """配置可视化"""
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd_curv = o3d.geometry.PointCloud()
        self.ctr = self.vis.get_view_control()

        """LOAM算法"""
        self.Cul_Curv = Cul_Curvature()
        self.loam = LOAM()

    def callback(self, data):
        """读取解析数据"""
        assert isinstance(data, PointCloud2)
        pcd_as_numpy_array = np.array(list(self.read_points(data)))
        self.pcn = self.label(pcd_as_numpy_array)
        
        self.loam.input(self.pcn)
        
        self.curv_pcn = self.Cul_Curv.process(self.pcn)

        """可视化点云"""
        self.vis.remove_geometry(self.o3d_pcd_curv)
        self.vis.remove_geometry(self.o3d_pcd)
        self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.pcn[:,:3]))
        self.o3d_pcd_curv = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.curv_pcn[:,:3]))
        self.o3d_pcd.paint_uniform_color([60/255, 80/255, 120/255])
        self.o3d_pcd_curv.paint_uniform_color([255/255, 0/255, 0/255])
        self.vis.add_geometry(self.o3d_pcd)
        self.vis.add_geometry(self.o3d_pcd_curv)
        #self.vis.run()
        self.vis.update_renderer()
        self.vis.poll_events()
    
    def label(self, pcn):
        """给点云标注角度和线"""
        scan_mat = np.zeros(pcn.shape[0])
        degree_mat = np.zeros(pcn.shape[0])
        
        if pcn.shape[0] == 28800:
            for i in range(pcn.shape[0]):
                scan_mat[i] = i % 16
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


def main(args = None):
    """
    point_cloud节点主程序
    """

    """配置节点"""
    rclpy.init(args = args)
    node = Node_PC("point_cloud")

    """保持节点"""
    rclpy.spin(node)
    rclpy.shutdown()
