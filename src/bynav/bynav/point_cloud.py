import time
import math
import struct
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d


class Node_PC(Node):
    """
    创建point_cloud节点
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("point_cloud节点已创建")

        """创建并初始化接收"""
        self.sub_point_cloud = self.create_subscription(PointCloud2,"/rslidar_points",self.callback,10)

        """配置可视化"""
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd_curv = o3d.geometry.PointCloud()
        self.ctr = self.vis.get_view_control()

        """计算曲率"""
        self.Cul_Curv = Cul_Curvature()

    def callback(self, data):
        """读取解析数据"""
        assert isinstance(data, PointCloud2)
        pcd_as_numpy_array = np.array(list(self.read_points(data)))


        self.pcn = pcd_as_numpy_array
        self.curv_pcn = self.Cul_Curv.process(self.pcn)
        if(self.Cul_Curv.edge_points != []):
            self.Cul_Curv.process2(self.pcn)

        """可视化点云"""
        self.vis.remove_geometry(self.o3d_pcd_curv)
        self.vis.remove_geometry(self.o3d_pcd)
        self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.pcn[:,:3]))
        self.o3d_pcd_curv = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.curv_pcn[:,:3]))
        
        """颜色"""
        self.o3d_pcd.paint_uniform_color([60/255, 80/255, 120/255])
        self.o3d_pcd_curv.paint_uniform_color([255/255, 0/255, 0/255])
        self.vis.add_geometry(self.o3d_pcd)
        self.vis.add_geometry(self.o3d_pcd_curv)
        self.vis.run()
        self.vis.update_renderer()
        self.vis.poll_events()
        

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

            datatype_fmt = 'f'
            datatype_length = 4
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

        return fmt


class Cul_Curvature():
    def __init__(self):
        self.processed_pcn = []
        self.edge_points = []
        self.plane_points = []
    
    def process(self, pcn):
        self.processed_pcn = []
        list = []
        list2 = []

        a, b = pcn.shape
        for i in range(a):
            x = pcn[i][0]
            y = pcn[i][1]
            z = pcn[i][2]
            
            if i % 16 >= 4:
                self.processed_pcn.append([x, y, z])
                    
        a = len(self.processed_pcn)
        for i in range(a):
            x = self.processed_pcn[i][0]
            y = self.processed_pcn[i][1]
            z = self.processed_pcn[i][2]

            curv = 0
            sum = [0, 0, 0]
            if((i - 12 * 5 >= 0 ) & (i + 12 * 5 < a)):
                for j in range(5):
                    sum[0] += (x - self.processed_pcn[i - 12 * j][0])
                    sum[0] += (x - self.processed_pcn[i + 12 * j][0])
                    sum[1] += (y - self.processed_pcn[i - 12 * j][1])
                    sum[1] += (y - self.processed_pcn[i + 12 * j][1])
                    sum[2] += (z - self.processed_pcn[i - 12 * j][2])
                    sum[2] += (z - self.processed_pcn[i + 12 * j][2])

                curv = sum[0] ** 2 + sum[1] ** 2 + sum[2] ** 2 
            
            if not math.isnan(curv): 
                if(curv < 100) & (curv > 0.2):
                    list.append([x, y, z, curv])
                    #print("\r curv = %s " % (curv), end = "")
                elif(curv < 0.1) & (curv > 0):
                    list2.append([x, y, z, curv])
                    #print("\r curv = %s " % (curv), end = "")
            
        list = np.array(list)
        list2 = np.array(list2)

        self.edge_points = list[:,:3]
        self.plane_points = list2[:,:3]

        return list

    def process2(self, pcn):
        a, b = pcn.shape
        for i in range(a):
            x = pcn[i][0]
            y = pcn[i][1]
            z = pcn[i][2]

            vec0 = np.array([x, y, z])
            vec_list = np.array(self.edge_points)
            distance = np.linalg.norm(vec_list - vec0, axis = 1)
            j = np.argmax(-distance)

            d = np.linalg.norm(vec_list[j] - vec_list[j-1])
            s = np.linalg.norm(np.cross(vec_list[j] - vec0, vec_list[j-1] - vec0))
            h = s / d

            print("\r h = %s " % (h), end = "")

        return 0


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
