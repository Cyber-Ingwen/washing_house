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
        self.ctr = self.vis.get_view_control()

        """计算曲率"""
        self.Cul_Curv = Cul_Curvature()

        '''handle pointcloud'''
        self.LEGO_cloudhandler=LEGO_cloudhandler()

    def callback(self, data):
        """读取解析数据"""
        assert isinstance(data, PointCloud2)
        pcd_as_numpy_array = np.array(list(self.read_points(data)))
        self.pcn = pcd_as_numpy_array
        self.pcn = self.Cul_Curv.process(self.pcn)

        """可视化点云"""
        self.vis.remove_geometry(self.o3d_pcd)
        self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_as_numpy_array[:,:3]))
        self.vis.add_geometry(self.o3d_pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

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
        pass
    
    def process(self, pcd):
        m=pcd
        print(m.shape)
        delete_index=[]
        #delete_angle=[]
        a, b = m.shape
        for i in range(a):
            x = m[i][0]
            y = m[i][1]
            z = m[i][2]
            
            phi = math.atan2(z, math.sqrt(x**2 + y**2))
            phi = phi * 180 / math.pi

            theta =  math.atan2(y, x)
            theta = theta * 180 / math.pi

            if not math.isnan(phi):
                print("\r phi = %s " % (phi), end = "")
            #time.sleep(0.4)
                '''if phi<-8:
                    delete_index.append(i)
                    #delete_angle.append(phi)
        #print(delete_index,delete_angle)
        array_select = np.delete(m,delete_index)
        print(array_select.shape)
        return array_select'''
    
    #def dataselect(self,pcd):

         
class LEGO_cloudhandler():
    def __init__(self):
        pass
    
    def startendangle(self,pcd): #to find start and end angle of the clooud
        a, b = pcb.shape
        startOrientation=math.atan2(pcb[0][1],pcb[0][0])
        endOrientation=math.atan2(pcb[a-1][1],pcb[a-1][0])+2*math.pi
        if endOrientation-startOrientation>3*math.pi:
            startOrientation+=2*math.pi
        elif endOrientation-startOrientation<math.pi:
            orientationDiff=endOrientation-startOrientation
        return startOrientation,endOrientation,orientationDiff
    
    def pointcloudproject(slef,pcd): #range image projection
        num,dem=pcd.shape
        for i in range(num):
            X=pcb[i][0]
            Y=pcb[i][1]
            Z=pcb[i][2]
            #give row and column index for the point
            verticalAngle=atan2(Z,math.sqrt(X*X+Y*Y))*180/math.pi#find the angle btween p-o and plane x-y
            rowID=(verticalAngle+15)/2 #15 refers to bottom angle and 2 refers to vertical angle resolution





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
