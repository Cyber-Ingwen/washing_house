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
        self.rangematrix=np.zeros((16,1800))
        self.index=np.zeros(28800)
        self.queueIndX=np.zeros(28800)
        self.queueIndY=np.zeros(28800)
        self.allPushedIndX=np.zeros(28800)#used to track points of a segmented object
        self.allPushedIndY=np.zeros(28800)
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
            rowID=(verticalAngle-(-15))/2 #15 refers to bottom angle and 2 refers to vertical angle resolution
            if rowID<0 or rowID>16: # finish all the labels
                continue
            horizonAngle =atan2(X,Y)*180/math.pi#find the angle on the plane x-y
            colID=-round((horizonAngle-90.0)/0.2)+1800/2#0.2 refers to horizontal angle resolution,1800 refers to horizon scan
            if colID>=1800:
                colID=-1800
            if colID<0 or colID>=1800:
                continue
            distance=math.sqrt(X*X+Y*Y+Z*Z)
            if distance<1.0: #filter out point winthin 1 meter around the lidar
                continue
            self.rangematrix[rowID][colID]=distance #put all the range in this array
            pcd[i][3]=distance
            index=colID+rowID*1800
            self.index[i]=index
        pcd=np.insert(pcd,4,values=index,axis=1)
        return pcd
    
    def markground(self,pcd): #mark ground points
        #marker:0 no valid info, -1 initial value,after validation,not ground, 1 ground
        groundmetrix=np.zeros(28800)
        for i in range(1799,-1,-1):
            for j in range(5,-1,-1):#why 5: here we have 4 scans that are supposed to scan to the ground
                lowerID=i+j*1800
                upperID=i+(j+1)*1800
                for m in range(len(self.index)):
                    if self.index[m]=lowerID:
                        low=m
                        for n in range(m,len(self.index)):
                            if self.index[n]=upperID:
                                up=n
                if groundmetrix[m]==-1 or groundmetrix[n]==-1:
                    continue
                A=self.index[low]
                x1=A[0]
                y1=A[1]
                z1=A[2]
                B=self.index[up]
                x2=B[0]
                y2=B[1]
                z2=B[2]
                disx=x1-x2
                disy=y1-y2
                disz=z1-z2
                angle=atan2(disz,math.sqrt(disx*disx+disy*disy))*/math.pi
                if abs(angle)<=10:
                    groundmetrix[up]=1
                    groundmetrix[low]=1
        return groundmetrix
    
    def labelcomponents(self,row,col):
        self.queueIndX[0]=row
        self.queueIndY[0]=col
        self.allPushedIndX[0]=row
        self.allPushedIndY[0]=col
        queuesize=1
        queuestartInd=0
        queueendInd=1
        allpushedindsize=1
        while(queuesize>0):#


        

    def cloudsegmentation(self,pcd,groundmetrix):#point cloud segmentation,to remove clusters with few points


                    






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
