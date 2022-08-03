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
        self.pcn = self.label(pcd_as_numpy_array)
        #self.pcn = self.Cul_Curv.process(self.pcn)
        self.pcn=self.LEGO_cloudhandler.pointcloudproject(self.pcn)
        self.pcn,self.ground=self.LEGO_cloudhandler.markground(self.pcn)
        self.pcn_groundlabel=self.LEGO_cloudhandler.cloudsegmentation(self.pcn)

        """可视化点云"""
        self.vis.remove_geometry(self.o3d_pcd)
        self.o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_as_numpy_array[:,:3]))
        self.vis.add_geometry(self.o3d_pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    def label(self, pcn):
        """给点云标注角度和线"""
        scan_mat = np.zeros(pcn.shape[0])
        degree_mat = np.zeros(pcn.shape[0])
        
        if pcn.shape[0] == 28800:
            for i in range(pcn.shape[0]):
                scan_mat[i] = (i % 16) if (i % 16) < 9 else 24 - (i % 16)
                #print(scan_mat[i])
                degree_mat[i] = i % 1800
                #print(degree_mat[i])
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
        self.lablematrix=np.zeros((16,1800))
        self.groundmetrix=np.zeros(28800)
        self.groundpoint=[]
        self.startindex=np.zeros(16)
        pass

    '''def startendangle(self,pcd): #to find start and end angle of the clooud
        a, b = pcd.shape
        startOrientation=math.atan2(pcd[0][1],pcd[0][0])
        endOrientation=math.atan2(pcd[a-1][1],pcd[a-1][0])+2*math.pi
        if endOrientation-startOrientation>3*math.pi:
            startOrientation+=2*math.pi
        elif endOrientation-startOrientation<math.pi:
            orientationDiff=endOrientation-startOrientation
        return startOrientation,endOrientation,orientationDiff'''
    
    def pointcloudproject(self,pcd): #range image projection
        num,dem=pcd.shape
        for i in range(num):
            X=pcd[i][0]
            Y=pcd[i][1]
            Z=pcd[i][2]
            #add according to the new format
            rowID=pcd[i][4]
            colID=pcd[i][5]
            rowID=int(rowID) #divide vertical angle resolution
            colID=int(colID)
            #print(colID)
            #print(type(rowID))
            '''#give row and column index for the point
            verticalAngle=atan2(Z,math.sqrt(X*X+Y*Y))*180/math.pi#find the angle btween p-o and plane x-y
            rowID=(verticalAngle-(-15))/2 #15 refers to bottom angle and 2 refers to vertical angle resolution
            if rowID<0 or rowID>16: # finish all the labels
                continue
            horizonAngle =atan2(X,Y)*180/math.pi#find the angle on the plane x-y
            colID=-round((horizonAngle-90.0)/0.2)+1800/2#0.2 refers to horizontal angle resolution,1800 refers to horizon scan
            if colID>=1800:
                colID=-1800
            if colID<0 or colID>=1800:
                continue'''
            distance=math.sqrt(X*X+Y*Y+Z*Z)
            if distance<1.0: #filter out point winthin 1 meter around the lidar
                continue
            self.rangematrix[rowID][colID]=distance #put all the range in this array
            #pcd[i][3]=distance
            index=colID+rowID*1800
            #print(i)
            if i >=28800:
                continue
            self.index[i]=index
            #print(self.index)
        #pcd=np.insert(pcd,4,values=index,axis=1)
        return pcd
    
    def markground(self,pcd): #mark ground points
        #marker:0 no valid info, -1 initial value,after validation,not ground, 1 ground
        for i in range(1800):
            for j in range(4):#why 4: here we have 4 scans that are supposed to scan to the ground
                lowerID=i+j*1800
                upperID=i+(j+1)*1800
                #print(lowerID,upperID)
                '''if self.rangematrix[j][i]<=0 or self.rangematrix[j+1][i]<=0:
                    self.groundmetrix[lowerID]=-1
                    continue
                else:
                    print(lowerID)'''
                x1=pcd[lowerID][0]
                y1=pcd[lowerID][1]
                z1=pcd[lowerID][2]
                x2=pcd[upperID][0]
                y2=pcd[upperID][1]
                z2=pcd[upperID][2]                
                disx=x1-x2               
                disy=y1-y2                
                disz=z1-z2     
                angle=math.atan2(disz,math.sqrt(disx*disx+disy*disy))*180/math.pi               
                if abs(angle)<=10:
                    self.groundmetrix[upperID]=1
                    self.groundmetrix[lowerID]=1
                self.groundpoint.append(lowerID)
                #print(self.groundmetrix[upperID],self.groundmetrix[lowerID],"------------")
        #print(self.groundmetrix)
        return pcd,self.groundpoint
    
    def labelcomponents(self,row,col):
        labelcount=1
        self.queueIndX[0]=row
        self.queueIndY[0]=col
        self.allPushedIndX[0]=row
        self.allPushedIndY[0]=col
        list_compare=[0,0]
        angle_hor=0.2/180*math.pi
        angle_ver=2/180*math.pi
        angle_bon=60/180*math.pi
        queuesize=1
        queuestartInd=0
        queueendInd=1
        allpushedindsize=1
        while(queuesize>0):#use BFS to find the neighbor point
            findpointIDX=queueIndX[queuestartInd] #find a point 
            findpointIDY=queueIndY[queueendInd]
            queuesize=queuesize-1
            queuestartInd=queuestartInd+1
            self.lablematrix[findpointIDX][findpointIDY]=labelcount #mark the poind we are going to search
            if findpointIDX+1 <0 or findpointIDX-1<0 or findpointIDX+1>16 or findpointIDX-1>16: #index should be within the boundary
                continue
            if findpointIDY+1>=1800:
                findpointIDY=0
            if findpointIDY-1<0:
                findpointIDY=1800
            if self.lablematrix[findpointIDX-1][findpointIDY]!=0:#check whether it has been coverd
                continue
            list_compare[0]=self.rangematrix[findpointIDX][findpointIDY] #this is the range between findpoint and lidar
            dis_left=self.rangematrix[findpointIDX-1][findpointIDY] #use col and row id to find the range from the left point to the lidar
            list_compare[1]=dis_left
            d1=max(list_compare)
            d2=min(list_compare)
            angel=atan2(d2*sin(angle_hor),(d1-d2*cos(angle_hor)))
            if angle>angle_bon:
                queueIndX[queueendInd]=findpointIDX-1
                queueIndX[queueendInd]=findpointIDY
                queuesize=queuesize+1
                queueendInd=queueendInd+1
                self.lablematrix[findpointIDX-1][findpointIDY]=labelcount
                allPushedIndX[allpushedindsize]=findpointIDX-1
                allPushedIndY[allpushedindsize]=findpointIDY
                allpushedindsize=allpushedindsize+1
            dis_right=self.rangematrix[findpointIDX+1][findpointIDY] #right point
            list_compare[1]=dis_right
            d1=max(list_compare)
            d2=min(list_compare)
            angel=atan2(d2*sin(angle_hor),(d1-d2*cos(angle_hor)))
            if angle>angle_bon:
                queueIndX[queueendInd]=findpointIDX+1
                queueIndX[queueendInd]=findpointIDY
                queuesize=queuesize+1
                queueendInd=queueendInd+1
                self.lablematrix[findpointIDX+1][findpointIDY]=labelcount
                allPushedIndX[allpushedindsize]=findpointIDX+1
                allPushedIndY[allpushedindsize]=findpointIDY
                allpushedindsize=allpushedindsize+1
            dis_up=self.rangematrix[findpointIDX][findpointIDY+1]#uppper point
            list_compare[1]=dis_up
            d1=max(list_compare)
            d2=min(list_compare)
            angel=atan2(d2*sin(angle_ver),(d1-d2*cos(angle_ver)))
            if angle>angle_bon:
                queueIndX[queueendInd]=findpointIDX
                queueIndX[queueendInd]=findpointIDY+1
                queuesize=queuesize+1
                queueendInd=queueendInd+1
                self.lablematrix[findpointIDX][findpointIDY+1]=labelcount
                allPushedIndX[allpushedindsize]=findpointIDX
                allPushedIndY[allpushedindsize]=findpointIDY+1
                allpushedindsize=allpushedindsize+1
            dis_low=self.rangematrix[findpointIDX][findpointIDY-1]#lower point
            list_compare[1]=dis_low
            d1=max(list_compare)
            d2=min(list_compare)
            angel=atan2(d2*sin(angle_ver),(d1-d2*cos(angle_ver)))
            if angle>angle_bon:
                queueIndX[queueendInd]=findpointIDX
                queueIndX[queueendInd]=findpointIDY-1
                queuesize=queuesize+1
                queueendInd=queueendInd+1
                self.lablematrix[findpointIDX][findpointIDY-1]=labelcount
                allPushedIndX[allpushedindsize]=findpointIDX
                allPushedIndY[allpushedindsize]=findpointIDY-1
                allpushedindsize=allpushedindsize+1
        
        check_seg=False
        labelCounts=0
        if allpushedindsize>=30: #check whether the cluster is valid
            check_seg=True
        if check_seg==True: #mark valid points
            labelCounts+=1
        else: #mark invalid points
            self.lablematrix[findpointIDX-1][findpointIDY]=9
            self.lablematrix[findpointIDX+1][findpointIDY]=9
            self.lablematrix[findpointIDX][findpointIDY-1]=9
            self.lablematrix[findpointIDX][findpointIDY+1]=9
        return self.lablematrix


    def cloudsegmentation(self,pcd):#point cloud segmentation,to remove clusters with few points
        for i in range(16):
            for j in range(1800):
                if self.lablematrix[i][j]==0: #to make labelmatrix
                    self.labelcomponents(i,j) 
        return self.lablematrix
        
        '''MMMM=[]
        sizeofsegment=0 #to deal with noise, ground point and interval point
        for m in range(16):
            self.startindex[i]=sizeofsegment-1+5 #to record each scan's valid start point
            for n in range(1800):
                number=n+m*1800
                if self.lablematrix[m][n]>0 or self.groundmetrix[number]==1:# deal with valid clusters or ground point
                    if self.lablematrix[m][n]==9:#deal with invalid one
                        if m>4 and n%5==0:
                            a=pcd[number]
                            MMMM.append(a)
                            continue
                        else:
                            continue
                    if self.groundmetrix[number]==1:
                        if n%5!=0 and n>5 and n<1795:
                            continue'''
                        


                    






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
