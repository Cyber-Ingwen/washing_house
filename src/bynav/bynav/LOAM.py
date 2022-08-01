import math
from operator import ne
import numpy as np
from yaml import scan


class LOAM():
    """
    LOAM算法主程序
    """
    def __init__(self):
        self.feature_extraction = FeatureExtraction()
        self.lidar_odometry = LidarOdometry()
        self.init_flag = 0

    def input(self, data):
        self.feature_extraction.process(data)
        
        if self.init_flag == 0:
            self.lidar_odometry.init(self.feature_extraction.features)
            self.init_flag = 1
        
        elif self.init_flag == 1:
            self.lidar_odometry.matching(self.feature_extraction.features)
            
        else:
            print("shabi")
        
    def output(self):
        return "shabi"


class FeatureExtraction():
    """
    LOAM算法提取特征
    """
    def __init__(self):
        self.edge_points = []
        self.plane_points = []
        self.features = []
    
    def process(self, pcn):
        print("__:", pcn.shape)
        self.processed_pcn = []
        self.edge_points = []
        self.plane_points = []

        """分割地面点"""
        #xdt 写在此处

        """分割地面点"""
        for i in range(pcn.shape[0]):
            if i % 16 >= 4:
                self.processed_pcn.append(pcn[i])

        """提取竖线和平面"""
        for i in range(len(self.processed_pcn)):
            x = self.processed_pcn[i][0]
            y = self.processed_pcn[i][1]
            z = self.processed_pcn[i][2]
            
            curv = 0
            sum = [0, 0, 0]
            
            if((i - 12 * 5 >= 0 ) & (i + 12 * 5 < len(self.processed_pcn))):
                for j in range(5):
                    next_index = i + 12 * j
                    last_index = i - 12 * j
                    sum[0] += (x - self.processed_pcn[last_index][0])
                    sum[0] += (x - self.processed_pcn[next_index][0])
                    sum[1] += (y - self.processed_pcn[last_index][1])
                    sum[1] += (y - self.processed_pcn[next_index][1])
                    sum[2] += (z - self.processed_pcn[last_index][2])
                    sum[2] += (z - self.processed_pcn[next_index][2])

                curv = sum[0] ** 2 + sum[1] ** 2 + sum[2] ** 2 
            
            if not math.isnan(curv): 
                if(curv < 100) & (curv > 0.2):
                    self.edge_points.append(self.processed_pcn[i])
                elif(curv < 0.1) & (curv > 0):
                    self.plane_points.append(self.processed_pcn[i])
        
        self.edge_points = np.array(self.edge_points)
        self.plane_points = np.array(self.plane_points)
        self.features = [self.edge_points, self.plane_points]
        
        print("__:", self.edge_points.shape)
        
        return 1


class LidarOdometry():
    """
    LOAM算法激光里程计
    """
    def __init__(self):
        self.last_features = []

    def init(self, features):
        self.last_features = features

    def matching(self, features):
        [edge_points, plane_points] = features
        [last_edge_points, last_plane_points] = self.last_features
        
        """边缘点匹配"""
        for i in range(edge_points.shape[0]):
            edge_point = np.array(edge_points[i][:3])
            last_points = np.array(last_edge_points[:,:3])
            distance = np.linalg.norm(last_points - edge_point, axis = 1)
            nearest_index = np.argmax(-distance)

            d = np.linalg.norm(last_points[nearest_index] - last_points[nearest_index - 1 if nearest_index-1 >= 0 else nearest_index + 1])
            s = np.linalg.norm(np.cross(last_points[nearest_index] - edge_point, last_points[nearest_index - 1 if nearest_index - 1 >= 0 else nearest_index + 1] - edge_point))
            h = (s / d) if d != 0 else -1

            #print("\r h = %s " % (h), end = "")

        """平面点匹配"""
        for i in range(plane_points.shape[0]):
            plane_point = np.array(plane_points[i][:3])
            last_points = np.array(last_plane_points[:,:3])
            distance = np.linalg.norm(last_points - plane_point, axis = 1)
            nearest_index = np.argmax(-distance)
            
            near_angle_index = (nearest_index - 1) if (nearest_index - 1) >= 0 else (nearest_index + 1)
            for delta in range(16):
                if nearest_index + delta + 1 < last_plane_points.shape[0]:
                    if last_plane_points[nearest_index + delta + 1][3] == last_plane_points[nearest_index][3] :
                        near_scan_index = nearest_index + delta + 1
                        break
                else: 
                    if last_plane_points[nearest_index - delta - 1][3] == last_plane_points[nearest_index][3] :
                        near_scan_index = nearest_index - delta - 1
                        break

            s = (np.cross(last_points[nearest_index] - last_points[near_angle_index], last_points[nearest_index] - last_points[near_scan_index]))
            h = np.linalg.norm(np.dot(last_points[nearest_index] - plane_point, s / np.linalg.norm(s)))
            
        self.last_features = features
