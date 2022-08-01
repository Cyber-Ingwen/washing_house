import math
from operator import ne
import numpy as np


class LOAM():
    """
    LOAM算法主程序
    """
    def __init__(self):
        self.feature_extraction = FeatureExtraction()
        self.lidar_odometry = LidarOdometry()

    def input(self, data):
        self.feature_extraction.process(data)


class FeatureExtraction():
    """
    LOAM算法提取特征
    """
    def __init__(self):
        self.edge_points = []
        self.plane_points = []
        self.features = []
    
    def process(self, pcn):
        self.processed_pcn = []
        self.edge_points = []
        self.plane_points = []

        """分割地面点"""
        #xdt 写在此处

        """分割地面点"""
        for i in range(pcn.shape[0]):
            x = pcn[i][0]
            y = pcn[i][1]
            z = pcn[i][2]
            
            if i % 16 >= 4:
                self.processed_pcn.append([x, y, z])

        """提取竖线和平面"""
        for i in range(len(self.processed_pcn)):
            x = self.processed_pcn[i][0]
            y = self.processed_pcn[i][1]
            z = self.processed_pcn[i][2]

            curv = -1
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
                    self.edge_points.append([x, y, z, curv])
                elif(curv < 0.1) & (curv > 0):
                    self.plane_points.append([x, y, z, curv])
                elif curv == -1:
                    return 0
            
        self.edge_points = (np.array(self.edge_points))[:,:3]
        self.plane_points = (np.array(self.plane_points))[:,:3]
        self.features = [self.edge_points, self.plane_points]
        
        return 1


class LidarOdometry():
    """
    LOAM算法激光里程计
    """
    def __init__(self):
        self.features = []

    def init(self, features):
        self.last_features = features

    def matching(self, features):
        [edge_points, plane_points] = self.features
        
        """边缘点匹配"""
        for i in range(edge_points.shape[0]):
            edge_point = np.array(edge_points[i][:2])
            last_points = np.array(edge_points)
            distance = np.linalg.norm(last_points - edge_point, axis = 1)
            nearest_index = np.argmax(-distance)

            d = np.linalg.norm(last_points[nearest_index] - last_points[nearest_index - 1 if nearest_index-1 >= 0 else nearest_index + 1])
            s = np.linalg.norm(np.cross(last_points[nearest_index] - edge_point, last_points[nearest_index - 1 if nearest_index - 1 >= 0 else nearest_index + 1] - edge_point))
            h = (s / d) if d != 0 else -1

            print("\r h = %s " % (h), end = "")

        """平面点匹配"""
        for i in range(plane_points.shape[0]):
            plane_point = np.array(plane_points[i][:2])
            last_points = np.array(plane_points)
            distance = np.linalg.norm(last_points - plane_point, axis = 1)
            nearest_index = np.argmax(-distance)

            d = np.linalg.norm(last_points[nearest_index] - last_points[nearest_index - 1 if nearest_index-1 >= 0 else nearest_index + 1])
            s = np.linalg.norm(np.cross(last_points[nearest_index] - plane_point, last_points[nearest_index - 1 if nearest_index - 1 >= 0 else nearest_index + 1] - plane_point))
            h = (s / d) if d != 0 else -1

            print("\r h = %s " % (h), end = "")
        
        self.last_features = features
        