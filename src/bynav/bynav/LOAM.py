import math
import numpy as np


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
        self.lidar_odometry.process(self.feature_extraction.features)
        
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
        
        return 1


class LidarOdometry():
    """
    LOAM算法激光里程计
    """
    def __init__(self):
        self.last_features = []
        self.init_flag = 0
        self.T_list = []
        
    def process(self, features):
        """主程序"""
        if self.init_flag == 0:
            self.last_features = features
            self.init_flag = 1
            
        elif self.init_flag == 1:
            self.NewtonGussian(features)
            self.last_features = features

    def NewtonGussian(self, features):
        """牛顿高斯法优化"""
        x = np.zeros(6)
        j = self._get_jacobi()
        self.matching(features, x)
        f = np.array([self.D1, self.D2])
        x = x - np.matmut(np.matmut(np.linalg.inv(np.matmul(j.T, j)), j.T), f)
        self.T_list.append(x)
        
        return 1
    
    def matching(self, features, T):
        """特征点匹配"""
        [edge_points, plane_points] = features
        [last_edge_points, last_plane_points] = self.last_features
        edge_points = self.transform(edge_points, T)
        plane_points = self.transform(plane_points, T)
        
        print("___ep_____:", edge_points.shape)
        
        """边缘点匹配"""
        for i in range(edge_points.shape[0]):
            edge_point = np.array(edge_points[i][:3])
            last_points = np.array(last_edge_points[:,:3])
            distance = np.linalg.norm(last_points - edge_point, axis = 1)
            nearest_index = np.argmax(-distance)

            d = np.linalg.norm(last_points[nearest_index] - last_points[nearest_index - 1 if nearest_index-1 >= 0 else nearest_index + 1])
            s = np.linalg.norm(np.cross(last_points[nearest_index] - edge_point, last_points[nearest_index - 1 if nearest_index - 1 >= 0 else nearest_index + 1] - edge_point))
            h = (s / d) if d != 0 else -1

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

    def _get_jacobi(self):
        j_11 = 0
        j_12 = 0
        j_13 = 0
        j_14 = 0
        j_15 = 0
        j_16 = 0
        j_21 = 0
        j_22 = 0
        j_23 = 0
        j_24 = 0
        j_25 = 0
        j_26 = 0
        
    def _get_R(self, T):
        alpha = T[0]
        beta = T[1]
        gamma = T[2]
        R = np.array([[np.cos(beta)*np.cos(gamma), -np.sin(gamma)*np.cos(beta), np.sin(beta)], [np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha), -np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma), -np.sin(alpha)*np.cos(beta)], [np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha), np.cos(alpha)*np.cos(beta)]])
        
        return R

    def _get_t(self, T):
        x = T[3]
        y = T[4]
        z = T[5]
        t = np.array([x, y, z])
        
        return t
    
    def transform(self, x, T):
        R = self._get_R(T)
        t = self._get_t(T)
        x_vect, x_label = x[:,:3], x[:,3:]
        x_vect = np.einsum("ij,nj->ni", R, x_vect) + t
        x = np.concatenate((x_vect, x_label), axis=1)
        
        return x