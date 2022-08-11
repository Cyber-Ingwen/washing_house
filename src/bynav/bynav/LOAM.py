import math
import numpy as np
from math import *
import time


class LOAM():
    """
    LOAM算法主程序
    """
    def __init__(self):
        self.feature_extraction = FeatureExtraction()
        self.lidar_odometry = LidarOdometry()

    def input(self, data):
        self.feature_extraction.process(data)
        self.lidar_odometry.process(self.feature_extraction.features)
    
    def output(self, pcn):
        pcn = pcn[:,:3]
        pcn = self.lidar_odometry.transform(pcn, self.lidar_odometry.T)
        #pcn = self.feature_extraction.plane_points
        
        return pcn


class FeatureExtraction():
    """
    LOAM算法提取特征
    """
    def __init__(self):
        self.edge_points = []
        self.plane_points = []
        self.features = []
        self.edge_points_index = []
        self.plane_points_index = []
        
        self.LEGO_cloudhandler = LEGO_cloudhandler()
    
    def process(self, pcn):
        self.processed_pcn = []
        self.edge_points = []
        self.plane_points = []
        self.edge_points_index = []
        self.plane_points_index = []

        """分割地面点"""
        pcn = self.LEGO_cloudhandler.pointcloudproject(pcn)
        pcn,self.ground_point_index = self.LEGO_cloudhandler.markground(pcn)
        self.ground_point = pcn[self.ground_point_index, :]

        """分割地面点"""
        for i in range(pcn.shape[0]):
            if i % 16 >= 4:
                self.processed_pcn.append(pcn[i])

        """提取竖线和平面"""
        for sector in range(6):
            curv_list = []
            for i in range(int(len(self.processed_pcn)/6)):
                i = i + int(len(self.processed_pcn) / 6 ) * sector
                
                x = self.processed_pcn[i][0]
                y = self.processed_pcn[i][1]
                z = self.processed_pcn[i][2]
                r0 = x**2 + y**2 + z**2
                
                curv = 0
                sum = [0, 0, 0]
                sum2 = 0
                
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
                        
                        sum2 += (x - self.processed_pcn[last_index][0])**2 + (y - self.processed_pcn[last_index][1])**2 + (z - self.processed_pcn[last_index][2])**2
                        sum2 += (x - self.processed_pcn[next_index][0])**2 + (y - self.processed_pcn[next_index][1])**2 + (z - self.processed_pcn[next_index][2])**2

                    curv = (sum[0] ** 2 + sum[1] ** 2 + sum[2] ** 2) / (sum2)
                    
                    next_index = i + 12 * 5
                    last_index = i - 12 * 5
                    rl = self.processed_pcn[last_index][0]**2 + self.processed_pcn[last_index][1]**2 + self.processed_pcn[last_index][2]**2
                    rn = self.processed_pcn[next_index][0]**2 + self.processed_pcn[next_index][1]**2 + self.processed_pcn[next_index][2]**2
                    if (abs(rl - r0) / r0 > 0.2) or (abs(rn - r0) / r0 > 0.2):
                        #self.edge_points.append(self.processed_pcn[i])
                        curv = nan
                        
                    curv_list.append(curv)
                    
            index = np.argsort(np.array(curv_list))
            edge_index = []
            plane_index = []

            for j in range(len(index)):
                if (len(plane_index) >= 20):
                    break
                if curv_list[index[j]] > 0:
                    flag = 1
                    for k in range(5):
                        if (index[j] + k < len(curv_list)):
                            if (curv_list[index[j]] > curv_list[index[j] + k]): flag = 0
                        if (index[j] - k > 0):
                            if (curv_list[index[j]] > curv_list[index[j] - k]): flag = 0
                    if (flag == 1):
                        plane_index.append(index[j])
                
            index = np.flipud(index)
            for j in range(len(index)):
                if (len(edge_index) >= 20):
                    break
                if ((not(math.isnan(curv_list[index[j]]))) & (curv_list[index[j]] < 100)):
                    flag = 1
                    for k in range(5):
                        if (index[j] + k < len(curv_list)):
                            if (curv_list[index[j]] < curv_list[index[j] + k]): flag = 0
                        if (index[j] - k > 0):
                            if (curv_list[index[j]] < curv_list[index[j] - k]): flag = 0
                    if (flag == 1):
                        edge_index.append(index[j])
            
            for ind in edge_index:
                ind += int(len(self.processed_pcn) / 6 ) * sector
                if sector == 0:
                    ind += 12 * 5
                self.edge_points.append(self.processed_pcn[ind])
                self.edge_points_index.append(ind)

            for ind in plane_index:
                ind += int(len(self.processed_pcn) / 6 ) * sector
                if sector == 0:
                    ind += 12 * 5
                self.plane_points.append(self.processed_pcn[ind])
                self.plane_points_index.append(ind)
         
        self.edge_points = np.array(self.edge_points)
        self.plane_points = np.array(self.plane_points)
        self.features = [self.edge_points, self.plane_points, self.edge_points_index, self.plane_points_index]
        
        return 1


class LidarOdometry():
    """
    LOAM算法激光里程计
    """
    def __init__(self):
        self.last_features = []
        self.init_flag = 0
        self.T_list = []
        self.T = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        
        self.flag = 0
        self.var = []
        
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
        x = np.array([0.1, 0.1, 0.1, 1, 1, 1])
        
        print("___________")
        for num in range(3):
            f, j = self.matching(features, x)
            x = (x.reshape(6,1) - 3 * np.matmul(np.matmul(np.array(np.linalg.inv(np.matmul(j.T, j) + 1e-2 * np.eye(6))), j.T), f)).reshape(6)
            self.T = x
            #print("\r x = %s ,f = %s" % (x.reshape(6), np.linalg.norm(f)), end = "")
            if np.linalg.norm(f)<10:
                break
            
        self.T_list.append(x)
        
        return 1
    
    def matching(self, features, T):
        """特征点匹配"""
        [edge_points, plane_points, self.edge_points_index, self.plane_points_index] = features
        [last_edge_points, last_plane_points, _, _] = self.last_features
        
        raw_edge_points = edge_points
        raw_plane_points = plane_points
        edge_points = self.transform(edge_points, T)
        plane_points = self.transform(plane_points, T)
        
        n = edge_points.shape[0]
        n = edge_points.shape[0] + plane_points.shape[0]
        F, J = np.zeros(n),  np.zeros((n, 6))
        
        """边缘点匹配"""
        for i in range(edge_points.shape[0]):
            edge_point = np.array(edge_points[i][:3])
            last_points = np.array(last_edge_points[:,:3])
            
            if self.flag == 0:
                distance = np.linalg.norm(last_points - edge_point, axis = 1)
                nearest_index = np.argmax(-distance)
                near_angle_index = (nearest_index - 1) if (nearest_index - 1) >= 0 else (nearest_index + 1)
                
                self.var = [nearest_index, near_angle_index]
                self.flag = 0
            
            [nearest_index, near_angle_index] = self.var
            d = np.linalg.norm(last_points[nearest_index] - last_points[near_angle_index])
            s = np.linalg.norm(np.cross(last_points[nearest_index] - edge_point, last_points[near_angle_index] - edge_point))
            h = (s / d)
            
            J[i] = self._get_jacobi_edge(raw_edge_points[i][:3], last_points[nearest_index], last_points[near_angle_index], T)
            F[i] = h
        
        """平面点匹配"""
        for i in range(plane_points.shape[0]):
            plane_point = np.array(plane_points[i][:3])
            last_points = np.array(last_plane_points[:,:3])
            distance = np.linalg.norm(last_points - plane_point, axis = 1)
            nearest_index = np.argmax(-distance)
            
            near_angle_index = (nearest_index - 1) if (nearest_index - 1) >= 0 else (nearest_index + 1)
        
            for delta in range(32):
                if nearest_index + delta + 1 < last_plane_points.shape[0]:
                    if last_plane_points[nearest_index + delta + 1][3] == last_plane_points[nearest_index][3] :
                        near_scan_index = nearest_index + delta + 1
                        break
                    else:
                        near_scan_index = nearest_index + delta + 1
                else: 
                    if last_plane_points[nearest_index - delta - 1][3] == last_plane_points[nearest_index][3] :
                        near_scan_index = nearest_index - delta - 1
                        break

            s = (np.cross(last_points[nearest_index] - last_points[near_angle_index], last_points[nearest_index] - last_points[near_scan_index]))
            h = np.dot(last_points[nearest_index] - plane_point, s / np.linalg.norm(s))
            
            if math.isnan(h): h = 0
            if not math.isnan(h):
                J[i + edge_points.shape[0]] = 1 * self._get_jacobi_plane(raw_plane_points[i][:3], last_points[nearest_index], last_points[near_angle_index], last_points[near_scan_index], T)
                F[i + edge_points.shape[0]] = 1 * h 
            else:
                J[i + edge_points.shape[0]] = self._get_jacobi_plane(raw_plane_points[i][:3], last_points[nearest_index], last_points[near_angle_index], last_points[near_scan_index], T)
                F[i + edge_points.shape[0]] = h
                print("h==Nan")
            
        return F.reshape((n, 1)), J.reshape((n, 6))
    
    def matching2(self, features, T):
        """特征点匹配"""
        last_edge_points = []
        last_edge_points_index = []
        raw_edge_points = edge_points
        raw_plane_points = plane_points
        edge_points = self.transform(edge_points, T)
        plane_points = self.transform(plane_points, T)
        
        n = edge_points.shape[0]
        n = edge_points.shape[0] + plane_points.shape[0]
        F, J = np.zeros(n),  np.zeros((n, 6))
        
        for i in range(edge_points.shape[0]):
            edge_point = edge_points[i][:3]
            last_points = last_edge_points[:,:3]
            
            distance = np.linalg.norm(last_points - edge_point, axis = 1)
            nearest_index = np.argmax(-distance)
            nearest_index = last_edge_points_index[nearest_index]
            near_angle_index = (nearest_index - 1) if (nearest_index - 1) >= 0 else (nearest_index + 1)
            
            
                
        
        
    def _get_jacobi_edge(self, p1, p2, p3, T):
        [x_1, y_1, z_1], [x_2, y_2, z_2], [x_3, y_3, z_3] = p1, p2, p3
        [alpha, beta, gamma, x, y, z] = T
        
        j11 = (2*(x_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(-np.sin(alpha)*np.cos(gamma) - np.sin(beta)*np.sin(gamma)*np.cos(alpha)) - z_1*np.cos(alpha)*np.cos(beta))*(-x - x_1*np.cos(beta)*np.cos(gamma) + x_2 + y_1*np.sin(gamma)*np.cos(beta) - z_1*np.sin(beta)) + 2*(x_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(-np.sin(alpha)*np.cos(gamma) - np.sin(beta)*np.sin(gamma)*np.cos(alpha)) - z_1*np.cos(alpha)*np.cos(beta))*(x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta)))*(-(x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) + (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta))) + (2*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - z_1*np.sin(alpha)*np.cos(beta))*(-x - x_1*np.cos(beta)*np.cos(gamma) + x_3 + y_1*np.sin(gamma)*np.cos(beta) - z_1*np.sin(beta)) + 2*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - z_1*np.sin(alpha)*np.cos(beta))*(x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta)))*((x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) - (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2)) + ((x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) - (x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)))*(2*(-x_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)) - y_1*(-np.sin(alpha)*np.cos(gamma) - np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z_1*np.cos(alpha)*np.cos(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) + 2*(x_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(-np.sin(alpha)*np.cos(gamma) - np.sin(beta)*np.sin(gamma)*np.cos(alpha)) - z_1*np.cos(alpha)*np.cos(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2) + 2*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - z_1*np.sin(alpha)*np.cos(beta))*(-x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) - y - y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) + y_2 + z_1*np.sin(alpha)*np.cos(beta)) + 2*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - z_1*np.sin(alpha)*np.cos(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)))
        j12 = ((x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) - (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2))*(2*(-x_1*np.sin(beta)*np.cos(gamma) + y_1*np.sin(beta)*np.sin(gamma) + z_1*np.cos(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) + 2*(x_1*np.sin(beta)*np.cos(gamma) - y_1*np.sin(beta)*np.sin(gamma) - z_1*np.cos(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2) + 2*(-x_1*np.cos(alpha)*np.cos(beta)*np.cos(gamma) + y_1*np.sin(gamma)*np.cos(alpha)*np.cos(beta) - z_1*np.sin(beta)*np.cos(alpha))*(-x - x_1*np.cos(beta)*np.cos(gamma) + x_3 + y_1*np.sin(gamma)*np.cos(beta) - z_1*np.sin(beta)) + 2*(-x_1*np.cos(alpha)*np.cos(beta)*np.cos(gamma) + y_1*np.sin(gamma)*np.cos(alpha)*np.cos(beta) - z_1*np.sin(beta)*np.cos(alpha))*(x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))) + (-(x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) + (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)))*(2*(-x_1*np.sin(beta)*np.cos(gamma) + y_1*np.sin(beta)*np.sin(gamma) + z_1*np.cos(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)) + 2*(x_1*np.sin(beta)*np.cos(gamma) - y_1*np.sin(beta)*np.sin(gamma) - z_1*np.cos(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) + 2*(x_1*np.sin(alpha)*np.cos(beta)*np.cos(gamma) - y_1*np.sin(alpha)*np.sin(gamma)*np.cos(beta) + z_1*np.sin(alpha)*np.sin(beta))*(-x - x_1*np.cos(beta)*np.cos(gamma) + x_2 + y_1*np.sin(gamma)*np.cos(beta) - z_1*np.sin(beta)) + 2*(x_1*np.sin(alpha)*np.cos(beta)*np.cos(gamma) - y_1*np.sin(alpha)*np.sin(gamma)*np.cos(beta) + z_1*np.sin(alpha)*np.sin(beta))*(x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))) + ((x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) - (x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)))*(2*(-x_1*np.sin(alpha)*np.cos(beta)*np.cos(gamma) + y_1*np.sin(alpha)*np.sin(gamma)*np.cos(beta) - z_1*np.sin(alpha)*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) + 2*(x_1*np.sin(alpha)*np.cos(beta)*np.cos(gamma) - y_1*np.sin(alpha)*np.sin(gamma)*np.cos(beta) + z_1*np.sin(alpha)*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2) + 2*(-x_1*np.cos(alpha)*np.cos(beta)*np.cos(gamma) + y_1*np.sin(gamma)*np.cos(alpha)*np.cos(beta) - z_1*np.sin(beta)*np.cos(alpha))*(-x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) - y - y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) + y_2 + z_1*np.sin(alpha)*np.cos(beta)) + 2*(-x_1*np.cos(alpha)*np.cos(beta)*np.cos(gamma) + y_1*np.sin(gamma)*np.cos(alpha)*np.cos(beta) - z_1*np.sin(beta)*np.cos(alpha))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)))
        j13 = ((x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) - (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2))*(2*(x_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)))*(-x - x_1*np.cos(beta)*np.cos(gamma) + x_3 + y_1*np.sin(gamma)*np.cos(beta) - z_1*np.sin(beta)) + 2*(x_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)))*(x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta)) + 2*(-x_1*np.sin(gamma)*np.cos(beta) - y_1*np.cos(beta)*np.cos(gamma))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) + 2*(x_1*np.sin(gamma)*np.cos(beta) + y_1*np.cos(beta)*np.cos(gamma))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2)) + (-(x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) + (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)))*(2*(x_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.sin(gamma)*np.cos(alpha)))*(-x - x_1*np.cos(beta)*np.cos(gamma) + x_2 + y_1*np.sin(gamma)*np.cos(beta) - z_1*np.sin(beta)) + 2*(x_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.sin(gamma)*np.cos(alpha)))*(x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta)) + 2*(-x_1*np.sin(gamma)*np.cos(beta) - y_1*np.cos(beta)*np.cos(gamma))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)) + 2*(x_1*np.sin(gamma)*np.cos(beta) + y_1*np.cos(beta)*np.cos(gamma))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta))) + ((x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) - (x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)))*(2*(x_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)))*(-x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) - y - y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) + y_2 + z_1*np.sin(alpha)*np.cos(beta)) + 2*(x_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) + 2*(-x_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_1*(-np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.sin(gamma)*np.cos(alpha)))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) + 2*(x_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.sin(gamma)*np.cos(alpha)))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2))
        j14 = (-2*y_2 + 2*y_3)*(-(x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) + (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta))) + (2*z_2 - 2*z_3)*((x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) - (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2))
        j15 = (2*x_2 - 2*x_3)*(-(x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) + (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta))) + (-2*z_2 + 2*z_3)*((x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) - (x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)))
        j16 = (-2*x_2 + 2*x_3)*((x + x_1*np.cos(beta)*np.cos(gamma) - x_2 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3) - (x + x_1*np.cos(beta)*np.cos(gamma) - x_3 - y_1*np.sin(gamma)*np.cos(beta) + z_1*np.sin(beta))*(x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2)) + (2*y_2 - 2*y_3)*((x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_2)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_3 - z_1*np.sin(alpha)*np.cos(beta)) - (x_1*(np.sin(alpha)*np.sin(gamma) - np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + z + z_1*np.cos(alpha)*np.cos(beta) - z_3)*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - y_2 - z_1*np.sin(alpha)*np.cos(beta)))
        
        j = np.array([j11, j12, j13, j14, j15, j16])
        j = j / ((-x_2 + x_3)**2 + (-y_2 + y_3)**2 + (-z_2 + z_3)**2)

        return j
    
    def _get_jacobi_plane(self, p1, p2, p3, p4, T):
        [x_1, y_1, z_1], [x_2, y_2, z_2], [x_3, y_3, z_3], [x_4, y_4, z_4] = p1, p2, p3, p4
        [alpha, beta, gamma, x, y, z] = T
        
        j21 = ((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3))*(x_1*(np.sin(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) - z_1*np.sin(alpha)*np.cos(beta)) + (-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3))*(x_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)) + y_1*(-np.sin(alpha)*np.cos(gamma) - np.sin(beta)*np.sin(gamma)*np.cos(alpha)) - z_1*np.cos(alpha)*np.cos(beta))
        j22 = ((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3))*(-x_1*np.cos(alpha)*np.cos(beta)*np.cos(gamma) + y_1*np.sin(gamma)*np.cos(alpha)*np.cos(beta) - z_1*np.sin(beta)*np.cos(alpha)) + (-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3))*(x_1*np.sin(alpha)*np.cos(beta)*np.cos(gamma) - y_1*np.sin(alpha)*np.sin(gamma)*np.cos(beta) + z_1*np.sin(alpha)*np.sin(beta)) + ((-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3))*(-x_1*np.sin(beta)*np.cos(gamma) + y_1*np.sin(beta)*np.sin(gamma) + z_1*np.cos(beta))
        j23 = (x_1*(np.sin(alpha)*np.cos(gamma) + np.sin(beta)*np.sin(gamma)*np.cos(alpha)) + y_1*(-np.sin(alpha)*np.sin(gamma) + np.sin(beta)*np.cos(alpha)*np.cos(gamma)))*((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3)) + (x_1*(-np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma)) + y_1*(-np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.sin(gamma)*np.cos(alpha)))*(-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3)) + ((-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3))*(-x_1*np.sin(gamma)*np.cos(beta) - y_1*np.cos(beta)*np.cos(gamma))
        j24 = (-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3)
        j25 = -(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3)
        j26 = (-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3)
        
        j = np.array([j21, j22, j23, j24, j25, j26])
        j = j / np.sqrt(((-x_2 + x_3)*(y_3 - y_4) - (x_3 - x_4)*(-y_2 + y_3))**2 + (-(-x_2 + x_3)*(z_3 - z_4) + (x_3 - x_4)*(-z_2 + z_3))**2 + ((-y_2 + y_3)*(z_3 - z_4) - (y_3 - y_4)*(-z_2 + z_3))**2)

        return j
        
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
    
    def pointcloudproject(self,pcd): #range image projection
        num,dem=pcd.shape
        for i in range(num):
            X=pcd[i][0]
            Y=pcd[i][1]
            Z=pcd[i][2]
            #add according to the new format
            rowID=pcd[i][3]
            colID=pcd[i][4]
            rowID=int(rowID) #divide vertical angle resolution
            colID=int(colID)
            #print(colID)
            #print(type(rowID))
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
                lowerID = i * 16 + j
                upperID = i * 16 + (j+1)
                #print(lowerID,upperID)
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
        return pcd, self.groundpoint
    
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
            findpointIDX=self.queueIndX[queuestartInd] #find a point 
            findpointIDY=self.queueIndY[queueendInd]
            findpointIDX=int(findpointIDX)
            findpointIDY=int(findpointIDY)
            queuesize=queuesize-1
            queuestartInd=queuestartInd+1
            self.lablematrix[findpointIDX][findpointIDY]=labelcount #mark the poind we are going to search
            if findpointIDX+1 <0 or findpointIDX-1<0 or findpointIDX+1>16 or findpointIDX-1>16: #index should be within the boundary
                continue
            if self.lablematrix[findpointIDX-1][findpointIDY]!=0:#check whether it has been coverd
                continue
            if self.lablematrix[findpointIDX+1][findpointIDY]!=0:#check whether it has been coverd
                continue
            if findpointIDY+1>=1800:
                findpointIDY=0
            if self.lablematrix[findpointIDX][findpointIDY+1]!=0:#check whether it has been coverd
                continue
            if findpointIDY-1<0:
                findpointIDY=1800
            if self.lablematrix[findpointIDX][findpointIDY-1]!=0:#check whether it has been coverd
                continue
            list_compare[0]=self.rangematrix[findpointIDX][findpointIDY] #this is the range between findpoint and lidar
            dis_left=self.rangematrix[findpointIDX-1][findpointIDY] #use col and row id to find the range from the left point to the lidar
            list_compare[1]=dis_left
            d1=max(list_compare)
            d2=min(list_compare)
            angel=atan2(d2*sin(angle_hor),(d1-d2*cos(angle_hor)))
            if angle>angle_bon:
                self.queueIndX[queueendInd]=findpointIDX-1
                self.queueIndX[queueendInd]=findpointIDY
                queuesize=queuesize+1
                queueendInd=queueendInd+1
                self.lablematrix[findpointIDX-1][findpointIDY]=labelcount
                self.allPushedIndX[allpushedindsize]=findpointIDX-1
                self.allPushedIndY[allpushedindsize]=findpointIDY
                allpushedindsize=allpushedindsize+1
            dis_right=self.rangematrix[findpointIDX+1][findpointIDY] #right point
            list_compare[1]=dis_right
            d1=max(list_compare)
            d2=min(list_compare)
            angle=atan2(d2*sin(angle_hor),(d1-d2*cos(angle_hor)))
            if angle>angle_bon:
                self.queueIndX[queueendInd]=findpointIDX+1
                self.queueIndX[queueendInd]=findpointIDY
                queuesize=queuesize+1
                queueendInd=queueendInd+1
                self.lablematrix[findpointIDX+1][findpointIDY]=labelcount
                self.allPushedIndX[allpushedindsize]=findpointIDX+1
                self.allPushedIndY[allpushedindsize]=findpointIDY
                allpushedindsize=allpushedindsize+1
            dis_up=self.rangematrix[findpointIDX][findpointIDY+1]#uppper point
            list_compare[1]=dis_up
            d1=max(list_compare)
            d2=min(list_compare)
            angel=atan2(d2*sin(angle_ver),(d1-d2*cos(angle_ver)))
            if angle>angle_bon:
                self.queueIndX[queueendInd]=findpointIDX
                self.queueIndX[queueendInd]=findpointIDY+1
                queuesize=queuesize+1
                queueendInd=queueendInd+1
                self.lablematrix[findpointIDX][findpointIDY+1]=labelcount
                self.allPushedIndX[allpushedindsize]=findpointIDX
                self.allPushedIndY[allpushedindsize]=findpointIDY+1
                allpushedindsize=allpushedindsize+1
            dis_low=self.rangematrix[findpointIDX][findpointIDY-1]#lower point
            list_compare[1]=dis_low
            d1=max(list_compare)
            d2=min(list_compare)
            angel=atan2(d2*sin(angle_ver),(d1-d2*cos(angle_ver)))
            if angle>angle_bon:
                self.queueIndX[queueendInd]=findpointIDX
                self.queueIndX[queueendInd]=findpointIDY-1
                queuesize=queuesize+1
                queueendInd=queueendInd+1
                self.lablematrix[findpointIDX][findpointIDY-1]=labelcount
                self.allPushedIndX[allpushedindsize]=findpointIDX
                self.allPushedIndY[allpushedindsize]=findpointIDY-1
                allpushedindsize=allpushedindsize+1
        
        check_seg=False
        labelCounts=0
        if allpushedindsize>=30: #check whether the cluster is valid
            check_seg=True
        if check_seg==True: #mark valid points
            labelCounts+=1
        else: #mark invalid points
            self.lablematrix[findpointIDX-1][findpointIDY]=9
            if findpointIDX+1<16:
                self.lablematrix[findpointIDX+1][findpointIDY]=9
            self.lablematrix[findpointIDX][findpointIDY-1]=9
            self.lablematrix[findpointIDX][findpointIDY+1]=9
        #print(self.lablematrix)
        return self.lablematrix


    def cloudsegmentation(self,pcd):#point cloud segmentation,to remove clusters with few points
        for i in range(16):
            for j in range(1800):
                #print(j)
                if self.lablematrix[i][j]==0: #to make labelmatrix
                    self.labelcomponents(i,j) 
        #print(self.lablematrix)
        return self.lablematrix
    