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
    
    def output(self, pcn):
        pcn = pcn[:,:3]
        pcn = self.lidar_odometry.transform(pcn, self.lidar_odometry.T)
        
        return pcn


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
                elif(curv < 2e-5) & (curv > 0):
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
        for num in range(10):
            f, j = self.matching(features, x)
            x = (x.reshape(6,1) - np.matmul(np.matmul(np.array(np.linalg.inv(np.matmul(j.T, j) + 1e-2 * np.eye(6))), j.T), f)).reshape(6)
            self.T = x
            print("\r x = %s ,f = %s" % (x.reshape(6), np.linalg.norm(f)), end = "")
            if np.linalg.norm(f)<10:
                break
            
        self.T_list.append(x)
        
        return 1
    
    def matching(self, features, T):
        """特征点匹配"""
        [edge_points, plane_points] = features
        [last_edge_points, last_plane_points] = self.last_features
        
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

