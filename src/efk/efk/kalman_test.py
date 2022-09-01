import numpy as np
import pandas as pd
 
class Kalman:
    B = 0
    u = 0  
    K = float('nan')  # 卡尔曼增益无需初始化
    z = float('nan')  # 这里无需初始化，每次使用kf_update之前需要输入观察值z
    P = np.diag(np.ones(6))  # 初始P设为0,查到的资料p值越小，估计值越可靠，但是也不知道初始值的设定有什么影响
 
    # 初始状态
    x = []
    G = [] #更新
 
    # 状态转移矩阵A
    # 和线性系统的预测机制有关，这里的线性系统是上一刻的位置加上速度等于当前时刻的位置，而速度本身保持不变
    A = np.eye(6) + np.diag(np.ones((1, 3))[0, :], 3) * 0.015
    A[3][3] = 2
    A[4][4] = 2
    A[5][5] = 2
 
    # 预测噪声协方差矩阵Q：假设预测过程上叠加一个高斯噪声，协方差矩阵为Q
    # 大小取决于对预测过程的信任程度。如果运动目标在y轴上的速度可能不匀速，那么可以把这个对角矩阵的最后一个值调大，Q越大，估计值也不可靠
    Q = np.diag(np.ones(6)) * 0.1
 
    # 观测矩阵H：z = H * x
    # 这里的状态是（坐标x， 坐标y，坐标z, 速度x， 速度y, 速度z），观察值是（坐标x， 坐标y, 坐标z），所以H = eye(3, 6)
    H = np.eye(3, 6)
 
    # 观测噪声协方差矩阵R：假设观测过程上存在一个高斯噪声，协方差矩阵为R
    # 大小取决于对观察过程的信任程度。如果观测结果中的坐标x值常常很准确，那么矩阵R的第一个值应该比较小
    R = np.diag(np.ones(3)) * 0.1
 
    def kf_init(self,px, py, pz, vx, vy, vz, v_form_x, v_form_y, v_form_z):
    # 状态x为（坐标x， 坐标y，坐标z, 速度x， 速度y, 速度z），观测值z为（坐标x， 坐标y, 坐标z）
        self.B = np.zeros((6,3))
        self.B[0][0] = - 0.00005
        self.B[0][1] = - 0.00005
        self.B[0][2] = - 0.00005
        self.B[1][0] = - 1
        self.B[1][1] = - 1
        self.B[1][2] = - 1  
        self.u = np.zeros((3,1))
        self.u[0] = v_form_x
        self.u[0] = v_form_y
        self.u[0] = v_form_z
        self.K = float('nan')
        self.z = float('nan')
        self.P = np.diag(np.ones(6))
        self.x = np.zeros((6,1))
        self.x[0] = px
        self.x[1] = py
        self.x[2] = pz
        self.x[3] = vx
        self.x[4] = vy
        self.x[5] = vz
        self.G = np.zeros((6,1))
        self.A = np.eye(6) + np.diag(np.ones((1, 3))[0, :], 3) * 0.01
        self.Q = np.diag(np.ones(6)) * 0.1
        self.H = np.eye(3, 6)
        self.R = np.diag(np.ones(3)) * 0.1
        return self
 
 
    def kf_update(self):
        #计算不考虑误差的预测值
        AA = np.dot(self.A, self.x) #6x1
        BB = np.dot(self.B,self.u) #6x1
        x_ = np.array(AA) + BB
    
        #计算先验的协方差
        b1 = np.dot(self.A, self.P) #6x6
        b2 = np.dot(b1, np.transpose(self.A)) #6x6
        p_ = np.array(b2) + self.Q 
    
        #计算卡尔曼增益
        z1 = np.dot(p_, np.transpose(self.H)) #6x3
        z2 = np.dot(self.H, p_) #3x6
        z3 = np.dot(z2, np.transpose(self.H)) #3x3
        z4 = np.array(z3) + self.R 
        z5 = np.linalg.matrix_power(z4, -1) #取逆 3x3
        self.K = np.dot(z1, z5) #6x3
 
        #data fusion
        d1 = np.dot(self.H, x_) #3x1
        d2 = np.array(self.z) - np.array(d1) 
        d3 = np.dot(self.K, d2) #6x1
        self.x = np.array(x_) + np.array(d3) 
 
        #更新后验协方差
        e1 = np.dot(self.K, self.H) #6x6
        e2 = np.dot(e1, p_) #6x6
        self.P = np.array(p_) - np.array(e2)
 
        #更新
        self.G = x_
        return self

 
    def Kalman_Fliter(self,px, py, pz, vx, vy, vz, v_form_x, v_form_y, v_form_z, x_clo, y_clo, z_clo):
        kf_record = []
        kf_p = []
        kalman_filter_params = self.kf_init(px, py, pz, vx, vy, vz, v_form_x, v_form_y, v_form_z)
        kalman_filter_params.z = np.zeros((3,1))
        kalman_filter_params.z[0] = x_clo
        kalman_filter_params.z[1] = y_clo
        kalman_filter_params.z[2] = z_clo
        #kalman_filter_params.z = np.transpose([x_clo, y_clo, z_clo])  # 设置当前时刻的观测位置
        #print("----------------------------------------------")
        #print(kalman_filter_params.z)
        #print("**************************************************************")
        kalman_filter_params = self.kf_update()  # 卡尔曼滤波
        kf_record.append(np.transpose(kalman_filter_params.x))
        kf_p.append(np.transpose(kalman_filter_params.x))
        #print(kalman_filter_params.x) 
        return kf_p
        