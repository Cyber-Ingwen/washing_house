import numpy as np
import math
import matplotlib.pyplot as plt

# 求出两个点之间的向量弧度，向量方向由点1指向点2
def GetTheAngleOfTwoPoints(x1,y1,x2,y2):
    return math.atan2(y2-y1,x2-x1)

# 求出两个点的距离
def GetDistOfTwoPoints(x1,y1,x2,y2):
    return math.sqrt(math.pow(x2-x1,2) + math.pow(y2-y1,2))

# 在pt_set点集中找到距(p_x，p_y)最近点的id
def GetClosestID(p_x,p_y,pt_set):
    id = 0
    min = 10000000
    for i in range(pt_set.shape[1]):
        dist = GetDistOfTwoPoints(p_x,p_y,pt_set[0][i],pt_set[1][i])
        if dist < min:
            id = i
            min = dist
    return id

# 求两个点集之间的平均点距
def DistOfTwoSet(set1,set2):
    loss = 0;
    for i in range(set1.shape[1]):
        id = GetClosestID(set1[0][i],set1[1][i],set2)
        dist = GetDistOfTwoPoints(set1[0][i],set1[1][i],set2[0][id],set2[1][id])
        loss = loss + dist
    return loss/set1.shape[1]

def ICP(sourcePoints,targetPoints):
    A = targetPoints # A是标准点云
    B = sourcePoints # B是源点云

    iteration_times = 0 # 迭代次数为0
    dist_now = 1 # A,B两点集之间初始化距离
    dist_improve = 1 # A,B两点集之间初始化距离提升
    dist_before = DistOfTwoSet(A,B) # A,B两点集之间距离
    while iteration_times < 10 and dist_improve > 0.001: # 迭代次数小于10 并且 距离提升大于0.001时，继续迭代
        x_mean_target = A[0].mean() # 获得A点云的x坐标轴均值。
        y_mean_target = A[1].mean() # 获得A点云的y坐标轴均值。
        x_mean_source = B[0].mean() # 获得B点云的x坐标轴均值。
        y_mean_source = B[1].mean() # 获得B点云的y坐标轴均值。

        A_ = A - np.array([[x_mean_target],[y_mean_target]]) # 获得A点云的均一化后点云A_
        B_ = B - np.array([[x_mean_target],[y_mean_target]]) # 获得B点云的均一化后点云B_

        w_up = 0 # w_up，表示角度公式中的分母
        w_down = 0 # w_up，表示角度公式中的分子
        for i in range(A_.shape[1]): # 对点云中每个点进行循环
            j = GetClosestID(A_[0][i],A_[1][i],B) # 在B点云中找到距(A_[0][i],A_[1][i])最近点的id
            w_up_i = A_[0][i]*B_[1][j] - A_[1][i]*B_[0][j] # 获得求和公式，分母的一项
            w_down_i = A_[0][i]*B_[0][j] + A_[1][i]*B_[1][j] # 获得求和公式，分子的一项
            w_up = w_up + w_up_i
            w_down = w_down + w_down_i

        TheRadian = math.atan2(w_up,w_down) # 分母与分子之间的角度
        x = x_mean_target - math.cos(TheRadian)*x_mean_source - math.sin(TheRadian)*y_mean_source # x即为x轴偏移量
        y = x_mean_target + math.cos(TheRadian)*x_mean_source - math.sin(TheRadian)*y_mean_source # y即为y轴偏移量
        R = np.array([[math.cos(TheRadian),math.sin(TheRadian)],[-math.sin(TheRadian),math.cos(TheRadian)]]) # 由θ得到旋转矩阵。

        B = np.matmul(R,B) + np.array([x],[y])

        iteration_times = iteration_times + 1 # 迭代次数+1
        dist_now = DistOfTwoSet(A,B) # 变换后两个点云之间的距离
        dist_improve = dist_before - dist_now # 这一次迭代，两个点云之间的距离提升。
        print("迭代第"+str(iteration_times)+"次，损失是"+str(dist_now)+",提升了"+str(dist_improve)) # 打印迭代次数、损失距离、损失提升
        dist_before = dist_now # 将"现在距离"赋值给"以前距离"，开始下一轮迭代循环。

    return B