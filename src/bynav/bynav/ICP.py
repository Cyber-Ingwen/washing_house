import open3d as o3d
import numpy as np
import time


time_0 = time.perf_counter() *1000

#读取电脑中的 ply 点云文件
source = o3d.io.read_point_cloud("plys/1.ply")  #source 为需要配准的点云
target = o3d.io.read_point_cloud("plys/0.ply")  #target 为目标点云

#为两个点云上上不同的颜色
source.paint_uniform_color([1, 0.706, 0])    #source 为黄色
target.paint_uniform_color([0, 0.651, 0.929])#target 为蓝色

processed_source, outlier_index = source.remove_radius_outlier(nb_points=16, radius=0.5)

processed_target, outlier_index = target.remove_radius_outlier(nb_points=16, radius=0.5)

x = np.array(processed_source.points)
y = np.array(processed_target.points)

x = (x[0:4000]).T
y = (y[0:4000]).T

#o3d.visualization.draw_geometries([source, target])

xm = np.mean(x, axis=1)
ym = np.mean(y, axis=1)

H = np.matmul((x.T - xm.T).T, (y.T - ym.T))

u,s,v = np.linalg.svd(H)

A = np.array([[1, 0, 0],
            [0, 1, 0],
            [0, 0, np.linalg.det(np.matmul(v, u.T))]])

print(A)

#R = np.matmul(v, np.matmul(A, u.T))
R = np.matmul(v, u.T)
T = ym - np.matmul(R, xm)

temp = np.matmul(R, x)

y_pred = temp.T + T


target_pred = o3d.geometry.PointCloud()
target_pred.points = o3d.utility.Vector3dVector(y_pred)

target_pred.paint_uniform_color([0.22, 0.651, 0.333])

o3d.visualization.draw_geometries([target_pred, processed_target, processed_source])
