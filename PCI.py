#!/usr/bin/env python
# coding: utf-8

# In[4]:


import open3d as o3d
import numpy as np

print("test loading ply point cloud")
data_1=o3d.io.read_point_cloud('./dataset/plys/0.ply')
d_1=np.asarray(data_1.points)
P_0=d_1[:10000]#start point

data_2=o3d.io.read_point_cloud('./dataset/plys/6.ply')
d_2=np.asarray(data_1.points)
P_1=d_2[:10000]#end point


# In[56]:


data_1.paint_uniform_color([1,0.706,0])#start point colored yellow
data_2.paint_uniform_color([0,0.651,0.929])#end point colored blue 


# In[35]:


#calculate center
P_0_mean=np.mean(P_0,axis=0)
print(P_0_mean)
P_1_mean=np.mean(P_1,axis=0)
#P_1_mean
#print(P_1[1][1])
#print(P_1[1])
#test=P_1-P_1_mean
Q_0=P_0-P_0_mean
Q_0_T=Q_0.T
Q_1=P_1-P_1_mean
#Wprint(P_0_T)


# In[40]:


#using least square method on metrix 
#tartget function **1/2 min sigma((P_0(i)-P_0_mean-R(P_1(i)-P_1_mean))), where R is what we want
#traverse all the elements in the array
#for i in range(10000):
#    X=P_1[i]-P_1_mean
#    Y=P_0[i]-P_0_mean
###W=np.identity(10000)
#X=P_1-P_1_mean
#Y=P_0-P_0_mean
#Y_T=Y.T
#print(Y)
#print(Y_T)
##define convariance matrix Con
#Con=X.dot(W.dot(Y_T)) #A.dot(B) refers to matrix multiplication
#print(Con)
#Con.shape


#define convariance matrix W
#sum=0
W=np.empty((3,3))
for i in range(10000):
    X=Q_0_T[:,i]
    Y=Q_1[i]
    #w=x.dot(y)
    #sum+=w
    for x in range(3):
        for y in range(3):
            W[x][y]=X[x]*Y[y]
print(W)


# In[50]:


#do SVD on Con
#the right singular value
F=(W.T).dot(W)
#print(F)
#Eigenvalue decomposition
eig_vals,eig_vecs=np.linalg.eig(F)
V=eig_vecs
F2=(W).dot(W.T)
eig_vals,eig_vecs1=np.linalg.eig(F2)
U=eig_vecs1
A=np.identity(3)
A[0][0]=eig_vals[0]
A[1][1]=eig_vals[1]
A[2][2]=eig_vals[2]


# In[52]:


#solve R
R=U.dot(V.T)
print(R)


# In[55]:


#solve t
t=P_0.T-R.dot(P_1.T)
t_f=t.T
print(t.T)


# In[65]:


e_P_0=R.dot(P_0.T)+t
m=e_P_0.T


# In[63]:


vis=o3d.visualization.Visualizer()
vis.create_window()


# In[68]:


start = o3d.geometry.PointCloud()
start.points = o3d.utility.Vector3dVector(m)
end = o3d.geometry.PointCloud()
end.points = o3d.utility.Vector3dVector(P_1)


# In[72]:


start.paint_uniform_color([1,0.3,0])#start point colored yellow
end.paint_uniform_color([0,0.2,0.929])#end point colored blue 


# In[ ]:


o3d.visualization.draw_geometries([data_1,data_2,start,end])


# In[ ]:




