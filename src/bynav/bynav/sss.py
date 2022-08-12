import math
import numpy as np
from math import *
import time
points = [[-1,-2,0,1,2,3,5,8], [-3,-4,0,-3,5,9,2,1]]
points = np.array(points)
print(points.shape)
x = np.array(points[0,:])
print(x.shape)
y = np.array(points[1,:])
print(y.shape)

mask = np.logical_and(np.logical_and(x>=0, x<=5), np.logical_and(y>=0, y<=5))
print(mask)
# mask = array([False, False,  True, False,  True, False,  True, False])
print(x.shape)
print(y.shape)
x = x[mask] # x = array([0, 2, 5])
y = y[mask] # y = array([0, 5, 2])
points = [x,y]
print(x.shape)
print(y.shape)
print(points)
points = np.array(points)
print(points)
print(points.shape)
print(x)
print(y)

a = [1,2,3]
b = [4,5,6]
a = np.array(a)
b = np.array(b)
a = a[np.newaxis,:]
b = b[np.newaxis,:]
c = np.concatenate((a,b),axis=0)
print(c.shape)
b = [4,5,6]
print(b)
b = []
print(b)
