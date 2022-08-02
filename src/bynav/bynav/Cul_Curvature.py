import math
import numpy as np


class Cul_Curvature():
    def __init__(self):
        self.processed_pcn = []
        self.edge_points = []
        self.plane_points = []
    
    def process(self, pcn):
        self.processed_pcn = []
        list = []
        list2 = []

        a, b = pcn.shape
        for i in range(a):
            x = pcn[i][0]
            y = pcn[i][1]
            z = pcn[i][2]
            
            if i % 16 >= 4:
                self.processed_pcn.append([x, y, z])
                    
        a = len(self.processed_pcn)
        for i in range(a):
            x = self.processed_pcn[i][0]
            y = self.processed_pcn[i][1]
            z = self.processed_pcn[i][2]

            curv = 0
            sum = [0, 0, 0]
            if((i - 12 * 5 >= 0 ) & (i + 12 * 5 < a)):
                for j in range(5):
                    sum[0] += (x - self.processed_pcn[i - 12 * j][0])
                    sum[0] += (x - self.processed_pcn[i + 12 * j][0])
                    sum[1] += (y - self.processed_pcn[i - 12 * j][1])
                    sum[1] += (y - self.processed_pcn[i + 12 * j][1])
                    sum[2] += (z - self.processed_pcn[i - 12 * j][2])
                    sum[2] += (z - self.processed_pcn[i + 12 * j][2])

                curv = sum[0] ** 2 + sum[1] ** 2 + sum[2] ** 2 
            
            if not math.isnan(curv): 
                if(curv < 100) & (curv > 0.2):
                    list.append([x, y, z, curv])
                    #print("\r curv = %s " % (curv), end = "")
                elif(curv < 0.1) & (curv > 0):
                    list2.append([x, y, z, curv])
                    #print("\r curv = %s " % (curv), end = "")
            
        list = np.array(list)
        list2 = np.array(list2)

        self.edge_points = list[:,:3]
        self.plane_points = list2[:,:3]

        return list

    def process2(self, pcn):
        a, b = pcn.shape
        for i in range(a):
            x = pcn[i][0]
            y = pcn[i][1]
            z = pcn[i][2]

            vec0 = np.array([x, y, z])
            vec_list = np.array(self.edge_points)
            distance = np.linalg.norm(vec_list - vec0, axis = 1)
            j = np.argmax(-distance)

            d = np.linalg.norm(vec_list[j] - vec_list[j-1])
            s = np.linalg.norm(np.cross(vec_list[j] - vec0, vec_list[j-1] - vec0))
            h = s / d

            #print("\r h = %s " % (h), end = "")

        return 0