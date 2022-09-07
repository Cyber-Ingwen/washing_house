import rclpy
from rclpy.node import Node
import numpy as np
from pylab import *
from nav_msgs.msg import OccupancyGrid, Odometry
import copy


class AStar_node(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(("\033[132m----> "+str(name)+" Started.\033[0m"))
        
        self.sub_map = self.create_subscription(OccupancyGrid, "/map", self.callback_map, 10)
        self.sub_odom = self.create_subscription(Odometry, "/frame_odom2", self.callback_odom, 10)
        self.pub_map = self.create_publisher(OccupancyGrid, "/map2", 10) 
        
        self.w, self.h = 100, 100
        self.r = 0.2
        
        self.start = [0, 0]
        self.goal = [1, 1]
        
    def callback_odom(self, data):
        x = data.pose.pose.position.x - self.w / 2
        y = data.pose.pose.position.y - self.h / 2
        self.start = [int(x / self.r), int(y / self.r)]
    
    def callback_map(self, grid_map):
        map = self._process_map(grid_map)
        a = AStar(map, self.start, self.goal)
        a.main()
        a.path_backtrace()
        self.map_publish(a.draw_direction_point(), grid_map)
        
    def _process_map(self, grid_map):
        width = int(self.w / self.r)
        height = int(self.h / self.r)
        map = np.full((width, height), int(10), dtype=np.int8)
        for i in range(len(grid_map.data)):
            x = i % width
            y = int((i - x) / width)
            if grid_map.data[i] == 100:
                if (x < width and y < height):
                    map[x, y] = 0
                else:
                    print("warning", x, y)
               
        print("start:", self.start)
        if (self.start != [0, 0]):
            map[self.start[0], self.start[1]] = 7
                
        return map
    
    def map_publish(self, map, map_msg):
        grid_map = OccupancyGrid()

        width = int(self.w / self.r)
        height = int(self.h / self.r)

        grid_map.header.frame_id = "map"
        grid_map.header.stamp = map_msg.header.stamp
        grid_map.info.map_load_time = map_msg.header.stamp
        grid_map.info.resolution = self.r
        grid_map.info.width = width
        grid_map.info.height = height
        grid_map.info.origin.position.x = -self.w / 2
        grid_map.info.origin.position.y = -self.h / 2
        grid_map.info.origin.position.z = 0.0
        grid_map.info.origin.orientation.x = 0.0
        grid_map.info.origin.orientation.y = 0.0
        grid_map.info.origin.orientation.z = 0.0
        grid_map.info.origin.orientation.w = 1.0
        
        for i in range(width * height):
            grid_map.data.append(-1)
        
        for x in range(width):
            for y in range(height):
                ind = x + y * width
                if (map[x, y] == 0):
                    grid_map.data[ind] = 100
                elif (map[x, y] == 10):
                    grid_map.data[ind] = 0
                elif (map[x, y] == 3):
                    grid_map.data[ind] = -50
                elif (map[x, y] == 7):
                    grid_map.data[ind] = -126
                else:
                    grid_map.data[ind] = -1
        
        self.pub_map.publish(grid_map)
            
            
class AStar(object):
    """
    创建一个A*算法类
    """
    def __init__(self, map, start, goal):
        """
        初始化
        """
        self.map = map
        self.start = np.array(start)
        self.goal = np.array(goal)

        self.f = 0
        self.g = 0
        self.last_point = np.array([])
        self.current_point = np.array([])
        self.open = np.array([[], []])
        self.closed = np.array([[], []])
        self.record_direction = np.array([[], [], [], []])
        self.best_path_array = np.array([[], []])
        self.point_f = np.array([[], [], []])
        
        self.w, self.h = 100, 100
        self.r = 0.2

    def h_value_tem(self, cur_p):
        """
        计算拓展节点和终点的h值
        """
        h = (cur_p[0] - self.goal[0]) ** 2 + (cur_p[1] - self.goal[1]) ** 2
        h = np.sqrt(h)

        return h

    def g_value(self):
        """
        累计的g值
        """
        self.g = self.g + self.g_value_tem(self.current_point, self.last_point)

    def g_value_tem(self, chl_p, cu_p):
        """
        计算拓展节点和父节点的g值
        """
        g1 = cu_p[0] - chl_p[0]
        g2 = cu_p[1] - chl_p[1]
        g = np.sqrt(g1 ** 2 + g2 ** 2)

        return g

    def f_value_tem(self, chl_p, cu_p):
        """
        求出的是临时g值和h值的和加上累计g值得到全局f值
        """
        f = self.g_value_tem(chl_p, cu_p) + self.h_value_tem(cu_p)
        
        return f

    def min_f(self):
        """
        找出open中f值最小的节点坐标
        """
        tem_f = []
        for i in range(self.open.shape[1]):
            f_value = self.f_value_tem(self.current_point, self.open[:, i]) + self.g
            tem_f.append(f_value)
        index = tem_f.index(min(tem_f))
        location = self.open[:, index]

        return index, location

    def child_point(self, x):
        """
        拓展的子节点坐标
        """
        for j in range(-1, 2):
            for q in range(-1, 2):
                if j == 0 and q == 0:
                    continue
                m = np.array([x[0] + j, x[1] + q])

                if self.map[int(m[0]), int(m[1])] == 0:
                    continue

                self.direction(x, m)

                b = self.judge_location(m, self.closed)
                if b == 1:
                    np.delete(self.record_direction, -1, axis=1)
                    continue

                a = self.judge_location(m, self.open)
                if a == 1:
                    np.delete(self.record_direction, -1, axis=1)

                    continue

                self.open = np.c_[self.open, m]

    def judge_location(self, m, list_co):
        """
        判断拓展点是否在open表或者closed表中
        """
        jud = 0
        for i in range(list_co.shape[1]):

            if m[0] == list_co[0, i] and m[1] == list_co[1, i]:

                jud = jud + 1

        return jud

    def direction(self, father_point, son_point):
        """
        建立每一个节点的方向便于在closed表中选出最佳路径
        """
        x = son_point[0] - father_point[0]
        y = son_point[1] - father_point[1]
        xy = [son_point[0], son_point[1], x, y]
        self.record_direction = np.c_[self.record_direction, xy]

    def path_backtrace(self):
        """
        回溯closed表中的最短路径
        """
        best_path = self.goal
        self.best_path_array = np.array([[self.goal[0]], [self.goal[1]]])
        j = 0
        while j <= self.record_direction.shape[1]:
            for i in range(self.record_direction.shape[1]):
                if best_path[0] == self.record_direction[0][i] and best_path[1] == self.record_direction[1][i]:
                    x = self.record_direction[0][i]-self.record_direction[2][i]
                    y = self.record_direction[1][i]-self.record_direction[3][i]
                    best_path = [x, y]
                    self.best_path_array = np.c_[self.best_path_array, best_path]
                    break
                else:
                    continue
            j = j+1
            
    def draw_direction_point(self):
        map_direction = copy.deepcopy(self.map)
        for i in range(self.best_path_array.shape[1]):
            x = self.best_path_array[:, i]
            if (x[0] == self.start[0]) and (x[1] == self.start[1]):
                continue
            elif (x[0] == self.goal[0]) and (x[1] == self.goal[1]):
                continue
            elif (abs(int(x[0]))<(self.w / self.r) and abs(int(x[1])) < (self.w / self.r)):
                map_direction[int(x[0]), int(x[1])] = 6
                
        # plt.figure()
        # plt.imshow(map_direction, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # xlim(-1, 50)
        # ylim(-1, 50)
        # plt.show()
        
        return map_direction

    def main(self):
        """
        main函数
        """
        self.open = np.column_stack((self.open, self.start))
        self.current_point = self.start
        # self.closed
        ite = 1
        while ite <= 4000:
                if self.open.shape[1] == 0:
                    print('没有搜索到路径！')
                    return

                self.last_point = self.current_point

                index, self.current_point = self.min_f()

                # 选取open表中最小f值的节点作为best，放入closed表
                self.closed = np.c_[self.closed, self.current_point]

                if self.current_point[0] == self.goal[0] and self.current_point[1] == self.goal[1]:
                    return
                
                if abs(self.current_point[0]) >= (self.w / self.r - 1) or abs(self.current_point[1]) >= (self.h / self.r - 1):
                    self.goal = self.current_point
                    return

                self.child_point(self.current_point)

                self.open = delete(self.open, index, axis=1)

                self.g_value()

                ite = ite+1           
           
             
class MAP(object):
    def draw_direction_point(self, a):
        """
        从终点开始，根据记录的方向信息，画出搜索的路径图
        """
        map_direction = copy.deepcopy(a.map)
        for i in range(a.best_path_array.shape[1]):
            x = a.best_path_array[:, i]
            map_direction[int(x[0]), int(x[1])] = 3

        # plt.imshow(map_direction, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
        # xlim(-1, 100)
        # ylim(-1, 100)
        
        return map_direction

    def draw_three_axes(self, a):
        # plt.figure()
        self.draw_direction_point(a)

        # plt.show()
                        
                
def main(args=None):
    rclpy.init(args=args) 
    node = AStar_node("a_star_node")  
    rclpy.spin(node) 
    rclpy.shutdown()