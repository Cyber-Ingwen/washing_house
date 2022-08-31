import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from scipy.linalg import expm

    
class KalmanFilterNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(name + "节点已创建")
        
        self.sub_imu = self.create_subscription(Imu, "/imu", self.callback_imu, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        
        self.filter = ESKalmanFilter()
        self.inputs = None
        self.recive_odom_flag = 0
        
    def callback_imu(self, data):
        a = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
        omega = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
        imu_data = [a, omega]
    
    def callback_odom(self, data):
        x = data.pose.pose.position.x
        data.pose.pose.position.y = y
        data.pose.pose.position.z = z
    
    def update(self):
        self.filter.update(self.inputs)
        

class ESKalmanFilter():
    def init(self):
        self.delta_t = 0.01
        self.P = 0.01 * np.eye(18)
        self.Q = 0.01 * np.eye(18)
        self.R = 0.01 * np.eye(6)
        
        self.nominal_state = np.zeros([18])
        self.error_state = np.zeros([18])
        
        self.g = np.array([0, 0, 9.8])
        
    def update(self, inputs):
        [imu_data, odom_data] = inputs
        A = self._get_A(imu_data)
        H = self._get_H()
        z = odom_data
        
        delta_x = self.error_state
        x = self.nominal_state
        
        delta_x_pr = A @ delta_x
        P_pr = A @ self.P @ A.T + self.Q 
        K = P_pr @ H.T @ np.linalg.inv(H @ P_pr @ H.T + self.R)
        x_po = K @ (z - H @ (x + delta_x_pr))
        
        self.P = P_pr - K @ H @ P_pr
        x += x_po
        self.nominal_state = x
        
        return self.nominal_state

    def _get_A(self, imu_data):
        #const
        one_matrix = np.eye(3)
        zero_matrix = np.zeros((3, 3))
        delta_t_matrix = one_matrix * self.delta_t
        
        #imu
        [a, omega] = imu_data
        
        #norminal
        theta = self.nominal_state[6:9]
        ba = self.nominal_state[9:12]
        bg = self.nominal_state[12:15]
        Rot = self._exp(theta)
        
        R1_matrix = -Rot @ self._get_skew(a - ba) * self.delta_t
        R1_matrix = self._exp(Rot)
        R2_matrix = -Rot * self.delta_t
        E_matrix = self._exp(-(omega - bg) * self.delta_t)
        
        return np.block([[one_matrix, delta_t_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix], 
                         [zero_matrix, one_matrix, R1_matrix, R2_matrix, zero_matrix, delta_t_matrix],
                         [zero_matrix, zero_matrix, E_matrix, zero_matrix, -delta_t_matrix, zero_matrix],
                         [zero_matrix, zero_matrix, zero_matrix, one_matrix, zero_matrix, zero_matrix],
                         [zero_matrix, zero_matrix, zero_matrix, zero_matrix, one_matrix, zero_matrix],
                         [zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, one_matrix]])
        
    def _get_H(self):
        one_matrix = np.eye(3)
        zero_matrix = np.zeros((3, 3))
        
        h1 = np.block([[one_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix], 
                       [zero_matrix, zero_matrix, one_matrix, zero_matrix, zero_matrix, zero_matrix]])
        
        theta = self.nominal_state[6:9]
        J = self._get_jacobi(theta)
        
        h2 = np.block([[one_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix], 
                       [zero_matrix, one_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix],
                       [zero_matrix, zero_matrix, J, zero_matrix, zero_matrix, zero_matrix],
                       [zero_matrix, zero_matrix, zero_matrix, one_matrix, zero_matrix, zero_matrix],
                       [zero_matrix, zero_matrix, zero_matrix, zero_matrix, one_matrix, zero_matrix],
                       [zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, one_matrix]])
        
        return h1 @ h2
        
    def _get_skew(self, x):
        if(x.shape != (3, 3)): print("warning:", x.shape)
        
        return np.array([[0, -x[2], x[1]],
                         [x[2], 0, -x[0]],
                         [-x[1], x[0], 0]])
        
    def _exp(self, x):
        return expm(self._get_skew(x))
    
    def _get_jacobi(self, theta):
        a = np.linalg.norm(theta)
        b = theta / a
        
        temp = 0.5 * a * (1 / np.tan(0.5 * a))
        j = temp * np.eye(3) + (1 - temp) * b * b.T + 0.5 * a * self._get_skew(b)
        
        return j
        
        
class KalmanFilter():
    def __init__(self):
        self.A = np.zeros([6, 6])
        self.B = np.zeros([6, 6])
        self.H = np.zeros([3, 6])
        
        self.x = np.zeros([6])
        
        self.P = np.zeros([6, 6])
        self.Q = np.zeros([6, 6])
        self.R = np.zeros([3, 3])
        
    def update(self, inputs):
        [u, z] = inputs
        x_pr = self.A @ self.x + self.B @ u
        P_pr = self.A @ self.P @ self.A.T + self.Q 
        K = P_pr @ self.H.T @ np.linalg.inv(self.H @ P_pr @ self.H.T + self.R)
        x_po = x_pr + K @ (z - self.H @ x_pr)
        
        # self.P = P_pr + K @ (self.Q - H @ P_pr @ H.T) @ K.T
        self.P = P_pr - K @ self.H @ P_pr
        self.x = x_po
        
        return x_po
        
def main(args=None):
    rclpy.init(args=args) 
    node = KalmanFilterNode("eskf_node")  
    rclpy.spin(node) 
    rclpy.shutdown()
