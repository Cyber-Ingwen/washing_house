import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation as Rota
from sensor_msgs.msg import Imu


class KalmanFilterNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(name + "节点已创建")
        
        self.sub_imu = self.create_subscription(Imu, "/imu", self.callback_imu, 10)
        self.sub_odom = self.create_subscription(Odometry, "/frame_odom2", self.callback_odom, 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom3", 10) 
        self.pub_path = self.create_publisher(Path, "/path2", 10) 
        
        self.filter = ESKalmanFilter()
        self.path = Path()
        self.inputs = [1, 1]
        self.recive_odom_flag = 0
        self.count = 0
        self.last_time = 0
        
    def callback_imu(self, data):
        a = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
        omega = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
        imu_data = [a, omega]
        self.inputs[0] = imu_data
        
        self.update()
            
    def callback_odom(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        
        q1 = data.pose.pose.orientation.x
        q2 = data.pose.pose.orientation.y
        q3 = data.pose.pose.orientation.z
        q4 = data.pose.pose.orientation.w
        Rq = [q1, q2, q3, q4]
        Rm = (Rota.from_quat(Rq)).as_matrix()
        Rm = np.array(Rm)
        theta = logm(Rm)
        
        odom_data = [x, y, z, theta[2, 1], theta[0, 2], theta[1, 0]]
        self.inputs[1] = odom_data
        
        self.recive_odom_flag = 1
    
    def update(self):
        if self.recive_odom_flag == 1:
            # Δt
            [sec, nano] = self.get_clock().now().seconds_nanoseconds()
            now_time = sec + 1e-9 * nano
            self.filter.delta_t = now_time - self.last_time if(self.last_time != 0) else(0.1)
            self.last_time = now_time
            
            res =  self.filter.update(self.inputs)
            self.pub()
            self.recive_odom_flag = 0
            
    def pub(self):
        odom = Odometry()
        odom.pose.pose.position.x = self.filter.nominal_state[0]
        odom.pose.pose.position.y = self.filter.nominal_state[1]
        odom.pose.pose.position.z = self.filter.nominal_state[2]
        
        theta = [self.filter.nominal_state[6], self.filter.nominal_state[7], self.filter.nominal_state[8]]
        theta = self.filter.nominal_state[6:9]
        R = self.filter._exp(theta)
        q = (Rota.from_matrix(R)).as_quat()
        
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.header.frame_id = "map"
        
        self.pub_odom.publish(odom)
        
        pose_stamped = PoseStamped()
        
        pose_stamped.header.stamp = odom.header.stamp
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = odom.pose.pose
        self.path.poses.append(pose_stamped)
        self.path.header.stamp = odom.header.stamp
        self.path.header.frame_id = "map"
        self.pub_path.publish(self.path)
        
        # print("\r", self.filter.nominal_state[15], self.filter.nominal_state[16], self.filter.nominal_state[17], end="", flush = True)
        

class ESKalmanFilter():
    def __init__(self):
        self.delta_t = 0.5
        one_matrix = np.eye(3)
        zero_matrix = np.zeros((3, 3))
        self.P = np.eye(18) * 1e-2
        self.Q = np.block([[zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix], 
                           [zero_matrix, one_matrix * 1e-2, zero_matrix, zero_matrix, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, one_matrix * 1e-3, zero_matrix, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, one_matrix * 1e-3, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, zero_matrix, one_matrix * 1e-3, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix]])
        self.R = np.block([[one_matrix * 1e-2, zero_matrix], 
                           [zero_matrix, one_matrix * 1e-4]])
        
        self.nominal_state = np.zeros([18])
        self.error_state = np.zeros([18])
        
        self.ba = np.array([-0.46, -0.26, 0])
        self.g = np.array([0, 0, -9.8])
        self.nominal_state[9:12] = self.ba
        self.nominal_state[15:18] = self.g
        
    def update(self, inputs):
        [imu_data, odom_data] = inputs
        A = self._get_A(imu_data)
        H = self._get_H()
        z = odom_data
        
        x = self.nominal_state
        
        P_pr = A @ self.P @ A.T + self.Q 
        K = P_pr @ H.T @ np.linalg.inv(H @ P_pr @ H.T + self.R)
        x_po = K @ (z - H @ x)
        
        self.P = P_pr - K @ H @ P_pr
        x += x_po
        self.nominal_state = x
        
        print("\r", x_po[6:9], end="", flush = True)
        
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
        return np.array([[0, -x[2], x[1]],
                         [x[2], 0, -x[0]],
                         [-x[1], x[0], 0]])
        
    def _exp(self, x):
        return expm(self._get_skew(x))
    
    def _get_jacobi(self, theta):
        a = np.linalg.norm(theta)
        if (a != 0):
            b = theta / a
            
            temp = 0.5 * a * (1 / np.tan(0.5 * a))
            j = temp * np.eye(3) + (1 - temp) * b @ b.T + 0.5 * a * self._get_skew(b)
        else:
            j = np.eye(3)
        
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
