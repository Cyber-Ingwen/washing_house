import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation as Rota
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt


class KalmanFilterNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(("\033[1;32m----> "+str(name)+" Started.\033[0m"))
        
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
        self.last_z = None
        
    def callback_imu(self, data):
        a = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
        omega = [data.angular_velocity.x * np.pi / 180, data.angular_velocity.y * np.pi / 180, data.angular_velocity.z * np.pi / 180]
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
        
        correct_z = theta[1, 0]
        # correct_z = theta[1, 0] - np.pi
        # if(self.last_z != None):
        #     if correct_z - self.last_z < -np.pi:
        #         correct_z += 2 * np.pi
        #     if correct_z - self.last_z > np.pi:
        #         correct_z -= 2 * np.pi

        self.last_z = correct_z
        
        # print("\r", correct_z, theta[1, 0], end="", flush = True)
        
        odom_data = [x, y, z, theta[2, 1], theta[0, 2], correct_z]
        self.inputs[1] = odom_data
        
        self.recive_odom_flag = 1
    
    def update(self):
        self.filter.init(self.inputs)
        if self.recive_odom_flag == 1:
            # Δt
            [sec, nano] = self.get_clock().now().seconds_nanoseconds()
            now_time = sec + 1e-9 * nano
            self.filter.delta_t = now_time - self.last_time if(self.last_time != 0) else(0.1)
            self.last_time = now_time
            
            self.filter.update(self.inputs)
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
        

class ESKalmanFilter():
    def __init__(self):
        self.delta_t = 0.5
        one_matrix = np.eye(3)
        zero_matrix = np.zeros((3, 3))
        self.P = np.eye(18) * 1e-6
        # self.P = np.array([[4.25911283e-05,-1.81217681e-09,-7.33217963e-10,2.96520252e-04,-2.60368665e-08,-7.67855854e-09,-2.59363180e-17,1.68913650e-17,4.32174011e-17,2.38493023e-06,3.74066059e-06,7.94598083e-08,1.29241826e-12,-8.24435471e-13,-2.71960446e-12,4.21164253e-06,-1.05419501e-07,2.55232514e-08],[-1.81217681e-09,4.25936206e-05,-6.73675861e-10,-3.12352554e-08,2.96559202e-04,2.31956495e-09,-1.66272799e-17,-2.41767720e-17,1.78398822e-17,-3.77316581e-06,2.36892563e-06,-3.09793203e-07,7.86219246e-13,1.15018228e-12,-4.39039278e-13,6.91508826e-08,4.23433797e-06,-4.61295828e-08],[-7.33217963e-10,-6.73675861e-10,4.26031403e-05,-1.52360803e-08,-2.38683240e-08,2.96709516e-04,5.62534570e-18,2.17425373e-17,3.86105250e-19,3.07699286e-07,-1.22208072e-07,-4.54313024e-06,-7.63319203e-13,-1.75949131e-12,-3.55364521e-14,-3.79029064e-08,2.99583085e-08,4.33458685e-06],[2.96520252e-04,-3.12352554e-08,-1.52360803e-08,8.02814189e-03,-4.52283786e-07,-1.79144870e-07,-1.03187110e-12,7.12956315e-13,-2.14008732e-12,3.76884900e-05,5.90201922e-05,1.33310693e-06,1.86362144e-11,-1.18845606e-11,-3.93232760e-11,6.64820928e-05,-1.66203446e-06,4.05171801e-07],[-2.60368665e-08,2.96559202e-04,-2.38683240e-08,-4.52283786e-07,8.02875116e-03,-1.72262011e-07,-7.27888336e-13,-1.06228726e-12,1.00571628e-12,-5.95424492e-05,3.74417725e-05,-4.61723189e-06,1.13357613e-11,1.65647463e-11,-6.35165734e-12,1.09094887e-06,6.68379248e-05,-7.21573769e-07],[-7.67855854e-09,2.31956495e-09,2.96709516e-04,-1.79144870e-07,-1.72262011e-07,8.03112462e-03,-9.08281463e-13,-5.31787226e-13,-2.06810743e-14,4.67223587e-06,-1.71303538e-06,-7.17258916e-05,-1.10346223e-11,-2.54278955e-11,-5.09176056e-13,-5.98366261e-07,4.58404841e-07,6.84208311e-05],[-2.59363180e-17,-1.66272799e-17,5.62534570e-18,-1.03187110e-12,-7.27888336e-13,-9.08281463e-13,6.66155628e-07,-7.19479748e-12,-2.90230453e-09,9.87513518e-20,1.33747295e-18,7.57433157e-19,-1.22734840e-11,1.33048468e-16,5.34724644e-14,1.14627293e-18,1.99171870e-19,-1.38502883e-18],[1.68913650e-17,-2.41767720e-17,2.17425373e-17,7.12956315e-13,-1.06228726e-12,-5.31787226e-13,-7.19479738e-12,6.66132415e-07,8.26602489e-10,-8.95427158e-19,1.91330945e-19,4.49051407e-18,1.32271582e-16,-1.22730565e-11,-1.52287946e-14,-1.65475814e-18,3.60740784e-18,-5.86863073e-18],[4.32174011e-17,1.78398822e-17,3.86105250e-19,-2.14008732e-12,1.00571628e-12,-2.06810743e-14,-2.90230453e-09,8.26602489e-10,9.99573009e-07,-1.75292719e-18,-9.13541170e-18,-2.36490201e-20,5.34721432e-14,-1.52304369e-14,-1.84164633e-11,-8.20722429e-18,-3.21593696e-18,-8.53228152e-19],[2.38493023e-06,-3.77316581e-06,3.07699286e-07,3.76884900e-05,-5.95424492e-05,4.67223587e-06,9.87513518e-20,-8.95427158e-19,-1.75292719e-18,9.37829587e-05,-5.72166181e-08,1.28451663e-07,-9.40431954e-15,-3.88550576e-15,1.45595422e-13,-3.81339386e-06,2.45999480e-06,9.58432553e-07],[3.74066059e-06,2.36892563e-06,-1.22208072e-07,5.90201922e-05,3.74417725e-05,-1.71303538e-06,1.33747295e-18,1.91330945e-19,-9.13541170e-18,-5.72166181e-08,9.38353027e-05,3.25948385e-09,-3.14365591e-14,-1.80558650e-14,4.98503146e-13,-3.26074676e-06,-4.20647948e-06,-1.65523361e-08],[7.94598083e-08,-3.09793203e-07,-4.54313024e-06,1.33310693e-06,-4.61723189e-06,-7.17258916e-05,7.57433157e-19,4.49051407e-18,-2.36490201e-20,1.28451663e-07,3.25948385e-09,9.34858930e-05,-8.77433499e-14,-3.69940072e-13,3.61156958e-15,6.04207880e-07,-3.34451475e-07,2.19939696e-06],[1.29241826e-12,7.86219246e-13,-7.63319203e-13,1.86362144e-11,1.13357613e-11,-1.10346223e-11,-1.22734840e-11,1.32271582e-16,5.34721432e-14,-9.40431954e-15,-3.14365591e-14,-8.77433499e-14,9.99228451e-07,-8.00595015e-15,2.18458131e-14,-3.15667885e-14,2.47584440e-14,1.35924864e-13],[-8.24435471e-13,1.15018228e-12,-1.75949131e-12,-1.18845606e-11,1.65647463e-11,-2.54278955e-11,1.33048471e-16,-1.22730565e-11,-1.52304369e-14,-3.88550576e-15,-1.80558650e-14,-3.69940072e-13,-8.00595015e-15,9.99228468e-07,-1.61706118e-14,9.49737388e-14,-2.32481339e-13,4.76039052e-13],[-2.71960446e-12,-4.39039278e-13,-3.55364521e-14,-3.93232760e-11,-6.35165734e-12,-5.09176056e-13,5.34724644e-14,-1.52287946e-14,-1.84164633e-11,1.45595422e-13,4.98503146e-13,3.61156958e-15,2.18458131e-14,-1.61706118e-14,9.99228660e-07,4.72316793e-13,1.35147365e-13,4.79318723e-14],[4.21164253e-06,6.91508826e-08,-3.79029064e-08,6.64820928e-05,1.09094887e-06,-5.98366261e-07,1.14627293e-18,-1.65475814e-18,-8.20722429e-18,-3.81339386e-06,-3.26074676e-06,6.04207880e-07,-3.15667885e-14,9.49737388e-14,4.72316793e-13,8.92518923e-05,4.40861215e-08,-2.41686657e-08],[-1.05419501e-07,4.23433797e-06,2.99583085e-08,-1.66203446e-06,6.68379248e-05,4.58404841e-07,1.99171870e-19,3.60740784e-18,-3.21593696e-18,2.45999480e-06,-4.20647948e-06,-3.34451475e-07,2.47584440e-14,-2.32481339e-13,1.35147365e-13,4.40861215e-08,8.92335962e-05,1.79235811e-08],[2.55232514e-08,-4.61295828e-08,4.33458685e-06,4.05171801e-07,-7.21573769e-07,6.84208311e-05,-1.38502883e-18,-5.86863073e-18,-8.53228152e-19,9.58432553e-07,-1.65523361e-08,2.19939696e-06,1.35924864e-13,4.76039052e-13,4.79318723e-14,-2.41686657e-08,1.79235811e-08,8.90404150e-05]])
        self.P = np.block([[zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix], 
                           [zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, one_matrix * 1e-4, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, zero_matrix, one_matrix * 1e-6, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, one_matrix * 1e-4]])
        self.Q = np.block([[zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix], 
                           [zero_matrix, one_matrix * 2.5e-3, zero_matrix, zero_matrix, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, one_matrix * 2.5e-3, zero_matrix, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, one_matrix * 3e-9, zero_matrix, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, zero_matrix, one_matrix * 5.4e-12, zero_matrix],
                           [zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix, zero_matrix]])
        self.R = np.block([[one_matrix * 1e-4, zero_matrix], 
                           [zero_matrix, one_matrix * 1e-6]])
        
        self.nominal_state = np.zeros([18])
        self.error_state = np.zeros([18])
        
        self.theta = np.array([0, 0, np.pi])
        self.ba = np.array([-0.46, -0.26, 0])
        self.bg = np.array([3.5e-4, 1.5e-3, 1.1e-3])
        self.g = np.array([0, 0, -9.815])
        self.nominal_state[6:9] = self.theta
        self.nominal_state[9:12] = self.ba
        self.nominal_state[12:15] = self.bg
        self.nominal_state[15:18] = self.g
        
        self.count = 0
        self.list = []
        
    def init(self, inputs):
        if (self.count < 100):
            [imu_data, odom_data] = inputs
            [a, omega] = imu_data
            
            a = np.array(a)
            omega = np.array(omega)
            self.ba = self.count / (self.count + 1) * self.ba + a * 1 / (self.count + 1)
            self.bg = self.count / (self.count + 1) * self.bg + omega / (self.count + 1)
            
            self.count += 1 
            
            self.list.append((self.ba[0]))
            print("\r", self.count, "--", self.ba[0], end="", flush = True)
            
        if (self.count == 100):
            fig, ax = plt.subplots(1, 1)
            ax.plot(self.list, color="#7744FF", lw=3)
            
            plt.show()
        
        if (self.count == 100):
            self.nominal_state[9:12] = self.ba
            self.nominal_state[12:15] = self.bg
        
    def update(self, inputs):
        if (self.count < 100): return 
        
        [imu_data, odom_data] = inputs
        A = self._get_A(imu_data)
        H = self._get_H()
        z = odom_data
        
        x = self.nominal_state
        
        x = self.update_nominal_state(x, imu_data)
        P_pr = A @ self.P @ A.T + self.Q 
        K = P_pr @ H.T @ np.linalg.inv(H @ P_pr @ H.T + self.R)
        x_po = K @ (z - H @ x)
        
        self.P = P_pr - K @ H @ P_pr
        x += x_po
        self.nominal_state = x
        
        temp = np.linalg.det(H @ P_pr @ H.T + self.R)
        print("\r", temp, end="", flush = True)
        
        # if (temp <= 2e-19):
        #     print(self.P)
        
        # print("\r", self.nominal_state[9:12], end="", flush = True)
        
        return self.nominal_state
    
    def update_nominal_state(self, x, imu_data):
        [a, omega] = imu_data
        theta = x[6:9]
        bg = x[9:12]
        ba = x[12:15]
        g = x[15:18]
        Rot = self._exp(theta)
        
        x[0:3] += x[3:6] * self.delta_t
        x[3:6] += Rot @ (a - ba) * self.delta_t + g * self.delta_t
        x[6:9] += self._get_vect(Rot @ self._get_skew(omega - bg)) * self.delta_t
        
        return x

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
        E_matrix = one_matrix - self._get_skew((omega - bg) * self.delta_t)
        
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
        
    def _get_vect(self, x):
        return np.array([x[2, 1], x[0, 2], x[1, 0]])
        
    def _exp(self, x):
        return expm(self._get_skew(x))
    
    def _get_jacobi(self, theta):
        a = np.linalg.norm(theta)
        if (a == 0.0):
            j = np.eye(3)
        elif (a == 2 * np.pi):
            j = np.eye(3)
        elif (a != 0) & (a != 2 * np.pi):
            b = theta / a
            
            temp = 0.5 * a * (1 / np.tan(0.5 * a))
            j = temp * np.eye(3) + (1 - temp) * b.reshape(3,1) @ b.reshape(1,3) + 0.5 * a * self._get_skew(b)
        else:
            print("_____________WARNING____________")
            print(a)
        
        return j
        
        
def main(args=None):
    rclpy.init(args=args) 
    node = KalmanFilterNode("eskf_node")  
    rclpy.spin(node) 
    rclpy.shutdown()
