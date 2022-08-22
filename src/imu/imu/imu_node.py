from re import T
from statistics import NormalDist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import math
from std_msgs.msg import String
from  matplotlib import pyplot

import sys,os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

from imu_fliter_node import IMUPublisher
'''x_linear = 0
y_linear = 0
z_linear = 0
x_angle = 0
y_angle = 0
z_angle = 0
q0 = float(1.0)
q1 = float(0.0)
q2 = float(0.0)
q3 = float(0.0)'''

class IMUSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(Imu,'/imu',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, kk):
        data=kk
        x_linear = data.linear_acceleration.x
        y_linear = data.linear_acceleration.y
        z_linear = data.linear_acceleration.z
        x_angle = data.angular_velocity.x
        y_angle = data.angular_velocity.y
        z_angle = data.angular_velocity.z
        #print("1")
        m = mahony()
        #print(x_linear,'--',y_linear,'--',z_linear,'/n',x_angle ,'--',y_angle,'--',z_angle,'/n')
        m.mahony_imu(x_linear,y_linear,z_linear,x_angle,y_angle,z_angle)
        #print(x_l,'--',y_l,'--',z_l,'/n',x_a ,'--',y_a,'--',z_a,'/n')
        #print(x_linear,'--',y_linear,'--',z_linear,'/n',x_angle ,'--',y_angle,'--',z_angle,'/n')
        #x_linear=np.array(list(x_linear))
        #print(data)
        #print(data.angular_velocity.x)
        #print(data.linear_acceleration.y)
        #self.get_logger().info('节点已创建')
        k = IMUPublisher()
        k.timer_callback(x_angle,y_angle,z_angle,x_linear,y_linear,z_linear,m.q0,m.q1,m.q2,m.q3)

class mahony():
    q0 = float(1.0)
    q1 = float(0.0)
    q2 = float(0.0)
    q3 = float(0.0)
    ki = 2.0*0.0 #2 * integral gain (Ki)
    kp = 2.0*5.0 #2 * proportional gain (Kp)
    integeral_x = 0.0 #integral error terms scaled by Ki
    integeral_y = 0.0
    integeral_z = 0.0
    
    def mahony_imu(self,x_g,y_g,z_g,x_a,y_a,z_a):
        #print("2")
        if not((x_a == 0.0) and (y_a == 0.0) and (y_a == 0.0)):
            #print("3")
            Normcenter = Solution.invSqrt(x_a * x_a + y_a * y_a + z_a * z_a) #Normalise
            #print(Normcenter)
            x_a *= Normcenter
            y_a *= Normcenter
            z_a *= Normcenter
            #重力分量
            v_x = self.q1 * self.q3 - self.q0 * self.q2
            v_y = self.q0 * self.q1 + self.q2 * self.q3
            v_z = self.q0 * self.q0 - 0.5 + self.q3 * self.q3
            #error
            e_x = (y_a * v_z - z_a * v_y)
            e_y = (z_a * v_x - x_a * v_z)
            e_z = (x_a * v_y - y_a * v_x)
            if(self.ki > 0): #对误差进行积分运算
                self.integeral_x += self.ki * e_x * (1.0 / 125)
                self.integeral_y += self.ki * e_y * (1.0 / 125)
                self.integeral_z += self.ki * e_z * (1.0 / 125)
                x_g += self.integeral_x
                y_g += self.integeral_y
                z_g += self.integeral_z
            else:
                self.integeral_x = 0.0
                self.integeral_y = 0.0
                self.integeral_z = 0.0
            x_g += self.kp * e_x #对误差进行比例运算
            y_g += self.kp * e_y
            z_g += self.kp * e_z

        #integrate rate of change of quaternion
        x_g *= 0.5 * (1.0 / 100)
        y_g *= 0.5 * (1.0 / 100)
        z_g *= 0.5 * (1.0 / 100)
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += (-qb * x_g - qc * y_g - self.q3 * z_g)
        self.q1 += (qa * x_g + qc * z_g - self.q3 * y_g)
        self.q2 += (qa * y_g - qb * z_g + self.q3 * x_g)
        self.q3 += (qa * z_g + qb * y_g - qc * x_g)
        #normalise quaternion
        Norm1 = Solution.invSqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= Norm1
        self.q1 *= Norm1
        self.q2 *= Norm1
        self.q3 *= Norm1
        #print(self.q0,"---",self.q1,"---",self.q2,"---",self.q3,"---")
        #print("\r%s,%s,%s",(x_g,y_g,z_g), end = "", flush=True)
        value = [[1 - 2 * self.q2 * self.q2 - 2 * self.q3 * self.q3, 2 * self.q1 * self.q2 - 2 * self.q3 * self.q0,2 * self.q1 * self.q3 + 2 * self.q2 * self.q0], 
        [2 * self.q1 * self.q2 + 2 * self.q3 * self.q0, 1 - 2 * self.q1 * self.q1 - 2 * self.q3 * self.q3, 2 * self.q2 * self.q3 - 2 * self.q1 * self.q0],
        [2 * self.q1 * self.q3 - 2 * self.q2 * self.q0,2 * self.q2 * self.q3 + 2 * self.q1 * self.q0,1-2 * self.q1 * self.q1 - 2 * self.q2 * self.q2]]
        R = np.array(value)
        print(R)
        return 1

class Solution():
    def invSqrt(num):
        # t = num
        # t = 0x5f3759df - (t/2.0)
        # while not ((t * t <= num) and ((t+1) * (t+1) > num)):
        #     t = ((num / t) + t) / 2.0
        # return t
        t = num
        t = math.sqrt(t)
        t = 1 / t
        return t

def main(args=None):
    rclpy.init(args=args)
    IMU_subscriber = IMUSubscriber()

    rclpy.spin(IMU_subscriber)
    IMU_subscriber.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()