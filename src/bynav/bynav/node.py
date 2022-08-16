import time
import math
import rclpy
from rclpy.node import Node
from serial import Serial
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


class Node_bynav(Node):
    """
    创建bynav节点
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("bynav 节点已创建")

        """创建并初始化发布"""
        self.gps_pub = self.create_publisher(NavSatFix,"gps", 10) 
        self.imu_pub = self.create_publisher(Imu,"imu", 10) 

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        """打开串口"""
        port = "/dev/ttyUSB0"
        baudrate = 115200
        self.ser = Serial(port, baudrate)

        self.get_logger().info("串口已打开")

        self.gps_msg = NavSatFix()
        self.imu_msg = Imu()
        self.gps_flag = 0
        self.imu_flag = 0

    def Eular2Quat(self, EularAngle_list):
        """欧拉角转四元数"""
        [Roll, Pitch, Azimuth] = EularAngle_list

        cy = math.cos(Azimuth * 0.5)
        sy = math.sin(Azimuth * 0.5)
        cp = math.cos(Pitch * 0.5)
        sp = math.sin(Pitch * 0.5)
        cr = math.cos(Roll * 0.5)
        sr = math.sin(Roll * 0.5)
    
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        Quat_list = [x, y, z, w]
        
        return Quat_list

    def timer_callback(self):
        """读取解析数据"""
        line = self.ser.readline()
        line = str(line)
        #line = "#RAWIMUA,ICOM4,0,0.0,FINESTEERING,2107,37454.000,00000000,0000,68;2107,37454.000000000,00000000,-2116037,15254,-3991,1707,2161,3258*ab408b44"
        line = str(line)
        list = line.split(',')

        """gps数据"""
        if str(list[0]) == "$GPGGA":
            if list[1] != "":
                self.gps_msg.latitude = float(list[2])/100
                self.gps_msg.longitude = float(list[4])/100
                self.gps_msg.altitude = float(list[9])

                self.gps_flag = 1

            else:
                self.gps_flag = 0

        elif str(list[0]) == "b'#RAWIMUA":
            """imu校正数据"""
            header, list = line.split(';')
            list = list.split(',')

            if list[1] != "":
                PitchRate = float((list[8].split('*'))[0])
                RollRate = -float(list[7])
                YawRate = float(list[6])
                LateralAcc = float(list[5])
                LongitudinalAcc = -float(list[4])
                VerticalAcc = float(list[3])

                
                PitchRate *= 3.0517578125e-05
                RollRate *= 3.0517578125e-05
                YawRate *= 3.0517578125e-05
                LateralAcc *= 3.74094e-06
                LongitudinalAcc *= 3.74094e-06
                VerticalAcc *= 3.74094e-06

                self.imu_msg.angular_velocity.x = PitchRate
                self.imu_msg.angular_velocity.y = RollRate
                self.imu_msg.angular_velocity.z = YawRate

                self.imu_msg.linear_acceleration.x = LateralAcc
                self.imu_msg.linear_acceleration.y = LongitudinalAcc
                self.imu_msg.linear_acceleration.z = VerticalAcc

                self.imu_flag = 1

            else:
                self.imu_flag = 0

        """发布数据"""
        if(self.imu_flag == 1):
            self.imu_pub.publish(self.imu_msg)
            self.get_logger().info("发布imu消息")
        elif(self.imu_flag == 0):
            self.imu_pub.publish(self.imu_msg)
            self.get_logger().info("imu无信号")

            
    def timer_callback2(self):
        """读取解析数据"""

        line = self.ser.readline()
        line = str(line)

        """gps数据"""
        if str(list[0]) == "$GPGGA":
            if list[1] != "":
                self.gps_msg.latitude = float(list[2])/100
                self.gps_msg.longitude = float(list[4])/100
                self.gps_msg.altitude = float(list[9])

                self.gps_flag = 1

            else:
                self.gps_flag = 0
                self.gps_pub.publish(self.gps_msg)
                self.get_logger().info("gps无信号")

        
        elif str(list[0]) == "#CORRIMUDATAA":
            """imu校正数据"""
            header, list = line.split(';')
            list = list.split(',')

            if list[1] != "":
                PitchRate = float(list[2])
                RollRate = float(list[3])
                YawRate = float(list[4])
                LateralAcc = float(list[5])
                LongitudinalAcc = float(list[6])
                VerticalAcc = float(list[7])

                self.imu_msg.angular_velocity.x = PitchRate
                self.imu_msg.angular_velocity.y = RollRate
                self.imu_msg.angular_velocity.z = YawRate

                self.imu_msg.linear_acceleration.x = LateralAcc
                self.imu_msg.linear_acceleration.y = LongitudinalAcc
                self.imu_msg.linear_acceleration.z = VerticalAcc

                if(self.imu_flag == -1): self.imu_flag = 1
                else: self.imu_flag = -1

            else:
                self.imu_flag = 0

        elif str(list[0]) == "#INSATTA":
            """imu欧拉角"""
            header, list = line.split(';')
            list = list.split(',')

            if list[1] != "":
                """欧拉角转四元数"""
                Roll = float(list[2])
                Pitch = float(list[3])
                Azimuth = float(list[4])

                x, y, z, w = self.Eular2Quat([Roll, Pitch, Azimuth])

                self.imu_msg.orientation.x = x
                self.imu_msg.orientation.y = y
                self.imu_msg.orientation.z = z
                self.imu_msg.orientation.w = w

                if(self.imu_flag == -1): self.imu_flag = 1
                else: self.imu_flag = -1

            else:
                self.imu_flag = 0


        """发布数据"""
        if(self.gps_flag == 1):
            self.gps_pub.publish(self.gps_msg)
            self.get_logger().info("发布gps消息")
        elif(self.gps_flag == 0):
            self.gps_pub.publish(self.gps_msg)
            self.get_logger().info("gps无信号")

        if(self.imu_flag == 1):
            self.imu_pub.publish(self.imu_msg)
            self.get_logger().info("发布imu消息")
        elif(self.imu_flag == 0):
            self.imu_pub.publish(self.imu_msg)
            self.get_logger().info("imu无信号")


def main(args = None):
    """
    bynav节点主程序
    """

    """配置节点"""
    rclpy.init(args = args)
    node = Node_bynav("bynav")

    """保持节点"""
    rclpy.spin(node)
    rclpy.shutdown()
