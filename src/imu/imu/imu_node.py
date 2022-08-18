import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from std_msgs.msg import String


class IMUSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(Imu,'/imu',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, kk):
        data=kk
        #print(data.angular_velocity.x)
        print(data.linear_acceleration.y)
        self.get_logger().info('节点已创建')

def main(args=None):
    rclpy.init(args=args)
    IMU_subscriber = IMUSubscriber()

    rclpy.spin(IMU_subscriber)
    IMU_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()