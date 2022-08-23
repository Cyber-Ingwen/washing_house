import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu_filter', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self,x_angle,y_angle,z_angle,x_linear,y_linear,z_linear,q0,q1,q2,q3):
        self.imu_msg = Imu()
        self.imu_msg.angular_velocity.x = x_angle
        self.imu_msg.angular_velocity.y = y_angle
        self.imu_msg.angular_velocity.z = z_angle
        self.imu_msg.linear_acceleration.x = x_linear
        self.imu_msg.linear_acceleration.y = y_linear
        self.imu_msg.linear_acceleration.z = z_linear
        self.imu_msg.orientation.x = q1
        self.imu_msg.orientation.y = q2
        self.imu_msg.orientation.z = q3
        self.imu_msg.orientation.w = q0
        self.imu_msg.header.frame_id = "imu-link"
        self.publisher_.publish(self.imu_msg)
        self.get_logger().info('imu filter send')
        


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
