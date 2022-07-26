import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class Node_PC(Node):
    """
    创建point_cloud节点
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("point_cloud节点已创建")

        """创建并初始化接收"""
        self.sub_point_cloud = self.create_subscription(PointCloud2,"/rslidar_points",self.callback,10)

    def callback(self, data):
        """读取解析数据"""
        assert isinstance(data, PointCloud2)
        for i in range(len(data.data)):
            if(data.data[i]>255):
                print("\r%s" % (data.data[i]), end="")


def main(args = None):
    """
    point_cloud节点主程序
    """

    """配置节点"""
    rclpy.init(args = args)
    node = Node_PC("point_cloud")

    """保持节点"""
    rclpy.spin(node)
    rclpy.shutdown()
