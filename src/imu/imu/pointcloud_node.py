import math
import struct
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('pc_subscriber')
        self.subscription = self.create_subscription(PointCloud2,'/rslidar_points',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self,data):
        """读取解析数据"""
        assert isinstance(data, PointCloud2)
        pcd_as_numpy_array = np.array(list(self.read_points(data)))
        self.pcn = self.label(pcd_as_numpy_array)
        self.get_logger().info('节点已创建')
    

    def read_points(self, cloud):
        """读取点云数据"""
        assert isinstance(cloud, PointCloud2)
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        unpack_from = struct.Struct(fmt).unpack_from

        for v in range(height):
            offset = row_step * v
            for u in range(width):
                yield unpack_from(data, offset)
                offset += point_step

    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        """获取数据格式"""
        fmt = '>' if is_bigendian else '<'

        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset)):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            else:
                datatype_fmt = 'f'
                datatype_length = 4
                fmt += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt


def main(args=None):
    rclpy.init(args=args)
    PC_subscriber = PointCloudSubscriber()

    rclpy.spin(PC_subscriber)
    PC_subscriber.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()