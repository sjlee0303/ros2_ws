import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import struct

from interfaces_pkg.msg import ConeInfo, ConeInfoArray

class ConeinfoNode(Node):
    def __init__(self):
        super().__init__('cone_info_node')
        self.sub = self.create_subscription(
            PointCloud2, '/cones/colored_centroids', self.cb, 10)
        self.pub = self.create_publisher(ConeInfoArray, '/cones/cone_info', 10)

    def cb(self, msg: PointCloud2):
        cloud = self.pc2_to_array(msg)
        cone_list = []
        for pt in cloud:
            x, y, z = pt[:3]
            rgb_f = pt[3]
            r, g, b = self.unpack_rgb_float(rgb_f)
            dist = float(np.linalg.norm([x, y, z]))

            cone = ConeInfo()
            cone.x = float(x)
            cone.y = float(y)
            cone.z = float(z)
            cone.distance = dist
            cone.cone_color = self.rgb_to_color_name(r, g, b)
            cone_list.append(cone)

        # 거리 기준 오름차순 정렬
        cone_list.sort(key=lambda cone: cone.distance)

        msg_out = ConeInfoArray()
        msg_out.cones = cone_list
        self.pub.publish(msg_out)

    def pc2_to_array(self, msg: PointCloud2) -> np.ndarray:
        # PointCloud2의 필드 형식이 float32 [x, y, z, rgb]라고 가정
        return np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)

    def unpack_rgb_float(self, rgb_float: float):
        rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]
        r = (rgb_int >> 16) & 0xFF
        g = (rgb_int >> 8) & 0xFF
        b = rgb_int & 0xFF
        return r, g, b

    def rgb_to_color_name(self, r, g, b):
        if b > r and b > g:
            return "blue"
        else:
            return "yellow"

def main(args=None):
    rclpy.init(args=args)
    node = ConeinfoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
