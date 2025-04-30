import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.bridge = CvBridge()
        self.img = None
        self.colorsrc = 'rgb'

        self.sub = self.create_subscription(
            Image,
            '/your/image/topic/image_raw',
            self.cb_image,
            10
        )

        self.pointcloud_pub = self.create_publisher(PointCloud2, '/your/image/as/a/pointcloud', 10)
        self.get_logger().info("Image Viewer Node started")
        self.create_timer(0.01, self.image_to_pointcloud)

    def cb_image(self, msg):
        self.img = msg

    def image_to_pointcloud(self):
        try:
            loaded_image = self.bridge.imgmsg_to_cv2(self.img, desired_encoding='bgr8')
            if loaded_image is None:
                self.get_logger().error("Failed to load image")
                return
        except Exception as e:
            self.get_logger().error(f"Error loading image: {e}")
            return

        resized_image = cv2.resize(loaded_image, (0, 0), fx=0.1, fy=0.1)

        self.publish_pointcloud(resized_image)

    def publish_pointcloud(self, image):
        height, width, _ = image.shape
        scale_factor = 0.02

        pointcloud = PointCloud2()
        pointcloud.header.frame_id = 'map'
        pointcloud.header.stamp = self.get_clock().now().to_msg()

        pointcloud.width = width
        pointcloud.height = height

        pointcloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16
        pointcloud.row_step = pointcloud.point_step * width
        pointcloud.is_dense = True

        qx = -0.5
        qy = 0.5
        qz = -0.5
        qw = 0.5
        rotation_matrix = np.array([
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)]
        ])

        points = []
        for i in range(height):
            for j in range(width):
                x = j * scale_factor - (width / 2) * scale_factor
                y = i * scale_factor - (height / 2) * scale_factor
                z = height * scale_factor 
                rotated_point = np.dot(rotation_matrix, np.array([x, y, z]))
                x, y, z = rotated_point
                b, g, r = image[i, j]
                rgb = (r << 16) | (g << 8) | b
                points.append(struct.pack('fffI', x, y, z, rgb))

        pointcloud.data = b''.join(points)
        self.pointcloud_pub.publish(pointcloud)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()