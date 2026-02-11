#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PIL import Image as PILImage
import numpy as np
import os


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_path', 'carrie_perception/example_images/solvay_conference_1927.jpg'),
                ('topic', '/camera/image_raw'),
            ])

        self.image_path = self.get_parameter('image_path').value
        self.topic = self.get_parameter('topic').value

        self.publisher = self.create_publisher(Image, self.topic, 10)
        self.bridge = CvBridge()

        # Load image
        if not os.path.exists(self.image_path):
            self.get_logger().error(f'Image not found: {self.image_path}')
            rclpy.shutdown()
            return

        pil_image = PILImage.open(self.image_path).convert('RGB')
        cv_image = np.array(pil_image)

        self.msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
        self.msg.header.frame_id = 'camera'

        self.timer = self.create_timer(5, self.timer_callback)
        
        self.get_logger().info(f'Publishing image continuously on {self.topic} ')

    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main():
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
