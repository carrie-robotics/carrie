#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PIL import Image as PILImage
import numpy as np
import os


class ImageOncePublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_path', 'carrie_perception/example_images/solvay_conference_1927.jpg'),
                ('topic', '/camera/image_raw'),
            ]
        )

        image_path = self.get_parameter('image_path').value
        topic = self.get_parameter('topic').value

        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        # Load image
        if not os.path.exists(image_path):
            self.get_logger().error(f'Image not found: {image_path}')
            rclpy.shutdown()
            return

        pil_image = PILImage.open(image_path).convert('RGB')
        cv_image = np.array(pil_image)

        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
        msg.header.frame_id = 'camera'

        # Publish once
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published image once on {topic}: {image_path}'
        )


def main():
    rclpy.init()
    node = ImageOncePublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
