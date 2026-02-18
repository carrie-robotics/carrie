#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PIL import Image as PILImage
import numpy as np
import os

from carrie_interfaces.srv import DetectObjects


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_path', 'carrie_perception/example_images/solvay_conference_1927.jpg'),
                ('prompt', 'person'),
            ])

        self.image_path = self.get_parameter('image_path').value
        self.prompt = self.get_parameter('prompt').value

        self.bridge = CvBridge()
        self.client = self.create_client(DetectObjects, 'detect_objects')

        # load image
        if not os.path.exists(self.image_path):
            self.get_logger().error(f'Image not found: {self.image_path}')
            rclpy.shutdown()
            return

        pil_image = PILImage.open(self.image_path).convert('RGB')
        cv_image = np.array(pil_image)

        self.image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
        self.image_msg.header.frame_id = 'camera'

        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info(f'Image client ready, calling detect_objects every 5s')

    def timer_callback(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('detect_objects service not available, waiting...')
            return

        self.image_msg.header.stamp = self.get_clock().now().to_msg()

        request = DetectObjects.Request()
        request.image = self.image_msg
        request.prompt = self.prompt

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service response: {response.message}')
            else:
                self.get_logger().error(f'Service failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call raised exception: {e}')


def main():
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
