#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from transformers import Sam3Model, Sam3Processor
from PIL import Image as PILImage
import torch
import numpy as np


class Sam3Detector(Node):
    def __init__(self):
        super().__init__('sam3_detector')

        # parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('prompt', "person"),
                ('threshold', 0.5),
                ('mask_threshold', 0.5),
            ])

        self.prompt = self.get_parameter('prompt').value
        self.threshold = self.get_parameter('threshold').value
        self.mask_threshold = self.get_parameter('mask_threshold').value

        # device
        self.device = torch.device('mps' if torch.backends.mps.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')

        # load model
        self.model = Sam3Model.from_pretrained("facebook/sam3").to(self.device)
        self.processor = Sam3Processor.from_pretrained("facebook/sam3")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.mask_pub = self.create_publisher(Image, '/perception/masks', 10)

        self.get_logger().info('SAM3 detector node ready')

    def image_callback(self, msg: Image):
        # convert ROS image -> PIL
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        pil_image = PILImage.fromarray(cv_image)

        # preprocess
        inputs = self.processor(images=pil_image, text=self.prompt, return_tensors='pt').to(self.device)

        # inference
        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_instance_segmentation(
            outputs,
            threshold=self.threshold,
            mask_threshold=self.mask_threshold,
            target_sizes=[pil_image.size[::-1]]
        )[0]

        # publish detections
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for i, score in enumerate(results["scores"]):
            det = Detection2D()
            det.header = msg.header

            box = results["boxes"][i]
            det.bbox.center.x = float((box[0] + box[2]) / 2)
            det.bbox.center.y = float((box[1] + box[3]) / 2)
            det.bbox.size_x = float(box[2] - box[0])
            det.bbox.size_y = float(box[3] - box[1])

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.prompt
            hyp.hypothesis.score = float(score)

            det.results.append(hyp)
            detections_msg.detections.append(det)

        self.detection_pub.publish(detections_msg)

        # publish masks (stacked)
        masks = results["masks"].cpu().numpy().astype(np.uint8) * 255
        combined_mask = np.max(masks, axis=0)

        mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)


def main():
    rclpy.init()
    node = Sam3Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
