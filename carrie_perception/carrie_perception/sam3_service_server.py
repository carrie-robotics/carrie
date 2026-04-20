#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge
from transformers import Sam3Model, Sam3Processor
import torch
import numpy as np

from carrie_interfaces.srv import DetectObjects


def get_device() -> torch.device:
    if torch.cuda.is_available():
        return torch.device('cuda')
    elif torch.backends.mps.is_available():
        return torch.device('mps')
    else:
        return torch.device('cpu')


def xyxy_to_cwh(box) -> tuple[float, float, float, float]:
    cx = float((box[0] + box[2]) / 2.0)
    cy = float((box[1] + box[3]) / 2.0)
    w  = float(box[2] - box[0])
    h  = float(box[3] - box[1])
    return cx, cy, w, h


def masks_to_images(masks: torch.Tensor) -> list[np.ndarray]:
    return [mask for mask in masks.cpu().numpy().astype(np.uint8) * 255]


def build_bounding_boxes(results: dict) -> tuple[list[BoundingBox2D], list[float]]:
    boxes = []
    scores = []
    for i, score in enumerate(results["scores"]):
        cx, cy, w, h = xyxy_to_cwh(results["boxes"][i])

        bbox = BoundingBox2D()
        bbox.center.position.x = cx
        bbox.center.position.y = cy
        bbox.center.theta = 0.0
        bbox.size_x = w
        bbox.size_y = h

        boxes.append(bbox)
        scores.append(float(score))

    return boxes, scores


def build_mask_messages(masks: torch.Tensor, header, bridge: CvBridge) -> list:
    mask_msgs = []
    for mask_img in masks_to_images(masks):
        mask_msg = bridge.cv2_to_imgmsg(mask_img, encoding='mono8')
        mask_msg.header = header
        mask_msgs.append(mask_msg)
    return mask_msgs


class Sam3Detector(Node):
    def __init__(self):
        super().__init__('sam3_detector')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('confidence_threshold', 0.5),
                ('mask_threshold', 0.5),
            ])

        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.mask_threshold = self.get_parameter('mask_threshold').value

        self.device = get_device()
        self.get_logger().info(f'Using device: {self.device}')

        self.model = Sam3Model.from_pretrained("facebook/sam3").to(self.device)
        self.processor = Sam3Processor.from_pretrained("facebook/sam3")

        self.bridge = CvBridge()

        self.srv = self.create_service(DetectObjects, 'detect_objects', self.detect_callback)
        self.get_logger().info('SAM3 detector service ready')

    def detect_callback(self, request: DetectObjects.Request, response: DetectObjects.Response):
        prompt = request.prompt
        if not prompt:
            response.success = False
            response.error_message = "No prompt provided"
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')

            inputs = self.processor(images=cv_image, text=prompt, return_tensors='pt').to(self.device)

            with torch.no_grad():
                outputs = self.model(**inputs)

            h, w = cv_image.shape[:2]
            results = self.processor.post_process_instance_segmentation(
                outputs,
                threshold=self.confidence_threshold,
                mask_threshold=self.mask_threshold,
                target_sizes=[(h, w)]
            )[0]

            boxes, scores = build_bounding_boxes(results)
            mask_msgs = build_mask_messages(results["masks"], request.image.header, self.bridge)

            response.header = request.image.header
            response.prompt = prompt
            response.boxes = boxes
            response.scores = scores
            response.masks = mask_msgs
            response.success = True
            response.error_message = ""
            self.get_logger().info(f"Detected {len(boxes)} object(s)")

        except Exception as e:
            self.get_logger().error(f"Detection failed: {e}")
            response.success = False
            response.error_message = str(e)

        return response


def main():
    rclpy.init()
    node = Sam3Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()