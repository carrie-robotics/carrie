#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge
from transformers import Sam3Model, Sam3Processor
import cv2
import torch
import numpy as np
import os

from carrie_perception import helper_functions
from carrie_interfaces.srv import DetectObjects

def get_device() -> torch.device:
    if torch.cuda.is_available():
        return torch.device('cuda')
    elif torch.backends.mps.is_available():
        return torch.device('mps')
    else:
        return torch.device('cpu')

def build_bounding_boxes(results : dict) -> tuple[list[BoundingBox2D], list[float]]:
    # build flat boxes + scores lists
    boxes = []
    scores = []
    for i, score in enumerate(results["scores"]):
        box_data = results["boxes"][i]

        bbox = BoundingBox2D()
        bbox.center.position.x = float((box_data[0] + box_data[2]) / 2.0)
        bbox.center.position.y = float((box_data[1] + box_data[3]) / 2.0)
        bbox.center.theta = 0.0
        bbox.size_x = float(box_data[2] - box_data[0])
        bbox.size_y = float(box_data[3] - box_data[1])

        boxes.append(bbox)
        scores.append(float(score))

    return boxes, scores

def combined_mask(masks: np.ndarray, h: int, w: int) -> np.ndarray:
    if masks.shape[0] > 0:
        combined_mask = np.max(masks, axis=0)
    else:
        combined_mask = np.zeros((h, w), dtype=np.uint8)
    
    return combined_mask

class Sam3Detector(Node):
    def __init__(self):
        super().__init__('sam3_detector')

        # parameters
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
        
        # load model
        self.model = Sam3Model.from_pretrained("facebook/sam3").to(self.device)
        self.processor = Sam3Processor.from_pretrained("facebook/sam3")

        self.bridge = CvBridge()

        self.srv = self.create_service(DetectObjects, 'detect_objects', self.detect_callback)

        self.output_saved = False
        os.makedirs("carrie_perception/sam3_output", exist_ok=True)

        self.get_logger().info('SAM3 detector service ready')

    def detect_callback(self, request: DetectObjects.Request, response: DetectObjects.Response):
        prompt = request.prompt
        if not request.prompt:
            response.success = False
            response.message = "No prompt provided"
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')

            # preprocess
            inputs = self.processor(images=cv_image, text=prompt, return_tensors='pt').to(self.device)

            # inference
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

            masks = results["masks"].cpu().numpy().astype(np.uint8) * 255
            combined_mask_img = combined_mask(masks, h, w)

            mask_msg = self.bridge.cv2_to_imgmsg(combined_mask_img, encoding='mono8')
            mask_msg.header = request.image.header

            # for debugging and demo purposes (should be deleted later)
            if not self.output_saved:
                sam_result_for_helper = {
                    "masks": results["masks"].cpu(),
                    "scores": results["scores"].cpu(),
                    "boxes": results["boxes"].cpu() if "boxes" in results else None,
                }

                detections = helper_functions.from_sam(sam_result=sam_result_for_helper)
                detections = detections[detections.confidence > self.confidence_threshold]

                if len(detections) > 0:
                    annotated = helper_functions.annotate(cv_image, detections, label=prompt)
                    output_path = os.path.join("carrie_perception/sam3_output", "annotated_output.jpg")
                    cv2.imwrite(output_path, cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
                    self.get_logger().info(f"Saved annotated debug image to {output_path}")
                    self.output_saved = True

            # populate response
            response.header = request.image.header
            response.prompt = prompt
            response.boxes = boxes
            response.scores = scores
            response.mask = mask_msg
            response.success = True
            response.message = f"Detected {len(boxes)} object(s)"
            self.get_logger().info(response.message)

        except Exception as e:
            self.get_logger().error(f"Detection failed: {e}")
            response.success = False
            response.message = str(e)

        return response


def main():
    rclpy.init()
    node = Sam3Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
