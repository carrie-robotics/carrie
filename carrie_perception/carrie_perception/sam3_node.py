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
import os

from carrie_perception import helper_functions


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

        self.output_saved = False
        os.makedirs("carrie_perception/sam3_output", exist_ok=True)

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

            cx = float((box[0] + box[2]) / 2.0)
            cy = float((box[1] + box[3]) / 2.0)
            w = float(box[2] - box[0])
            h = float(box[3] - box[1])

            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.center.theta = 0.0

            det.bbox.size_x = w
            det.bbox.size_y = h

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.prompt
            hyp.hypothesis.score = float(score)

            det.results.append(hyp)
            detections_msg.detections.append(det)

        self.detection_pub.publish(detections_msg)

        masks = results["masks"].cpu().numpy().astype(np.uint8) * 255

        if masks.shape[0] > 0:
            combined_mask = np.max(masks, axis=0)
        else:
            combined_mask = np.zeros((pil_image.size[1], pil_image.size[0]), dtype=np.uint8)

        mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        # for dubugging and demo purpose (should be deleted later)
        if not self.output_saved:
            sam_result_for_helper = {
                "masks": results["masks"].cpu(),
                "scores": results["scores"].cpu(),
                "boxes": results["boxes"].cpu() if "boxes" in results else None
            }

            detections = helper_functions.from_sam(sam_result=sam_result_for_helper)
            detections = detections[detections.confidence > self.threshold]
            
            if len(detections) > 0:
                annotated = helper_functions.annotate(pil_image, detections, label=self.prompt)
                output_path = os.path.join("carrie_perception/sam3_output", "annotated_output.jpg")
                annotated.save(output_path)
                self.get_logger().info(f"Saved annotated debug image to {output_path}")
                self.output_saved = True

def main():
    rclpy.init()
    node = Sam3Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
