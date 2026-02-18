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
from carrie_interfaces.srv import DetectObjects


class Sam3Detector(Node):
    def __init__(self):
        super().__init__('sam3_detector')

        # parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('threshold', 0.5),
                ('mask_threshold', 0.5),
            ])

        self.threshold = self.get_parameter('threshold').value
        self.mask_threshold = self.get_parameter('mask_threshold').value

        # device
        self.device = torch.device('mps' if torch.backends.mps.is_available() else 'cpu')
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
        prompt = request.prompt if request.prompt else 'person'

        try:
            # convert ROS image -> PIL
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='rgb8')
            pil_image = PILImage.fromarray(cv_image)

            # preprocess
            inputs = self.processor(
                images=pil_image,
                text=prompt,
                return_tensors='pt'
            ).to(self.device)

            # inference
            with torch.no_grad():
                outputs = self.model(**inputs)

            results = self.processor.post_process_instance_segmentation(
                outputs,
                threshold=self.threshold,
                mask_threshold=self.mask_threshold,
                target_sizes=[pil_image.size[::-1]]
            )[0]

            # build Detection2DArray
            detections_msg = Detection2DArray()
            detections_msg.header = request.image.header

            for i, score in enumerate(results["scores"]):
                det = Detection2D()
                det.header = request.image.header

                box = results["boxes"][i]

                cx = float((box[0] + box[2]) / 2.0)
                cy = float((box[1] + box[3]) / 2.0)
                w  = float(box[2] - box[0])
                h  = float(box[3] - box[1])

                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.center.theta = 0.0
                det.bbox.size_x = w
                det.bbox.size_y = h

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = prompt
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)

                detections_msg.detections.append(det)

            # build combined mask
            masks = results["masks"].cpu().numpy().astype(np.uint8) * 255

            if masks.shape[0] > 0:
                combined_mask = np.max(masks, axis=0)
            else:
                combined_mask = np.zeros(
                    (pil_image.size[1], pil_image.size[0]), dtype=np.uint8
                )

            mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, encoding='mono8')
            mask_msg.header = request.image.header

            # for debugging and demo purposes (should be deleted later)
            if not self.output_saved:
                sam_result_for_helper = {
                    "masks": results["masks"].cpu(),
                    "scores": results["scores"].cpu(),
                    "boxes": results["boxes"].cpu() if "boxes" in results else None,
                }

                detections = helper_functions.from_sam(sam_result=sam_result_for_helper)
                detections = detections[detections.confidence > self.threshold]

                if len(detections) > 0:
                    annotated = helper_functions.annotate(pil_image, detections, label=prompt)
                    output_path = os.path.join("carrie_perception/sam3_output", "annotated_output.jpg")
                    annotated.save(output_path)
                    self.get_logger().info(f"Saved annotated debug image to {output_path}")
                    self.output_saved = True

            response.detections = detections_msg
            response.mask = mask_msg
            response.success = True
            response.message = f"Detected {len(detections_msg.detections)} object(s)"
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
