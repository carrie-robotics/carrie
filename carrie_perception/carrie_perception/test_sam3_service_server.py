import unittest
from unittest.mock import patch, MagicMock
import numpy as np
import torch
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

from carrie_interfaces.srv import DetectObjects
from carrie_perception.sam3_service_server import xyxy_to_cwh, masks_to_images, get_device, build_bounding_boxes, build_mask_messages

class TestXyxyToCwh(unittest.TestCase):

    def test_basic_conversion(self):
        cx, cy, w, h = xyxy_to_cwh([10., 20., 50., 60.])
        self.assertEqual(cx, 30.0)
        self.assertEqual(cy, 40.0)
        self.assertEqual(w,  40.0)
        self.assertEqual(h,  40.0)

    def test_returns_floats(self):
        cx, cy, w, h = xyxy_to_cwh([0, 0, 10, 10])
        self.assertIsInstance(cx, float)
        self.assertIsInstance(w,  float)

    def test_zero_size_box(self):
        cx, cy, w, h = xyxy_to_cwh([5., 5., 5., 5.])
        self.assertEqual(cx, 5.0)
        self.assertEqual(w,  0.0)


class TestMasksToImages(unittest.TestCase):

    def test_correct_number_of_masks(self):
        masks = torch.ones((3, 480, 640), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(len(result), 3)

    def test_output_shape(self):
        masks = torch.ones((2, 480, 640), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(result[0].shape, (480, 640))

    def test_values_scaled_to_255(self):
        masks = torch.ones((1, 100, 100), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(result[0].max(), 255)

    def test_zero_mask_stays_zero(self):
        masks = torch.zeros((1, 100, 100), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(result[0].max(), 0)

    def test_empty_masks(self):
        masks = torch.zeros((0, 480, 640), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(len(result), 0)


class TestGetDevice(unittest.TestCase):

    def test_device_is_valid(self):
        device = get_device()
        self.assertIn(device.type, ['cuda', 'mps', 'cpu'])


class TestBuildBoundingBoxes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # rclpy may already be initialised if tests run in the same process.
        # This guard prevents a double init error.
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def _make_results(self, boxes, scores):
        return {
            "boxes":  torch.tensor(boxes,  dtype=torch.float32),
            "scores": torch.tensor(scores, dtype=torch.float32),
        }

    def test_correct_number_of_boxes(self):
        results = self._make_results([[10., 20., 50., 60.], [0., 0., 10., 10.]], [0.9, 0.8])
        boxes, scores = build_bounding_boxes(results)
        self.assertEqual(len(boxes), 2)
        self.assertEqual(len(scores), 2)

    def test_scores_preserved(self):
        results = self._make_results([[0., 0., 10., 10.]], [0.75])
        _, scores = build_bounding_boxes(results)
        self.assertAlmostEqual(scores[0], 0.75, places=4)

    def test_empty_results(self):
        results = self._make_results([], [])
        boxes, scores = build_bounding_boxes(results)
        self.assertEqual(len(boxes), 0)
        self.assertEqual(len(scores), 0)

    def test_theta_is_zero(self):
        """BoundingBox2D has a rotation field. We always set it to 0."""
        results = self._make_results([[0., 0., 10., 10.]], [0.9])
        boxes, _ = build_bounding_boxes(results)
        self.assertEqual(boxes[0].center.theta, 0.0)


class TestBuildMaskMessages(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_correct_number_of_messages(self):
        masks = torch.ones((3, 480, 640), dtype=torch.int64)
        msgs = build_mask_messages(masks, Header(), CvBridge())
        self.assertEqual(len(msgs), 3)

    def test_encoding_is_mono8(self):
        masks = torch.ones((1, 100, 100), dtype=torch.int64)
        msgs = build_mask_messages(masks, Header(), CvBridge())
        self.assertEqual(msgs[0].encoding, 'mono8')

    def test_header_is_propagated(self):
        header = Header()
        header.frame_id = 'camera'
        masks = torch.ones((2, 100, 100), dtype=torch.int64)
        msgs = build_mask_messages(masks, header, CvBridge())
        self.assertEqual(msgs[0].header.frame_id, 'camera')
        self.assertEqual(msgs[1].header.frame_id, 'camera')

    def test_empty_masks(self):
        masks = torch.zeros((0, 480, 640), dtype=torch.int64)
        msgs = build_mask_messages(masks, Header(), CvBridge())
        self.assertEqual(len(msgs), 0)


# monkey patches for Sam3Model and Sam3Processor
class TestDetectCallback(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def _make_node(self, MockModel, MockProcessor):
        MockModel.from_pretrained.return_value.to.return_value = MagicMock()
        mock_processor = MagicMock()
        MockProcessor.from_pretrained.return_value = mock_processor
        mock_processor.return_value.to.return_value = MagicMock()
        
        mock_processor.post_process_instance_segmentation.return_value = [{
            "scores": torch.tensor([0.9, 0.8]),
            "boxes":  torch.tensor([[10., 20., 50., 60.],
                                    [100., 200., 150., 250.]]),
            "masks":  torch.ones((2, 480, 640), dtype=torch.int64),
        }]

        from carrie_perception.sam3_service_server import Sam3Detector
        return Sam3Detector()

    def _make_request(self, prompt='person'):
        request = DetectObjects.Request()
        request.image = Image()
        request.image.header = Header()
        request.image.height = 480
        request.image.width  = 640
        request.image.encoding = 'rgb8'
        request.image.step = 640 * 3
        request.image.data = bytes(np.zeros((480, 640, 3), dtype=np.uint8))
        request.prompt = prompt
        return request

    @patch('carrie_perception.sam3_service_server.Sam3Processor')
    @patch('carrie_perception.sam3_service_server.Sam3Model')
    def test_success(self, MockModel, MockProcessor):
        node = self._make_node(MockModel, MockProcessor)
        response = node.detect_callback(self._make_request(), DetectObjects.Response())

        self.assertTrue(response.success)
        self.assertEqual(response.error_message, "")
        self.assertEqual(len(response.boxes), 2)
        self.assertEqual(len(response.scores), 2)
        self.assertEqual(len(response.masks), 2)

        node.destroy_node()

    @patch('carrie_perception.sam3_service_server.Sam3Processor')
    @patch('carrie_perception.sam3_service_server.Sam3Model')
    def test_empty_prompt_returns_failure(self, MockModel, MockProcessor):
        """Callback should reject an empty prompt before touching the model."""
        MockModel.from_pretrained.return_value.to.return_value = MagicMock()
        MockProcessor.from_pretrained.return_value = MagicMock()

        from carrie_perception.sam3_service_server import Sam3Detector
        node = Sam3Detector()

        response = node.detect_callback(self._make_request(prompt=''), DetectObjects.Response())

        self.assertFalse(response.success)
        self.assertEqual(response.error_message, "No prompt provided")

        node.destroy_node()

    @patch('carrie_perception.sam3_service_server.Sam3Processor')
    @patch('carrie_perception.sam3_service_server.Sam3Model')
    def test_model_exception_returns_failure(self, MockModel, MockProcessor):
        """If the model raises, success should be False and error_message populated."""
        mock_model = MagicMock()
        mock_model.side_effect = RuntimeError("GPU out of memory")
        MockModel.from_pretrained.return_value.to.return_value = mock_model

        mock_processor = MagicMock()
        MockProcessor.from_pretrained.return_value = mock_processor
        mock_processor.return_value.to.return_value = MagicMock()

        from carrie_perception.sam3_service_server import Sam3Detector
        node = Sam3Detector()

        response = node.detect_callback(self._make_request(), DetectObjects.Response())

        self.assertFalse(response.success)
        self.assertIn("GPU out of memory", response.error_message)

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()