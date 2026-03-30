import unittest
import numpy as np
import torch

from carrie_perception.sam3_service_server import xyxy_to_cwh, masks_to_images


class TestXyxyToCwh(unittest.TestCase):

    def test_basic_conversion(self):
        cx, cy, w, h = xyxy_to_cwh([10., 20., 50., 60.])
        self.assertEqual(cx, 30.0)
        self.assertEqual(cy, 40.0)
        self.assertEqual(w,  40.0)
        self.assertEqual(h,  40.0)

    def test_non_square_box(self):
        """A wide, short box — width and height should differ."""
        cx, cy, w, h = xyxy_to_cwh([100., 200., 150., 250.])
        self.assertEqual(cx, 125.0)
        self.assertEqual(cy, 225.0)
        self.assertEqual(w,   50.0)
        self.assertEqual(h,   50.0)

    def test_returns_floats(self):
        """Even if we pass integers, output should be floats."""
        cx, cy, w, h = xyxy_to_cwh([0, 0, 10, 10])
        self.assertIsInstance(cx, float)
        self.assertIsInstance(w,  float)

    def test_zero_size_box(self):
        """A box where both corners are the same point."""
        cx, cy, w, h = xyxy_to_cwh([5., 5., 5., 5.])
        self.assertEqual(cx, 5.0)
        self.assertEqual(w,  0.0)


class TestMasksToImages(unittest.TestCase):
    """Tests for the masks_to_images tensor-to-numpy conversion function."""

    def test_correct_number_of_masks(self):
        """If the model returns 3 masks, we should get 3 images back."""
        masks = torch.ones((3, 480, 640), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(len(result), 3)

    def test_output_shape(self):
        """Each output image should match the H x W of the input mask."""
        masks = torch.ones((2, 480, 640), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(result[0].shape, (480, 640))

    def test_values_scaled_to_255(self):
        """
        The model outputs 1s where an object is detected.
        We scale by *255 so that a '1' mask pixel becomes 255 (white)
        in the output image. This is standard for binary masks.
        """
        masks = torch.ones((1, 100, 100), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(result[0].max(), 255)

    def test_zero_mask_stays_zero(self):
        """A mask of all zeros should produce an all-black image."""
        masks = torch.zeros((1, 100, 100), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(result[0].max(), 0)

    def test_empty_masks(self):
        """Zero detections — should return an empty list, not crash."""
        masks = torch.zeros((0, 480, 640), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(len(result), 0)

    def test_output_dtype_is_uint8(self):
        """The output arrays must be uint8 (8-bit image, values 0-255)."""
        masks = torch.ones((1, 10, 10), dtype=torch.int64)
        result = masks_to_images(masks)
        self.assertEqual(result[0].dtype, np.uint8)


if __name__ == '__main__':
    unittest.main()