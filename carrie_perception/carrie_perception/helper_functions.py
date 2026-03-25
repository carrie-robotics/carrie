import supervision as sv
from PIL import Image
from typing import Optional
import torch
import numpy as np

def from_sam(sam_result: dict) -> sv.Detections:
    """
    Convert Hugging Face SAM3 output to supervision Detections.
    sam_result comes from processor.post_process_instance_segmentation()
    """
    # Masks from Hugging Face have shape: (num_masks, height, width)
    # Not (num_masks, 1, height, width) like the original SAM3
    masks = sam_result["masks"]
    
    # Convert to numpy and ensure correct shape
    if isinstance(masks, torch.Tensor):
        mask_np = masks.cpu().numpy()
    else:
        mask_np = masks
    
    # If masks have shape (N, 1, H, W), squeeze to (N, H, W)
    if len(mask_np.shape) == 4 and mask_np.shape[1] == 1:
        mask_np = mask_np.squeeze(1)  # Remove the channel dimension
    
    # Scores
    scores = sam_result["scores"]
    if isinstance(scores, torch.Tensor):
        scores_np = scores.cpu().numpy()
    else:
        scores_np = scores
    
    # Calculate bounding boxes from masks
    xyxy = []
    for i in range(mask_np.shape[0]):
        mask = mask_np[i]
        # Find where mask is True/1
        y_indices, x_indices = np.where(mask > 0)
        
        if len(x_indices) > 0 and len(y_indices) > 0:
            x1, x2 = np.min(x_indices), np.max(x_indices)
            y1, y2 = np.min(y_indices), np.max(y_indices)
            xyxy.append([x1, y1, x2, y2])
        else:
            xyxy.append([0, 0, 0, 0])  # Empty box if no mask
    
    # Return in supervision format
    return sv.Detections(
        xyxy=np.array(xyxy, dtype=np.float32),
        confidence=scores_np.astype(np.float32),
        mask=mask_np.astype(bool)  # Convert to boolean for mask annotator
    )

COLOR = sv.ColorPalette.from_hex([
    "#ffff00", "#ff9b00", "#ff8080", "#ff66b2", "#ff66ff", "#b266ff",
    "#9999ff", "#3399ff", "#66ffff", "#33ff99", "#66ff66", "#99ff00"
])


def annotate(image: np.ndarray, detections: sv.Detections, label: Optional[str] = None) -> np.ndarray:
    h, w = image.shape[:2]
    text_scale = sv.calculate_optimal_text_scale(resolution_wh=(w, h))

    mask_annotator = sv.MaskAnnotator(
        color=COLOR,
        color_lookup=sv.ColorLookup.INDEX,
        opacity=0.6
    )
    box_annotator = sv.BoxAnnotator(
        color=COLOR,
        color_lookup=sv.ColorLookup.INDEX,
        thickness=1
    )
    label_annotator = sv.LabelAnnotator(
        color=COLOR,
        color_lookup=sv.ColorLookup.INDEX,
        text_scale=0.4,
        text_padding=5,
        text_color=sv.Color.BLACK,
        text_thickness=1
    )

    annotated_image = image.copy()
    annotated_image = mask_annotator.annotate(annotated_image, detections)
    annotated_image = box_annotator.annotate(annotated_image, detections)

    if label:
        labels = [
            f"{label} {confidence:.2f}"
            for confidence in detections.confidence
        ]
        annotated_image = label_annotator.annotate(annotated_image, detections, labels)

    return annotated_image
