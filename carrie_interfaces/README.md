# 🤖 carrie_interfaces
Contains a custom service definition used in package `carrie_perception`. The service client sends segmentation requests to the service server using these interfaces.

## DetectObjects.srv
### Request Structure
```shell
sensor_msgs/Image image
string prompt
```
The client makes a request by sending an image and a text prompt describing what to segment in that image.

### Response Structure
```shell
vision_msgs/Detection2DArray detections
sensor_msgs/Image mask
bool success
string message
```
The service server performs the segmentation on the image received based on the text prompt using SAM3 foundation model and returns:

- **detections**: Bounding boxes and confidence scores as a 2D array
- **mask**: Binary segmentation mask
- **success**: Whether the operation succeeded
- **message**: Status message (e.g., "Detected 3 object(s)")
