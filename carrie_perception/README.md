# carrie_perception
Contains a ros2 node for performing prompt based image segmentation using SAM3 foundation model. This node subscribes to the camera topic and publishes the 2D detections and the binary segmentation masks based on the text prompt. 

## 📂 Directories
- `carrie_perception` contains the sam3 node that performs the segmentation. The entire node runs irrespective of CUDA. It runs on mps (on macos), if mps is not detected then the node runs on the cpu.   
- `config` parameters that effect the segmentation can be set here 
- `example_images` contains some sample images on which segmentation could be run and output could be visualized 


## SAM3 Segmentation demo
After you have built the pixi environment by typing:  
```shell
pixi run build
```
you can type: 

```shell
pixi run sam3-detection
```
The weights should be automatically loaded and sam3_node will be waiting for an image to be published on topic `/camera/image_raw`

Open another terminal and type:
```shell
pixi run publish-test-image
```
This node publishes an image from the `example_images` on the topic `/camera/image_raw`

Following is an example of how the output will look like. A folder called as `sam3_output` containg the segmentation results would be generated automatically.   

The `prompt` is set to `woman` in `config/sam3_params.yaml` file. Change this parameter to play with the segmentation results. 

<p align="center"> <img src="docs/result_solvay_conference.png" alt="rviz carrie" width="1000"/> </p>

