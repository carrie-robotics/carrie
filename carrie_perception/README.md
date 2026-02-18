# 🤖 carrie_perception
Contains ros2 service server and service client nodes for performing prompt based image segmentation using SAM3 foundation model. The client sends the image on which the segmentation will be performed and the prompt for segmentation. The server receives this request and perform segmentation. The server returns the response which is the detections as a 2D array and the binary segmentation mask.

#### **Important:** 
SAM3 is a gated foundation model. Before running this package you must: 
1. Requst access on Hugging Face by clicking [here](https://huggingface.co/facebook/sam3)
2. Be logged in locally (`huggingface-cli login`)
3. Have a valid access token

## 📂 Directories
- `carrie_perception` contains the server and client nodes that performs the segmentation.
- `config` parameters that affect the segmentation can be set here
- `example_images` contains some sample images on which segmentation could be run and output could be visualized

**Note:** The client and server are making use of a custom service definition called as `DetectObjects.srv`and is defined inside ros2 package called as `carrie_interfaces`

## 💻 Hardware Achitecture
The entire architecture runs irrespective of CUDA availability. It runs on mps (on macos), if mps is not detected then the nodes run on the cpu.

## SAM3 Segmentation demo
After you have built the pixi environment by typing:
```shell
pixi run build
```
you can launch the service server by typing:

```shell
pixi run sam3-detection-server
```
The weights should be automatically loaded and sam3_node will be waiting for a segmentation request from client.

to launch the service client, open another terminal and type:
```shell
pixi run sam3-client
```

Following is an example of how the output will look like. A folder called as `sam3_output` containg the segmentation results would be generated automatically.

The `prompt` is set to `woman` in `config/sam3_params.yaml` file. Change this parameter to play with the segmentation results.

<p align="center"> <img src="docs/result_solvay_conference.png" alt="rviz carrie" width="1000"/> </p>