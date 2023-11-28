# SPDA-SLAM
## An SuperPoint with Domain Adaptation SLAM System


## Test Environment
### Dependencies
* OpenCV 4.2
* Eigen 3
* Ceres 2.0.0 
* G2O (tag:20230223_git)
* TensorRT 8.4 
* CUDA 11.6
* python
* onnx
* ROS noetic
* Boost
* Glog

For **Nvidia GeForce RTX 40 series**, please use TensorRT 8.5 and CUDA 11.8 instead.

## Data
The data should be organized using the Automous Systems Lab (ASL) dataset format just like the following:

```
dataroot
├── cam0
│   └── data
│       ├── 00001.jpg
│       ├── 00002.jpg
│       ├── 00003.jpg
│       └── ......
└── cam1
    └── data
        ├── 00001.jpg
        ├── 00002.jpg
        ├── 00003.jpg
        └── ......
```

## Build
```
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

