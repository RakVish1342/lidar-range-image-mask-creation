### Lidar Range Image Semantic Segmentation Mask Creation

This is a tool to segment and label 3D pointclouds using CloudCompare to create the corresponding 2D semantic segmentation masks for range images of lidar scans. 

A PCD pointcloud is read into CloudCompare (CC). CC is then used to segment and label pointclouds in 3D. The labelled pointclouds are saved in ASCII format. This ASCII format is then used to create a modified 2D range image. The modified 2D range image is thresholded to obtain the final 2D semantic segmentation masks. These masks can be fed to neural networks to perform semantic segmentation on lidar range images.

#### Usage:


**Youtube Walkthrouh:** [https://youtu.be/B61WNd7R_w4](https://youtu.be/B61WNd7R_w4)


1. **As Standalone CPP Package (lidar_seg/)**

Run each executable only after the previous has finished executing.

```
cd lidar_seg/
mkdir build/
cd build/
cmake ..
make
./rangeImageVisualization
./create_masks.py (OR) python3 create_masks.py
```


2. **As ROS Package (lidar_seg_ros/)**

Run each ROS node only after the previous has finished executing.

```
catkin build
source devel/setup.sh
roscore
rosrun lidar_seg_ros rangeImageVisualization
rosrun lidar_seg_ros create_masks.py
```
