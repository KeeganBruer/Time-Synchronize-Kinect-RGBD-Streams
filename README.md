# Time-Synchronize-Kinect-RGBD-Streams
## Robot Operating System (ROS)
http://wiki.ros.org/ <br>
ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS is licensed under an open source, BSD license. 
<br><br>
Current Version: Noetic

## The Point Cloud Library
http://wiki.ros.org/pcl <br>
The Point Cloud Library (or PCL) for point cloud processing - development The PCL framework contains numerous state-of-the art algorithms including filtering, feature estimation, surface reconstruction, registration, model fitting and segmentation. 
<br><br>
Current Version: <br>
Package was leveraged to preform Point Cloud Registration on incoming data streams from multiple Kinect360 RGBD Cameras. 

## Launch Project:
```
> rosrun time_sync_kinects time_sync_kinects [kinect#] [topics]
```
Example:
```
> rosrun time_sync_kinects time_sync_kinects 2 /camera1/depth/points /camera2/depth/points
```

## Launch Gazebo Test World
```
> rosrun gazebo_ros gazebo ./worlds/test.world -u
```
