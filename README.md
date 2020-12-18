# Time-Synchronize-Kinect-RGBD-Streams
## Packages:
- **Robot Operating System (ROS)** - http://wiki.ros.org/ <br> ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS is licensed under an open source, BSD license.<br><br> Current Version: Noetic

- **The Point Cloud Library** - http://wiki.ros.org/pcl <br> The Point Cloud Library (or PCL) for point cloud processing - development The PCL framework contains numerous state-of-the art algorithms including filtering, feature estimation, surface reconstruction, registration, model fitting and segmentation.<br><br> Package was leveraged to preform Point Cloud Registration on incoming data streams from multiple Kinect360 RGBD Cameras. 

## Directory/File Structure
- ### /src
  - **time_sync_kinects.cpp** <br> The project's main file, contains the main method and the "On Kinnect Recieved" callback. The main method starts threads to complete simultanious point cloud registration on multiple connect streams. After all registrations are sucessful, the main method starts a single new thread to preform the time synchronization.
  - **registration.cpp** <br> This file contains the point cloud registration function that is called for each new thread. It preforms pointcloud registration between the first kinect stream and a stream specified in the parameters.
  - **combination.cpp** <br> This file contains the time synchronization algorithm. The alogorithm increments by a specified time interval and interpolates between the nearest point clouds.
- ### /worlds
  - **test.world** <br> Gazebo_ros world file that contains a scene with two Microsoft Kinect360 RGBD cameras and enough detail for the PCL Library's feature based point cloud registration algorithm to work correctly.
  
  
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
