# Time-Synchronize-Kinect-RGBD-Streams
## Packages:
- **Robot Operating System (ROS)** - http://wiki.ros.org/ <br> ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS is licensed under an open source, BSD license.<br><br> Current Version: Noetic

- **The Point Cloud Library** - http://wiki.ros.org/pcl <br> The Point Cloud Library (or PCL) for point cloud processing - development The PCL framework contains numerous state-of-the art algorithms including filtering, feature estimation, surface reconstruction, registration, model fitting and segmentation.<br><br> Package was leveraged to preform Point Cloud Registration on incoming data streams from multiple Kinect360 RGBD Cameras. 

## Directory/File Structure
- ### /src
  - **time_sync_kinects.cpp** <br> The project's main file, contains the main method and the "On Kinnect Recieved" callback. The main method starts threads to complete simultanious point cloud registration on multiple connect streams. After all registrations are sucessful, the main method starts a single new thread to preform the time synchronization.
- ### /registration  
  - **registration.cpp** <br> This file contains the point cloud registration function that is called for each new thread. It preforms pointcloud registration between the first kinect stream and a stream specified in the parameters.
  - **combination.cpp** <br> This file contains the time synchronization algorithm. The alogorithm increments by a specified time interval and interpolates between the nearest point clouds.
- ### /data_collection
  - **tf_broadcaster.py** <br> This file contains a node to broadcast a tf transform, used in data collection to collect the positional data of the cameras. If using automatic registration, this node is not needed.
  - [**data_collector.py**](https://github.com/KeeganBruer/Time-Synchronize-Kinect-RGBD-Streams/blob/main/src/data_collection/tf_broadcaster.py) <br> This file contains a node to collect training data based the the specifications detailed [here](https://github.com/KeeganBruer/Time-Synchronize-Kinect-RGBD-Streams/tree/main/src/data_collection/README.md)
- ### /gazebo_utils
  - [**model_mover.py**](https://github.com/KeeganBruer/Time-Synchronize-Kinect-RGBD-Streams/blob/main/src/gazebo_utils/model_mover.py) <br> This file broadcasts a message to move a gazebo model.  
- ### /launch
  - **start_gazebo.launch** <br> This file launchs gazebo with the /world/cube_test.world
  - **start_collection.launch** <br> This file launchs four tf transform broadcaster and a data_collection node.
- ### /worlds
  - **test.world** <br> Gazebo_ros world file that contains a scene with two Microsoft Kinect360 RGBD cameras and enough detail for the PCL Library's feature based point cloud registration algorithm to work correctly.
  - **cube_test.world** <br> Gazebo_ros world file that contains a scene with four Microsoft Kinect360 RGBD cameras and a single cube.

## Install Package:
```
> git clone https://github.com/KeeganBruer/Time-Synchronize-Kinect-RGBD-Streams.git time_sync_kinects
> catkin_make
```
## Launch Data Collection
```
> roslaunch time_sync_kinects start_collection.launch
```

## Launch Automatic Registration:
```
> roslaunch time_sync_kinects start_registration.launch
```

## Launch Gazebo Test World
```
> roslaunch time_sync_kinects start_gazebo.launch
```
