# Time-Synchronize-Kinect-RGBD-Streams

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
