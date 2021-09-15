## Setting Up Simulator For Data Collection
For the generation of training data, we are using the Gazebo simulator. Our first task is getting the positional data for each of the depth cameras and this is accomplished in ROS using Transform Frames. For a frame to represent a world position it has to be relative to a world frame, so we add a world frame to our simulation and give each depth camera their own frame. I have not found an achievable way to get gazebo to publish tf transforms of models, because of that the frame positions are set by creating a transform publisher and manually setting the position of those transforms to the pose of the model in gazebo. Another method could be to use a point cloud registration algorithm that will set one camera as the world frame and register other frames relative to the first. This method that uses a point cloud registration algorithm would be able to create the transform frames automatically based on the depth camera's inputs.

## Representations Option 1
The distribution we are encoding takes in the position of the depth camera, the current time,  and an xy location as the X; and the distances received from the depth camera at the xy location, given in the X, as the Y.  This way of representing the sensors will treat each of the depth samples as a context point that are strongly connected to the specific value of the depth sensors. This strong connection to the distance sensors will result in less noise in the final samples, but requires a massive amount of context points (one per depth per time). A representation of the inputs and output is shown below. 
<br>
<br>
[Camera X,  Camera Y,  Camera Z, Camera Roll,  Camera Pitch,  Camera Yaw, Time, x, y]
 =>
[Distance]
 <br>
 <br>
To collect this data, we run a ROS node that will listen for any number of kinect cameras and transform frames. The ROS node uses the transform frame to fill out the first 6 arguments of X, the x, y, z, pitch, roll and yaw of the camera. The node then uses the timestamp of the depth image fill out the Time argument of X.  The node then loops over the width and height of the depth image filling out the last arguments of X, x and y. Finally, the node uses an x and y, given in the X, to access the depth from the depth image and stores that as the Y.
<br>
<br>
All the sensor data is then saved into two individual 2D arrays, one for all the Xs and another for the Ys. Then the data is saved into an .npz file as "sample_set_x" and "sample_set_y". The .npz file will also include the "repr_type" attribute, set to "1", as an easy way to access which format the data was collected in. 

## Representations Option 2
The distribution we are encoding takes in the position of the depth camera, and the current time as the X; and all the distances received from the depth camera as the Y.  This way of representing the sensors will treat the depth samples as being random but that shouldn't be an issue because a probability distribution of a 3D object moving over time is not strongly connected to the specific value of the depth sensors. This week connection to the distance sensors will result in a lot more noise in the final samples, but would reduce the number of context points by a ton (one per time, rather than one per depth per time). A representation of the inputs and output is shown below. 
<br>
<br>
[Camera X,  Camera Y,  Camera Z, Camera Roll,  Camera Pitch,  Camera Yaw, Time]
 => 
[D1, D2, D3, ... Dn-1, Dn] 

## Representations Option 4
[Camera1 X,  Camera1 Y,  Camera1 Z, Camera1 T, Camera2 X,  Camera2 Y,  Camera2 Z, Camera2 T]
 => 
[distance_to_intersection / distance_to_second_point] 
<br>
<br>
 To collect this data, we run a ROS node that will listen for any number of kinect cameras and transform frames. The ROS node uses the transform frame to fill out the first 6 arguments of X, the x, y, z, pitch, roll and yaw of the camera. The node also uses the timestamp of the depth image fill out the last argument of X. Finally, the node uses the received depth image to construct Y, an array of all distance values.
<br>
<br>
All the sensor data is then saved into two individual 2D arrays, one for all the Xs and another for the Ys. Then the data is saved into an .npz file as "sample_set_x" and "sample_set_y". The .npz file will also include the "repr_type" attribute, set to "2", as an easy way to access which format the data was collected in. 
