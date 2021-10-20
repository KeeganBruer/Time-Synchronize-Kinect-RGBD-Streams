#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf
import numpy as np
import os
import threading
import math

sample_split = 1000000
sample_type = 4
view_distance = 10
bridge = CvBridge()
start_time = None
thread_list = []
X = []
Y = []
save_count = 0

def callback(ros_image, args):
    global bridge
    global start_time
    global X
    global Y
    global sample_split
    global sample_type
    global save_count
    global view_distance
    global thread_list
    _id = args[0]
    listener = args[1]
    tf_topic = args[2]
    try:
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        #print(depth_image.dtype)
    except Exception as e:
        print(e)
    depth_array = np.array(depth_image, dtype=np.float32)
    (trans,rot) = listener.lookupTransform("world", tf_topic, rospy.Time(0))
    position = [round(float(x), 4) for x in trans]
    #print("rotation")
    #print(rot)
    rotation = [round(float(x), 4) for x in tf.transformations.euler_from_quaternion(rot)]
    if (sample_type in [1, 2, 3]):
        pose = [*position, *rotation] #position and rotation
    elif (sample_type == 4):
        pose = [*position]
    timestamp = ros_image.header.stamp.secs + (ros_image.header.stamp.nsecs * 0.000000001)
    
    if start_time == None:
        start_time = timestamp
    timestamp = round(timestamp-start_time, 4)
    
    if (sample_type == 1):
        sensor_x = [*pose, timestamp]
    elif (sample_type == 2):
        pass
    elif (sample_type == 3):
        sensor_x = [timestamp]
    if (sample_type in [1,2,3]):
        sensor_y = depth_array.flatten()
    elif (sample_type == 4):
        if len(thread_list) < 40:
            save_depth_image(depth_image, pose, rot, timestamp, save_count)
            save_count+=1
        else:
            thread_list[0].join()
            thread_list.pop(0)
    if len(X) >= sample_split:
        out_x = X[0:sample_split]
        out_y = Y[0:sample_split]
        
        X = X[sample_split:len(X)]
        Y = Y[sample_split:len(Y)]
        print("\rCollected {0:4d} points in format {1}, leaving {2} points to be saved next".format(len(out_x), sample_type, len(X)), end="")
        x = threading.Thread(target=save_data, args=(out_x, out_y, save_count))
        save_count += 1

def save_depthimage_to_type_4(depth_array, save_count, pose, rot, timestamp):
    w = len(depth_array)
    h = len(depth_array[0])
    X = []
    Y = []
    print(w, h)
    for x in range(w):
        for y in range(h):
            origin = pose[0:3]
            shift_x = x - (w/2) #Shift so 0,0 is in the center
            shift_y = y - (h/2)
            pixel_q = get_quaternion(shift_x, shift_y, w, h, 1.0472) #get quaternion of pixel
            #quaternion in terms of the world origin
            q = quaternion_multiply(pixel_q, rot)
            #q = pixel_q
            forward = [
                2 * (q[1]*q[3] + q[0]*q[2]),  # X
                2 * (q[2]*q[3] + q[0]*q[1]),  # Y
                1- 2 * (q[1]*q[1] + q[2]*q[2])# Z
            ]
            new_point = [
                origin[0] + view_distance * forward[0],
                origin[1] + view_distance * forward[1],
                origin[2] + view_distance * forward[2]
            ]

            sensor_x = [*origin, timestamp, *new_point, timestamp]
            sensor_y = depth_array[x][y] / view_distance
            X.append(sensor_x)
            Y.append(sensor_y)
        #print(str(x)+" ", end="", flush=True)
    print(len(X))
    save_depth_image(depth_array, save_count)

def get_quaternion(x, y, w, h, fov):
    x_angle = (x/float(w))*fov #Get angle if x is -w/2 then x/w is -1/2. then the angle is -1/2 * fov (60deg) = -30deg.
    y_angle = (y/float(h))*fov
    #print("{0:.2f} {1:.2f} \n".format(x_angle, y_angle), end="")
    xq = get_quaternion_about([1, 0, 0], x_angle)
    yq = get_quaternion_about([0, 1, 0], y_angle)
    zq = get_quaternion_about([0, 0, 1], 0)
    q = quaternion_multiply(quaternion_multiply(xq, yq), zq)
    #print(q)
    return q
def get_quaternion_about(axis, angle):
    factor = math.sin(angle/2)
    x = axis[0] * factor
    y = axis[1] * factor
    z = axis[2] * factor
    w = math.cos(angle/2)
    return [w, x, y, z]


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def save_data(sample_x, sample_y, save_count):
    print("\nSaving {0} points".format(len(sample_x)))
    print("Saving to {0}/catkin_ws/src/time_sync_kinects/samples/sample_set_{1}".format(os.path.expanduser("~"), save_count))
    np.savez_compressed('{0}/catkin_ws/src/time_sync_kinects/samples/sample_set_{1}'.format(os.path.expanduser("~"), save_count), sample_set_x=sample_x, sample_set_y=sample_y, repr_type=sample_type)
    del sample_x[:]
    del sample_y[:]
def save_depth_image(depth_array, pose, rot, timestamp, save_count):
    np.savez_compressed(
        '{0}/catkin_ws/src/time_sync_kinects/raw_data/sample_set_{1}'.format(os.path.expanduser("~"), save_count), 
        depth_array=depth_array,
        pose=pose,
        rotation=rot,
        timestamp=timestamp,
        repr_type="depth_array"
    )


def listener():
    global sample_split
    global sample_type
    for i in range(3, 0, -1):
        print("\r{}".format(i), end="")
        rospy.sleep(1)
    rospy.init_node('data_collector', anonymous=True)
    sample_split = int(rospy.get_param("~save_split", None)) if rospy.get_param("~save_split", None) else 1000
    sample_type = rospy.get_param("~collection_type", None) if rospy.get_param("~collection_type", None) else 2
    tf_topics = rospy.get_param("~tf_topics")
    print(tf_topics)
    tf_topics = [x.strip() for x in tf_topics.replace("[", "").replace("]", "").split(",")]
    print(tf_topics)
    img_topics= rospy.get_param("~depth_topics")
    img_topics = [x.strip() for x in img_topics.replace("[", "").replace("]", "").split(",")]
    listener = tf.TransformListener()
    for i in range(len(img_topics)):
        tf_topic = tf_topics[i]
        img_topic = img_topics[i]
        print("tf: {0}".format(tf_topic))
        print("cm: {0}".format(img_topic))
        listener.waitForTransform(tf_topic, "world", rospy.Time(), rospy.Duration(30.0))
        rospy.Subscriber(img_topic, Image, callback, (i, listener, tf_topic))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
