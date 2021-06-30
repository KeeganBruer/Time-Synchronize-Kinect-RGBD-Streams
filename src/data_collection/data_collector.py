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


sample_split = 0
sample_type = 2
bridge = CvBridge()
start_time = None
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
    _id = args[0]
    listener = args[1]
    tf_topic = args[2]
    try:
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
    except Exception as e:
        print(e)
    depth_array = np.array(depth_image, dtype=np.float32)
    (trans,rot) = listener.lookupTransform("world", tf_topic, rospy.Time(0))
    position = [round(float(x), 4) for x in trans]
    rotation = [round(float(x), 4) for x in tf.transformations.euler_from_quaternion(rot)]
    pose = [*position, *rotation]
    
    timestamp = ros_image.header.stamp.secs + (ros_image.header.stamp.nsecs * 0.000000001)
    
    if start_time == None:
        start_time = timestamp
    timestamp = round(timestamp-start_time, 4)

    sensor_x = [*pose, timestamp]
    sensor_y = depth_array
    X.append(sensor_x)
    Y.append(sensor_y)
    print("\rCollected {0:4d} points".format(len(X)), end="")

    if len(X) >= sample_split:
        out_x = X
        out_y = Y
        X = []
        Y = []
        save_data(out_x, out_y, save_count)
        save_count += 1
        

def save_data(sample_x, sample_y, save_count):
    print("Saving {0} points".format(len(sample_x)))
    print("Saving to {0}/catkin_ws/src/time_sync_kinects/sample_set_{1}".format(os.path.expanduser("~"), save_count))
    np.savez_compressed('{0}/catkin_ws/src/time_sync_kinects/sample_set_{1}'.format(os.path.expanduser("~"), save_count), sample_set_x=sample_x, sample_set_y=sample_y, repr_type=sample_type)
    del sample_x[:]
    del sample_y[:]

def listener():
    global sample_split
    global sample_type
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
        print(img_topic)
        listener.waitForTransform(tf_topic, "world", rospy.Time(), rospy.Duration(4.0))
        rospy.Subscriber(img_topic, Image, callback, (i, listener, tf_topic))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
