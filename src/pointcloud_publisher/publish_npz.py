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
import glob
from geometry_msgs.msg import Point32
import std_msgs.msg 
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1)
]

def publish_pointclouds(ptcld_publisher, save_location):
    print(os.path.abspath(save_location) + '/*.npz')
    
    file_paths = glob.glob(os.path.abspath(save_location) + '/*.npz')
    print(file_paths)
    for i in range(len(file_paths)):
        data = np.load(file_paths[i])
        sample_set_x = data["sample_set_x"]
        sample_set_y = data["sample_set_y"]

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "map"
        points = []
        for i in range(len(sample_set_x)):
            point1 = sample_set_x[i][0:3]
            p_time = sample_set_x[i][3]
            point2 = sample_set_x[i][4:7]
            distance = sample_set_y[i]
            #print("point {} point {} p_time {} distance {}".format(point1, point2, p_time, distance))
            midpoint = [
                (point1[0]+point2[0])* distance, 
                (point1[1]+point2[1]) * distance, 
                point1[2]+point2[2] * distance
            ]
            points.append(point1)
            points.append(midpoint)
        print("Publishing a pointcloud with {} points".format(len(points)))
        out_ptcld = point_cloud2.create_cloud(h, fields, points)
        ptcld_publisher.publish(out_ptcld)

def listener():
    rospy.init_node('data_collector', anonymous=True)
    save_location = rospy.get_param("~save_location", "./samples")
    publish_path = rospy.get_param("~publish_topic", "/pointcloud_publisher/output_cloud")
    print(save_location)
    ptcld_publisher = rospy.Publisher(publish_path, PointCloud2, queue_size=10)
    
    publish_pointclouds(ptcld_publisher, save_location)
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    listener()

