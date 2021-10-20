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

sample_split = 1000000
sample_type = 4
view_distance = 10
bridge = CvBridge()
start_time = None
thread_list = []
X = []
Y = []

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
    save_data(X,  Y, save_count)

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


def listener():
    save_location = '{0}/catkin_ws/src/time_sync_kinects/raw_data'.format(os.path.expanduser("~"))
    print(os.path.abspath(save_location) + '/*.npz')
    save_count = 0 
    file_paths = sorted(glob.glob(os.path.abspath(save_location) + '/*.npz'), key=lambda x: int(os.path.basename(x).replace(".npz", "").split('_')[-1]))
    for file_path in file_paths:
        frame_npz = np.load(file_path)
        depth_array = frame_npz["depth_array"]
        pose = frame_npz["pose"]
        rot = frame_npz["rotation"]
        timestamp = frame_npz["timestamp"]
        print("{0:0.4f} {1} {2}".format(timestamp, pose, rot))
        save_depthimage_to_type_4(depth_array, save_count, pose, rot, timestamp)
        save_count += 1

if __name__ == '__main__':
    listener()
