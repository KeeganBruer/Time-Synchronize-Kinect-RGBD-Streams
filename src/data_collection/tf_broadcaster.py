#!/usr/bin/env python3  
import roslib
roslib.load_manifest('time_sync_kinects')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node("tf_broadcaster")
    name = rospy.get_param('~tf_name')
    br = tf.TransformBroadcaster()
    pose = [float(x.strip()) for x in rospy.get_param('~pose').split(" ")]
    count = rospy.get_param('~count')
    count = count if count else 10
    for i in range(count):
        print(pose)
        br.sendTransform((pose[0], pose[1], pose[2]),
            tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5]),
            rospy.Time.now(),
            "{0}".format(name),
            "world"
        )
        rospy.sleep(1)
    rospy.spin()
