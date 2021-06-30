#!/usr/bin/env python3
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('set_pose')
    rospy.wait_for_service('/gazebo/set_model_state')
    z = 0.0
    z_inc = 0.01
    while not rospy.is_shutdown():
        state_msg = ModelState()
        state_msg.model_name = 'unit_box'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        
        if z >= 2.0:
            z_inc = -0.01
        if z <= 0.0:
            z_inc = 0.01
        z += z_inc

        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        rospy.sleep(0.001)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
