#!/usr/bin/env python3
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('set_pose')
    steps_params = rospy.get_param("~steps", "0, 0, 0.005")
    steps = [float(x.strip()) for x in steps_params.split(",")]
    print(steps)
    steps[0] = rospy.get_param("~x_step", steps[0])
    steps[1] = rospy.get_param("~y_step", steps[1])
    steps[2] = rospy.get_param("~z_step", steps[2])
    
    max_pos_params = rospy.get_param("~max_pos", "1, 1, 5")
    max_pos = [float(x.strip()) for x in max_pos_params.split(",")]
    
    min_pos_params = rospy.get_param("~min_pos", "-1, -1, 0")
    min_pos = [float(x.strip()) for x in min_pos_params.split(",")]

    model_name = rospy.get_param("~model_name", "wall")
    
    init_pos_params = rospy.get_param("~init_pos", "0, 0, 2")
    pos = [float(x.strip()) for x in init_pos_params.split(",")]
    print(steps)
    print(max_pos)
    print(min_pos)
    print(model_name)
    rospy.wait_for_service('/gazebo/set_model_state')
    

    inc = steps[:] #copy the array? not sure if not having [:] copies by pointer.

    while not rospy.is_shutdown():
        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose.position.x = pos[0]
        state_msg.pose.position.y = pos[1]
        state_msg.pose.position.z = pos[2]
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        
        for i in range(3):
            if pos[i] >= max_pos[i]:
                inc[i] = -1 * steps[i]
            if pos[i] <= min_pos[i]:
                inc[i] = steps[i]
            pos[i] += inc[i]
        #print(pos)
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
