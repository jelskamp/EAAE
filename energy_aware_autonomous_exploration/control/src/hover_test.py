#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def hover():
    rospy.init_node('uav_hover', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state')

    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    hover_state = ModelState()
    hover_state.model_name = 'iris'  
    hover_state.pose.position.x = 0.0
    hover_state.pose.position.y = 0.0
    hover_state.pose.position.z = 2.0  # Set UAV hover altitude
    hover_state.pose.orientation.w = 1.0
    hover_state.twist.linear.x = 0.0
    hover_state.twist.linear.y = 0.0
    hover_state.twist.linear.z = 0.0
    hover_state.reference_frame = 'world'

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        set_model_state(hover_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        hover()
    except rospy.ROSInterruptException:
        pass
