#!/usr/bin/env python
import rospy
import numpy as np
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import*


def carry_box():
    
    
    state = arm("robot::base_footprint","")
    state_msg = ModelState()
    state_msg.model_name = 'beer'
    state_msg.pose.position.x = state.link_state.pose.position.x
    state_msg.pose.position.y = state.link_state.pose.position.y
    state_msg.pose.position.z = 0.3
    grip_state = grip(state_msg)

def main():
    carry_box()


rospy.init_node('carry_box')
grip = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
rospy.wait_for_service('/gazebo/set_model_state')
arm = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
rospy.wait_for_service('/gazebo/get_link_state')
while not rospy.is_shutdown():
    main()