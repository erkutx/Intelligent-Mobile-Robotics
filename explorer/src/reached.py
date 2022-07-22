#!/usr/bin/env python

import rospy
import actionlib
import tf
import roslib
import threading
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def callback(msg):
    global detecttion
    detecttion = msg.data



class Patrol:

    def __init__(self):
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def set_goal_to_point(self, point):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, point[2])
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()


if __name__ == '__main__':
    rospy.init_node('patrolling')
    try:
        p = Patrol()

        p.set_goal_to_point((-0.694251266082, 8.2121617605, 1.591))
        p.set_goal_to_point((-8.74428653305, 8.2121617605, 1.591))
        p.set_goal_to_point((-8.74428653305, 10.0, 1.591))

    except rospy.ROSInterruptException:
		rospy.logerr("Something went wrong when sending the goal")