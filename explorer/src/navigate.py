#!/usr/bin/env python
import rospy
import actionlib
import tf
import roslib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


get_pos = 0
marker_size  = 50 #- [cm]
stopp = 0
add = -20 
yaw = 3.14
n = 0
m = 0
a = 0
b = 0
c = 0

def callback(msg):
    global x, y, z
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.orientation.z

if get_pos == 0:
    odo_sub = rospy.Subscriber('/odom', Odometry, callback)
    get_pos = 1

# Get a move_base action client
rospy.init_node('patrolling')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
rospy.loginfo('Connecting to move_base...')
client.wait_for_server()
rospy.loginfo('Connected to move_base.')



def set_goal_to_point(point):

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

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()



def main():
    point = (-10, 10, 0.000)
    set_goal_to_point(point)
    print"1st patient reached"



main()