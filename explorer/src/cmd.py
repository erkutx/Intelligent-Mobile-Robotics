#!/usr/bin/env python3
import rospy 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def odom_callback(msg):
    global x, y, z
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.orientation.z

front = 0

def scan_callback(msg):
    global front, left, right, length
    length  = len(msg.ranges)
    front = msg.ranges[0]
    left = msg.ranges[90]
    right = msg.ranges[270]

def wanderbot():
    twist.angular.z = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    if front > 1.5 :
        twist.linear.x = 0.2
        cmd_vel_pub.publish(twist)
        print ("moving forward", front)
    else :
        twist.angular.z = 1
        cmd_vel_pub.publish(twist)
        print ("left turn")



# def avoid():
# 	while front > 2 :
# 		twist.linear.x = 0.2
# 		cmd_vel_pub.publish(twist)
# 		print ("distance until collision", front)
# 	twist.linear.x = 0

# 	print ("~2m before collision reached", front)
# def meter():
#     while x < 1 :
#         twist.linear.x = 0.2
#         cmd_vel_pub.publish(twist)
#         print ("going 1 meter", x)
#     print ("achieved", x)
#     twist.linear.x = 0
#     cmd_vel_pub.publish(twist)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
twist = Twist()
odom_sub =  rospy.Subscriber('/odom', Odometry, odom_callback)
scan_sub =  rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.init_node('Wall')
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    #print (x, y, z)
    wanderbot()
    # meter()
    # avoid()
    #print(front)
    rospy.sleep(0.1)