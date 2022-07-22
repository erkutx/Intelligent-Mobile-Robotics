#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge

def image_callback(msg):
	bridge = cv_bridge.CvBridge()
	cv2.namedWindow("window", 1)
	image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
	cv2.imshow("window", image)
	cv2.waitKey(3)

image_sub = rospy.Subscriber('/camera/image_raw',Image, image_callback)
rospy.init_node('camera')
rospy.spin()