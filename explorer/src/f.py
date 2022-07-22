
#!/usr/bin/env python
import rospy, cv2, cv_bridge
import actionlib
import tf
import roslib
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

get_pos = 0

def callback(msg):
    global x, y, z
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.orientation.z

if get_pos == 0:
	odo_sub = rospy.Subscriber('/odometry/filtered', Odometry, callback)
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


marker_size  = 50 #- [cm]
stopp = 0
add = -20 
yaw = 3.14
n = 0
m = 0
a = 0
b = 0
c = 0



def main(msg):
    global x, y, z, stopp, n, m, add, yaw, a,b,c
    calib_path  = ""
    camera_matrix   = np.loadtxt("/home/markhoor/catkin_ws/src/perimeter_robot/perimeter_control/src/cameraMatrix.txt", delimiter=',')
    camera_distortion   = np.loadtxt('/home/markhoor/catkin_ws/src/perimeter_robot/perimeter_control/src/cameraDistortion.txt', delimiter=',')


    #--- Define the aruco dictionary
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters  = aruco.DetectorParameters_create()


    #--- Capture the videocamera (this may also be a video or a picture)
    cap = cv_bridge.CvBridge()



    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    #while True:

    #-- Read the camera frame
    frame = cap.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    #cap = cv2.VideoCapture(frame)
    #ret, frame = cap.read(frame)

    #-- Convert in gray scale
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # remember, OpenCV stores color images in Blue, Green, Red

    #-- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                              cameraMatrix=camera_matrix, distCoeff=camera_distortion)

    if ids is not None:


	    if stopp == 0:
	        if ids[0] == 0:
	            print"start point"
	            print "Row = " + str(ids[0])
	            point = (x+1,y+2.5,z+yaw)
	            print "goal = " + str(point)
	            set_goal_to_point(point)
	            print"1st target reached"
	            point = (-4.36195762768,2.56197709286,3.14)
	            set_goal_to_point(point)
	            point = (-20.4521223022,2.45150109369,3.14)
	            print "2nd goal = " + str(point)
	            set_goal_to_point(point)
	            print"2nd target reached"
	            stopp=1
	
	    if m == 0:
	        if ids[0] == 1:
	            print" "
	            print "Row = " + str(ids[0])
	            point = (-23.9765862781,5.13382682985,1.57)
	            print "goal = " + str(point)
	            set_goal_to_point(point)
	            print"1st target reached"
	            point = (-0.472503035281,5.88936294893,0)
	            print "2nd goal = " + str(point)
	            set_goal_to_point(point)
	            print"2nd target reached"
	            m=1
	
	    if n == 0:
	        if ids[0] == 2:
	            print"  "
	            print "Row = " + str(ids[0])
	            point = (1.83417150179,7.42037664332,1.57)
	            print "goal = " + str(point)
	            set_goal_to_point(point)
	            print"1st target reached"
	            point = (-20.4019386169,8.43916912737,3.14)
	            print "2nd goal = " + str(point)
	            set_goal_to_point(point)
	            print"2nd target reached"
	            n=1
	
	    if a == 0:
	        if ids[0] == 3:
	            print" "
	            print "Row = " + str(ids[0])
	            point = (-23.0989291768,10.4641937369,1.57)
	            print "goal = " + str(point)
	            set_goal_to_point(point)
	            print"1st target reached"
	            point = (-0.305869328947,12.2,0)
	            print "2nd goal = " + str(point)
	            set_goal_to_point(point)
	            print"2nd target reached"
	            a=1
	
	    if b == 0:
	        if ids[0] == 4:
	            print"  "
	            print "Row = " + str(ids[0])
	            point = (1.83194627774,14.11835762,1.57)
	            print "goal = " + str(point)
	            set_goal_to_point(point)
	            print"1st target reached"
	            point = (-22.0721272105,14.8184407585,3.14)
	            print "2nd goal = " + str(point)
	            set_goal_to_point(point)
	            print"2nd target reached"
	            point = (-12.3975016859,-2.49016217513,0)
	            set_goal_to_point(point)
	            print"put the grapes into freezer"
	            b=1
	

if __name__ == '__main__':

    image_sub = rospy.Subscriber('/up_left_d435/camera/image', Image, main)
    rospy.spin()