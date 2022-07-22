#!/usr/bin/env python
import rospy, cv2, cv_bridge
import actionlib
import tf
import roslib
import threading
import cv2.aruco as aruco
import sys, time, math
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy
from sensor_msgs.msg import Image
import dlib
import numpy as np
import face_recognition

test_gates = 1
test_freeman = 1

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

def image_callback(msg):
    global old_faces,test_gates,test_freeman,frame
    new = True
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

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

def image_test():
    global old_faces,test_gates,test_freeman,frame

    # Load a sample picture and learn how to recognize it.
    gates_image = face_recognition.load_image_file("/home/markhor/catkin_ws/src/explorer/src/gates.jpeg")
    gates_face_encoding = face_recognition.face_encodings(gates_image)[0]

    # Load a second sample picture and learn how to recognize it.
    freeman_image = face_recognition.load_image_file("/home/markhor/catkin_ws/src/explorer/src/morgan.jpeg")
    freeman_face_encoding = face_recognition.face_encodings(freeman_image)[0]

    # Create arrays of known face encodings and their names
    known_face_encodings = [gates_face_encoding, freeman_face_encoding]
    known_face_names = ["Bill Gates", "Morgan Freeman"]

    # Initialize some variables
    face_locations = []
    face_encodings = []
    face_names = []
    process_this_frame = True
    # Grab a single frame of video
    # Resize frame of video to 1/4 size for faster face recognition processing
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_small_frame = frame[:, :, ::-1]

    # Only process every other frame of video to save time
    # Find all the faces and face encodings in the current frame of video
    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    face_names = []
    for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                #print matches
                name = "Unknown"

                # # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                     first_match_index = matches.index(True)
                     name = known_face_names[first_match_index]
                     
                if name == "Bill Gates" and test_gates == 1:

                    test_gates = 5
                    print name
                    photo_name = str(name) + '.jpg'
                    cv2.imwrite(photo_name, frame)

                if name == "Morgan Freeman" and test_freeman == 1:
                    test_freeman = 5
                    print name
                    photo_name = str(name) + '.jpg'
                    cv2.imwrite(photo_name, frame)
                           
                face_names.append(name)

def main():
    global x, y, z, stopp, n, m, add, yaw, a,b,c
    point = (4, 3, 1.57)
    set_goal_to_point(point)
    print"1st point reached"
    image_test()
    point = (0, 0, 3.12)
    set_goal_to_point(point)
    print"2nd point reached"
    point = (-7, 0, 3.12)
    set_goal_to_point(point)
    print"3rd reached"
    image_test()
    point = (-7, 3.5, 2)
    set_goal_to_point(point)
    print"4th reached"
    image_test()
    point = (-6.5, 0, -1.57)
    set_goal_to_point(point)
    print"5th reached"
    point = (0, 0, 0)
    set_goal_to_point(point)
    print"6th reached"


image_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)
main()