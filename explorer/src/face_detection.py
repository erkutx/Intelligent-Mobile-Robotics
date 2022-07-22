#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
import dlib

old_faces = []

def image_callback(msg):
    global old_faces
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    detector = dlib.get_frontal_face_detector()


    image = cv2.resize(image, (0, 0), fx=1, fy=1)

    faces = detector(image, 1)
    if len(old_faces) < len(faces):
        old_faces = []
        for face in faces:
            tracker = dlib.correlation_tracker()
            tracker.start_track(image, face)
            old_faces.append(tracker)
    else:
        for i, tracker in enumerate(old_faces):
            quality = tracker.update(image)
            if quality > 7:
                pos = tracker.get_position()
                pos = dlib.rectangle(
                    int(pos.left()),
                    int(pos.top()),
                    int(pos.right()),
                    int(pos.bottom()),
                )
                cv2.rectangle(image, (pos.left(), pos.top()), (pos.right(), pos.bottom()),
                              (100, 200, 100))
            else:
                old_faces.pop(i)

    cv2.imshow("window", image)
    key = cv2.waitKey(1) & 0xFF

rospy.init_node('camera')
image_sub = rospy.Subscriber('/camera/image_raw', Image, image_callback)
rospy.spin()
