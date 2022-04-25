#!/usr/bin/python3

from glob import glob
import rospy

import cv2
import numpy as np
from cv_bridge import CvBridge

import geometry_msgs.msg as gmsg
import actionlib_msgs.msg as amsg
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseActionFeedback
from sensor_msgs.msg import Image


last_feedback = MoveBaseActionFeedback()
parking_feedbacks = []


def add_feedback(msg):
    global last_feedback
    last_feedback = msg


ground_image = Image()


def add_image(msg):
    global ground_image
    ground_image = msg
    find_parking_space(msg)


image_number = 0


def find_parking_space(img):
    bridge = CvBridge()

    # image = cv2.imread(img)
    image = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
    # cv2.imwrite('test.png', image)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    # Threshold and morph close
    thresh = cv2.threshold(sharpen, 130, 255, cv2.THRESH_BINARY)[1]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find contours and filter using threshold area
    cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    min_area = 100000
    max_area = 280000
    global image_number
    for c in cnts:
        area = cv2.contourArea(c)
        if area > min_area and area < max_area:
            print("got a square")
            x,y,w,h = cv2.boundingRect(c)
            print("-----", x, ";", y, ";", w, ";", h)
            ROI = image[y:y+h, x:x+w]
            # cv2.imwrite('{}.png'.format(image_number), ROI)
            cv2.rectangle(image, (x, y), (x + w, y + h), (36,255,12), 2)
            cv2.imwrite('{}.png'.format(image_number), image)
            image_number += 1

            center_x, center_y = (x+w)/2, (y+h)/2
            # parking position is current position (feedback) + 
            # + transformacija središča kvadrata iz slike v pozicijo

            # marker ?


if __name__ == '__main__':
    
    rospy.init_node('parking_node')

    # goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)

    # square_marker_pub = rospy.Publisher('/square_markers', MarkerArray, queue_size=1000)

     feedback_pub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callback=add_feedback, queue_size=1)

    ground_img_sub = rospy.Subscriber('/arm_camera/rgb/image_raw', Image, callback=add_image, queue_size=1)

    rospy.sleep(1)
    
    r = rospy.Rate(10)
    ind = 0
    
    while not rospy.is_shutdown():
        # find_parking_space(ground_image)
            
        r.sleep()
