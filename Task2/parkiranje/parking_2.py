#!/usr/bin/python3

# TA SKRIPTA JE TISTA KI DELA!

from glob import glob
import os

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

ground_image = Image()

image_number = 1
thresh_image_number = 1


def add_feedback(msg):
    global last_feedback
    last_feedback = msg


def add_image(msg):
    global ground_image
    ground_image = msg
    # find_parking_space(msg)


def find_parking_space(img):
    bridge = CvBridge()

    image = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
    global thresh_image_number

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # cv2.imwrite('parking/x_gray_{}.png'.format(thresh_image_number), gray)
    
    # thresh = cv2.threshold(sharpen, 253, 255, cv2.THRESH_BINARY_INV)[1]
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV)[1]
    print("delam sliko")
    # cv2.imwrite('src/exercise7/scripts/kvadrati/thresh_{}.png'.
    #                 format(thresh_image_number), thresh)
    thresh_image_number += 1

    # opening and closing
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    # Find contours and filter using threshold area
    cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    min_area = 5000
    global image_number
    for c in cnts:
        area = cv2.contourArea(c)
        if area > min_area:
            x,y,w,h = cv2.boundingRect(c)
            print("-----", x, ";", y, ";", w, ";", h)
            if w >= 200 and h >= 200 and 0.60 < h/w < 1.67:
                print("got a square")
                cv2.imwrite('src/exercise7/scripts/kvadrati/thresh_used_{}.png'.
                    format(thresh_image_number), thresh)
                ROI = image[y:y+h, x:x+w]
                cv2.rectangle(image, (x, y), (x + w, y + h), (36,255,12), 2)
                cv2.imwrite('/home/subic/ROS/src/exercise7/scripts/kvadrati/{}.png'.
                    format(image_number), image)
                image_number += 1

                center_x, center_y = (x+w)/2, (y+h)/2
                print("center of the square is: ", center_x, center_y)
                # parking position is current position (feedback) + 
                # + transformacija središča kvadrata iz slike v pozicijo


if __name__ == '__main__':
    rospy.init_node('parking_node')

    # goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)

    # square_marker_pub = rospy.Publisher('/square_markers', MarkerArray, queue_size=1000)

    feedback_pub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callback=add_feedback, queue_size=1)

    ground_img_sub = rospy.Subscriber('/arm_camera/rgb/image_raw', Image, callback=add_image, queue_size=1)

    rospy.sleep(1)
    
    r = rospy.Rate(2)
    ind = 0
    
    # da zbrišem slike v mapi iz prejšnjega poganjanja
    files = glob('src/exercise7/scripts/kvadrati/*')
    for f in files:
        os.remove(f)

    while not rospy.is_shutdown():
        find_parking_space(ground_image)
            
        r.sleep()
