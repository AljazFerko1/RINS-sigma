#!/usr/bin/python3

# TA SKRIPTA JE TISTA KI DELA!

from glob import glob
import os
import math
from shutil import move

from cv2 import line

import rospy

import cv2
import numpy as np
from cv_bridge import CvBridge

from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseActionFeedback
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID


last_feedback = MoveBaseActionFeedback()
parking_feedbacks = []

ground_image = Image()

image_number = 1
thresh_image_number = 1

target_x, target_y = 0, 0


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
    cv2.imwrite('src/exercise7/scripts/kvadrati/thresh_{}.png'.
                    format(thresh_image_number), thresh)
    thresh_image_number += 1

    # opening and closing
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    # Find contours and filter using threshold area
    cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    global image_number
    global target_x
    global target_y
    min_area = 3000
    best_ratio_dif = 1000
    got_square = False
    for c in cnts:
        area = cv2.contourArea(c)
        if area > min_area:
            x,y,w,h = cv2.boundingRect(c)
            print("-----", x, ";", y, ";", w, ";", h)
            
            if h > w:
                ratio = w/h
            else:
                ratio = h/w

            if w >= 200 and h >= 200 and 0.60 < ratio < 1.67:
                got_square = True
                print("===> got a square")
                cv2.imwrite('src/exercise7/scripts/kvadrati/thresh_used_{}.png'.
                    format(thresh_image_number), thresh)

                # ROI = image[y:y+h, x:x+w]
                cv2.rectangle(image, (x, y), (x + w, y + h), (36,255,12), 2)
                cv2.imwrite('/home/subic/ROS/src/exercise7/scripts/kvadrati/{}.png'.
                    format(image_number), image)
                image_number += 1

                center_x, center_y = w/2+x, h/2+y
                print("center of the square is: ", center_x, center_y)

                if abs(1-ratio) < best_ratio_dif:
                    best_ratio_dif = abs(1-ratio)
                    target_x, target_y = int(center_x), int(center_y)
                    

                # parking position is current position (feedback) + 
                # + transformacija sredi????a kvadrata iz slike v pozicijo
    
    if got_square:
        cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        cancel_msg = GoalID()
        cancel_publisher.publish(cancel_msg)
        move_to_parking()


# za premikanje do centra kvadrata (v parking)
def move_to_parking():
    # premikanje s twist messagi
    twist_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    
    offset_to_turtlebot = 50    # parameter za ocenit, razdalja od turtlebota
    triangle_a = abs(320-target_x)
    triangle_b = 480 - target_y + offset_to_turtlebot
    triangle_c = math.sqrt(triangle_b**2 + triangle_a**2)
    angle = math.atan(triangle_a/triangle_b) * 0.75
    # if target_x <= 320:
    #     angle = -1 * angle

    print("a=", triangle_a, "b=", triangle_b, "c=", triangle_c, "angle=", angle)
    
    # obrat v smer
    twist_msg = Twist()
    if target_x <= 320:
        angular_speed = 0.08
    else:
        angular_speed = -0.08
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = angular_speed
    twist_publisher.publish(twist_msg)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while(current_angle < angle):
        twist_publisher.publish(twist_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = abs(angular_speed)*(t1-t0)
    
    twist_msg.angular.z = 0
    twist_publisher.publish(twist_msg)

    # premik naprej proti ??eljeni to??ki
    linear_speed = 0.12
    twist_msg.linear.x = linear_speed
    twist_msg.angular.z = 0

    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance < triangle_c/500):
        twist_publisher.publish(twist_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = linear_speed*(t1-t0)

    twist_msg.linear.x = 0
    twist_publisher.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node('parking_node')

    # goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)
    # square_marker_pub = rospy.Publisher('/square_markers', MarkerArray, queue_size=1000)

    feedback_pub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callback=add_feedback, queue_size=1)

    ground_img_sub = rospy.Subscriber('/arm_camera/rgb/image_raw', Image, callback=add_image, queue_size=1)

    rospy.sleep(1)
    
    r = rospy.Rate(2)
    
    # da zbri??em slike v mapi iz prej??njega poganjanja
    files = glob('src/exercise7/scripts/kvadrati/*')
    for f in files:
        os.remove(f)

    # cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
    # cancel_msg = GoalID()
    # cancel_publisher.publish(cancel_msg)

    while not rospy.is_shutdown():
        find_parking_space(ground_image)
        r.sleep()
