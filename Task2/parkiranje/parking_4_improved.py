#!/usr/bin/python3

# TA VERZIJA NAJ SE UPORABLJA!

from glob import glob
import os
import math

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from move_base_msgs.msg import MoveBaseActionFeedback
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String


image_width = 640
image_height = 480

last_feedback = MoveBaseActionFeedback()
parking_feedbacks = []

ground_image = Image()

image_number = 1
thresh_image_number = 1

start_parking = False


def add_feedback(msg):
    global last_feedback
    last_feedback = msg


def add_image(msg):
    global ground_image
    ground_image = msg
    
    
def parking_status(msg):
    global start_parking
    start_parking = True


def find_parking_space(img):
    # povezano s subscriberjem za začetek iskanja parkinga
    global start_parking
    if not start_parking:
        print("waiting for parking ...")
        return
    
    bridge = CvBridge()

    image = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
    global thresh_image_number

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # cv2.imwrite('parking/x_gray_{}.png'.format(thresh_image_number), gray)
    
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV)[1]
    print("delam sliko")
    # cv2.imwrite('src/exercise7/scripts/kvadrati/thresh_{}.png'.
    #                 format(thresh_image_number), thresh)
    thresh_image_number += 1

    # opening and closing
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    # iskanje rdeče pike
    img_hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask = mask0+mask1
    red_img = close.copy()
    red_img[np.where(mask==0)] = 0
    red_img[np.where(mask!=0)] = 150

    merged_img = merge_images(red_img, close)
    # cv2.imwrite('src/exercise7/scripts/kvadrati//x_red_{}.png'
    #          .format(thresh_image_number), red_img)
    # cv2.imwrite('src/exercise7/scripts/kvadrati/merged_{}.png'
    #          .format(thresh_image_number), merged_img)

    # če je rdeča pika na sliki
    if 150 in merged_img:
        red_dot_center = mean_point(merged_img, 150)
        print(red_dot_center)

        # poiščemo najbližjo točko roba parkirišča
        offset = 25
        if offset*3 <= red_dot_center[0] < image_height-offset*3 and \
            offset*3 <= red_dot_center[1] < image_width-offset*3:
            parking_point_e = closest_parking_point_e(red_dot_center, merged_img, offset)
            parking_point_m = closest_parking_point_m(red_dot_center, merged_img, offset)
            
            parking_point = (int((parking_point_e[0]+parking_point_e[0])/2), 
                             int((parking_point_m[1]+parking_point_m[1])/2))
            
            if parking_point_e[0] >= 0 and parking_point_e[1] >= 0 and \
                    parking_point_m[0] >= 0 and parking_point_m[1] >= 0:
                
                # izračunaj točko za robota
                vektor = (red_dot_center[0]-parking_point[0], 
                            red_dot_center[1]-parking_point[1])
                # dist_between_points = euclidean_dist(red_dot_center, parking_point)
                target_y = int(8*vektor[0] + red_dot_center[0])
                target_x = int(8*vektor[1] + red_dot_center[1])

                print(parking_point, "--> (", target_x, ",", target_y, ")")
                # merged_rgb = cv2.cvtColor(merged_img, cv2.COLOR_GRAY2RGB)
                # merged_rgb[parking_point[0], :, 2] = 255
                # merged_rgb[:, parking_point[1], 2] = 255
                # merged_rgb[red_dot_center[0], :, 0] = 255
                # merged_rgb[:, red_dot_center[1], 0] = 255
                # target_y_plot = target_y if target_y < image_height else image_height-1
                # target_x_plot = target_x if target_x < image_width else image_width-1
                # merged_rgb[target_y_plot, :, 1] = 255
                # merged_rgb[:, target_x_plot, 1] = 255
                # cv2.imwrite('src/exercise7/scripts/kvadrati/merged_rgb_{}.png'
                #         .format(thresh_image_number), merged_rgb)

                # da se premakneš proti točki
                cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
                cancel_msg = GoalID()
                cancel_publisher.publish(cancel_msg)
                move_to_parking(target_y, target_x)


# za premikanje do centra kvadrata (v parking)
def move_to_parking(target_y, target_x):
    twist_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    end_parking_publisher = rospy.Publisher('end_parking', String, queue_size=10)
    
    offset_to_turtlebot = 70    # parameter za ocenit, razdalja od turtlebota
    triangle_a = abs(320-target_x)
    triangle_b = 480 - target_y + offset_to_turtlebot
    triangle_c = math.sqrt(triangle_b**2 + triangle_a**2)
    angle = math.atan(triangle_a/triangle_b) * 0.80

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

    # premik naprej proti željeni točki
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
    
    # ko končamo parkiranje
    end_parking_publisher.publish("finished")

# za združevanje slike parkirišča in rdeče pike
def merge_images(red_dot, square):
    image = np.copy(square)
    image[np.where(red_dot==150)] = 150
    return image


# za računanje središča rdeče pike
def mean_point(image, color_value):
    coordinates = np.where(image == color_value)
    # print(coordinates)
    avg_y = int(np.average(coordinates[0]))
    avg_x = int(np.average(coordinates[1]))
    return (avg_y, avg_x)


# za računanje najbližjega dela roba parkirišča rdeči piki (evklidska razdalja)
def closest_parking_point_e(red_dot_coordinates, image, offset=30):
    start_y = red_dot_coordinates[0] - offset
    start_x = red_dot_coordinates[1] - offset

    min_dist = 1000000
    best_coordinates = (-1, -1)
    for i in range(start_y, start_y+offset*2):
        for j in range(start_x, start_x+offset*2):
            if image[i, j] == 255:
                distance_to_point = euclidean_dist((i,j), red_dot_coordinates)
                if distance_to_point < min_dist:
                    min_dist = distance_to_point
                    best_coordinates = (i, j)

    return best_coordinates


# za računanje najbližjega dela roba parkirišča rdeči piki (manhattanska razdalja)
def closest_parking_point_m(red_dot_coordinates, image, offset=30):
    start_y = red_dot_coordinates[0] - offset
    start_x = red_dot_coordinates[1] - offset

    min_dist = 1000000
    best_coordinates = (-1, -1)
    for i in range(start_y, start_y+offset*2):
        for j in range(start_x, start_x+offset*2):
            if image[i, j] == 255:
                distance_to_point = manhattan_dist((i,j), red_dot_coordinates)
                if distance_to_point < min_dist:
                    min_dist = distance_to_point
                    best_coordinates = (i, j)

    return best_coordinates


def euclidean_dist(a, b):
    dist = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    return dist


def manhattan_dist(a, b):
    dist = abs(a[0]-b[0]) + abs(a[1]-b[1])
    return dist


if __name__ == '__main__':
    rospy.init_node('parking_node')

    feedback_pub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callback=add_feedback, queue_size=1)
    ground_img_sub = rospy.Subscriber('/arm_camera/rgb/image_raw', Image, callback=add_image, queue_size=1)
    start_parking_sub = rospy.Subscriber('start_parking', String, callback=parking_status, queue_size=1)
    
    rospy.sleep(1)
    r = rospy.Rate(4)
    
    # da zbrišem slike v mapi iz prejšnjega poganjanja
    files = glob('src/exercise7/scripts/kvadrati/*')
    for f in files:
        os.remove(f)
    
    while not rospy.is_shutdown():
        find_parking_space(ground_image)
        r.sleep()
