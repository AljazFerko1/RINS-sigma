#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import math
import itertools as it

from os.path import dirname, join

import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from task1.msg import Message
from test import food_classification

from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter



class food_recognizer:
    def __init__(self):
        rospy.init_node('food_recognizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        #self.face_detector = dlib.get_frontal_face_detector()
        #protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        #modelPath = join(dirname(__file__), "res10_300x300_ssd_iter_140000.caffemodel")
        """protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__), "foodero_model_1.pt")

        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)
        """
        # A help variable for holding the dimensions of the image
        
        self.pub = rospy.Publisher("found_food", String, queue_size=10)
        
        self.dims = (0, 0, 0)
        
        self.food_cnt = 0
        
        self.tries = 0

        # Subscribe to the image and/or depth topic
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)


        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        
    def find_food(self):
        global start
        
        
        print('I got a new image!')

        # Get the next rgb and depth images that are posted from the camera
        try:
            rgb_image_message = rospy.wait_for_message("/arm_camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        #cv2.imshow(rgb_image_message)
        #rgb_image_message.show()

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        """img_path = "/home/domen/ROS/src/task1/scripts/foods/food1_" + str(self.food_cnt) + ".jpg"
        cv2.imwrite(img_path, rgb_image)
        food_name = food_classification(img_path)
        self.pub.publish(food_name)
        self.food_cnt += 1
        start = False"""
            
        self.tries += 1
        if self.tries > 20:
            img_path = "/home/domen/ROS/src/task1/scripts/foods/food_" + str(self.food_cnt) + ".jpg"
            cv2.imwrite(img_path, rgb_image)
            food_name = food_classification(img_path)
            self.pub.publish(food_name)
            self.food_cnt += 1
            self.tries = 0
            start = False
            return
        
        #gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        
        img = cv2.equalizeHist(cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY))
        rgb_img = np.copy(rgb_image)
        # Binarize the image, there are different ways to do it
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 7, 25) #def 15,25

        # Extract contours
        cnts, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        #cnts, hier = cv2.findContours(gray, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        
        filtered_cnts = []
        for cnt in cnts:
            rect = cv2.minAreaRect(cnt)
            aspect_ratio = 0
            if rect[1][1] > 0:
                aspect_ratio = float(rect[1][0])/rect[1][1]
            area = cv2.contourArea(cnt)
            #k = cv2.isContourConvex(cnt)
            #print(k)
            #print(area)
            if 700 < area < 1500:
                print(aspect_ratio)
            if 0.25 < aspect_ratio < 4 and 700 < area < 1500:
                x,y,w,h = cv2.boundingRect(cnt)
                filtered_cnts.append(cnt)
                img_path = "/home/domen/ROS/src/task1/scripts/foods/food_" + str(self.food_cnt) + ".jpg"
                cv2.imwrite(img_path, rgb_img[y:y+h,x:x+w])
                food_name = food_classification(img_path)
                self.pub.publish(food_name)
                self.food_cnt += 1
                start = False
        
        #print("Velikost: ", len(filtered_cnts))

        # Example how to draw the contours, only for visualization purposes
        cv2.drawContours(img, cnts, -1, (255, 0, 0), 3)
        
        #cv2.imshow("Contour window",img)
        #cv2.waitKey(1)

        # Fit elipses to all extracted contours
        elps = [cv2.fitEllipse(cnt) for cnt in filtered_cnts if cnt.shape[0] >= 5]
        
        for el in elps:
            cv2.ellipse(rgb_img, el, (0, 255, 0), 2)
            
        cv2.imshow("elipsa", rgb_img)
        cv2.waitKey(1)
        
    
start = False
              
def signal(msg):   
    global start      
    start = True  
          
                    
def main():

    food_recognition = food_recognizer()
    
    rospy.Subscriber("find_food", String, callback=signal)
    global start

    rate = rospy.Rate(10)
    

    while not rospy.is_shutdown():
        if start:
            food_recognition.find_food()

        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()