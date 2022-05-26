#!/usr/bin/python3

from http import client
from multiprocessing.connection import Client, wait
import queue
import rospy
import re
import cv2
import sys
import roslib

import geometry_msgs.msg as gmsg
import time
import actionlib_msgs.msg as amsg
from std_msgs.msg import String
from exercise6.msg import Message
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import actionlib
import numpy as np
from sound_play.libsoundplay import SoundClient
from math import atan2
from tf.transformations import euler_from_quaternion
import pyzbar.pyzbar as pyzbar
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import ColorRGBA
import speech_recognition as sr


status_array = []
vec = [None] * 2
check_rings = False
ring_points = Message()
ori = [None] * 2

last_feedback = gmsg.PoseWithCovarianceStamped()

face_cnt = 0
faces_dict = {}

cylinder_cnt = 0
cylinder_points = np.zeros((4, 2), dtype=float)
cylinder_orientation = [None] * 4
foods = [None] * 4
check_food = False

#params = cv2.aruco.DetectorParameters_create()
#params.adaptiveThreshConstant = 25
adaptiveThreshWinSizeStep = 2


def info(msg):
    global status_array
    status_array = msg.status_list


class QRExtractor:
    def __init__(self):
        #rospy.init_node('image_converter', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()
        #self.extracted_data = False
        self.deliveries = {}
        # Subscribe to the image and/or depth topic
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback)

    def parse_data(self, data):
        data = re.sub(", ", ",", str(data)[2:-1])
        for delivery in data.split(","):
            name, food = delivery.split(" ")
            self.deliveries[name] = food
            #print(f"{name} : {food}")
        print(f"{self.deliveries = }")
        self.image_sub.unregister()
        #self.extracted_data = True

    def image_callback(self, data):
        # if self.extracted_data:
        #    print("Data already extracted...")
        #    return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

            # Find a QR code in the image
        decodedObjects = pyzbar.decode(cv_image)

        # print(decodedObjects)

        if len(decodedObjects) == 1:
            dObject = decodedObjects[0]
            print("Found 1 QR code in the image!")
            print("Data: ", dObject.data, '\n')
            self.parse_data(dObject.data)
            # Visualize the detected QR code in the image
            #points  = dObject.polygon
            # if len(points) > 4 :
            #    hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            #    hull = list(map(tuple, np.squeeze(hull)))
            # else:
            #    hull = points

            # Number of points in the convex hull
            #n = len(hull)

            # Draw the convext hull
            # for j in range(0,n):
            #    cv2.line(cv_image, hull[j], hull[ (j+1) % n], (0,255,0), 2)

            #cv2.imshow('Warped image',cv_image)
            # cv2.waitKey(1)

        elif len(decodedObjects) == 0:
            print("No QR code in the image")
        else:
            print("Found more than 1 QR code")


class SpeechTranscriber:
    def __init__(self):
        #rospy.init_node('speech_transcriber', anonymous=True)

        # The documentation is here: https://github.com/Uberi/speech_recognition

        # The main interface to the speech recognition engines
        self.sr = sr.Recognizer()

        # These are the methods that are available to us for recognition.
        # Please note that most of them use an internet connection and currently they are using
        # a default API user/pass, so there are restrictions on the number of requests we can make.
        # recognize_bing(): Microsoft Bing Speech
        # recognize_google(): Google Web Speech API
        # recognize_google_cloud(): Google Cloud Speech - requires installation of the google-cloud-speech package
        # recognize_houndify(): Houndify by SoundHound
        # recognize_ibm(): IBM Speech to Text
        # recognize_sphinx(): CMU Sphinx - requires installing PocketSphinx
        # recognize_wit(): Wit.ai

        # An interface to the default microphone
        self.mic = sr.Microphone()
        self.soundhandle = SoundClient()
        self.arm_pub = rospy.Publisher("/arm_command", String, queue_size=1)

        # You can get the list of available devices: sr.Microphone.list_microphone_names()
    # You can set the fault microphone like this: self. mic = sr.Microphone(device_index=3)
    # where the device_index is the position in the list from the first command.
    @staticmethod
    def pretty_str(text):
        resp = text + "."
        return resp.capitalize()

    def have_a_talk(self, food):
        self.soundhandle.say(f'Here is your {food}.', 'voice_kal_diphone', 1.0)
        print(f"R: Here is your {food}.")
        time.sleep(0.6)
        resp = self.recognize_speech()
        print(f"Y: {self.pretty_str(resp)}")
        self.soundhandle.say(f'Will you pay with cash or credit card?',
                             'voice_kal_diphone', 1.0)
        print(f"R: Will you pay with cash or credit card?")
        time.sleep(2)
        resp = self.recognize_speech()
        print(f"Y: {self.pretty_str(resp)}")
        if "card" in resp:
            self.arm_pub.publish("wave_right")
            print(f"I am waving right, because you decided to pay with a card.")
        elif "cash" in resp:
            self.arm_pub.publish("wave_left")
            print(f"I am waving left, because you decided to pay with cash.")
        else:
            print(
                f"I am not waving, because you did not decide to pay with a card or cash.")
        self.soundhandle.say(f"How satisfied were you with the service\
                                on the scale from 1 to 5?",
                             'voice_kal_diphone', 1.0)
        print("R: How satisfied were you with the service on the scale from 1 to 5?")
        time.sleep(3.2)
        resp = self.recognize_speech()
        try:
            ratting = [int(s) for s in resp if s.isdigit()][0]
            print(f"Y: {self.pretty_str(resp)}\nI extracted the rating {ratting}")
            self.soundhandle.say(f"Thank you for ratting me with {ratting}. Goodbye.",
                                 'voice_kal_diphone', 1.0)
            print(f"R: Thank you for ratting me with {ratting}. Goodbye.")
        except:
            got_ratting = False
            for idx, number in enumerate(["one", "two", "three", "four", "five"]):
                if number in resp:
                    ratting = idx + 1
                    print(
                        f"Y: {self.pretty_str(resp)}\nI extracted the rating {ratting}")
                    self.soundhandle.say(f"Thank you for ratting me with {ratting}. Goodbye.",
                                         'voice_kal_diphone', 1.0)
                    print(
                        f"R: Thank you for ratting me with {ratting}. Goodbye.")
                    got_ratting = True
                    break
            if not got_ratting:
                print(
                    f"Y: {self.pretty_str(resp)}, but I did not understand your ratting.")
                self.soundhandle.say(f"Thank you and goodbye.",
                                     'voice_kal_diphone', 1.0)
                print(f"R: Thank you and goodbye.")
        self.arm_pub.publish("wave_neutral")

    def recognize_speech(self):
        with self.mic as source:
            #print('Adjusting mic for ambient noise...')
            self.sr.adjust_for_ambient_noise(source)
            print('SPEAK NOW!')
            audio = self.sr.listen(source)

        #print('I am now processing the sounds you made.')
        recognized_text = ''
        try:
            recognized_text = self.sr.recognize_google(audio)
        except sr.RequestError as e:
            print('API is probably unavailable', e)
        except sr.UnknownValueError:
            print('Did not manage to recognize anything.')

        return recognized_text


def move_robot():
    """goals_array = [(-0.402, 0.433),
                    (-1.00, 2.14),
                    (1.00, 2.70),
                    (1.14, 1.00),
                    (2.20, 1.00),
                    (3.15, 0.00),
                    (1.08, -1.05)]"""

    """goals_array = [(-0.402, 0.433),
                    (1.08, -1.05),
                    (2.9, 0.63),
                    (2.5, 1.16),
                    (1.30, 1.10),
                    (1.00, 2.70),
                    (-1.00, 2.14)] """

    """goals_array = [(-0.402, 0.433),
                   (0.06, -0.74),
                   (1.49, -0.83),
                   (3.24, -0.26),
                   (2.36, 0.825),
                   (1.40, 1.03),
                   (1.05, 2.41),
                   (-0.887, 2.07)]"""
    
    """goals_array = [(-0.402, 0.433),
                   (-0.887, 2.07),
                   (1.05, 2.41),
                   (1.40, 1.03),
                   (2.36, 1.13),
                   (3.65, -0.50),
                   (1.40, -1.089),
                   (0.06, -0.74)] """
    
    goals_array = [(-0.402, 0.433),
                   (0.06, -0.74),
                   (1.40, -1.089),
                   (3.65, -0.50),
                   (2.36, 1.13),
                   (1.40, 1.03),
                   (1.05, 2.41),
                   (-0.887, 2.07)]

    global cylinder_cnt
    global face_cnt
    global check_rings
    global ring_points

    approached_cyl = 0
    approached_face = 0

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    arm_pub.publish("ring_detection")
    goal = MoveBaseGoal()

    for ind in range(len(goals_array)):
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.orientation.z = -1
        goal.target_pose.pose.position.x = goals_array[ind][0]
        goal.target_pose.pose.position.y = goals_array[ind][1]
        goal.target_pose.header.stamp = rospy.Time.now()

        client.send_goal(goal)
        rospy.loginfo("Sending goal.")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal reached.")

            twist_msg = gmsg.Twist()
            rospy.loginfo("Spining the robot.")
            start_time = rospy.Time.now().secs
            while rospy.Time.now().secs - start_time < 21:
                twist_msg.angular.z = 0.3
                cmd_vel.publish(twist_msg)

            if cylinder_cnt > approached_cyl:
                move_to(approached_cyl, cylinder_cnt, "cylinder")
                approached_cyl = cylinder_cnt

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.orientation.z = 0.809
    goal.target_pose.pose.orientation.w = 0.588
    goal.target_pose.pose.position.x = 2.2
    goal.target_pose.pose.position.y = 1.2
    goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    rospy.loginfo("Approaching parking space.")
    wait = client.wait_for_result()

    arm_pub.publish("parking")
    print("QR init")
    qre = QRExtractor()
    print("Sent parking command")
    start_park_pub.publish("Parking_for_qr")
    # TODO parking

    global food_dict, faces_dict
    while True:
        time.sleep(1)
        print("Waiting to be parked...")
        if parked:
            print("Speach init")
            st = SpeechTranscriber()
            parked = False
            while True:
                if len(qre.deliveries) > 0:
                    twist_msg.angular.z = 0.0
                    cmd_vel.publish(twist_msg)
                    break  
                twist_msg.angular.z = 0.3
                cmd_vel.publish(twist_msg)
            print(f"Parked! {qre.deliveries = }")
            for name, food in qre.deliveries.items():
                print(f"Now serving {name} with {food}.")
                print(f"{food_dict[food] = }")
                go_to_cylinder(food_dict[food])
                print(f"{faces_dict[name] = }")
                go_to_face(faces_dict[name])
                st.have_a_talk(food)
            break
    print("Sent parking command2")
    start_park_pub.publish("Parking_for_end")
    # TODO CALL PARKING AGAIN


def go_to_cylinder(loc_dict):
    print("Got req to move to food")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = loc_dict["pose"][0]
    goal.target_pose.pose.position.y = loc_dict["pose"][1]
    goal.target_pose.pose.orientation.z = loc_dict["ori"][0]
    goal.target_pose.pose.orientation.w = loc_dict["ori"][1]
    goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    client.wait_for_result()
    print("I think I am at the food.")


def go_to_face(face_loc):
    print("Got req to move to face")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = face_loc[0][0]
    goal.target_pose.pose.position.y = face_loc[0][1]
    goal.target_pose.pose.orientation.z = face_loc[1][0]
    goal.target_pose.pose.orientation.w = face_loc[1][1]
    goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    client.wait_for_result()
    print("I think I am at the face.")


def move_to(min, max, object, name=""):
    global check_food
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    soundhandle = SoundClient()

    while min < max:

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"

        if object == "cylinder":
            """arm_pub.publish("food_detection")   
            goal.target_pose.pose.position.x = cylinder_points[min][0]  
            goal.target_pose.pose.position.y = cylinder_points[min][1]
            #goal.target_pose.pose.orientation.z = cylinder_orientation[min].z
            #goal.target_pose.pose.orientation.w = cylinder_orientation[min].w
            goal.target_pose.pose.orientation.z = cylinder_orientation[min][0]
            goal.target_pose.pose.orientation.w = cylinder_orientation[min][1]
            rospy.loginfo("Approaching cylinder.")
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            client.wait_for_result()
            """
            arm_pub.publish("food_detection")
            rospy.loginfo("Approaching cylinder.")
            speed = gmsg.Twist()
            goal_1 = gmsg.Point()
            goal_1.x = cylinder_points[min][0]
            goal_1.y = cylinder_points[min][1]
            while True:
                inc_x = goal_1.x - last_feedback.pose.pose.position.x
                inc_y = goal_1.y - last_feedback.pose.pose.position.y

                angle_to_goal = atan2(inc_y, inc_x)

                (_, _, theta) = euler_from_quaternion(
                    [last_feedback.pose.pose.orientation.x, last_feedback.pose.pose.orientation.y, last_feedback.pose.pose.orientation.z, last_feedback.pose.pose.orientation.w])
                if abs(angle_to_goal - theta) > 0.1:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.2
                else:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    cmd_vel.publish(speed)
                    break

                cmd_vel.publish(speed)

            goal.target_pose.pose.position.x = goal_1.x
            goal.target_pose.pose.position.y = goal_1.y
            #goal.target_pose.pose.orientation.z = cylinder_orientation[min].z
            #goal.target_pose.pose.orientation.w = cylinder_orientation[min].w
            goal.target_pose.pose.orientation.z = last_feedback.pose.pose.orientation.z
            goal.target_pose.pose.orientation.w = last_feedback.pose.pose.orientation.w
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            client.wait_for_result()

            cylinder_orientation[min][0] = last_feedback.pose.pose.orientation.z
            cylinder_orientation[min][1] = last_feedback.pose.pose.orientation.w
            food_pub.publish("start")

            check_food = True
            while check_food:
                continue
            rospy.logwarn(foods[cylinder_cnt - 1])
            arm_pub.publish("ring_detection")

        if object == "face":
            goal.target_pose.pose.position.x = faces_dict[name][0][0]
            goal.target_pose.pose.position.y = faces_dict[name][0][1]
            goal.target_pose.pose.orientation.z = faces_dict[name][1][0]
            goal.target_pose.pose.orientation.w = faces_dict[name][1][1]
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            rospy.loginfo(f"Approaching {name}.")
            client.wait_for_result()

        """if object == "face":
            soundhandle.say('Hello face', 'voice_kal_diphone', 1.0)"""
        rospy.sleep(1)

        min += 1


def robot_position(msg):
    global last_feedback
    last_feedback = msg


def rings(msg):
    global ring_points
    global check_rings
    ring_points = msg
    check_rings = True


def cylinder(msg):
    print(msg)
    rospy.logwarn("Cylinder callback")
    global cylinder_points
    global cylinder_orientation
    global cylinder_cnt

    v1 = [msg.x - last_feedback.pose.pose.position.x,
          msg.y - last_feedback.pose.pose.position.y]
    vec1 = 1 - (2 / (np.linalg.norm(v1) * 5))

    #cylinder_orientation[cylinder_cnt] = last_feedback.pose.pose.orientation
    cylinder_orientation[cylinder_cnt] = [msg.ori_z, msg.ori_w]
    cylinder_points[cylinder_cnt][0] = last_feedback.pose.pose.position.x + v1[0] * vec1
    cylinder_points[cylinder_cnt][1] = last_feedback.pose.pose.position.y + v1[1] * vec1
    # rospy.logerr(cylinder_orientation[cylinder_cnt])
    cylinder_cnt += 1


def face_position(msg):
    rospy.logwarn(msg.name)
    global faces_dict
    global face_cnt

    v1 = [msg.x - last_feedback.pose.pose.position.x,
          msg.y - last_feedback.pose.pose.position.y]
    vec1 = 1 - (3 / (np.linalg.norm(v1) * 5))

    faces_dict[msg.name] = ((last_feedback.pose.pose.position.x + v1[0] * vec1, last_feedback.pose.pose.position.y +
                            v1[1] * vec1), (last_feedback.pose.pose.orientation.z, last_feedback.pose.pose.orientation.w))
    """face_orientation[face_cnt] = last_feedback.pose.pose.orientation
    face_points[face_cnt][0] = last_feedback.pose.pose.position.x + v1[0] * vec1
    face_points[face_cnt][1] = last_feedback.pose.pose.position.y + v1[1] * vec1"""
    # rospy.logerr(cylinder_orientation[cylinder_cnt])
    face_cnt += 1


parked = False


def end_parking(park_msg):
    global parked
    parked = True


food_cnt = 0
food_dict = {}


def food(msg):
    global foods, check_food, food_cnt, food_dict, cylinder_points, cylinder_orientation
    print(f"Got {str(msg.data)} saving to index {food_cnt}")
    check_food = False
    #foods[food_cnt] = msg.data
    food_dict[str(msg.data)] = {
        "pose": cylinder_points[food_cnt], "ori": cylinder_orientation[food_cnt]}
    print(f"{food_dict = }")
    food_cnt += 1


if __name__ == '__main__':

    rospy.init_node('task3_move')

    goal_pub = rospy.Publisher(
        '/move_base_simple/goal', gmsg.PoseStamped, queue_size=10)

    cmd_vel = rospy.Publisher(
        '/cmd_vel_mux/input/navi', gmsg.Twist, queue_size=1)

    finshed_pub = rospy.Publisher("chatter1", Message, queue_size=1)

    food_pub = rospy.Publisher("find_food", String, queue_size=1)

    arm_pub = rospy.Publisher("/arm_command", String, queue_size=1)

    start_park_pub = rospy.Publisher("start_parking", String, queue_size=1)

    rospy.Subscriber("found_food", String, callback=food)

    rospy.Subscriber("cylinders", Message, callback=cylinder)

    rospy.Subscriber("rings", Message, callback=rings)

    rospy.Subscriber('/amcl_pose', gmsg.PoseWithCovarianceStamped,
                     callback=robot_position, queue_size=10)

    rospy.Subscriber('face_position', Message,
                     callback=face_position, queue_size=10)

    rospy.Subscriber('/move_base/status', amsg.GoalStatusArray,
                     callback=info, queue_size=10)

    rospy.Subscriber('end_parking', String,
                     callback=end_parking, queue_size=10)

    rospy.sleep(1)

    move_robot()
