#!/usr/bin/python3

from http import client
from multiprocessing.connection import Client, wait
import rospy

import geometry_msgs.msg as gmsg
import time
import actionlib_msgs.msg as amsg
from std_msgs.msg import String
from exercise6.msg import Message
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import actionlib
import numpy as np
from sound_play.libsoundplay import SoundClient


status_array = []
vec = [None] * 2
check_rings = False
ring_points = Message()
ori = [None] * 2

last_feedback = gmsg.PoseWithCovarianceStamped()

face_cnt = 0
face_points = np.zeros((10, 2), dtype=float)
face_orientation = [None] * 10

cylinder_cnt = 0
cylinder_points = np.zeros((4, 2), dtype=float)
cylinder_orientation = [None] * 4
foods = [None] * 4
check_food = False
                              
def info(msg):
    global status_array
    status_array = msg.status_list
    
    
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
    
    goals_array = [(-0.402, 0.433),
                (0.5, -1.0),
                (3, -0.063),
                (2.5, 1.16),
                (1.30, 1.10),
                (1.00, 2.70),
                (-1.00, 2.14)] 
    
    global cylinder_cnt
    global face_cnt
    global check_rings
    global ring_points
    
    approached_cyl = 0
    approached_face = 0
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    for ind in range(len(goals_array)):
        goal = MoveBaseGoal()
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
                
            """if face_cnt > approached_face:
                move_to(approached_face, face_cnt, "face")
                approached_face = face_cnt"""
                
            if cylinder_cnt > approached_cyl:
                move_to(approached_cyl, cylinder_cnt, "cylinder")
                approached_cyl = cylinder_cnt
                
            
def move_to(min, max, object):
    global check_food
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    soundhandle = SoundClient()
    
    while min < max:
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        
        if object == "cylinder": 
            goal.target_pose.pose.position.x = cylinder_points[min][0]  
            goal.target_pose.pose.position.y = cylinder_points[min][1]
            goal.target_pose.pose.orientation.z = cylinder_orientation[min].z
            goal.target_pose.pose.orientation.w = cylinder_orientation[min].w
            rospy.loginfo("Approaching cylinder.")
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            client.wait_for_result()
            food_pub.publish("start")
            check_food = True
            while check_food:
                continue
            rospy.logwarn(foods[cylinder_cnt - 1])
            
            
        if object == "face": 
            goal.target_pose.pose.position.x = face_points[min][0]  
            goal.target_pose.pose.position.y = face_points[min][1]
            goal.target_pose.pose.orientation.z = face_orientation[min].z
            goal.target_pose.pose.orientation.w = face_orientation[min].w
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            rospy.loginfo("Approaching face.")
            client.wait_for_result()
        
        
        if object == "face":
            soundhandle.say('Hello face', 'voice_kal_diphone', 1.0)
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
    rospy.logwarn("Cylinder callback")
    global cylinder_points
    global cylinder_orientation
    global cylinder_cnt
    
    v1 = [msg.x - last_feedback.pose.pose.position.x, msg.y - last_feedback.pose.pose.position.y]
    vec1 = 1 - (2 / (np.linalg.norm(v1) * 5))
    
    cylinder_orientation[cylinder_cnt] = last_feedback.pose.pose.orientation
    cylinder_points[cylinder_cnt][0] = last_feedback.pose.pose.position.x + v1[0] * vec1
    cylinder_points[cylinder_cnt][1] = last_feedback.pose.pose.position.y + v1[1] * vec1
    #rospy.logerr(cylinder_orientation[cylinder_cnt])
    cylinder_cnt += 1
    
def face_position(msg):
    rospy.logwarn("Face callback") 
    global face_points
    global face_orientation
    global face_cnt
    
    v1 = [msg.x - last_feedback.pose.pose.position.x, msg.y - last_feedback.pose.pose.position.y]
    vec1 = 1 - (3 / (np.linalg.norm(v1) * 5))
    
    face_orientation[face_cnt] = last_feedback.pose.pose.orientation
    face_points[face_cnt][0] = last_feedback.pose.pose.position.x + v1[0] * vec1
    face_points[face_cnt][1] = last_feedback.pose.pose.position.y + v1[1] * vec1
    #rospy.logerr(cylinder_orientation[cylinder_cnt])
    face_cnt += 1
    
def food(msg):
    global foods
    global check_food
    check_food = False
    foods[cylinder_cnt - 1] = msg.data
    

if __name__ == '__main__':
    
    rospy.init_node('task3_move')
    
    goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)

    cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', gmsg.Twist, queue_size=10)
    
    finshed_pub = rospy.Publisher("chatter1", Message, queue_size=1)
    
    food_pub = rospy.Publisher("find_food", String, queue_size=1)
    
    rospy.Subscriber("found_food", String, callback=food)
    
    rospy.Subscriber("cylinders", Message, callback=cylinder)
    
    rospy.Subscriber("rings", Message, callback=rings)
    
    rospy.Subscriber('/amcl_pose', gmsg.PoseWithCovarianceStamped, callback=robot_position, queue_size=10)
    
    rospy.Subscriber('face_position', Message, callback=face_position, queue_size=10)
    
    rospy.Subscriber('/move_base/status', amsg.GoalStatusArray, callback=info, queue_size=10)
    
    
    rospy.sleep(1)
    
    move_robot()