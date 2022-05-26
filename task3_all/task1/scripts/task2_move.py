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
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import numpy as np

status_array = []
vec = [None] * 2
check_cylinder = False
check_rings = False
ring_points = Message()
ori = [None] * 2

last_feedback = MoveBaseActionFeedback()
face_feedbacks = []
                              
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
    
    #goals_array = [(2.9, 0.63)]
    
    goals_array = [(-0.402, 0.433),
                    (1.08, -1.05),
                    (2.9, 0.63),
                    (2.5, 1.16),
                    (1.30, 1.10),
                    (1.00, 2.70),
                    (-1.00, 2.14)] 
    
    global check_cylinder
    global check_rings
    global ring_points
    
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
        #rospy.loginfo(client.get_goal_status_text)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            #rospy.loginfo(client.get_result())  
            rospy.loginfo("Goal reached.")
            
            twist_msg = gmsg.Twist()
            rospy.loginfo("Spining the robot.")
            start_time = rospy.Time.now().secs
            while rospy.Time.now().secs - start_time < 21:
                twist_msg.angular.z = 0.3
                cmd_vel.publish(twist_msg)
                
            if check_cylinder and check_rings:
                #xm, ym, direction = where_to_move(goals_array[ind])
                rospy.loginfo("Approaching green ring.")
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.pose.orientation.z = -1
                goal.target_pose.pose.position.x = ring_points.x + 0.8
                #goal.target_pose.pose.position.x = 1.9894442628092874 + 0.80
                goal.target_pose.pose.position.y = ring_points.y + 0.1
                #goal.target_pose.pose.position.y = -0.11611620124580124 + 0.1
                goal.target_pose.header.stamp = rospy.Time.now()
                
                client.send_goal(goal)
                client.wait_for_result()
                msg = Message()
                finshed_pub.publish(msg)
                return
            
def where_to_move(robot_position):
    
    xd = ring_points.x - robot_position[0]
    yd = ring_points.y - robot_position[1]
    direction = -1
    
    # TODO po potrebi se tole lahko spreminja ampak men velik bol deluje z 0.4
    safety_distance = 0.4

    if xd > 0:
        xm = robot_position[0] + xd - safety_distance
        if xm < robot_position[0]:
            xm = robot_position[0]
    else:
        xm = robot_position[0] + xd + safety_distance
        if xm > robot_position[0]:
            xm = robot_position[0]

    if yd > 0:
        ym = robot_position[1] + yd - safety_distance
        if ym < robot_position[1]:
            ym = robot_position[1]
    else:
        ym = robot_position[1] + yd + safety_distance
        if ym > robot_position[1]:
            ym = robot_position[1]

    return xm, ym, direction
            

def add_feedback(msg):
    global last_feedback
    last_feedback = msg
    
def rings(msg):
    global ring_points
    global check_rings
    ring_points = msg
    check_rings = True
    
def cylinder(msg):
    global check_cylinder
    check_cylinder = True
    

if __name__ == '__main__':
    
    rospy.init_node('task2_move')
    
    goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)

    status_sub = rospy.Subscriber('/move_base/status', amsg.GoalStatusArray, callback=info, queue_size=10)
    
    finshed_pub = rospy.Publisher("chatter1", Message, queue_size=1)
    
    rospy.Subscriber("chatter", Message, callback=cylinder)
    
    rospy.Subscriber("rings", Message, callback=rings)
    
    rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callback=add_feedback, queue_size=1)
    
    cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', gmsg.Twist, queue_size=10)
    
    rospy.sleep(3)
    
    move_robot()