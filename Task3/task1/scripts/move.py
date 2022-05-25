#!/usr/bin/python3

from http import client
from multiprocessing.connection import Client, wait
import rospy

import geometry_msgs.msg as gmsg
import time
import actionlib_msgs.msg as amsg
from std_msgs.msg import String
from task1.msg import Message
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import actionlib
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import numpy as np

status_array = []
vec = [None] * 2
check = False
ori = [None] * 2

last_feedback = MoveBaseActionFeedback()
#last_feedback = Odometry()
face_feedbacks = []
                              
def info(msg):
    global status_array
    status_array = msg.status_list
    
    
def move_robot():
    
    """goals_array = [(-1.1920931339263916, 0.2882213592529297),
            (0.30317771434783936, -0.9778342247009277),
            (2.03275990486145, -0.9131331443786621),
            (3.7269110679626465, -0.20372676849365234),
            (1.0462441444396973, 0.848088264465332),
            (2.3926682472229004, 1.8668413162231445),
            (0.585220217704773, 2.7978434562683105),
            (-1.4668368101119995, 2.1192233562469482),
            (-0.3466166853904724, 0.7900221347808838)]"""
    
    """goals_array = [(0.458000, 0.950000), #real_map
                   (-0.407000, 0.247),
                    (1.27, -0.325),
                    (0.21, -1.19)]"""
    
    """goals_array = [(-0.376000, -1.260000),  #real_map1
                (0.582000, -0.414),
                (0.0484, 0.770),
                (-1.07, -0.117)]"""
    
    goals_array = [(0.125000, -0.316000),  #real_map1
            (1.0000, 0.297),
            (0.633, 1.650),
            (-0.518, 0.856)]

    
    cnt = 0
    soundhandle = SoundClient()
    global check
    
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
            
            """if check:
                v1 = [vec[0] - goals_array[ind][0], vec[1] - goals_array[ind][1]]
                vec1 = 1 - (3 / (np.linalg.norm(v1) * 5))
                goal.target_pose.pose.position.x = goals_array[ind][0] + v1[0] * vec1
                goal.target_pose.pose.position.y = goals_array[ind][1] + v1[1] * vec1
                rospy.loginfo(f"ko gre do face {ori =}")
                goal.target_pose.pose.orientation.z = ori[0]
                goal.target_pose.pose.orientation.w = ori[1]
                goal.target_pose.header.stamp = rospy.Time.now()
                rospy.loginfo("Approcahing face.")
                
                client.send_goal(goal)
                client.wait_for_result()
                
                rospy.loginfo("Hello face! :D")
                soundhandle.say('Hello', 'voice_kal_diphone', 1.0)
                rospy.sleep(1)
                
                cnt += 1
                if cnt == 4:
                    return
                check = False"""
            
            #rospy.loginfo(f"Predn se spina {last_feedback.pose.pose.orientation =}")
            
            """twist_msg = gmsg.Twist()
            rospy.loginfo("Spining the robot.")
            start_time = rospy.Time.now().secs
            while rospy.Time.now().secs - start_time < 24:
                twist_msg.angular.z = 0.3
                cmd_vel.publish(twist_msg)"""
                
            z = -0.98
            rospy.loginfo("Spining the robot.")
            while z < 1:
                goal.target_pose.pose.position.x = goals_array[ind][0]
                goal.target_pose.pose.position.y = goals_array[ind][1]
                goal.target_pose.pose.orientation.z = z
                goal.target_pose.pose.orientation.w = 1 - z ** 2
                goal.target_pose.header.stamp = rospy.Time.now()
                client.send_goal(goal)
                client.wait_for_result()
                
                #rospy.loginfo(f"Ko se spina {last_feedback.pose.pose.orientation =}")
                
                if check:
                    v1 = [vec[0] - goals_array[ind][0], vec[1] - goals_array[ind][1]]
                    vec1 = 1 - (3 / (np.linalg.norm(v1) * 5))
                    goal.target_pose.pose.position.x = goals_array[ind][0] + v1[0] * vec1
                    goal.target_pose.pose.position.y = goals_array[ind][1] + v1[1] * vec1
                    rospy.loginfo(f"ko gre do face {ori =}")
                    goal.target_pose.pose.orientation.z = ori[0] - 0.02
                    goal.target_pose.pose.orientation.w = 1 - (ori[0] - 0.02) ** 2
                    goal.target_pose.header.stamp = rospy.Time.now()
                    rospy.loginfo("Approcahing face.")
                    
                    client.send_goal(goal)
                    client.wait_for_result()

                    rospy.loginfo("Hello face! :D")
                    soundhandle.say('Hello', 'voice_kal_diphone', 1.0)
                    rospy.sleep(1)
                    
                    cnt += 1
                    if cnt == 4:
                        return
                    check = False
                    
                z += 0.03
            
        
        
def new_face(data):
    global ori
    global last_feedback
    ori = [last_feedback.feedback.base_position.pose.orientation.z, last_feedback.feedback.base_position.pose.orientation.w]
    #ori = [last_feedback.pose.pose.orientation.z , last_feedback.pose.pose.orientation.w]
    rospy.loginfo(f"v new_face {ori =}")
    if not math.isnan(data.x) and not math.isnan(data.y) and not math.isnan(data.z):
        global vec
        global check  
        check = True
        vec = [data.x, data.y]
        
        
def add_feedback(msg):
    global last_feedback
    last_feedback = msg 
    

if __name__ == '__main__':
    
    rospy.init_node('hw3_node')
    
    goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)

    status_sub = rospy.Subscriber('/move_base/status', amsg.GoalStatusArray, callback=info, queue_size=10)

    rospy.Subscriber("task1_topic", Message, callback=new_face)
    
    #rospy.Subscriber("/odom", Odometry, callback=odometry)
    
    end_pub = rospy.Publisher("chatter", String, queue_size=1)
    
    rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callback=add_feedback, queue_size=10)
    
    cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', gmsg.Twist, queue_size=10)
    
    rospy.sleep(3)
    
    move_robot()
    
    
    end_pub.publish("I am done")
