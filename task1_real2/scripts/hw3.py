#!/usr/bin/python3

import rospy

import geometry_msgs.msg as gmsg
import time
import actionlib_msgs.msg as amsg
from std_msgs.msg import String
from task1.msg import Message
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction
from actionlib import simple_action_client


goals_array = [(-0.3466166853904724, 0.7900221347808838),
            (-1.4668368101119995, 2.1192233562469482),
            (0.585220217704773, 2.7978434562683105),
            (2.3926682472229004, 1.8668413162231445),
            (1.0462441444396973, 0.848088264465332),
            (3.7269110679626465, -0.20372676849365234),
            (2.03275990486145, -0.9131331443786621),
            (0.30317771434783936, -0.9778342247009277),
            (-1.1920931339263916, 0.2882213592529297)]

status_array = []
cur_x = 0
cur_y = 0
cur_z = 0
orient_z = 0
orient_w = 0
                              
def info(msg):
    global status_array
    status_array = msg.status_list
    
    
def move_robot(ind):
    goal = gmsg.PoseStamped()

    goal.header.frame_id = "map"
    goal.pose.orientation.z = -1
    goal.pose.position.x = goals_array[ind][0]
    goal.pose.position.y = goals_array[ind][1]
    goal.header.stamp = rospy.Time.now()
    
    goal_pub.publish(goal)
    
    while len(status_array) == 0:
        continue
        
    status = status_array[-1].text
    rospy.loginfo(status)
    while True:
        now = time.time() 
        status = status_array[-1].text         
        if status_array[-1].status == 4:
            rospy.logwarn(status)
            break
        elif status_array[-1].status == 3:
            rospy.loginfo(status)
            spin_robot(ind)
            break
        else:
            rospy.loginfo(status)
        elapsed = time.time() - now
        time.sleep(1.-elapsed) 
        
def spin_robot(ind):
    rotate = gmsg.PoseStamped()
    
    rotate.header.frame_id = "map"
    #rotate.pose.orientation.w = 0
    rotate.pose.position.x = goals_array[ind][0]
    rotate.pose.position.y = goals_array[ind][1]
    z = -0.9
    
    rospy.sleep(0.2)
    rospy.loginfo("Spining the robot.")
    while z < 1:
        rotate.pose.orientation.z = z
        rotate.pose.orientation.w = 1 - z ** 2
        rotate.header.stamp = rospy.Time.now()
        goal_pub.publish(rotate)
        rospy.sleep(0.75)
        z += 0.1
        
def new_face(data):
    if not math.isnan(data.x) and not math.isnan(data.y) and not math.isnan(data.z):
        vec = [data.x - cur_x, data.y - cur_y]
        approach = gmsg.PoseStamped()
    
        cancel = amsg.GoalID()
        goal_cancel.publish(cancel)
        
        approach.header.frame_id = "map"
        approach.pose.position.x = vec[0] * 0.9
        approach.pose.position.y = vec[1] * 0.9
        approach.pose.orientation.z = orient_z
        approach.pose.orientation.w = orient_w
        approach.header.stamp = rospy.Time.now()
        goal_pub.publish(approach)
        
        #print(approach)
        
        rospy.sleep(10)
    
def odometry(msg):
    global cur_x
    global cur_y
    global cur_z
    global orient_z
    global orient_w
    cur_x = msg.pose.pose.position.x
    cur_y = msg.pose.pose.position.y
    cur_z = msg.pose.pose.position.z
    orient_z = msg.pose.pose.orientation.z
    orient_w = msg.pose.pose.orientation.w
    

if __name__ == '__main__':
    
    rospy.init_node('hw3_node')

    goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)
    
    goal_cancel = rospy.Publisher('/move_base/cancel', amsg.GoalID, queue_size = 10)

    status_sub = rospy.Subscriber('/move_base/status', amsg.GoalStatusArray, callback=info, queue_size=10)

    #pub = rospy.Publisher('chatter', String, queue_size=1)
    rospy.Subscriber("task1_topic", Message, callback=new_face)
    
    rospy.Subscriber("/odom", Odometry, callback=odometry)

    rospy.sleep(1)
    
    r = rospy.Rate(1)
    ind = 0
        
    while not rospy.is_shutdown():
        if ind >= len(goals_array):
            #pub.publish("I am done.")
            break

        move_robot(ind)
        ind += 1 
            
        r.sleep()
        #rospy.spin()