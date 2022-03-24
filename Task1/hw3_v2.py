#!/usr/bin/python3

import sys
import rospy

import geometry_msgs.msg as gmsg
import time
import actionlib_msgs.msg as amsg
from std_msgs.msg import String

from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseActionFeedback


# TODO točke za originalne pozicije slik!
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

faces = []
checked_faces = 0

last_feedback = MoveBaseActionFeedback()
face_feedbacks = []


def info(msg):
    global status_array
    status_array = msg.status_list


def add_face(msg):
    global faces
    faces.append(msg)
    # print(msg)
    rospy.loginfo("-----------------faca dodana----------------------")
    
    # dodajanje položaja robota v trenutku ko smo zaznali faco
    global last_feedback
    global face_feedbacks
    face_feedbacks.append(last_feedback.feedback.base_position.pose)
    print("=> face detected from: ", last_feedback.feedback.base_position.pose.position.x, 
        ",", last_feedback.feedback.base_position.pose.position.y, ")")


def add_feedback(msg):
    global last_feedback
    last_feedback = msg


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
    
    # če smo v tem času zaznali kakšno faco
    global checked_faces
    print("face: (", checked_faces, "/", len(faces), ")")
    moved_to_goal = False
    while checked_faces < len(faces):
        move_to_face(checked_faces)
        moved_to_goal = True
        rospy.logwarn("POZDRAVLJEN, OBRAZ!")

        # TODO za zvok (insert code here)

        # TODO
        
        rospy.sleep(2.0)
        checked_faces += 1

    # TODO program se konča, ko obiščemo X število fac
    if checked_faces >= 1:
        rospy.logwarn("KONEC, OBISKALI SMO KONCNO STEVILO OBRAZOV!")
        sys.exit()
    
    elif moved_to_goal:
        move_robot(ind)


def move_to_face(face_ind):
    rospy.loginfo("gremo do face")

    xm, ym, direction = where_to_move(face_ind)
    print("----------> gremo na ", xm, ",", ym, ".")

    goal = gmsg.PoseStamped()

    goal.header.frame_id = "map"
    goal.pose.orientation.z = direction
    goal.pose.position.x = xm   #1.3
    goal.pose.position.y = ym   #1.0
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
            break
        else:
            rospy.loginfo(status)
        elapsed = time.time() - now
        time.sleep(1.-elapsed)


def where_to_move(face_ind):
    global faces
    global face_feedbacks
    
    robot_position = face_feedbacks[face_ind]

    face_position = faces[face_ind].markers[face_ind].pose.position
    print("lokacija face: ", face_position.x, ",", face_position.y, ")")
    
    xd = face_position.x - robot_position.position.x
    yd = face_position.y - robot_position.position.y
    direction = robot_position.orientation.z
    
    # TODO po potrebi se tole lahko spreminja ampak men velik bol deluje z 0.4
    safety_distance = 0.4

    if xd > 0:
        xm = robot_position.position.x + xd - safety_distance
        if xm < robot_position.position.x:
            xm = robot_position.position.x
    else:
        xm = robot_position.position.x + xd + safety_distance
        if xm > robot_position.position.x:
            xm = robot_position.position.x

    if yd > 0:
        ym = robot_position.position.y + yd - safety_distance
        if ym < robot_position.position.y:
            ym = robot_position.position.y
    else:
        ym = robot_position.position.y + yd + safety_distance
        if ym > robot_position.position.y:
            ym = robot_position.position.y

    return xm, ym, direction


 
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


if __name__ == '__main__':
    
    rospy.init_node('hw3_node')

    goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)

    status_sub = rospy.Subscriber('/move_base/status', amsg.GoalStatusArray, callback=info, queue_size=10)

    pub = rospy.Publisher('chatter', String, queue_size=1)

    face_marker_pub = rospy.Subscriber('/face_markers', MarkerArray, callback=add_face, queue_size=10)

    feedback_pub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callback=add_feedback, queue_size=1)

    rospy.sleep(1)
    
    r = rospy.Rate(1)
    ind = 0
    
    while not rospy.is_shutdown():
        if ind >= len(goals_array):
            pub.publish("I am done.")
            break

        move_robot(ind)
        ind += 1 
            
        r.sleep()
        