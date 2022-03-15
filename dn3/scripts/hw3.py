#!/usr/bin/python3

import rospy

import geometry_msgs.msg as gmsg
import time
import actionlib_msgs.msg as amsg


goals_array=[(1.710587501525879,-0.9444425106048584),
            (2.404578685760498, 2.5176405906677246),
            (-0.10917353630065918, 2.832955837249756),
            (-1.352656364440918, 0.38173580169677734),
            (3.868697166442871, -0.588979959487915)]  

status_array = []
                              
def info(msg):
    global status_array
    status_array = msg.status_list
    
def move_robot(ind):
    goal = gmsg.PoseStamped()

    goal.header.frame_id = "map"
    goal.pose.orientation.w = 1
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
            break
        else:
            rospy.loginfo(status)
        elapsed = time.time() - now
        time.sleep(1.-elapsed) 

if __name__ == '__main__':
    
    rospy.init_node('hw3_node')

    goal_pub = rospy.Publisher('/move_base_simple/goal', gmsg.PoseStamped, queue_size = 10)

    status_sub = rospy.Subscriber('/move_base/status', amsg.GoalStatusArray, callback=info, queue_size=10)

    rospy.sleep(1)
    
    r = rospy.Rate(1)
    ind = 0
        
    while not rospy.is_shutdown():
        if ind >= len(goals_array):
            break

        move_robot(ind)
        ind += 1 
            
        r.sleep()
    