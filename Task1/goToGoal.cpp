#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool goToPosition(){ //TODO ADD MSG FORMAT
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x =  //TODO FROM THE MSG READ DESIRED COORDINATES
    goal.target_pose.pose.position.y = 
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO_STREAM("Sending goal");
    //ROS_INFO_STREAM(allGoals[i][0]);
    //ROS_INFO_STREAM(allGoals[i][1]);
    ac.sendGoal(goal);

    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO_STREAM("Reached a goal");
    }
    else{
        ROS_INFO_STREAM("Failed to reach goal.");
    }
    return true;
}

MoveBaseClient ac;

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");
    ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO_STREAM("Waiting for move_base server");
    }

    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/moveService/move", goToPosition);

    ROS_INFO("I am waiting for movement goals");
    ros::spin();

    return 0;
}
