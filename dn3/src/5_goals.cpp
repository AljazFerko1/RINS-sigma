#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){


    double allGoals[5][2] = {{-1.0, 1.5}, {-0.7, -0.4}, {2.9, -1.15}, {2.4, 2.4}, {0.6, 1.0}};

    ros::init(argc, argv, "simple_navigation_goals");

    MoveBaseClient ac("move_base", true);


    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO_STREAM("Waiting for move_base server");
    }

    for(int i = 0; i < 5; i++){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();


        goal.target_pose.pose.position.x = allGoals[i][0];
        goal.target_pose.pose.position.y = allGoals[i][1];
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
    }
    ROS_INFO_STREAM("Finished trying all 5 goals");

    return 0;
}
