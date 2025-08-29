#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal_1;
    move_base_msgs::MoveBaseGoal goal_2;
    move_base_msgs::MoveBaseGoal goal_3;
    move_base_msgs::MoveBaseGoal goal_4;
    move_base_msgs::MoveBaseGoal goal_5;

    goal_1.target_pose.header.frame_id = "map"; goal_1.target_pose.pose.position.x=-1.0; goal_1.target_pose.pose.position.y= -2.0; goal_1.target_pose.pose.orientation.w= 1.0;
    goal_2.target_pose.header.frame_id = "map"; goal_2.target_pose.pose.position.x= 1.8; goal_2.target_pose.pose.position.y= -2.1; goal_2.target_pose.pose.orientation.w=  1.0;
    goal_3.target_pose.header.frame_id = "map"; goal_3.target_pose.pose.position.x= 1.7; goal_3.target_pose.pose.position.y=  2.0; goal_3.target_pose.pose.orientation.w=  1.0;
    goal_4.target_pose.header.frame_id = "map"; goal_4.target_pose.pose.position.x= 0.0; goal_4.target_pose.pose.position.y=  2.5; goal_4.target_pose.pose.orientation.w=  1.0;
    goal_5.target_pose.header.frame_id = "map"; goal_5.target_pose.pose.position.x=2.0; goal_5.target_pose.pose.position.y= -2.0; goal_5.target_pose.pose.orientation.w= 1.0;


    move_base_msgs::MoveBaseGoal goal_list [5] = {goal_1, goal_2, goal_4, goal_3, goal_5};

    for(int i=0; i<5; i++)
    {
        goal_list[i].target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal %d", i);
        ac.sendGoal(goal_list[i]);
        
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Mir moved to (%f , %f)", goal_list[i].target_pose.pose.position.x, goal_list[i].target_pose.pose.position.y);
        else
            ROS_INFO("Mir failed to achieve goal %d for some reason", i);
            // return 0;  
    }

    return 1;
}