/*
 * Author: Automatic Addison
 * Date: May 30, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * This ROS node sends the robot goals to move to a particular location on 
 * a map. I have configured this program to the map of my own apartment.
 *
 * 0 = EXIT PROCESS
 * 1 = E/V
 * 2 = Drop Point
 */
 
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
 
using namespace std;
 
// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
int main(int argc, char** argv){
   
  // Connect to ROS
  ros::init(argc, argv, "simple_navigation_goals");
 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
 
  // Wait for the action server to come up so that we can begin processing goals.
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
 
  int user_choice = 1;
  char choice_to_continue = 'Y';
  bool run = true;
     
  while(run) {
 
    // Ask the user where he wants the robot to go?
    cout << "\nWhere do you want the robot to go?" << endl;
    cout << "\n0 = EXIT PROCESS" << endl;
    cout << "\n1 = Origin" << endl;
    cout << "2 = Drop Point" << endl;
    cout << "3 = Turning Point" << endl;
    cout << "\nEnter a number: ";
    cin >> user_choice;
 
    // Create a new goal to send to move_base 
    move_base_msgs::MoveBaseGoal goal;
 
    // Send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
         
    bool valid_selection = true;
 
    // Use map_server to load the map of the environment on the /map topic. 
    // Launch RViz and click the Publish Point button in RViz to 
    // display the coordinates to the /clicked_point topic.
    switch (user_choice) {
      case 0:
        cout << "\n------------!!!!!EXIT!!!!-------------\n" << endl;
        run = false;
        break;
      case 1:
        cout << "\nGoal Location: E/V\n" << endl;
    //     goal.target_pose.pose.position.x = -0.96;
    // goal.target_pose.pose.position.y = 0.0;
    //     goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.position.x = -0.082;
        goal.target_pose.pose.position.y = -0.023;
            goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 2:
        cout << "\nGoal Location: Drop Point\n" << endl;
    //     goal.target_pose.pose.position.x = 1.6;
    // goal.target_pose.pose.position.y = 0.84;
    //     goal.target_pose.pose.orientation.w = 1.0;
           goal.target_pose.pose.position.x = 0.768;
        goal.target_pose.pose.position.y = -0.017;
            goal.target_pose.pose.orientation.w = 1.0;
        break;
      case 3:
         cout << "\nGoal Location: Turn Point\n" << endl;
    //      goal.target_pose.pose.position.x = 0.23;
    //  goal.target_pose.pose.position.y = 1.48;
    //     goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.position.x = 0.752; 
        goal.target_pose.pose.position.y = 0.859;
            goal.target_pose.pose.orientation.w = 1.0;
         break;
      
      default:
        cout << "\nInvalid selection. Please try again.\n" << endl;
        valid_selection = false;
    }       
         
    // Go back to beginning if the selection is invalid.
    if(!valid_selection) {
      continue;
    }
  
    // move_base 토픽 전송
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
 
    // 전송한 토픽 결과 대기
    ac.waitForResult();

    // 로봇 액션 결과 확인
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot has arrived at the goal location");
    else
      ROS_INFO("The robot failed to reach the goal location for some reason");
         
    // Ask the user if he wants to continue giving goals
    // do {
    //   cout << "\nWould you like to go to another destination? (Y/N)" << endl;
    //   cin >> choice_to_continue;
    //   choice_to_continue = tolower(choice_to_continue); // Put your letter to its lower case
    // } while (choice_to_continue != 'n' && choice_to_continue != 'y'); 
 
    // if(choice_to_continue =='n') {
    //     run = false;
    // }  
  }
   
  return 0;
}
