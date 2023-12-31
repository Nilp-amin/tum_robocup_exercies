#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <vector>
#include <map>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tiago_move/controller.h>

// Point A: [-3.93, -1.73, -0.00125]
// Point B: [-3.72, -8.58, 0.0013]
// Point C: [-1.21, -12.1, -0.0019]

namespace tiago_move {
// Creates a typedef for a SimpleActionClient that communicates with actions that adhere to the MoveBaseAction action interface.
// typedef /*#>>>>TODO: ACTION CLIENT TYPE*/</*#>>>>TODO: ACTION NAME*/> MoveBaseClient;
  Controller::Controller()  : ac{"move_base", true}, body_planner_{"arm_torso"}
  {
  }

  Controller::~Controller()
  {
  }

  bool Controller::initialize(ros::NodeHandle& nh)
  {
    /*#>>>>TODO:Exercise3 wait for the action server to connet*/
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    /*#>>>>TODO:Exercise3 Load the 3D coordinates of three waypoints from the ROS parameter server*/
    std::vector<double> waypoint_a;
    std::vector<double> waypoint_ab;
    std::vector<double> waypoint_b;
    std::vector<double> waypoint_bc;
    std::vector<double> waypoint_c;
    std::vector<double> waypoint_ca;
    std::vector<double> waypoint_initial;
    if (!ros::param::get("waypoint_A", waypoint_a))
      return false;
    if (!ros::param::get("waypoint_AB", waypoint_ab))
      return false;
    if (!ros::param::get("waypoint_B", waypoint_b))
      return false;
    if (!ros::param::get("waypoint_BC", waypoint_bc))
      return false;
    if (!ros::param::get("waypoint_C", waypoint_c))
      return false;
    if (!ros::param::get("waypoint_CA", waypoint_ca))
      return false;
    if (!ros::param::get("waypoint_initial", waypoint_initial))
      return false;
    /*#>>>>TODO:Exercise3 Store three waypoints in the vector nav_goals*/
    nav_goals.push_back(createGoal(waypoint_a));
    nav_goals.push_back(createGoal(waypoint_ab));
    nav_goals.push_back(createGoal(waypoint_b));
    nav_goals.push_back(createGoal(waypoint_bc));
    nav_goals.push_back(createGoal(waypoint_c));
    nav_goals.push_back(createGoal(waypoint_ca));
    nav_goals.push_back(createGoal(waypoint_initial));


    //#>>>>TODO:Exercise4 Load the target pose from parameter
    if(!ros::param::get("target_pose", target_pose))
      return false; 
    if (!ros::param::get("home_pose", home_pose))
      return false;
    //#>>>>TODO:Exercise4 Set the planner of your MoveGroupInterface
    body_planner_.setPlannerId("RRTConnectkConfigDefault");
    //#>>>>TODO:Exercise4 Set the reference frame of your target pose 
    body_planner_.setPoseReferenceFrame("base_link"); // probably not map, and actually the arm reference frame
    return true;
  }


  move_base_msgs::MoveBaseGoal Controller::createGoal(std::vector<double> &goal)
  {
      move_base_msgs::MoveBaseGoal goal_msg;

      goal_msg.target_pose.header.frame_id = "map";//#>>>>TODO: the reference frame name
      goal_msg.target_pose.header.stamp = ros::Time::now();
      goal_msg.target_pose.pose.position.x = goal[0];
      goal_msg.target_pose.pose.position.y = goal[1];
      goal_msg.target_pose.pose.orientation.w = goal[2];

      return goal_msg;
  }

  // Uncomment the function for Exercise 4
  int Controller::move_arm(std::vector<double> &goal)
  {
    // input: std::vector<double> &goal [x,y,z,r,p,y]
    //#>>>>TODO:Exercise4 Create a msg of type geometry_msgs::PoseStamped from the input vector body_planner_
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "base_link";

    
    goal_pose.pose.position.x = goal[0];
    goal_pose.pose.position.y = goal[1];
    goal_pose.pose.position.z = goal[2];

    tf2::Quaternion quaternion;
    quaternion.setRPY(goal[3], goal[4], goal[5]);
    goal_pose.pose.orientation = tf2::toMsg(quaternion);

    //#>>>>TODO:Exercise4 Set the target pose for the planner setPoseTarget function of your MoveGroupInterface instance
    body_planner_.setPoseTarget(goal_pose);

    ROS_INFO_STREAM("Planning to move " << 
                    body_planner_.getEndEffectorLink() << " to a target pose expressed in " <<
                    body_planner_.getPlanningFrame());

    body_planner_.setStartStateToCurrentState();
    body_planner_.setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    //set maximum time to find a plan
    body_planner_.setPlanningTime(5.0);

    //#>>>>TODO:Exercise4 Start the planning by calling member function "plan" and pass the motion_plan as argument
    body_planner_.plan(motion_plan);
    
    ROS_INFO_STREAM("Plan found in " << motion_plan.planning_time_ << " seconds");
    //#>>>>TODO:Exercise4 Execute the plan by calling member function "move" of the MoveGroupInterface instance
    body_planner_.move();
    
    
    return EXIT_SUCCESS;
 }
}
int main(int argc, char** argv){
  ros::init(argc, argv, "tiago_move");
  ros::NodeHandle nh;
      
  ros::AsyncSpinner spinner(4);
  spinner.start();
  tiago_move::Controller controller;
  if(!controller.initialize(nh))
  {
      ROS_ERROR_STREAM("tiago_move::Controller failed to initialize");
      return -1;
  }

  int goal_index = 0;
  while (ros::ok())
  {
      ROS_INFO("Sending goal %d", goal_index + 1);

      //#>>>>TODO:Exercise3 send the current goal to the action server with sendGoal function of SimpleActionClient instace.
      controller.ac.sendGoal(controller.nav_goals[goal_index]);
      //#>>>>TODO:Exercise3 blocks until this goal finishes with waitForResult function of SimpleActionClient instace.
      controller.ac.waitForResult();
      actionlib::SimpleClientGoalState goal_state = controller.ac.getState();
      if (goal_state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
      {
          ROS_INFO("Reached goal %d", goal_index + 1);
          goal_index = (goal_index + 1) % controller.nav_goals.size();
      }
      else
      {
          ROS_WARN("Failed to reach goal %d", goal_index + 1);
          ros::Duration(1.0).sleep();
      }
      //#>>>>TODO:Exercise4 Call the move_arm function at propoer waypoint
      if (goal_index == 5)
      {
        controller.move_arm(controller.target_pose);
        controller.move_arm(controller.home_pose);
      }
  }
  spinner.stop();
  return 0;
}


