/*
 * This file is part of the Robot Learning Lab stack at KIT
 *
 * TODO: clarify license and authorship with Udacity and their Pick and Place project
 */

#include <trajectory_sampler.h>

TrajectorySampler::TrajectorySampler(ros::NodeHandle nh)
  : move_group(PLANNING_GROUP)
{
  my_iiwa.init();

  // configure planner
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(10.0);

  // slow down movement of the robot
  move_group.setMaxVelocityScalingFactor(0.2);

  // temporary hardcode all the needed poses until parameter server is ready
  geometry_msgs::Pose home_bow, target_1, target_2;

  home_bow.position.x = 0.4;
  home_bow.position.y = 0.0;
  home_bow.position.z = 0.6;
  home_bow.orientation.x = 0.0;
  home_bow.orientation.y = 1.0;
  home_bow.orientation.z = 0.0;
  home_bow.orientation.w = 0.0;

  target_1.position.x = 0.5;
  target_1.position.y = -0.3;
  target_1.position.z = 0.3;
  target_1.orientation.x = 0.0;
  target_1.orientation.y = 1.0;
  target_1.orientation.z = 0.0;
  target_1.orientation.w = 0.0;

  target_2.position.x = 0.3;
  target_2.position.y = 0.3;
  target_2.position.z = 0.4;
  target_2.orientation.x = 0.0;
  target_2.orientation.y = 1.0;
  target_2.orientation.z = 0.0;
  target_2.orientation.w = 0.0;

  while (ros::ok()) {
      if (my_iiwa.getRobotIsConnected()) {

        // first move to home
        ROS_INFO("Moving to home");
        // moveToPose(home_bow);
        // set starting pose
        move_group.setStartStateToCurrentState();

        move_group.setNamedTarget("home_bow");
        success_plan = move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        ROS_INFO("Planning result: %s",
                 success_plan ? "SUCCEEDED" : "FAILED");

        if (success_plan) {
          success_plan = move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
          ROS_INFO("Plan execution result: %s",
                   success_plan ? "SUCCEEDED" : "FAILED");

          // stay a little a the target
          ros::Duration(3.0).sleep();
        }

        ROS_INFO("Moving to target 1");
        moveToPose(target_1);

        ROS_INFO("Moving to target 2");
        moveToPose(target_2);

      } else {
        ROS_WARN_STREAM("Robot is not connected...");
        ros::Duration(5.0).sleep(); // 5 seconds
    }
  }
}

void TrajectorySampler::moveToPose(const geometry_msgs::Pose target_pose)
{
  // set starting pose
  move_group.setStartStateToCurrentState();

  move_group.setPoseTarget(target_pose);
  success_plan = move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO("Planning result: %s",
           success_plan ? "SUCCEEDED" : "FAILED");

  if (success_plan) {
    success_plan = move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("Plan execution result: %s",
             success_plan ? "SUCCEEDED" : "FAILED");

    // stay a little a the target
    ros::Duration(3.0).sleep();
  }
}

TrajectorySampler::~TrajectorySampler() {}

int main (int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "trajectory_sampler");
  ros::NodeHandle nh;

  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  TrajectorySampler plan_sampler(nh);

  return 0;
}
