#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "example_behavior/goal_behavior.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  roborts_decision::GoalBehavior goal_behavior(chassis_executor, blackboard);
  ros::Rate rate(10);

  std::cout << "Behavior Test (goal_behavior) started !" << std::endl;

  while(ros::ok()){
    ros::spinOnce();
    goal_behavior.Run();
    rate.sleep();
  }


  return 0;
}




