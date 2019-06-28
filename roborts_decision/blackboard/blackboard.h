/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

namespace roborts_decision{

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path){

    new_goal_ = false;

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Goal pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    goal_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    ros::NodeHandle nh;

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

  }

  ~Blackboard() = default;


  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

 private:
  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Goal info
  ros::Subscriber goal_sub_;
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;

};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
