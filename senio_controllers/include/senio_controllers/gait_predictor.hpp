/**
 * @file gait_predictor.hpp
 * @author mik-p
 * @brief predicting gait patterns for arthropod robots using current state and commanded inputs for pose and velocity
 * @version 0.1
 * @date 2023-11-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cartesian_trajectory_interpolation/cartesian_state.h>

#include <senio_controllers/gait_member.hpp>
#include <senio_controllers/gait_pattern.hpp>
#include <senio_controllers/gait_trajectory.hpp>

namespace senio_controllers
{

/**
 * @brief class to predict gait patterns for arthropod robots
 * using current state and commanded inputs for pose and velocity
 * keeps track of current state and predicts next best gait pattern and foot position
 * for a given set of inputs
 *
 */
class GaitPredictor : public GaitPatternBase
{
public:
  GaitPredictor(ros::NodeHandle& pnh)
  {
    // get the list of gait members
    std::vector<std::string> gait_members;
    pnh.getParam("gait_members", gait_members);

    _gait_members.resize(gait_members.size());

    // for each gait member
    for (const auto& member : gait_members)
    {
      // create a new gait member
      GaitMember gait_member;

      // add the leg to the gait member

      // add the gait member to the list of gait members
      _gait_members.push_back(gait_member);
    }
  }

  /**
   * @brief predicts next best gait pattern and foot position
   * for a given set of inputs
   *
   * @param[in] inputs commanded inputs for pose and velocity
   * @param[out] outputs predicted gait pattern and foot position
   */
  void predict(const geometry_msgs::Pose& input_pose, std::vector<ros_controllers_cartesian::CartesianState>& state)
  {
    // set the prediction for each gait member
    for (auto& leg : _gait_members)
    {
      const ros_controllers_cartesian::CartesianState predicted_state;
      leg.state(predicted_state);
    }
  }

  void predict(const geometry_msgs::Pose& input_pose, const geometry_msgs::Twist& input_twist,
               std::vector<ros_controllers_cartesian::CartesianState>& state)
  {
  }
};

}  // namespace senio_controllers
