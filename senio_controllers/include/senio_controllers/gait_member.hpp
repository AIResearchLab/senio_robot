/**
 * @file gait_member.hpp
 * @author mik-p
 * @brief a gait member is a single leg in a gait pattern
 * @version 0.1
 * @date 2023-11-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <cartesian_trajectory_interpolation/cartesian_state.h>

#include <senio_controllers/gait_trajectory.hpp>

namespace senio_controllers
{
/**
 * @brief a gait member is a single leg in a gait pattern
 */
class GaitMember : public SmoothedGaitTrajectory
{
public:
  GaitMember() : _in_gait(false)
  {
  }

  /**
   * @brief set the gait member to be in gait
   *
   * @param in_gait
   */
  void in_gait(const bool& in_gait)
  {
    _in_gait = in_gait;
  }

  const bool in_gait() const
  {
    return _in_gait;
  }

  /**
   * Determines if the gait member is in contact with the ground.
   *
   * @return true if the gait member is in contact with the ground, false otherwise
   */
  const bool in_contact() const
  {
    // get the gait state information from the predicted state trajectory
    return phase(cartToGaitState(_predicted_state)) == GaitTrajectoryBase::gait_phase::contact;
  }

  /**
   * Check if the robot is currently in the swing phase of the gait trajectory.
   *
   * @return true if the robot is in the swing phase, false otherwise
   */
  const bool in_swing() const
  {
    GaitTrajectoryBase::gait_phase phase = this->phase(cartToGaitState(_predicted_state));
    return phase == GaitTrajectoryBase::gait_phase::upswing || phase == GaitTrajectoryBase::gait_phase::downswing;
  }

  /**
   * @brief return the predicted cartesian state of the gait member
   *
   * @return const ros_controllers_cartesian::CartesianState
   */
  const ros_controllers_cartesian::CartesianState prediction() const
  {
    return _predicted_state;
  }

  /**
   * @brief set the predicted cartesian state of the gait member
   *
   * @param state
   */
  void state(const ros_controllers_cartesian::CartesianState& state)
  {
    _predicted_state = state;
  }

protected:
  /**
   * @brief convert a cartesian state to a gait state
   *
   * @param cart_state
   * @return const geometry_msgs::Vector3
   */
  const geometry_msgs::Vector3 cartToGaitState(const ros_controllers_cartesian::CartesianState cart_state) const
  {
    geometry_msgs::Vector3 gait_state;
    gait_state.x = cart_state.p.x();
    gait_state.y = cart_state.p.y();
    gait_state.z = cart_state.p.z();
    return gait_state;
  }

private:
  ros_controllers_cartesian::CartesianState _predicted_state;
  bool _in_gait;
};

}  // namespace senio_controllers
