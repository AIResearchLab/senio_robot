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

  const bool in_gait() const
  {
    return _in_gait;
  }

  const bool in_contact() const
  {
    return phase(_predicted_state) == GaitTrajectoryBase::gait_phase::contact;
  }

  const bool in_swing() const
  {
    GaitTrajectoryBase::gait_phase phase = this->phase(_predicted_state);
    return phase == GaitTrajectoryBase::gait_phase::upswing || phase == GaitTrajectoryBase::gait_phase::downswing;
  }

  const ros_controllers_cartesian::CartesianState prediction() const
  {
    return _predicted_state;
  }

  void state(const ros_controllers_cartesian::CartesianState& state)
  {
    _predicted_state = state;
  }

private:
  ros_controllers_cartesian::CartesianState _predicted_state;
  bool _in_gait;
};

}  // namespace senio_controllers
