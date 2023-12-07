/**
 * @file cartesian_trajectory_leg_controller.hpp
 * @author mik-p
 * @brief inherits from CartesianTrajectoryController adding leg specific functionality
 * @version 0.1
 * @date 2023-11-20
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <cartesian_trajectory_controller/cartesian_trajectory_controller.h>
#include <cartesian_trajectory_interpolation/cartesian_state.h>

namespace senio_controllers
{

/**
 * @brief cartesian trajectory leg controller
 *
 */
template <class HWInterface>
class CartesianTrajectoryLegController
  : public cartesian_trajectory_controller::CartesianTrajectoryController<HWInterface>
{
public:
  /**
   * @brief default constructor
   *
   */
  CartesianTrajectoryLegController() : _is_body_allocated(false)
  {
  }

  /**
   * @brief allocate ownership of command to the body controller
   *
   */
  void allocate()
  {
    _is_body_allocated = true;

    // if the action server is running, stop it
    this->preemptCB();
    // if (action_server_->isActive())
    // {
    //   action_server_->setPreempted();
    // }
  }

  /**
   * @brief deallocate ownership of command to the body controller
   * can now use cartesian trajectory controller action server
   *
   */
  void deallocate()
  {
    _is_body_allocated = false;
  }

  /**
   * @brief Set the Desired Cartesian State object
   *
   * @param state
   */
  void setDesiredCartesianState(const ros_controllers_cartesian::CartesianState& state)
  {
    _desired_state = state;
  }

  /**
   * @brief override the update function to allow access to the updateCommand function in the ControlPolicy base
   * class
   *
   * @param time
   * @param period
   */
  void update(const ros::Time& time, const ros::Duration& period) override
  {
    if (_is_body_allocated)
    {
      // maintain preemption
      allocate();

      // update the control policy base controller with desired state
      // ControlPolicy::updateCommand(_desired_state);
      this->updateCommand(_desired_state);
    }
    else
    {
      // action interface update
      // CartesianTrajectoryController<HWInterface>::update(time, period);
      this->update(time, period);
    }
  }

private:
  // allow access to lower level updateCommand function to override the cartesian trajectory controller action server
  bool _is_body_allocated;
  ros_controllers_cartesian::CartesianState _desired_state;
};

}  // namespace senio_controllers
