/**
 * @file gait_trajectory.hpp
 * @author mik-p
 * @brief Gait trajectory implementations
 * @version 0.1
 * @date 2023-11-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>

#include <cartesian_control_msgs/CartesianTrajectory.h>

#include <cartesian_trajectory_interpolation/cartesian_state.h>
#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>

namespace senio_controllers
{
/**
 * @brief base class for gait trajectory helper classes
 *
 */
class GaitTrajectoryBase : public ros_controllers_cartesian::CartesianTrajectory
{
public:
  enum gait_phase
  {
    contact,
    upswing,
    downswing,
    unbound,
  };

  /**
   * @brief keep track of the critical points of a gait to work out the phase of a point along the gait
   *
   */
  class GaitState
  {
  public:
    GaitState(const std::vector<Eigen::Vector3d>& stroke, const Eigen::Vector3d& swing_height,
              const double& swing_length)
      : contact_segment(stroke)
    {
      // set the swing transition regions using the maximum point of the swing trajectory
      // first get the direction of the stroke
      const Eigen::Vector3d stroke_direction = (stroke.back() - stroke.front()).normalized();

      // multiply the vector by the swing length and shift the vector by the height
      const Eigen::Vector3d swing_vector = (stroke_direction * swing_length) + swing_height;

      // set the point where the swing is maximum
      upswing_maximum = stroke.front() + swing_vector;
    }

    /**
     * @brief find the best approximate phase of a given point on the gait
     *
     * @param current_state
     * @return const gait_phase
     */
    const gait_phase phase(const Eigen::Vector3d& current_state) const
    {
      // we need a tolerance to be able to calculate the gait phase if the current state is not exactly on the
      // gait trajectory
      const double state_tolerance = 0.01;

      // create new coordinate bases using the contact segment direction and the upswing maximum
      // the contact segment direction will be x in the new coordinate system
      // the upswing direction will be z in the new coordinate system
      const Eigen::Vector3d contact_segment_direction = (contact_segment.back() - contact_segment.front()).normalized();
      const Eigen::Vector3d upswing_maximum_direction = (upswing_maximum - contact_segment.front()).normalized();

      // create a rotation matrix to rotate the current state into the new coordinate bases
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix.col(0) = contact_segment_direction;
      rotation_matrix.col(1) = contact_segment_direction.cross(upswing_maximum_direction);
      rotation_matrix.col(2) = upswing_maximum_direction;

      // rotate the current state into the new coordinate bases
      const Eigen::Vector3d rotated_state = rotation_matrix * current_state;

      // if the rotated state is above the contact segment in the upswing direction then it is probably swinging
      if (rotated_state.z() > state_tolerance)
      {
        // and less than the swing length in the contact segment direction then the gait phase is upswing
        if (rotated_state.x() <= (upswing_maximum - contact_segment.front()).norm())
        {
          // return upswing
          return gait_phase::upswing;
        }
        else
        {
          // return downswing
          return gait_phase::downswing;
        }
      }

      // if the rotated state is in the contact segment
      if (rotated_state.x() >= 0.0 && rotated_state.x() <= (contact_segment.back() - contact_segment.front()).norm())
      {
        // return contact
        return gait_phase::contact;
      }

      // cannot determine the gait phase
      return gait_phase::unbound;
    }

    /**
     * @brief get the rotation matrix for the gait frame
     *
     * @return const Eigen::Matrix3d
     */
    const Eigen::Matrix3d frame()
    {
      // create new coordinate bases using the contact segment direction and the upswing maximum
      // the contact segment direction will be x in the new coordinate system
      // the upswing direction will be z in the new coordinate system
      const Eigen::Vector3d contact_segment_direction = (contact_segment.back() - contact_segment.front()).normalized();
      const Eigen::Vector3d upswing_maximum_direction = (upswing_maximum - contact_segment.front()).normalized();

      // create a rotation matrix to rotate the current state into the new coordinate bases
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix.col(0) = contact_segment_direction;
      rotation_matrix.col(1) = contact_segment_direction.cross(upswing_maximum_direction);
      rotation_matrix.col(2) = upswing_maximum_direction;

      return rotation_matrix;
    }

    // transition regions
    std::vector<Eigen::Vector3d> contact_segment;
    Eigen::Vector3d upswing_maximum;
  };

public:
  /***/
  virtual void compute_key_points(const geometry_msgs::Pose& stroke_start, const double& stroke_length,
                                  const double& stroke_height) = 0;

  /**
   * Calculates the gait phase based on the state.
   *
   * @param state The state of the robot represented by a geometry_msgs::Vector3 object.
   *
   * @return The gait phase computed based on the state.
   *
   * @throws None
   */
  const gait_phase phase(const geometry_msgs::Vector3& state) const
  {
    return _state->phase(Eigen::Vector3d(state.x, state.y, state.z));
  }

protected:
  /***/
  void state(const GaitState& state)
  {
    _state = std::make_shared<GaitState>(state);
  }

  void state(const std::vector<Eigen::Vector3d>& stroke, const Eigen::Vector3d& swing_height,
             const double& swing_length)
  {
    _state = std::make_shared<GaitState>(stroke, swing_height, swing_length);
  }

protected:
  std::shared_ptr<GaitState> _state;
  std::vector<Eigen::Vector3d> _gait_key_points;
};

/**
 * @brief a three point gait trajectory with a height and a stroke length creating a triangular gait pattern
 *
 */
class ThreePointGaitTrajectory : public GaitTrajectoryBase
{
public:
  /***/
  virtual void compute_key_points(const geometry_msgs::Pose& stroke_start, const double& stroke_length,
                                  const double& stroke_height) override
  {
    // create a reference frame aligned with the stroke start orientation quaternion
    Eigen::Matrix3d reference_frame;
    reference_frame = Eigen::Quaterniond(stroke_start.orientation.w, stroke_start.orientation.x,
                                         stroke_start.orientation.y, stroke_start.orientation.z)
                          .toRotationMatrix();

    // create a new gait trajectory
    _gait_key_points.resize(3);

    // set the first point to the stroke start
    _gait_key_points[0] = Eigen::Vector3d(stroke_start.position.x, stroke_start.position.y, stroke_start.position.z);

    // set the second point to the stroke start plus the swing length and the stroke height
    _gait_key_points[1] = _gait_key_points[0] + (reference_frame * Eigen::Vector3d(swing_length, 0.0, stroke_height));

    // set the third point to the stroke start plus the stroke length
    _gait_key_points[2] = _gait_key_points[0] + (reference_frame * Eigen::Vector3d(stroke_length, 0.0, 0.0));

    // vector of vectors from the stroke start to the stroke end
    std::vector<Eigen::Vector3d> stroke;
    stroke.push_back(_gait_key_points[0]);
    stroke.push_back(_gait_key_points[2]);

    // set the gait state
    state(stroke, reference_frame * Eigen::Vector3d(0.0, 0.0, stroke_height), swing_length);
  }

protected:
  double swing_length;
};

/**
 * @brief a triangular gait with a debounced contact phase which directs the trajectory just before contact to align the
 * friction force with the travel direction
 *
 */
class DebouncedGaitTrajectory : public ThreePointGaitTrajectory
{
public:
  /***/
  void compute_key_points(const geometry_msgs::Pose& stroke_start, const double& stroke_length,
                          const double& stroke_height) override
  {
    // do parent computation
    ThreePointGaitTrajectory::compute_key_points(stroke_start, stroke_length, stroke_height);

    // get the rotation matrix for the gait frame from the gait state
    const Eigen::Matrix3d gait_frame = _state->frame();

    // get the contact point and shift up and ahead by a small amount
    const Eigen::Vector3d debounce_point =
        _gait_key_points[2] + (debounce_length * (gait_frame * (Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitZ())));

    // add a debounce key point before the contact point
    _gait_key_points.insert(_gait_key_points.end() - 2, debounce_point);
  }

protected:
  double debounce_length;
};

/**
 * @brief a gait that smoothed between key points using derivative constraints
 *
 */
class SmoothedGaitTrajectory : public DebouncedGaitTrajectory
{
public:
  /***/
  void compute_key_points(const geometry_msgs::Pose& stroke_start, const double& stroke_length,
                          const double& stroke_height) override
  {
    // do parent computation
    DebouncedGaitTrajectory::compute_key_points(stroke_start, stroke_length, stroke_height);

    // get the second derivative between points
    for (size_t i = 0; i < _gait_key_points.size() - 2; i++)
    {
      const Eigen::Vector3d second_derivative =
          (_gait_key_points[i + 2] - _gait_key_points[i + 1]) - (_gait_key_points[i + 1] - _gait_key_points[i]);

      // if the magnitude is greater than the maximum then insert a new point that reduces the magnitude
      if (second_derivative.norm() > ddx_max)
      {
        // insert a new point that reduces the magnitude
        _gait_key_points.insert(_gait_key_points.begin() + i + 1,
                                _gait_key_points[i + 1] + (ddx_max / second_derivative.norm()) * second_derivative);
      }
    }

    // apply constraints to the key points
  }

protected:
  double ddx_max;
};

}  // namespace senio_controllers
