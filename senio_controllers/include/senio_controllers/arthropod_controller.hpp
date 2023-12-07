/**
 * @file arthropod_controller.hpp
 * @author mik-p
 * @brief  Arthropod controller header file defining ros controller for arthropod robots
 * @version 0.1
 * @date 2023-11-16
 *
 *
 */

#pragma once

#include <ros/ros.h>

#include <controller_interface/multi_interface_controller.h>
#include <speed_scaling_interface/speed_scaling_interface.h>

#include <cartesian_trajectory_interpolation/cartesian_state.h>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <senio_controllers/cartesian_trajectory_leg_controller.hpp>
#include <senio_controllers/ground_estimator.hpp>
#include <senio_controllers/gait_predictor.hpp>

namespace senio_controllers
{

/**
 * @brief Arthropod controller class definition
 * the arthropod controller is a ros controller for arthropod robots it can determine foot positions for different
 * gaits orient the robot body in 6dof space and position each leg end effector in 3d space
 *
 */
template <class HWInterface>
class ArthropodController
  : public controller_interface::MultiInterfaceController<HWInterface, scaled_controllers::SpeedScalingInterface>
{
public:
  ArthropodController()
    : _robot_base_frame("base_link")
    , _robot_footprint_frame("base_footprint")
    , _odom_frame("odom")
    , _enable_odom_tf(true)
    , _publish_cmd(false)
    , _pub_rate_hz(25.0)
  {
  }

  virtual ~ArthropodController(){};

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override
  {
    // get controlller parameters
    // rate
    controller_nh.param("publish_rate", _pub_rate_hz, 25.0);
    // publish commands
    controller_nh.param("publish_cmd", _publish_cmd, false);
    // publish odom tf
    controller_nh.param("enable_odom_tf", _enable_odom_tf, true);

    // the list of legs
    std::vector<std::string> legs;
    if (!controller_nh.getParam("legs", legs))
    {
      ROS_ERROR("No legs given (namespace: %s)", controller_nh.getNamespace().c_str());
      return false;
    }

    // the robot base frame
    controller_nh.param("robot_base_frame", _robot_base_frame);

    // odom frame
    controller_nh.param("odom_frame", _odom_frame);

    // the robot footprint frame
    controller_nh.param("robot_footprint_frame", _robot_footprint_frame);

    // the robot tree
    if (!kdl_parser::treeFromParam("robot_description", _robot_tree))
    {
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }

    // instantiate the legs
    // _legs.resize(legs.size());
    // the order that the legs appear in config is the order in the gait patterns
    for (const auto& leg : legs)
    {
      ros::NodeHandle leg_nh(controller_nh.getNamespace() + "/" + leg);

      std::unique_ptr<CartesianTrajectoryLegController<HWInterface>> l =
          std::make_unique<CartesianTrajectoryLegController<HWInterface>>();

      if (!l->init(hw, root_nh, leg_nh))
      {
        ROS_ERROR("Failed to initialize leg %s", leg.c_str());
        return false;
      }

      _legs.push_back(std::move(l));
    }

    // initialize the gait predictor
    // pass reference to the legs
    _gait_predictor = std::make_unique<GaitPredictor>(controller_nh);

    // if the gait members are not the same size as the legs array then error
    if (!_gait_predictor->validate())
    {
      ROS_ERROR("Gait pattern doesn't match legs count");
      return false;
    }

    // initialize the ground estimator
    _ground_estimator = std::make_unique<GroundEstimator>(_robot_footprint_frame);

    // initialize the subs
    _sub_twist = controller_nh.subscribe("cmd_vel", 1, &ArthropodController::cmd_vel_cb, this);
    _sub_pose = controller_nh.subscribe("cmd_pose", 1, &ArthropodController::cmd_pose_cb, this);

    // initialize the pubs
    _pub_odom.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "/odom", 100));
    _pub_tf.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf", 100));

    if (_publish_cmd)
    {
      _pub_pose.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(controller_nh, "/pose", 100));
      _pub_twist.reset(
          new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "/twist", 100));
    }

    // initialize the foot polygon publisher
    _pub_foot_poly.reset(
        new realtime_tools::RealtimePublisher<geometry_msgs::PolygonStamped>(controller_nh, "/contact_poly", 1));

    // initialize the ground estimation publisher
    _pub_ground_est.reset(new realtime_tools::RealtimePublisher<sensor_msgs::PointCloud2>(controller_nh, "/ground", 5));

    // set the pub timer
    _pub_timer = root_nh.createTimer(ros::Duration(1.0 / _pub_rate_hz), &ArthropodController::pub_timer_cb, this);

    return true;
  }

  void starting(const ros::Time& time) override
  {
    // initialize the legs
    for (auto& leg : _legs)
    {
      leg->starting(time);
    }
  }

  void stopping(const ros::Time& time) override
  {
    // stop the legs
    for (auto& leg : _legs)
    {
      leg->stopping(time);
    }
  }

  void update(const ros::Time& time, const ros::Duration& period) override
  {
    // if there is a pose or twist command then allocate the legs to the body controller
    // read the commands from the buffers and check if they are valid for the current time window
    const geometry_msgs::TwistStamped twist_cmd = *_twist_cmd_buffer.readFromRT();
    const geometry_msgs::PoseStamped pose_cmd = *_pose_cmd_buffer.readFromRT();

    // if they are valid then update the actual commands
    double cmd_window = 0.1;
    if (pose_cmd.header.stamp + ros::Duration(cmd_window) > time ||
        twist_cmd.header.stamp + ros::Duration(cmd_window) > time)
    {
      // limit pose command to reachable region
      _actual_pose_cmd = pose_cmd;

      // calculate the cartesian commands for each leg for desired pose
      std::vector<ros_controllers_cartesian::CartesianState> cart_cmds;
      cart_cmds.resize(_legs.size());
      _gait_predictor->predict(pose_cmd.pose, cart_cmds);

      // limit the twist command
      _actual_twist_cmd = twist_cmd;

      // mix in twist command with current pose command
      // calculate the cartesian commands for each leg for desired twist
      _gait_predictor->predict(_actual_pose_cmd.pose, _actual_twist_cmd.twist, cart_cmds);

      // set the desired state for each leg
      for (size_t idx = 0; idx < _legs.size(); idx++)
      {
        // allocate the leg to the body controller
        _legs[idx]->allocate();
        // update the legs desired states
        _legs[idx]->setDesiredCartesianState(cart_cmds[idx]);
      }
    }
    else
    {
      // deallocate the legs from the body controller
      for (auto& leg : _legs)
      {
        leg->deallocate();
      }
    }

    // update the legs
    // if they are not allocated to the body controller
    // this will update the action servers for the legs
    // allowing cartesian trajectory controllers to be used for each leg
    for (auto& leg : _legs)
    {
      leg->update(time, period);
    }
  }

private:
  /**
   * @brief callback for twist command
   *
   * @param msg
   */
  void cmd_vel_cb(const geometry_msgs::Twist& msg)
  {
    // update the twist command
    geometry_msgs::TwistStamped cmd;
    // add time stamp to twist command to mark its validity window
    cmd.header.stamp = ros::Time::now();
    cmd.twist = msg;
    // write the twist command to the buffer
    _twist_cmd_buffer.writeFromNonRT(cmd);
  }

  /**
   * @brief callback for pose command
   *
   * @param msg
   */
  void cmd_pose_cb(const geometry_msgs::Pose& msg)
  {
    // update the pose command
    geometry_msgs::PoseStamped cmd;
    // add time stamp to pose command to mark its validity window
    cmd.header.stamp = ros::Time::now();
    cmd.pose = msg;
    // write the pose command to the buffer
    _pose_cmd_buffer.writeFromNonRT(cmd);
  }

  /**
   * @brief callback for pub timer
   *
   * @param event
   */
  void pub_timer_cb(const ros::TimerEvent& event)
  {
    update_odom(event.current_real);
    update_tf(event.current_real);

    // if publishing the commands the update them
    if (_publish_cmd)
    {
      if (_pub_twist->trylock())
      {
        _pub_twist->msg_ = _actual_twist_cmd;
        _pub_twist->msg_.header.frame_id = _robot_base_frame;
        _pub_twist->unlockAndPublish();
      }
      if (_pub_pose->trylock())
      {
        _pub_pose->msg_ = _actual_pose_cmd;
        _pub_pose->msg_.header.frame_id = _robot_base_frame;
        _pub_pose->unlockAndPublish();
      }
    }

    // create the foot polygon message
    if (_pub_foot_poly && _pub_foot_poly->trylock())
    {
      _pub_foot_poly->msg_.header.stamp = event.current_real;
      _pub_foot_poly->msg_.header.frame_id = _robot_footprint_frame;

      _pub_foot_poly->msg_.polygon = _gait_predictor->contact_polygon();

      _pub_foot_poly->unlockAndPublish();
    }

    // create the ground estimation message
    if (_pub_ground_est && _pub_ground_est->trylock())
    {
      // get current contacts
      const std::vector<ros_controllers_cartesian::CartesianState> contacts =
          _gait_predictor->current_contacts(current_ee_locs());

      // convert to points
      std::vector<geometry_msgs::Point> contact_points;
      for (const auto& contact : contacts)
      {
        geometry_msgs::Point p;
        p.x = contact.p.x();
        p.y = contact.p.y();
        p.z = contact.p.z();
        contact_points.push_back(p);
      }

      // estimate the ground
      _pub_ground_est->msg_ = _ground_estimator->estimate(_pub_ground_est->msg_, contact_points);

      // stamp message and send
      _pub_ground_est->msg_.header.stamp = event.current_real;
      _pub_ground_est->msg_.header.frame_id = _robot_footprint_frame;
      _pub_ground_est->unlockAndPublish();
    }
  }

  /**
   * @brief update the tf message
   *
   * @param time
   */
  void update_tf(const ros::Time& time)
  {
    // update the tf message
    if (_pub_tf->trylock())
    {
      _pub_tf->msg_.transforms.resize(1);

      if (_enable_odom_tf)
      {
        _pub_tf->msg_.transforms.resize(2);

        // odom to base transform
        _pub_tf->msg_.transforms[1].header.stamp = time;
        _pub_tf->msg_.transforms[1].header.frame_id = _odom_frame;
        _pub_tf->msg_.transforms[1].child_frame_id = _robot_base_frame;
        _pub_tf->msg_.transforms[1].transform.translation.x = 0.0;
        _pub_tf->msg_.transforms[1].transform.translation.y = 0.0;
        _pub_tf->msg_.transforms[1].transform.translation.z = 0.0;
        _pub_tf->msg_.transforms[1].transform.rotation.x = 0.0;
        _pub_tf->msg_.transforms[1].transform.rotation.y = 0.0;
        _pub_tf->msg_.transforms[1].transform.rotation.z = 0.0;
        _pub_tf->msg_.transforms[1].transform.rotation.w = 1.0;
      }

      // base to footprint transform
      _pub_tf->msg_.transforms[0].header.stamp = time;
      _pub_tf->msg_.transforms[0].header.frame_id = _robot_footprint_frame;
      _pub_tf->msg_.transforms[0].child_frame_id = _robot_base_frame;
      _pub_tf->msg_.transforms[0].transform = _gait_predictor->footprint_transform();

      _pub_tf->unlockAndPublish();
    }
  }

  /**
   * @brief update the odometry message
   *
   * @param time
   */
  void update_odom(const ros::Time& time)
  {
    // update the odometry message
    if (_pub_odom->trylock())
    {
      _pub_odom->msg_.header.stamp = time;
      _pub_odom->msg_.header.frame_id = _odom_frame;
      _pub_odom->msg_.child_frame_id = _robot_base_frame;
      _pub_odom->msg_.pose.pose.position.x = 0.0;
      _pub_odom->msg_.pose.pose.position.y = 0.0;
      _pub_odom->msg_.pose.pose.position.z = 0.0;
      _pub_odom->msg_.pose.pose.orientation.x = 0.0;
      _pub_odom->msg_.pose.pose.orientation.y = 0.0;
      _pub_odom->msg_.pose.pose.orientation.z = 0.0;
      _pub_odom->msg_.pose.pose.orientation.w = 1.0;
      _pub_odom->msg_.twist.twist.linear.x = 0.0;
      _pub_odom->msg_.twist.twist.linear.y = 0.0;
      _pub_odom->msg_.twist.twist.linear.z = 0.0;
      _pub_odom->msg_.twist.twist.angular.x = 0.0;
      _pub_odom->msg_.twist.twist.angular.y = 0.0;
      _pub_odom->msg_.twist.twist.angular.z = 0.0;
      _pub_odom->unlockAndPublish();
    }
  }

  /**
   * @brief Get the current cartesian position of the foot end effectors
   *
   * @return const std::vector<ros_controllers_cartesian::CartesianState>
   */
  const std::vector<ros_controllers_cartesian::CartesianState> current_ee_locs()
  {
    std::vector<ros_controllers_cartesian::CartesianState> current_ee_locs;

    for (const auto& leg : _legs)
    {
      const ros_controllers_cartesian::CartesianState state = leg->getState();

      // add it to the list
      current_ee_locs.push_back(state);
    }

    return current_ee_locs;
  }

protected:
  // basic config
  bool _enable_odom_tf;
  bool _publish_cmd;
  std::string _robot_base_frame;
  std::string _odom_frame;

  // leg config
  std::vector<std::unique_ptr<CartesianTrajectoryLegController<HWInterface>>> _legs;
  KDL::Tree _robot_tree;
  std::string _robot_footprint_frame;

  // control variables
  realtime_tools::RealtimeBuffer<geometry_msgs::TwistStamped> _twist_cmd_buffer;
  geometry_msgs::TwistStamped _actual_twist_cmd;
  realtime_tools::RealtimeBuffer<geometry_msgs::PoseStamped> _pose_cmd_buffer;
  geometry_msgs::PoseStamped _actual_pose_cmd;

  // gait and estimation
  std::unique_ptr<GroundEstimator> _ground_estimator;
  std::unique_ptr<GaitPredictor> _gait_predictor;
  // whole body limits
  std::vector<geometry_msgs::Pose> _pose_limits;
  std::vector<geometry_msgs::Twist> _twist_limits;

  // subs
  ros::Subscriber _sub_twist;
  ros::Subscriber _sub_pose;

  // pubs
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> _pub_tf;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> _pub_odom;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> _pub_twist;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> _pub_pose;
  // gait publisher
  //   realtime_tools::RealtimePublisher<senio_msgs::Gait> pub_gait_array_;
  // foot end effector publisher
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PolygonStamped>> _pub_foot_poly;
  // ground estimation publisher
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::PointCloud2>> _pub_ground_est;

  // action servers for rolling, standing, and collapsing

  // pub timers
  double _pub_rate_hz;
  ros::Timer _pub_timer;
};

/**
 * @brief not implemented
 *
 */
// template <>
// class ArthropodController<hardware_interface::EffortJointInterface>
//   : public ArthropodController<hardware_interface::JointHandle>
// {
// };

/**
 * @brief
 *
 * @tparam PositionJointInterface
 */
// template <>
// class ArthropodController<hardware_interface::PositionJointInterface>
//   : public ArthropodController<hardware_interface::JointHandle>
// {
// };

/**
 * @brief
 *
 * @tparam VelocityJointInterface
 */
// template <>
// class ArthropodController<hardware_interface::VelocityJointInterface>
//   : public ArthropodController<hardware_interface::JointHandle>
// {
// };

}  // namespace senio_controllers
