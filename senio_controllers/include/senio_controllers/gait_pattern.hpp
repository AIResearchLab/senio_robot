/**
 * @file gait_pattern.hpp
 * @author mik-p
 * @brief a gait pattern
 * @version 0.1
 * @date 2023-11-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cartesian_trajectory_interpolation/cartesian_state.h>

#include <senio_controllers/gait_member.hpp>

namespace senio_controllers
{
/***/
class GaitPatternBase
{
public:
  /***/
  const bool validate(const size_t& num_legs)
  {
    if (_gait_members.size() <= num_legs)
    {
      return false;
    }

    return true;
  }

  /***/
  const geometry_msgs::Transform footprint_transform()
  {
    geometry_msgs::Transform tf;

    // get the current cartesian points in contact
    const std::vector<ros_controllers_cartesian::CartesianState> current_contacts = this->current_contacts();

    // get the centroid of the contact points
    // this is the center of the footprint
    // or origin of the footprint frame in the cartesian control frame
    // which is normally the base frame
    // fill the transform translation
    tf.translation = GaitPatternBase::centroid(current_contacts);

    // find the normal of the plane of the contact points in roll pitch and yaw
    const geometry_msgs::Vector3 normal = GaitPatternBase::normal(current_contacts);
    double pitch = std::atan2(-normal.z, std::sqrt(normal.x * normal.x + normal.y * normal.y));
    double roll = std::atan2(normal.x, normal.y);

    // and make a quaternion to represent the orientation of the footprint frame
    tf2::Quaternion q;
    q.setRPY(roll, pitch, 0.0);

    // normalize the quaternion
    q.normalize();

    // fill the transform rotation
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
    tf.rotation.w = q.w();

    // return the transform
    return tf;
  }

  /**
   * @brief return a polygon representing the foot contacts
   *
   * @return const geometry_msgs::Polygon
   */
  const geometry_msgs::Polygon contact_polygon()
  {
    geometry_msgs::Polygon contact_poly;

    // get the current cartesian points in contact
    const std::vector<ros_controllers_cartesian::CartesianState> current_contacts = this->current_contacts();

    contact_poly.points.resize(current_contacts.size());

    // fill the contact polygon with the current points
    for (size_t idx = 0; idx < current_contacts.size(); idx++)
    {
      contact_poly.points[idx].x = current_contacts[idx].p.x();
      contact_poly.points[idx].y = current_contacts[idx].p.y();
      contact_poly.points[idx].z = current_contacts[idx].p.z();
    }

    return contact_poly;
  }

  /**
   * @brief Get the current cartesian points in contact with the ground
   *
   * @return const std::vector<ros_controllers_cartesian::CartesianState>
   */
  const std::vector<ros_controllers_cartesian::CartesianState> current_contacts()
  {
    std::vector<ros_controllers_cartesian::CartesianState> current_contacts;

    for (const auto& leg : _gait_members)
    {
      // if the leg is in the contact phase of trajectory
      // then add the contact point to the list
      if (leg.in_contact())
      {
        // add it to the list of current contacts
        current_contacts.push_back(leg.prediction());
      }
    }

    return current_contacts;
  }

  // static helpers
  /**
   * @brief calculates the 3-space centroid of a set of cartesian states
   *
   * @param points
   * @return const geometry_msgs::Vector3
   */
  static const geometry_msgs::Vector3 centroid(const std::vector<ros_controllers_cartesian::CartesianState>& points)
  {
    geometry_msgs::Vector3 centroid;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid.z = 0.0;
    for (const auto& point : points)
    {
      centroid.x += point.p.x();
      centroid.y += point.p.y();
      centroid.z += point.p.z();
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    centroid.z /= points.size();

    return centroid;
  }

  /**
   * @brief get the normal vector of a plane defined be a set of cartesian states
   *
   * @param points
   * @return const geometry_msgs::Vector3
   */
  static const geometry_msgs::Vector3 normal(const std::vector<ros_controllers_cartesian::CartesianState>& points)
  {
    geometry_msgs::Vector3 normal;
    normal.x = 0.0;
    normal.y = 0.0;
    normal.z = 0.0;

    for (size_t i = 0; i < points.size(); ++i)
    {
      const auto& p1 = points[i];
      const auto& p2 = points[(i + 1) % points.size()];

      normal.x += (p1.p.y() - p2.p.y()) * (p1.p.z() + p2.p.z());
      normal.y += (p1.p.z() - p2.p.z()) * (p1.p.x() + p2.p.x());
      normal.z += (p1.p.x() - p2.p.x()) * (p1.p.y() + p2.p.y());
    }

    double length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    normal.x /= length;
    normal.y /= length;
    normal.z /= length;

    return normal;
  }

protected:
  std::vector<GaitMember> _gait_members;
};

}  // namespace senio_controllers
