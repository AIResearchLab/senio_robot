/**
 * @file ground_estimator.hpp
 * @author mik-p
 * @brief estimating ground height from a set of points representing the ground plane based on location of foot end
 * effectors
 * @version 0.1
 * @date 2023-11-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

namespace senio_controllers
{
/**
 * @brief GroundEstimator class definition
 * the ground estimator class estimates the height of the ground plane based on the location of foot end effectors
 * it uses a set of points representing the ground plane and the location of the foot end effectors to estimate the
 * height of the ground plane
 *
 */
class GroundEstimator
{
public:
  GroundEstimator(const std::string& ground_frame) : _ground_frame(ground_frame)
  {
  }

  ~GroundEstimator();

  /**
   * @brief estimates a point cloud representing the real ground based on the location contacts and a previous estimate
   *
   * @param[in] points representing the previous ground point cloud
   * @param[in] foot contact locations
   * @return sensor_msgs::PointCloud2 representing the estimated ground plane
   */
  const sensor_msgs::PointCloud2 estimate(const sensor_msgs::PointCloud2& prev_est,
                                          const std::vector<geometry_msgs::Point>& foot_ee_locs)
  {
    // define a small region in space to use as a
    double dx = 0.1;
    double dy = 0.1;
    double dz = 0.1;

    // for each

    sensor_msgs::PointCloud2 est;
    return est;
  }

private:
  std::string _ground_frame;
};
}  // namespace senio_controllers
