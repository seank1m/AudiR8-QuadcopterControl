#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <math.h>
#include <mutex>
#include <algorithm>
#include <numeric>
#include <iostream>

/*!
 *  \brief    LaserProcessing Class 
 *  \details  Scan and detects object using a laser within the range detectable of the Ackerman vehicle
 *  \author   Sean Kim
 *  \date     2024-05-21
 */

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::msg::LaserScan laserScan);

  /*! @brief Count number of readings belonging to objects (not infinity, nan or max range) from the last
  * laser scan provided (either via @sa newScan or @sa LaserProcessing constructor)
  * thread-safe function, internally creates a copy fo laserScan_ 
  *
  * @return the number of laser readings that belong to objects
  */
  unsigned int countObjectReadings();


  /*! @brief Accepts a new laserScan, threadsafe function
   *  @param[in]    laserScan  - laserScan supplied
   */
  void newScan(sensor_msgs::msg::LaserScan laserScan);

  /*! @brief Converts a local point to a global point
   *  @param[in]    localPoint - point in local coordinates
   *  @param[in]    currentPose - current pose of the vehicle
   *  @return global point
   */
  geometry_msgs::msg::Point localToGlobal(geometry_msgs::msg::Point localPoint, geometry_msgs::msg::Pose currentPose);

  unsigned int countSegmentReadings(geometry_msgs::msg::Pose currentPose);

  std::vector<geometry_msgs::msg::Point> getCones(geometry_msgs::msg::Pose currentPose);

  std::vector<geometry_msgs::msg::Point> getRoadBoundaries(geometry_msgs::msg::Pose currentPose);

  std::vector<geometry_msgs::msg::Point> getRoadCentre(geometry_msgs::msg::Pose currentPose);

private:
  /*! @brief Returns the cartesian position of laer reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
   geometry_msgs::msg::Point polarToCart(unsigned int index);

   /*! @brief Given two points (only x,y are used), returns the slope slope of the lines connecting them
    *  @param[in] p1 - first point
    *  @param[in] p2 - second point
    *  @return slope of line between the points in radians
    */
  double angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2);

  sensor_msgs::msg::LaserScan laserScan_;
  std::mutex mtx; //!< Mutex to protect the laserScan_ from being accessed by multiple threads
  unsigned int objectReadings_; //!< Number of readings belonging to objects

  float coneTolerance_;
  float roadWidth_;
  std::vector<std::pair<double, double>> cones_; //!< Vector of cones detected by the laser in pairs 
};

#endif // LASERPROCESSING_H