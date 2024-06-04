#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "pfms_types.h"
#include "tf2/LinearMath/Quaternion.h"

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>

/**
 * @brief Controller Class
 * 
 * This class is a subclass of the ControllerInterface class. It is used to control the Controller platform.
 * 
 */

class Controller: public rclcpp::Node
{
public:
    // Default constructors should set all attributes to a default value
    Controller();
    // Destructor
    ~Controller();

    /**
     returns current odometry information
     @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
    */
    pfms::nav_msgs::Odometry getOdometry(void);

    /**
     Connected to callback function to set the goal
     @param msg The goal to set
    */
    void setGoals(const geometry_msgs::msg::PoseArray& msg);

    /**
     * Updates the internal pose
     *
     * Calback function for the odometry subscriber
    */
    void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    /**
     returns total time in motion by platform, time when stationary not included
     @return total time in motion since started [s]
    */
    double timeTravelled(void);

protected:
    /**
     * Checks if the goal has been reached.
     *
     * Update own pose before calling!
     * @return true if the goal is reached
     */
    bool goalReached();
    
    // geometry_msgs::msg::Pose odo_; //!< A protected copy of odometry information
    pfms::nav_msgs::Odometry odo_; //!< A protected copy of odometry information
    nav_msgs::msg::Odometry odoMsg_; //!< A protected copy of odometry information
    geometry_msgs::msg::PoseArray goals_;
    geometry_msgs::msg::Point goal_; //!< A protected copy of goal
    pfms::nav_msgs::Odometry estimatedPose_; //!< A protected copy of estimated pose
    bool goalSet_; //!< A protected copy of goal set

    std::mutex mtxGoals_; //!< A mutex to protect the odometry information
    std::mutex mtxOdo_; //!< A mutex to protect the odometry information
    
    double tolerance_; //!< A protected copy of tolerance value
    double totalTime_; //!< A private copy of time in motion
    double distanceTravelled_; //!< A private copy of distance travelled

    bool RUNNING_;


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdo_; //<! Subscription to odometry
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subGoal_;//<! Subscription to goal

};

#endif // CONTROLLER_H
