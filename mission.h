
#ifndef MISSION_H
#define MISSION_H


#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "laserprocessing.h"
#include "tf2/LinearMath/Quaternion.h"
#include "audi.h"
#include "pfmshog.h"

#include <vector>
#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>
#include <deque>

/**
 * @brief Mission Class
 * 
 * This class is a subclass of the MissionInterface class. It is used to control the Mission platform.
 * 
 */

namespace MarkerArray 
{
  enum MarkerType 
  {
    CYLINDER,  // Cone marker, needs to be CYLINDER type, radius 0.2m & height 0.5m
    CUBE       // Road marker, needs to be CUBE type of size 0.5m
  };
}

class Mission: public rclcpp::Node
{
public:
    /**
     * The Default constructor
     * @sa ControllerInterface and @sa MissionInterface for more information
     */
    Mission();
    // Destructor
    ~Mission();

    /*! @brief - A callback for the service
     *  
     * @param[in] request - The request object
     * @param[out] response - The response object which contains a number of valid readings in last laser scan received "Readings: <number of readings>"
     * if no laser scan is received, the response will contain error message "ERROR: No laser data available"
     */
    void request(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    
    /*! @brief - A callback for the timer
     * We will simply do some logging here in this function as an example
     */
    void timerCallback();

    /*! @brief - A function that will be run in a separate thread
     * We will simply do some logging here in this function as an example
     */
    void threadFunction();

    void goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

    void pubConesDetected(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubCones, std::vector<geometry_msgs::msg::Point>& cones);

    void pubRoadCentrePos(const std::vector<geometry_msgs::msg::Point>& roadCentres);

    bool roadBoundaries(nav_msgs::msg::Odometry currentOdo);

private:
    /*! @brief LaserScan Callback
     *
     *  @param std::shared_ptr<sensor_msgs::msg::LaserScan - The laserscan message as a const pointer
     *  @note This function and the declaration are ROS specific to handle callbacks
     */
    void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg);

    /**
     * @brief Runs the mission, non blocking call
     */
    void run(void);

    visualization_msgs::msg::MarkerArray produceMarker(std::vector<geometry_msgs::msg::Point> object, rclcpp::Time stamp);

    void setMarkerType(MarkerArray::MarkerType markerType_);

    void progress(void);

    double distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt);//<! distance between the arguments
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdo_; //<! Subscription to odometry
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subGoal_;//<! Subscription to goal
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLaser_;//<! Subscription to laser scan

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubVisualMarker_; //!< Publisher for markers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubCones_; //!< Publisher for cones
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_; //!< Pointer to the service object rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;//!< Pointer to the laser scan subscriber
    std::unique_ptr<LaserProcessing> laserProcessingPtr_;//!< Pointer to the laser processing object

    MarkerArray::MarkerType markerType_; //!< Marker Type
    visualization_msgs::msg::MarkerArray markerArray_; //!< Marker Array

    struct CountID // Counts the number of cones, centre road positions and goals
        {
            unsigned int cones;
            unsigned int goals;
            unsigned int road;
        } counter_; // Name for counter for markers

    std::thread* thread_; //!< Thread object pointer
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer object pointer, will be used to run a function at regular intervals
    
    nav_msgs::msg::Odometry odo_; //!< Storage of odometry
    geometry_msgs::msg::PoseArray goals_; //!< Storage of goals 
    std::vector<geometry_msgs::msg::Point> conesScanned_; //!< Storage of cones
    std::vector<geometry_msgs::msg::Point> roadCentres_; //!< Storage of road boundaries

    std::mutex mtxGoals_;//<! Mutex for goals_ and init_dist_to_goal_;
    std::mutex mtxOdo_;//<! Mutex for odo
    std::mutex mtxLaser_; //!< Mutex for laser scan
    
    double distToGoal_; //!< Storage of distance to goal
    double tolerance_;
    std::atomic<bool> chasing_;//<! Indicates if we are chasing (pursuing goal)
    Audi audi_;
    LaserProcessing laserProcessing_;

    struct LaserData
    {
      sensor_msgs::msg::LaserScan scan;
    } laserData_; 
};

#endif // RANGERFUSION_H
