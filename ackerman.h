#include "controller.h"
#include "audi.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"

class Ackerman: public Controller
{
public:
    // Default constructor should set all attributes to a default value
    Ackerman();
    // Destructor
    ~Ackerman();

    /**
     * Run controller in reaching goals - non blocking call
     */
    void reachGoal(void);
    
    /**
     Checks whether the platform can travel between origin and destination
    @param[in] origin The origin pose, specified as odometry for the platform
    @param[in] destination The destination point for the platform
    @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
    @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
    @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
    @return bool indicating the platform can reach the destination from origin supplied
    */

    void sendCmd (double brake, double throttle, double steering);
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, 
                                    pfms::geometry_msgs::Point goal, 
                                    double& distance, 
                                    double& time, 
                                    pfms::nav_msgs::Odometry& estimatedGoalPose);
    
    /**
     Getter for distance to be travelled to reach goal, updates at the platform moves to current goal
    @return distance to be travlled to goal [m]
    */
    double distanceToGoal(void);
    
    /**
     Getter for time to reach goal, updates at the platform moves to current goal
    @return time to travel to goal [s]
    */
    double timeToGoal(void);
    
    /**
     returns total distance travelled by platform
    @return total distance travelled since started [m]
    */
    double distanceTravelled(void);

    // void odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);

private:

    const double max_brake_torque_; //!< A private constant of max brake torque
    const double default_throttle_; //!< A private constant of default throttle

    double distance_; //!< A private copy of distance
    double time_; //!< A private copy of time
    double distanceTravelled_; //!< A private copy of distance travelled
    pfms::geometry_msgs::Point goal_; //!< A protected copy of goal
    pfms::nav_msgs::Odometry estimatedPose; //!< A private copy of estimated goal pose
    pfms::nav_msgs::Odometry currentOdo; //!< A private copy of current odometry
    std::vector<double> steeringAngle_; //!< A private copy of steering angles


    Audi audi_; //!< A private copy of audi
    std::thread* thread_; //!< A private thread for running controller

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubBrakeCmd_; //!< A private publisher for brake
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubSteerCmd_; //!< A private publisher for steering angle
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubThrottleCmd_; //!< A private publisher for throttle   

};

#endif // ACKERMAN_H
