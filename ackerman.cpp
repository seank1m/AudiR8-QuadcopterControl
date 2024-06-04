#include "ackerman.h"
#include "audi.h"
#include <thread>
#include <chrono>
#include <time.h>

Ackerman::Ackerman() : 
    max_brake_torque_(8000), 
    default_throttle_(0.1), 
    distance_(0.0),
    time_(0.0),
    estimatedPose({0,0,0,0,0,0,0,0}),
    distanceTravelled_(0.0)
{
    odo_ = getOdometry();
    subOdo_ = this->create_subscription<nav_msgs::msg::Odometry>("/orange/odom", 1000, std::bind(&Ackerman::odoCallback, this, std::placeholders::_1));
    
    pubBrakeCmd_ = this->create_publisher<std_msgs::msg::Float64>("/orange/brake_cmd", 10);
    pubSteerCmd_ = this->create_publisher<std_msgs::msg::Float64>("/orange/steering_cmd", 10);
    pubThrottleCmd_ = this->create_publisher<std_msgs::msg::Float64>("/orange/throttle_cmd", 10);

    thread_ = new std::thread(&Ackerman::reachGoal, this);

}

Ackerman::~Ackerman()
{
    thread_->join();
    delete thread_;
}
void Ackerman::reachGoal()
{
    while(rclcpp::ok()){
        for(unsigned int i = 0; i < goals_.poses.size(); i++)
        {
            pfms::nav_msgs::Odometry currentOdo = getOdometry();
            double currentSpeed = sqrt(pow(currentOdo.linear.x, 2) + pow(currentOdo.linear.y, 2));
            double steeringAngle = 0;
            double distance = 0;
            // Using this to get the goal position for info stream / debugging
            goal_.x = goals_.poses[i].position.x; // Set goal_ to currentGoal position (x)
            goal_.y = goals_.poses[i].position.y; // Set goal_ to currentGoal position (y)

            for(unsigned int j = 0; j < 500; j++)\
            {
                double brake = 0; // Initialise brake value to 0 
                double throttle = default_throttle_; // Initialise throttle value to default throttle value
                double dist2Goal = distanceToGoal();
                currentOdo = getOdometry(); // Get current odometry
                audi_.computeSteering(currentOdo, goal_, steeringAngle, distance);
                
                RCLCPP_INFO_STREAM(this->get_logger(), 
                    "Goal Number: " << i + 1 << " of " << goals_.poses.size());
                RCLCPP_INFO_STREAM(this->get_logger(), 
                    "Current Goal: {" << goal_.x << ", " << goal_.y << "}");
                RCLCPP_INFO_STREAM(this->get_logger(), 
                    "Odometry:  {" << currentOdo.position.x << ", " << currentOdo.position.y << "}");
                RCLCPP_INFO_STREAM(this->get_logger(), 
                    "Distance to Goal: " << dist2Goal);
                RCLCPP_INFO_STREAM(this->get_logger(), 
                    "Current Speed: {!!" << currentSpeed << "}");

                if(dist2Goal <= tolerance_)
                {
                    brake = max_brake_torque_;
                    throttle = 0;
                    break;
                }

                if(currentSpeed > 2.5)
                {
                    throttle = 0;
                }
                
                if(dist2Goal <= 5)
                {
                    brake = 200;

                    if(currentSpeed < 0.5)
                    {
                        throttle = default_throttle_ / 2; 
                        brake = 0;
                    } else 
                    {
                        throttle = 0;
                    }
                }
                sendCmd(brake, steeringAngle, throttle); // Send command to platform
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
}

void Ackerman::sendCmd (double brake, double steering, double throttle)
{
    std_msgs::msg::Float64 brakeMsg;
    std_msgs::msg::Float64 steeringMsg;
    std_msgs::msg::Float64 throttleMsg;

    brakeMsg.data = brake;
    steeringMsg.data = steering;
    throttleMsg.data = throttle;

    pubBrakeCmd_->publish(brakeMsg);
    pubSteerCmd_->publish(steeringMsg);
    pubThrottleCmd_->publish(throttleMsg);
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin, 
                                        pfms::geometry_msgs::Point goal, 
                                        double& distance, 
                                        double& time, 
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){
 
    bool checkO2D = audi_.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose);
    
    return checkO2D;
}

double Ackerman::distanceToGoal()
{
    currentOdo = getOdometry();
    audi_.checkOriginToDestination(currentOdo, goal_, distance_, time_, estimatedPose);
    
    return distance_;
}

double Ackerman::timeToGoal()
{
    currentOdo = getOdometry();
    audi_.checkOriginToDestination(currentOdo, goal_, distance_, time_, estimatedPose);
    
    return time_;
}

double Ackerman::distanceTravelled(){
    double dist = 0;
    double time = 0;
    currentOdo = getOdometry();
    pfms::geometry_msgs::Point posAtEnd = {currentOdo.position.x, currentOdo.position.y}; // Get the position at the end of run 
    audi_.checkOriginToDestination(odo_, posAtEnd, dist, time, estimatedPose);
    
    distanceTravelled_ += dist; // Add distance travelled to total distance travelled
    odo_ = currentOdo; // Update odometry                
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Sleep for 500 ms 

    return distanceTravelled_;
}