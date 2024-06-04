#include "controller.h"
#include "audi.h"
#include <cmath>
#include <time.h>
#include <thread>
#include <chrono>

Controller::Controller() : 
    Node("controller"),
    tolerance_(0.5),
    totalTime_(0.0),
    RUNNING_(false)
{
    subOdo_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/orange/odom", 1000, std::bind(&Controller::odoCallback, this, std::placeholders::_1));

    subGoal_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orange/goals", 1000, std::bind(&Controller::setGoals, this, std::placeholders::_1));
}

Controller::~Controller()
{
    // Destructor
}

void Controller::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    std::unique_lock<std::mutex> lock(mtxOdo_);
    odoMsg_ = *msg;
}
void Controller::setGoals(const geometry_msgs::msg::PoseArray& msg)
{
    std::unique_lock<std::mutex> lock(mtxGoals_);
    goals_ = msg;
    goalSet_ = true;
}

pfms::nav_msgs::Odometry Controller::getOdometry(){
    std::unique_lock<std::mutex> lock(mtxOdo_);
    pfms::nav_msgs::Odometry odo{0,0,0,0,0,0,0,0};
    odo.position.x = odoMsg_.pose.pose.position.x;
    odo.position.y = odoMsg_.pose.pose.position.y;
    
    tf2::Quaternion quat(odoMsg_.pose.pose.orientation.x,
                         odoMsg_.pose.pose.orientation.y,
                         odoMsg_.pose.pose.orientation.z,
                         odoMsg_.pose.pose.orientation.w);
    // Convert to yaw 
    double yaw = atan2(2.0 * (quat.x() * quat.y() + quat.w() * quat.z()), 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
    
    if(yaw < 0)
    {
        yaw = 2 * M_PI + yaw;
    }

    odo.yaw = yaw;
    odo.linear.x = odoMsg_.twist.twist.linear.x;
    odo.linear.y = odoMsg_.twist.twist.linear.y;

    mtxOdo_.unlock();
    return odo;
}



double Controller::timeTravelled()
{
    return totalTime_;
}

bool Controller::goalReached() 
{
    pfms::nav_msgs::Odometry currentOdo = getOdometry(); // Retrieve current odometry

    double dx = goal_.x - currentOdo.position.x;
    double dy = goal_.y - currentOdo.position.y;
    double dz = goal_.z - currentOdo.position.z;
    // Return the distance between the current position and the goal to determine if the goal has been reached
    return (sqrt(dx*dx + dy*dy + dz*dz) < tolerance_); 
}

