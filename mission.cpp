#include "mission.h"


Mission::Mission() : 
    Node("mission"), 
    chasing_(false), 
    tolerance_(0.5), 
    laserProcessing_(sensor_msgs::msg::LaserScan()),
    distToGoal_(0)
{
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "/orange/mission", std::bind(&Mission::request, this,std::placeholders::_1, std::placeholders::_2));

    subGoal_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orange/goals", 1000, std::bind(&Mission::goalsCallback, this, std::placeholders::_1));

    subOdo_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/orange/odom", 1000, std::bind(&Mission::odoCallback, this, std::placeholders::_1));

    subLaser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/orange/laserscan", 10, std::bind(&Mission::laserCallback, this, std::placeholders::_1));

    pubVisualMarker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/orange/visualization_marker", 10);

    pubCones_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/orange/cones", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Mission::timerCallback, this));
    thread_ = new std::thread(&Mission::run, this);
}

Mission::~Mission()
{
    thread_->join();
}

void Mission::goalsCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg)
{
    geometry_msgs::msg::PoseArray pt = *msg;

    goals_ = pt;

    RCLCPP_INFO_STREAM(this->get_logger(),"Have: " << goals_.poses.size() << " goals");
}

void Mission::odoCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    std::unique_lock<std::mutex> lck(mtxOdo_);
    odo_ = *msg;
}

void Mission::laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
{
    if(laserProcessingPtr_ == nullptr){
        laserProcessingPtr_ = std::make_unique<LaserProcessing>(*msg);
    }
    else{
        laserProcessingPtr_->newScan(*msg);
    }
}

void Mission::run()
{
    while(rclcpp::ok())
    {
        std::unique_lock<std::mutex> lck(mtxLaser_);
        nav_msgs::msg::Odometry odo;
        geometry_msgs::msg::Pose currentPose;
        geometry_msgs::msg::Point goal;

        if(goals_.poses.size()>0)
        {
            std::unique_lock<std::mutex> lck1(mtxOdo_);
            odo = odo_;
            lck1.unlock();
            
            std::unique_lock<std::mutex> lck2(mtxGoals_);
            goal = goals_.poses[0].position;
            lck2.unlock();
        }

        if(chasing_)
        {
            double dist = distance(odo,goal);

            if(dist<tolerance_){
                goals_.poses.erase(goals_.poses.begin());//Removoing element at front
                RCLCPP_INFO(this->get_logger(),"Reached and going for new goal");
                //Now lets get stats of the new goal
                if(goals_.poses.size()>0){                    
                    goal = goals_.poses[0].position;
                    distToGoal_=distance(odo,goal);
                }
            }

            if(goals_.poses.size()==0){
                chasing_=false;
            }
        }
        else
        {
            if(goals_.poses.size()>0){
                chasing_=true;
                goal = goals_.poses[0].position;
                distToGoal_=distance(odo,goal);
                RCLCPP_INFO(this->get_logger(),"Sending first goal");
            }
        }
    
        // std::vector<geometry_msgs::msg::Point> cones = laserProcessing_.getCones(currentPose);
        // if (cones.size() >= 2) 
        // {
        //     conesScanned_.clear();
        //     conesScanned_.push_back(cones[0]);
        //     conesScanned_.push_back(cones[1]);
        // }
        // else
        // {
        //     conesScanned_.clear();
        // }
        // pubConesDetected(pubVisualMarker_, cones);
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));// we sleep for 10 miliseconds
    }    
}

visualization_msgs::msg::MarkerArray Mission::produceMarker(std::vector<geometry_msgs::msg::Point> object, rclcpp::Time stamp){


      visualization_msgs::msg::MarkerArray markerArray;
      visualization_msgs::msg::Marker marker;
      // Set the frame ID and time stamp.
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      //We set lifetime (it will dissapear in this many seconds)
      marker.lifetime = rclcpp::Duration(1000,0); //zero is forever

      switch(markerType_) {
        case MarkerArray::MarkerType::CYLINDER: // Cone marker type

            marker.ns = "CONES"; //.ns = namespace, namespace for cones 

            // Increase ID counter
            marker.id = counter_.cones++;

            marker.type = visualization_msgs::msg::Marker::CYLINDER; // Marker is type CYLINDER, as required by assessment task 

            // Set the orientation of the marker to be 0,0,0,1 with 0 degree orientation(Quaternion)
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0; 

            // Setting and sending coloured marker 
            marker.color.a = 1.0; //a is alpha - transparency 1 is 100%. Best visibility hopefully
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            // Cones need a radius of 0.2m & height 0.5m, scale accordingly 
            marker.scale.x = 0.2; // 0.2m radius
            marker.scale.y = 0.2; // 0.2m radius
            marker.scale.z = 0.5; // 0.5m height

            // Position the object/marker
            marker.pose.position.x = object[0].x; // Set the x position of the marker to the x position of the object
            marker.pose.position.y = object[0].y; // Set the y position of the marker to the y position of the object
            marker.pose.position.z = object[0].z; // Set the z position of the marker to the z position of the object


            break;

        case MarkerArray::MarkerType::CUBE: // Road marker type 

            marker.ns = "ROAD";
            marker.id = counter_.road++;

            marker.type = visualization_msgs::msg::Marker::CUBE; // Marker is type CUBE, as required by assessment task 

            // Set the orientation of the marker to be 0,0,0,1 with 0 degree orientation(Quaternion)
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0; 

            //Send a marker with of colour yellow
            marker.color.a = 1.0;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.0;

            // Road needs to be 0.5m x 0.5m x 0.5m, scale accordingly
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5; 

            // Position the object/marker
            marker.pose.position.x = object[0].x;
            marker.pose.position.y = object[0].y;
            marker.pose.position.z = object[0].z;

            break;

      }
        // Set the marker action to ADD for new markers
        marker.action = visualization_msgs::msg::Marker::ADD;

        markerArray.markers.push_back(marker); // Add the marker to the marker array

        return markerArray; // Return the marker array 
}

void Mission::setMarkerType(MarkerArray::MarkerType type)
{
    markerType_ = type;
} 

void Mission::pubConesDetected(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubCones, 
                               std::vector<geometry_msgs::msg::Point>& cones)
{
    rclcpp::Time stamp = this->get_clock()->now(); // Get the current time stamp
    visualization_msgs::msg::MarkerArray conesMsg = produceMarker(cones, stamp); // Create a MarkerArray message to store a vector of cones detected  
    markerType_ = MarkerArray::MarkerType::CYLINDER; // Set the marker type to cone (CYLINDER)
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = "world";
    msg.header.stamp = this->get_clock()->now();
    for (const auto& cone : cones) {
        geometry_msgs::msg::Pose pose; // Create a new pose for each cone detected
        pose.position.x = cone.x; // Set the x value of the pose to the x value of the cone detected
        pose.position.y = cone.y; // Set the y value of the pose to the y value of the cone detected
        pose.position.z = 0.0;    //  Z position doesn't matter for this project because its on the ground but keep it here for now

        pose.orientation.x = 0.0; // Set the x value of the orientation to 0
        pose.orientation.y = 0.0; // Set the y value of the orientation to 0
        pose.orientation.z = 0.0; // Set the z value of the orientation to 0
        pose.orientation.w = 1.0; // Set the w value of the orientation to 1

        msg.poses.push_back(pose);
    } 

    pubVisualMarker_->publish(conesMsg); // Publish the PoseArray message with the cones detected

}

void Mission::pubRoadCentrePos(const std::vector<geometry_msgs::msg::Point>& roadCentres)
{
     markerType_ = MarkerArray::MarkerType::CUBE; // Set the marker type to road (CUBE)
     rclcpp::Time stamp = this->get_clock()->now(); // Get the current time stamp

    if (!roadCentres.empty()) {
        const auto& roadCentre = roadCentres.front(); 

        produceMarker(roadCentres, stamp); // Add the road centre to the road message

    }
}

void Mission::progress(){
    if(chasing_){
        nav_msgs::msg::Odometry odo;
        geometry_msgs::msg::Point goal;

        std::unique_lock<std::mutex> lck1(mtxOdo_);
        odo = odo_;
        lck1.unlock();
            
        std::unique_lock<std::mutex> lck2(mtxGoals_);
        goal = goals_.poses[0].position;
        lck2.unlock();

        double dist = distance(odo,goal);

        unsigned int progress = (unsigned int)(100.0*((distToGoal_-dist)/distToGoal_));

        RCLCPP_INFO_STREAM(this->get_logger(),"Progress:"<< progress << "%");
    }
}

void Mission::request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    if(laserProcessingPtr_ == nullptr){
        RCLCPP_ERROR_STREAM(this->get_logger(),"No laser data available");
        response->success = false;
        response->message = "ERROR No laser data available";
        return;
    }
    unsigned int readings = laserProcessingPtr_->countObjectReadings();
    RCLCPP_INFO_STREAM(this->get_logger(),"valid readings:" << readings);
    response->success = true;
    response->message = "Readings: " + std::to_string(readings);
}

void Mission::timerCallback(){
    //This is an example of logging, this will be called every 500ms
    RCLCPP_INFO(this->get_logger(), "Timer callback example");
}

void Mission::threadFunction(){
    while(rclcpp::ok()){
        //This function will run every second
         if(laserProcessingPtr_ != nullptr){
            unsigned int readings = laserProcessingPtr_->countObjectReadings(); // readings here is a local variable, so not dealing with threading
            RCLCPP_INFO_STREAM(this->get_logger(),"in thread valid readings:" << readings);
         }
        std::this_thread::sleep_for(std::chrono::seconds(1));// we sleep for 1 second as an example here
    }
}

double Mission::distance(nav_msgs::msg::Odometry odo, geometry_msgs::msg::Point pt){
    pfms::nav_msgs::Odometry updateOdo;
    pfms::geometry_msgs::Point goal;

    updateOdo.position.x = odo.pose.pose.position.x;
    updateOdo.position.y = odo.pose.pose.position.y;
    tf2::Quaternion q(
        odo.pose.pose.orientation.x,
        odo.pose.pose.orientation.y,
        odo.pose.pose.orientation.z,
        odo.pose.pose.orientation.w
    );
    updateOdo.yaw = q.getAngle();
    updateOdo.linear.x = odo.twist.twist.linear.x;
    updateOdo.linear.y = odo.twist.twist.linear.y;

    goal.x = pt.x;
    goal.y = pt.y;

    double time = 0;
    double dist = 0;
    pfms::nav_msgs::Odometry estimatedPose = {0,0,0,0,0,0,0,0};

    audi_.checkOriginToDestination(updateOdo, goal, dist, time, estimatedPose);

    return dist;
}

