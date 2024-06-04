#include "laserprocessing.h"

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::msg::LaserScan laserScan):
    laserScan_(laserScan), objectReadings_(0)
{
    coneTolerance_ = 0.5; // 50cm tolerance for cones to be considered as one
    roadWidth_ = 8.0; // 8m road width
}



unsigned int LaserProcessing::countObjectReadings()
{
    std::unique_lock<std::mutex> lck(mtx);
    sensor_msgs::msg::LaserScan laserScan = laserScan_;   
    lck.unlock();

    unsigned int count=0;
    std::vector<double> x;std::vector<double> y;
    for (unsigned int i = 0; i < laserScan_.ranges.size(); ++i)
    {
        if ((laserScan_.ranges.at(i) > laserScan_.range_min) &&
            (laserScan_.ranges.at(i) < laserScan_.range_max) &&
            !isnan(laserScan_.ranges.at(i)) &&
            isfinite(laserScan_.ranges.at(i))  ){

            count++;
        }
    }

    objectReadings_=count;
    return objectReadings_;
}

void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan){
    std::unique_lock<std::mutex> lck(mtx);
    laserScan_ = laserScan;    
}

geometry_msgs::msg::Point LaserProcessing::localToGlobal(geometry_msgs::msg::Point localPoint, geometry_msgs::msg::Pose currentPose)
{
    geometry_msgs::msg::Point globalPoint; // Initialise a point in the global plane 
    
    double yawAngle = tf2::getYaw(currentPose.orientation); // Get the yaw angle of the current pose

    globalPoint.x = (localPoint.x * cos(yawAngle) - localPoint.y * sin(yawAngle)) + 
                    (currentPose.position.x);      // Calculate the x coordinate of the global point

    globalPoint.y = (localPoint.x * sin(yawAngle) + localPoint.y * cos(yawAngle)) + 
                    (currentPose.position.y);      // Calculate the y coordinate of the global point
    
    globalPoint.z = 0.0; // Set the z coordinate of the global point to 0 as it is on the ground

    return globalPoint; // Return the global point
}

unsigned int LaserProcessing::countSegmentReadings(geometry_msgs::msg::Pose currentPose) 
{
    unsigned int count = 0; // Initialise count index for number of readings
    if(!cones_.empty()) 
    {
        return cones_.size(); // Return the number of cones detected
    }
    unsigned int i = 1; // Initialise index for the while loop
    // Start at index 1 (0 is the first element) instead of 0 because we are comparing the current element with the previous element
    while (i < laserScan_.ranges.size()) 
    {
        bool init = false;
        bool countStatus = true;
        unsigned int first = i;
        unsigned int last = i; // Initialise the first and last index of the cone
        double distance = 0; // Distance between first and last cone, placeholder

        if (isinf(laserScan_.ranges.at(i - 1)) || isnan(laserScan_.ranges.at(i - 1))) 
        {
            i++; // Increment the index if the previous element is infinite or not a number
            continue; // Continue to the next iteration of the while loop
        }

        while (countStatus && i < laserScan_.ranges.size()) 
        {
            if (isinf(laserScan_.ranges.at(i)) || isnan(laserScan_.ranges.at(i))) 
            {
                countStatus = false; // Exit loop if the current element is infinite or not a number
            }

            else 
            {
                // Convert the polar coordinates of the previous element to cartesian coordinates
                geometry_msgs::msg::Point p1 = localToGlobal(polarToCart(i - 1), currentPose); 

                // Convert the polar coordinates of the current element to cartesian coordinates
                geometry_msgs::msg::Point p2 = localToGlobal(polarToCart(i), currentPose); 

                double differenceX = p2.x - p1.x; // Calculate the difference in x coordinates
                double differenceY = p2.y - p1.y; // Calculate the difference in y coordinates

                distance = sqrt(differenceX * differenceX + differenceY * differenceY); // Calculate the distance between the two points

                if (distance < coneTolerance_) 
                {
                    if (!init) 
                    {
                        first = i - 1; // Set the first index of the cone
                        init = true; // Set the initialisation status to true
                    }
                    // last = i; // Set the last index of the cone
                }

                else 
                {
                    countStatus = false; // Exit loop if the distance between the two points is greater than the cone tolerance
                }
            }

            if (!countStatus) 
            {
                last = i - 1; // Set the last index of the cone
            }
            i++; // Increment the index
        }

        if (first != last) 
        {
            count++; // Increment the count index

            double dx = 0;
            double dy = 0;
            for (unsigned int j = first; j <= last; j++) 
            {
                float angle = laserScan_.angle_min + laserScan_.angle_increment * j; // Calculate the angle of the current element
                dx += laserScan_.ranges[j] * cos(angle); // Add the x coordinate to the sum of x coordinates
                dy += laserScan_.ranges[j] * sin(angle); // Add the y coordinate to the sum of y coordinates
            }

            dx  = dx / (last - first + 1); // Calculate the average x coordinate
            dy  = dy / (last - first + 1); // Calculate the average y coordinate

            cones_.push_back(make_pair(dx, dy)); // Add the average x and y coordinates to the cones vector
        }
    }
    return count; // Return the count index
}

geometry_msgs::msg::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::msg::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}

double LaserProcessing::angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

std::vector<geometry_msgs::msg::Point> LaserProcessing::getCones(geometry_msgs::msg::Pose currentPose)
{
    std::vector<geometry_msgs::msg::Point> conesDetected;
    unsigned int count = countSegmentReadings(currentPose); // Count the number of cones detected
    if (count < 2) // Check there are enough cones.. We need at least 2 (1 pair) for this program 
        {
            return {}; // If count is less than 2, return an empty list 
        }
    double closestDistance = std::numeric_limits<double>::max(); // Set the closest distance to the maximum value of distance
    unsigned int closestIndex = 0; // Set the closest index to 0
    unsigned int secondClosestIndex = 0; // Set the second closest index to 0

    for (size_t i = 0; i < cones_.size(); i++) // Loop through the cones vector
    {
        for (size_t j = i + 1; j < cones_.size(); j++) // Loop through the cones vector (next cone in vector)
        {
            double dx = cones_[i].first - cones_[j].first; // Calculate the difference in x coordinates between cone i and cone j
            double dy = cones_[i].second - cones_[j].second; // Calculate the difference in y coordinates between cone i and cone j
            double distance = sqrt(dx * dx + dy * dy); // Calculate the distance between the two cones

            if (distance < closestDistance) // If the distance between the two cones is less than the closest distance
            {
                closestDistance = distance; // Update the closest distance
                closestIndex = i; // Update the closest index
                secondClosestIndex = j; // Update the second closest index
            }
        }
    }

    geometry_msgs::msg::Point firstCone_Global; // First cone in the closest pair detected in global coordinates
    firstCone_Global.x = cones_[closestIndex].first; // Calculate the x coordinate of the cone in global coordinates
    firstCone_Global.y = cones_[closestIndex].second; // Calculate the y coordinate of the cone in global coordinates
    firstCone_Global.z = 0.0; // Set the z coordinate of the cone to 0
    firstCone_Global = localToGlobal(firstCone_Global, currentPose); // Convert the local cone to global cone

    geometry_msgs::msg::Point secondCone_Global; // Second cone in the closest pair detected in global coordinates
    secondCone_Global.x = cones_[secondClosestIndex].first; // Calculate the x coordinate of the cone in global coordinates
    secondCone_Global.y = cones_[secondClosestIndex].second; // Calculate the y coordinate of the cone in global coordinates
    secondCone_Global.z = 0.0; // Set the z coordinate of the cone to 0
    secondCone_Global = localToGlobal(secondCone_Global, currentPose); // Convert the local cone to global cone

    geometry_msgs::msg::Point firstCone; 
    firstCone.x = firstCone_Global.x; // Set the x coordinate of the first cone
    firstCone.y = firstCone_Global.y; // Set the y coordinate of the first cone
    firstCone.z = firstCone_Global.z; // Set the z coordinate of the first cone to 0

    geometry_msgs::msg::Point secondCone;
    secondCone.x = secondCone_Global.x; // Set the x coordinate of the second cone
    secondCone.y = secondCone_Global.y; // Set the y coordinate of the second cone
    secondCone.z = secondCone_Global.z; // Set the z coordinate of the second cone to 0

    conesDetected.push_back(firstCone); // Add the first cone to the cones vector
    conesDetected.push_back(secondCone); // Add the second cone to the cones vector
    return conesDetected; 
}

std::vector<geometry_msgs::msg::Point> LaserProcessing::getRoadBoundaries(geometry_msgs::msg::Pose currentPose) 
{
    std::vector<geometry_msgs::msg::Point> roadBoundaries;
    for (size_t i = 0; i < laserScan_.ranges.size(); i++) 
    {
        if(!std::isnan(laserScan_.ranges[i]) &&                                 // If the current element is not a number
        laserScan_.ranges[i] != std::numeric_limits<float>::infinity() &&       // If the current element is not infinite
        laserScan_.ranges[i] < laserScan_.range_max)                            // If the current element is less than the maximum range

        {
            float angle = laserScan_.angle_min + laserScan_.angle_increment * i; // Calculate the angle of the current element
            float range = laserScan_.ranges[i];                                  // Get the range of the current element
            double x = range * cos(angle);                                       // Calculate the x coordinate of the current element
            double y = range * sin(angle);                                       // Calculate the y coordinate of the current element

            double maxHeight = 0.5; // Maximum height of the road boundary
            double minHeight = 0.2; // Minimum height of the road boundary
            double maxRadius = 0.2; // Maximum radius of the road boundary
            double minRadius = 0.1; // Minimum radius of the road boundary

            // Now set the control values for boundary dimensions 
            double height = 0.5; // Height of the road boundary
            double radius = 0.2; // Radius of the road boundary

            if ((fabs(x) > maxRadius ||  // If the absolute value of x is greater than the maximum radius
                 fabs(y) > maxRadius ||  // If the absolute value of y is greater than the maximum radius
                 height < maxHeight  ||  // If the height is less than the maximum height
                 radius < maxRadius) ||  // If the radius is less than the maximum radius
                 fabs(x) < minRadius &&  // If the absolute value of x is less than the minimum radius
                 fabs(y) < minRadius &&  // If the absolute value of y is less than the minimum radius
                 height < minHeight &&   // If the height is less than the minimum height
                 radius < minRadius)     // If the radius is less than the minimum radius
            {
                geometry_msgs::msg::Point roadBoundaryPoint; // Create a point for the road boundary
                roadBoundaryPoint.x = x;      // Set the x coordinate of the road boundary point
                roadBoundaryPoint.y = y;      // Set the y coordinate of the road boundary point
                roadBoundaryPoint.z = height; // Set the z coordinate of the road boundary point to the height
                roadBoundaryPoint = localToGlobal(roadBoundaryPoint, currentPose); // Convert the local road boundary point to global road boundary point
                roadBoundaries.push_back(roadBoundaryPoint); // Add the road boundary point to the roadBoundaries vector
            }
        }
    }

    return roadBoundaries; // Return the roadBoundaries vector containing the detected road boundary points
}

std::vector<geometry_msgs::msg::Point> LaserProcessing::getRoadCentre(geometry_msgs::msg::Pose currentPose)
{
     std::vector<geometry_msgs::msg::Point> roadCentrePosition_;

    bool withinReading = false; // Flag for whether the current roadPosition is within the segment read by the laser 
    unsigned int count = 0; // Count of the number of road positions detected
    float maxDistance = std::numeric_limits<float>::max(); // Maximum distance for the road position
    float readingSum_X = 0.0; 
    float readingSum_Y = 0.0; // Sum of the x and y coordinates of the segment
    std::vector<geometry_msgs::msg::Point> readingSum; // Vector to store the road positions detected

    for (size_t i = 0; i < laserScan_.ranges.size() - 1; i++)
    {
        float currentAngle = laserScan_.angle_increment * laserScan_.angle_min + i ; // Calculate the angle of the current element
        float nextAngle = laserScan_.angle_increment * laserScan_.angle_min + (i + 1) ; // Calculate the angle of the next element
        float currentRoadPos_X = laserScan_.ranges[i] * cos(currentAngle); // Calculate the x coordinate of the current element
        float currentRoadPos_Y = laserScan_.ranges[i] * sin(currentAngle); // Calculate the y coordinate of the current element
        float nextRoadPos_X = laserScan_.ranges[i + 1] * cos(nextAngle);   // Calculate the x coordinate of the next element
        float nextRoadPos_Y = laserScan_.ranges[i + 1] * sin(nextAngle);   // Calculate the y coordinate of the next element

        // Distance between two points in a line
        float distance = sqrt(pow(nextRoadPos_X - currentRoadPos_X, 2) + pow(nextRoadPos_Y - currentRoadPos_Y, 2)); // Calculate the distance between the two points

        if (distance < coneTolerance_){ // If the distance between the two points is within the cone tolerance 
        
            if (!withinReading) // If current road position is not not within the laser reasings
            {
                withinReading = true;
            }
            readingSum_X += currentRoadPos_X; // Add the x coordinate of the current road position to the sum of x coordinates
            readingSum_Y += currentRoadPos_Y; // Add the y coordinate of the current road position to the sum of y coordinates
            count++;
        }
        else {
            if (withinReading){ // If the current road position is within the laser readings

                float centre_X = readingSum_X / count; // Calculate the average x coordinate of the segment
                float centre_Y = readingSum_Y / count; // Calculate the average y coordinates of the segment
                
                geometry_msgs::msg::Point roadCCentrePoint; // Create a point for the road centre
                roadCCentrePoint.x = centre_X; // Set the x coordinate of the road centre point
                roadCCentrePoint.y = centre_Y; // Set the y coordinate of the road centre point

                roadCCentrePoint = localToGlobal(roadCCentrePoint, currentPose); // Convert the local road centre point to global road centre point
                readingSum.push_back(roadCCentrePoint); // Add centre position to the readingSum vector

                readingSum_X = 0.0; // Reset the sum of x coordinates
                readingSum_Y = 0.0; // Reset the sum of y coordinates
                withinReading = false; // Set the bool back to false
                count = 0; // Reset the count of road positions detected
            }
        }
    }

    // Now check for last segment 
    if (withinReading){
        float centreX = readingSum_X / count;
        float centreY = readingSum_Y / count;

        geometry_msgs::msg::Point roadCentrePoint;
        roadCentrePoint.x = centreX;
        roadCentrePoint.y = centreY;

        roadCentrePoint = localToGlobal(roadCentrePoint, currentPose);
        readingSum.push_back(roadCentrePoint);
    }

    
    for (size_t i = 0; i < readingSum.size(); i++) { // Loop through each pair of road centre points
        for (size_t j = i + 1; j < readingSum.size(); j++)
        {
            float dx = readingSum[j].x - readingSum[i].x;
            float dy = readingSum[j].y - readingSum[i].y;
            float deltadist = sqrt(dx * dx + dy * dy); // Calculate the distance between the two points

            if ((readingSum[i].y * readingSum[j].y < 0) && (deltadist < maxDistance)){  
                maxDistance = deltadist; // Update the minDistance
                geometry_msgs::msg::Point roadCentrePoint;
               roadCentrePoint.x = (readingSum[i].x + readingSum[j].x) / 2.0;
               roadCentrePoint.y = (readingSum[i].y + readingSum[j].y) / 2.0;
               roadCentrePoint.z = 0.0;
               roadCentrePoint = localToGlobal(roadCentrePoint, currentPose); 
               roadCentrePosition_.push_back(roadCentrePoint); 
            }
        }
    }
    
    if (roadCentrePosition_.size() > 0) // If any road centres detected
    {
        sort(roadCentrePosition_.begin(), roadCentrePosition_.end(), 
        [](const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
        {
            double distance_p1 = sqrt(pow(p1.x, 2) + pow(p1.y, 2));
            double distance_p2 = sqrt(pow(p2.x, 2) + pow(p2.y, 2));

            return distance_p1 < distance_p2; // Sort the road centres based on the distance from the origin

        });
        roadCentrePosition_.resize(1); // Keep only the road centre between the closest pair of cones
    }

    return roadCentrePosition_; // Return the roadCentres vector containing the detected road centre points

}