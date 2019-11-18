
#include "TurtleBa.hpp"

TurtleBa::TurtleBa() {
    // Initialize the obstacle checker as False
    obstacleCheck = false;
    
    // Publish the velocity to the discovered topic: cmd_vel_mux/input/navi
    publishVelocity = nh.advertise <geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
    
    // Subcribe to the /scan topic and use the scannerCallback function
    subscribeSensor = nh.subscribe <sensor_msgs::LaserScan> ("/scan", 500, &WalkerAlgorithm::sensorCallback, this);
    
    // Initialize the turtlebot to be at rest
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    
    // Send the initial data to the robot
    publishVelocity.publish(msg);
}

void TurtleBa::sensorCallback (const sensor_msgs::LaserScan::ConstPtr& data) {
    double safeDistance = 0.8;
    for (auto distRange : data->ranges) {
        if (distRange < safeDistance) {
        obstacleCheck = true;
        return;
        }
    }
    obstacleCheck = false;
}

bool TurtleBa::isObstacle() {
    return obstacleCheck;
}

void TurtleBa::runAlgorithm() {
    // Set Publish frequency
    ros::Rate freqRate(10);
    
    while (ros::ok()) {
        // Check for obstacle and react as defined
        if (isObstacle()) {
            ROS_INFO_STREAM("Obstacle in path! Turning right.");
            // Set forward to zero and angular turn to one
            msg.linear.x = 0.0;
            msg.angular.z = 2.0;
        } else {
            ROS_INFO_STREAM("Moving forward.");
            msg.linear.x = 0.5;
            msg.angular.z = 0.0;
        }

        // Send the data to the robot
        publishVelocity.publish(msg);

        // Give ont-time control to ROS
        ros::spinOnce();

        // Sleep the remainder of the loop time
        freqRate.sleep();
    }
}

TurtleBa::~TurtleBa() {
  // Set all velocity values to zero
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  // Send zero values to turtlebot
  publishVelocity.publish(msg);
}


