

#ifndef GAZEBO_TESTS_INCLUDE_TURTLEBA_HPP
#define GAZEBO_TESTS_INCLUDE_TURTLEBA_HPP

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class TurtleBa {
 private:
  ros::NodeHandle nh

  ros::Publisher publishVelocity;

  ros::Subscriber subscribeSensor;

  geometry_msgs::Twist msg;

  bool obstacleCheck;

 public:
  TurtleBa();

  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& data);

  bool isObstacle();

  void runAlgorithm();

  ~TurtleBa();
};

#endif  // GAZEBO_TESTS_INCLUDE_TURTLEBA_HPP
