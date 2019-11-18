/*
Copyright (c) 2019, Ari Kupferberg
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**
 *  @file   turtleBa.hpp
 *  @author Ari Kupferberg
 *  @date   11/18/2019
 *  @brief  This is the header file for the TurtleBa Class
 */

#ifndef INCLUDE_TURTLEBA_HPP_
#define INCLUDE_TURTLEBA_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class TurtleBa {
 private:
  ros::NodeHandle nh;  ///< Create a node handle

  ros::Publisher publishVelocity;  ///< Publisher to the turtlebot 'velocity' topic.

  ros::Subscriber subscribeSensor;  ///< Subscriber to turtlebot 'laserscan' topic

  geometry_msgs::Twist msg;  ///< Variable used for publishing velocity messages

  bool obstacleCheck;  ///< Declare a variable to detect poosible collision

 public:
  /**
  *  @brief   This is the constructor for the Class object
  *  @param	  None
  *  @return	None
  */
  TurtleBa();

  /**
  *  @brief   This is the call back function for the subscriber
  *  @param	  data
  *  @return	None
  */
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& data);

  /**
  *  @brief   This is the function that checks if there is an obstacle
  *  @param	  None
  *  @return	boolean
  */
  bool isObstacle();

  /**
  *  @brief   This is the function that runs the implementation
  *  @param	  None
  *  @return	None
  */
  void runAlgorithm();

  /**
  *  @brief   This is the destructor for the Class object
  *  @param	  None
  *  @return	None
  */
  ~TurtleBa();
};

#endif  // INCLUDE_TURTLEBA_HPP_
