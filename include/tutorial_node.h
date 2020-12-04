#pragma once
// Put the preprocessor directives and include commands here

// Include files for ROS libraries.
#include <ros/console.h>
/** TODO: add ros.h include **/
#include <ros/timer.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

// Include generated message and service file headers.
#include <tutorial_node/exampleService.h>
#include <tutorial_node/exampleMessage.h>

// Include files for C++ libraries
#include <math.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>

class TutorialNode
{
public:
  TutorialNode();

private:
  // variables for ros interactions
  ros::NodeHandle nodeHandle_, privateNodeHandle_;
  ros::Subscriber countSubscriber_;
  ros::Publisher countPublisher_;
  /** TODO: add another ros Publisher with name "adderPublisher_" **/
  ros::Timer countTimer_;
  ros::ServiceServer countResetService_;

  std_msgs::Int32 numberCountMessage_;
  std_msgs::Float32 numberAdderMessage_;

  // declare constants
  const double pi = M_PI;
  
  // variable declaration for member attributes
  double publishingInterval_;
  double numberIncrementer_;
  /** TODO: add a member variable of type integer and name "numCount_" **/
  int numResets_;

  // function declaration for member functions
  void adderCallback_(const std_msgs::Float32::ConstPtr& msg);
  /** TODO: declare a function with name "publishCount" that has no input argument and returns void **/
  void incrementCount(const ros::TimerEvent&);
  bool resetCount_(tutorial_node::exampleServiceRequest& req, tutorial_node::exampleServiceResponse& res);
};
