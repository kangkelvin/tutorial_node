// TODO: basic C++ discipline, add include guard here

// Include files for ROS libraries.
#include <ros/console.h>
// TODO: we are using ROS, so add the ros.h include here
#include <ros/timer.h>
#include <std_msgs/Float32.h>
// TODO: we are using ros standard messages Int32, so include it here
#include <std_msgs/String.h>

// Include files for C++ libraries
#include <math.h>
#include <algorithm>
#include <chrono>
#include <iostream>

// TODO: C++ discipline, don't use "using namespace" unnecessarily

class TutorialNode
{
public:
  TutorialNode();

private:
  // variables for ros interactions
  ros::NodeHandle nodeHandle_, privateNodeHandle_;
  ros::Subscriber countSubscriber_, myNameSubscriber_;
  // TODO: we have two publishers, one for counter and one for adder, add the missing one here
  ros::Publisher adderPublisher_;
  ros::Timer countTimer_;

  std_msgs::Int32 numberCountMessage_;
  // TODO: we have two things to publish, add the missing message here

  // variable declaration for member attributes
  double publishingInterval_;
  // TODO: we need a variable to hold the variable on how much to increment by, declare it here
  int numCount_;
  int numResets_;
  std::string myName_;

  // function declaration for member functions
  double doCoolStuff(double num1, double num2);
  // TODO: we need a callback function to receive the adder_in messages, declare it here
  void myNameCallback(const std_msgs::String::ConstPtr& msg);
  void publishCount();
  void incrementCount(const ros::TimerEvent&);
};
