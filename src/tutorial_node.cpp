#include "tutorial_node.h"

TutorialNode::TutorialNode() : nodeHandle_(), privateNodeHandle_("~"), numCount_(0)
{
  // initialize the ros parameters
  privateNodeHandle_.param("publishing_interval", publishingInterval_, 1.0);
  // TODO: declared variable to hold the variable on how much to increment by, read the ros param here
  // and put into your declared variable

  // initalize timers
  // TODO: we just read a ros param on our desired publishing interval for the counter, put the param here
  // TODO: whenever the timer ticks, we want to run the function to increment the counter, put the function here
  countTimer_ = nodeHandle_.createTimer(ros::Duration(/*******/), &TutorialNode::/********/, this);

  /* initialize the publisher and subscriber */
  // topic subscriber
  // TODO: this node subscribes to /adder_in, define the subscriber here
  myNameSubscriber_ = nodeHandle_.subscribe("/my_name", 5, &TutorialNode::myNameCallback, this);

  // topic publisher
  // TODO: this node publishes on /counter_out, define the publisher here
  adderPublisher_ = nodeHandle_.advertise<std_msgs::Float32>("/adder_out", 1);
}

// Example of a cool algorithm that you will run
double TutorialNode::doCoolStuff(double num1, double num2)
{
  num1 += num2;  // adds the subscribed data
  ROS_INFO("The sum is = [%f]", num1);

  return num1;
}

/**
 * This callback method is called when the node recieves a count data
 * @param &msg is the timer action that triggers the event
 */
// TODO: define your adder_in callback here
void TutorialNode::/*******/(const std_msgs::Float32::ConstPtr& msg)
{
  double numberReceived = msg->data;  // extracts the data from the message

  ROS_INFO("The number recieved is %f", numberReceived);
  ROS_INFO("Adding the number recieved by %f", numberIncrementer_);

  // TODO: pass your subscribed message and ros param to your cool algorithm
  double output = /*************/;

  numberAdderMessage_.data = output;             // puts the data to the message.
  adderPublisher_.publish(numberAdderMessage_);  // publishes the message to the topic
}

/**
 * This method publishes the count
 */
void TutorialNode::publishCount()
{
  ROS_INFO("Publishing counter... [%d]", numCount_);
  // TODO: publish the counter
}

/**
 * This method increments the count by 1 every time the timer triggers an event
 */
void TutorialNode::incrementCount(const ros::TimerEvent&)
{
  numCount_ += 1;
  publishCount();
}

/**
 * This method reads the topic and store it to a class member variable
 */
void TutorialNode::myNameCallback(const std_msgs::String::ConstPtr& msg)
{
  // TODO: extract the string from the message and put it into the member variable myName_
}

/**
 * Main function to initialize the ros node and construct the node class
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tutorial_node");
  // TODO: create the object of your class
  ros::spin();
  return 0;
}