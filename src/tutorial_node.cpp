#include "tutorial_node.h"

/** TODO: initialise the variable "numCount_" to 0 in the constructor**/
TutorialNode::TutorialNode() : nodeHandle_(), privateNodeHandle_("~"), numCount_(<insert value here>)
{
  // initialize the ros parameters
  /** TODO: change the name of the parameter to "publishing_interval" and the default value to 1 **/
  privateNodeHandle_.param("<insert value here>", publishingInterval_, <insert value here>);
  privateNodeHandle_.param("number_incrementer", numberIncrementer_, 3.0);

  // initalize timers
  /** TODO: change the timer frequency by adjusting the timer interval to "publishingInterval_"**/
  countTimer_ = nodeHandle_.createTimer(ros::Duration(<insert value here>), &TutorialNode::incrementCount, this);

  /* initialize the topic and service publisher and subscriber */
  // topic subscriber
  /** TODO: change topic name of the subscriber to "/adder_in" and put the right callback function of adderCallback_()**/
  countSubscriber_ = nodeHandle_.subscribe("<insert value here>", 1, <insert value here>, this);
  // service publisher
  countResetService_ = nodeHandle_.advertiseService("reset_count", &TutorialNode::resetCount_, this);
  // topic publisher
  /** TODO: change topic name of the publisher to "/counter_out"**/
  countPublisher_ = nodeHandle_.advertise<std_msgs::Int32>("<insert value here>", 1);
  adderPublisher_ = nodeHandle_.advertise<std_msgs::Float32>("/adder_out", 1);
}

/**
 * This callback method is called when the node recieves a count data
 *
 * @param &msg is the topic received that triggers this function
 */
void TutorialNode::adderCallback_(const std_msgs::Float32::ConstPtr& msg)
{
  /** TODO: take the value of msg and store it in the variable numbersRecieved**/
  float numberRecieved = <insert code here>;  // extracts the data from the message
  ROS_INFO("The number recieved is %f", numberRecieved);

  ROS_INFO("Adding the number recieved by %f", numberIncrementer_);

  numberRecieved += numberIncrementer_;  // adds the subscribed data
  ROS_INFO("The sum is = [%f]", numberRecieved);

  numberAdderMessage_.data = numberRecieved;     // puts the data to the message.
  adderPublisher_.publish(numberAdderMessage_);  // publishes the message to the topic
}

/**
 * This method publishes the count to the ros param server
 */
void TutorialNode::publishCount()
{
  ROS_INFO("Publishing counter... [%d]", numCount_);
  numberCountMessage_.data = numCount_;          // inserts the data to the ros message
  /** TODO: publish the message numberCountMessage_ through the publisher countPublisher_**/
  countPublisher_.publish(<insert code here>);  // the publisher publishes the message and its contents.
}

/**
 * This method increments the count by 1 every time the timer triggers an event
 */
  /** TODO: put the appropriate function parameters**/
void TutorialNode::incrementCount(<insert code here>)
{
  numCount_ += 1;
  publishCount();
}

/**
 * This method resets the number of counts if the exampleService service is called
 * The return type of a ros service must be a boolean.
 */
bool TutorialNode::resetCount_(tutorial_node::exampleServiceRequest& req, tutorial_node::exampleServiceResponse& res)
{
  ROS_INFO("Resetting count");
  if (req.reset_counter)
  {
    numCount_ = 0;
  }
  ROS_INFO("Count has been reset to %d", numCount_);
  return true;
}

/**
 * Main function to initialize the ros node and construct the node class
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tutorial_node");
  TutorialNode node;
  ros::spin();
  return 0;
}