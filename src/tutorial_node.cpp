#include "tutorial_node.h"

TutorialNode::TutorialNode()
    : nodeHandle_(), privateNodeHandle_("~"), numCount_(0) {
  // initialize the ros parameters
  privateNodeHandle_.param("publishing_interval", publishingInterval_, 1.0);
  privateNodeHandle_.param("number_incrementer", numberIncrementer_, 3.0);
  // initalize timers
  countTimer_ = nodeHandle_.createTimer(ros::Duration(publishingInterval_),
                                        &TutorialNode::incrementCount,this);

  /* initialize the topic and service publisher and subscriber */
  // topic subscriber
  countSubscriber_ =
      nodeHandle_.subscribe("/adder_in", 1, &TutorialNode::adderCallback_, this);
  // service publisher
  countResetService_ =
      nodeHandle_.advertiseService("reset_count", &TutorialNode::resetCount_,this);
  // topic publisher
  countPublisher_ = nodeHandle_.advertise<std_msgs::Int32>("/counter_out", 1);
  adderPublisher_ = nodeHandle_.advertise <std_msgs::Float32>("/adder_out", 1);
}

/**
 * This callback method is called when the node recieves a count data
 *
 * @param &msg is the timer action that triggers the event
 */
void TutorialNode::adderCallback_(const std_msgs::Float32::ConstPtr &msg) {
  float numberRecieved = msg->data; // extracts the data from the message
  ROS_INFO("The number recieved is %f", numberRecieved); 

  ROS_INFO("Adding the number recieved by %f", numberIncrementer_);

  numberRecieved += numberIncrementer_; // adds the subscribed data
  ROS_INFO("The sum is = [%f]", numberRecieved);

  numberAdderMessage_.data = numberRecieved; // puts the data to the message.
  adderPublisher_.publish(numberAdderMessage_); // publishes the message to the topic
}

/**
 * This method publishes the count to the ros param server
 */
void TutorialNode::publishCount() {
  ROS_INFO("Publishing counter... [%d]", numCount_);
  numberCountMessage_.data = numCount_; // inserts the data to the ros message
  countPublisher_.publish(numberCountMessage_); // the publisher publishes the message and its contents.
}

/**
 * This method increments the count by 1 every time the timer triggers an event
 */
void TutorialNode::incrementCount(const ros::TimerEvent &) {
  numCount_ += 1;
  publishCount();
}

/**
 * This method resets the number of counts if the exampleService service is called
 * The return type of a ros service must be a boolean.
 */
bool TutorialNode::resetCount_(tutorial_node::exampleServiceRequest &req,
                  tutorial_node::exampleServiceResponse &res){
  ROS_INFO("Resetting count");
  if (req.reset_counter) {
    numCount_ = 0;
  }
  ROS_INFO("Count has been reset to %d", numCount_);
  return true;
}

/**
 * Main function to initialize the ros node and construct the node class
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "tutorial_node");
  TutorialNode node;
  ros::spin();
  return 0;
}