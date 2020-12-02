# Tutorial Node
This is the tutorial node to get you started writing your own node using ROS in C++. This node is a template that you should follow when writing your own ROS code.

## What this node does
1. Prints out a `counter` that works based on a timer.
2. Adds the value from the `/adder_in` topic.
3. Outputs the counter value in the `/counter_out` topic
4. Outputs the adder result to the `/adder_out` topic

## Useful ROS Tools
### 1. [rviz](http://wiki.ros.org/rviz)
![image](https://answers.ros.org/upfiles/14075513525710936.png)
rviz is the visualization application for visualization using 3d space.
You can use rviz to visualize 3d maps, lidar readings, camera output, transformation, and many more.

### 2. [rqt_plot](http://wiki.ros.org/rqt_plot)
![image](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_plot.png)
rqt_plot is a gui application that allow syou to visualize numeric values from your ros topics.


### 3. rostopic
1. rostopic pub
rostopic pub allows you to independently publish an information to a topic. 
```bash
rostopic pub <topic_name> <message_type> <message_attributes>
```

2. rostopic list
rostopic list will list down all the active topics that are published or subscribed.
```bash
rostopic list
```

3. rostopic echo
rostopic echo will list the values that is currently inside the ros param server.
```bash
rostopic echo <topic_name>
```

4. For further info, look at the ros wiki page for [rostopic](http://wiki.ros.org/rostopic)

### 4. [roswtf](http://wiki.ros.org/roswtf)
roswtf allows you to quickly check for errors in your ros code.
Just simply type:
```bash
roswtf
```

###:exclamation: Read the ros wiki for in depth for other ROS related information [here](http://wiki.ros.org/)

## Getting Started
Make sure to install ROS in your system by following the instruction in this [website](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).The link will guide you on how to install ROS and create a workspace folder in your system
>:bulb: Follow these steps carefully step by step, failure in doing so will end up in a faulty system.

Match the tutorials with your system specification (i.e. using **Ubuntu 18.04, ROS Melodic**)

## Building and Compiling ROS node
catkin_make is a build automation tool that is used to build and compile ros nodes.

To compile a workspace, let's say your `catkin_make` folder, you type in this command in your `catkin_ws` folder.

When you are inside a node directory, 
```
catkin_ws/src/<node_name>
```
Navigate back to the catkin_ws root folder
```
cd ../..
```
Then type in `catkin_make` to start the build process.
```
catkin_make
```
When your build is successful, you should see this on your terminal
<a href="https://ibb.co/b7bQ6wC"><img src="https://i.ibb.co/mSNbRZ2/Screenshot-from-2020-12-02-17-38-02.png" alt="Screenshot-from-2020-12-02-17-38-02" border="0"></a>

## What to do next?
You need to write your own ROS node using C++

You are going to make a simple substractor and multiplier node. 

There are some of the requirements:
1. Subscribe to the `/adder_out` and the `/counter_out` topic
2. Set the `substractorValue` of `5` as a ros param.
3. Set the `multiplyFactor` of `5.6` also as a ros param.
4. Substract the data you recieve from the `/adder_out` message with the `substractorValue`.
5. Multiply the data you recieved from the `/counter_out` message with the `multiplyFactor`.
6. Publish the adder to the `/adder_result` topic.
7. Publish the multiply result to `/multiply_result` topic.
8. [**OPTIONAL**] Generate a custom message to publish both of the messages simultaneously under `/calculate_both` message.

Make sure that the data type for the publiser should be `UInt8` and the multiplier should be `Float64`.