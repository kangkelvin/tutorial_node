# Tutorial Node
This is the tutorial node to get you started writing your own node using ROS in C++. This node is a template that you should follow when writing your own ROS code.

## Getting Started
Make sure to install ROS in your system by following the instruction in this [website](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).The link will guide you on how to install ROS and create a workspace folder in your system
>:bulb: Follow these steps carefully step by step, failure in doing so will end up in a faulty system.

Match the tutorials with your system specification (i.e. using **Ubuntu 18.04, ROS Melodic**)

## Cloning the repository
To install this node, you should clone it from the github repository in this [link](https://github.com/a-i-lab/tutorial_node).

Navigate to your `catkin_ws/src` folder
```
cd ~/catkin_ws/src
```
Clone the repository to your local machine
```
git clone https://github.com/a-i-lab/tutorial_node
```

## Building and Compiling ROS node
catkin_make is a build automation tool that is used to build and compile ros nodes.

To compile a workspace, let's say your `catkin_ws` folder, you type in this command in your `catkin_ws` folder.

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

>:bulb: Repeat this process every time you make changes to your code, including when you add another ros package to your system, or making small changes inside your source code.
---
## What the tutorial_node suppose to do

1. Prints out a `counter` that works based on a timer.
2. Adds the value from the `/adder_in` topic.
3. Outputs the counter value in the `/counter_out` topic
4. Outputs the adder result to the `/adder_out` topic

## What am I suppose to do?

1. Fill up all the TODO inside all these files in this order:
```bash
  include/tutorial_node.h
  src/tutorial_node.cpp
  CMakeLists.txt
  package.xml
  launch/tutorial_node.launch
```

2. Go back to your workspace directory and try to run `catkin_make`, debug your code until it is successfully compiled

3. Launch the node

4. Your node is expecting an input at the topic `/adder_in`, manually publish a msg at this topic using `rostopic pub` at 10Hz

5. Use the ros tools you have learnt to understand what is going on with the node

6. Afterwards, write your own ROS node using C++. You are going to make a simple substractor and multiplier node. 

   Here are some of the requirements:

   1. Subscribe to the `/adder_out` and the `/counter_out` topic
   2. Create a rosparam `substractorValue` with value of `5`
   3. Create a rosparam `multiplyFactor` with value `5.6` 
   4. Subtract the data you receive from the `/adder_out` message with the `substractorValue`. Publish the result on the topic `/result/subtract` as `std_msgs::Float64` at every `/adder_out` callback.
   5. Multiply the latest data you receive from the `/counter_out` message with the `multiplyFactor` and publish it on the topic `/result/multiply` as the linear x velocity in  `geometry_msgs::Twist` at 5Hz
   6. [**OPTIONAL**] Generate a custom message to publish both of the messages simultaneously under `/calculate_both` message.

---
## Useful ROS Tools
### 1. [rviz](http://wiki.ros.org/rviz)
![image](https://answers.ros.org/upfiles/14075513525710936.png)
rviz is the visualization application for visualization using 3d space.
You can use rviz to visualize 3d maps, lidar readings, camera output, transformation, and many more.

### 2. [rqt_plot](http://wiki.ros.org/rqt_plot)
![image](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_plot.png)
rqt_plot is a gui application that allow syou to visualize numeric values from your ros topics.

### 3. [rqt_graph](http://wiki.ros.org/rqt_graph)
![image](http://wiki.ros.org/rqt_graph?action=AttachFile&do=get&target=snap_rqt_graph_moveit_demo.png)
rqt_graph is a ROS visualization tool that allows you to see how the topics are connected from one node to another. 
```
rqt_graph
```

### 4. [rostopic](http://wiki.ros.org/rostopic)
1. rostopic pub
rostopic pub allows you to independently publish an information to a topic. 
```bash
rostopic pub <topic_name> <message_type> <message_attributes>
```
For example:
```bash
rostopic pub -r 15 /adder_in std_msgs/Float32 "data: 1.25"
```
This command will publish 1.25 on the topic "/adder_in" at a rate of 15Hz

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

For further info on `rostopic`, look at the ros wiki page for the topic linked in the title of this section

### 5. [roswtf](http://wiki.ros.org/roswtf)
roswtf allows you to quickly check for errors in your ros code.
Just simply type:
```bash
roswtf
```

### 6. [rosmsg](http://wiki.ros.org/Messages)
ROS uses messages to encapsluate data that is trasnferred between ros nodes.
There are many many different data types in ros. There are [native datatypes](http://wiki.ros.org/msg).
<a href="https://ibb.co/rm8WGWC"><img src="https://i.ibb.co/QfS5F52/Screenshot-from-2020-12-02-18-10-08.png" alt="Screenshot-from-2020-12-02-18-10-08" border="0"></a>

There are also packaged datatypes that are more specialized. You should look it up here. [common messages](http://wiki.ros.org/common_msgs?distro=melodic).

### 7. [rosrun](http://wiki.ros.org/rosbash#rosrun)
rosrun is a rosbash command that allows you to run executables from the your ros package.
```bash
rosrun <package> <executable>
```

### 8. [roslaunch](http://wiki.ros.org/roslaunch)
roslaunch is a command that allows you to run ros packages from a ROS launch file. The launch file has the extension `.launch` and iniside, you can put data values for the ros parameters. 

roslaunch will also start roscore if no roscore is started on the machine. roscore is essentially, a ros launch tool to start the core of the ROS system. You can look into it more [here](http://wiki.ros.org/roscore)

### 9. [rosbag](http://wiki.ros.org/rosbag)
rosbag is a ros simulation tool that allows you to record and play back to ROS topics. rosbag is useful for testing you ros nodes, and collecting real time data.
#### Playing a rosbag file
```
rosbag play <bagfile(s)>
```
There are several flags that you can use in the rosbag command, that you can see [here](http://wiki.ros.org/rosbag/Commandline#play)
#### Recording a rosbag file
```
rosbag record <topic_names>
```
For example:
```
rosbag record adder_in adder_out counter_in
```
There are flags that you can set in the rosbag command, you can see them [here](http://wiki.ros.org/rosbag/Commandline#record)

### 10. [rosnode](http://wiki.ros.org/rosnode)
rosnode is a useful tool to obtain information about the node. rosnode allows you to see the nodes that are running in your ros environment, as well as to kill the node processes.
#### 1. rosnode info - to display the status of a node (subscriber, publisher)
```
rosnode info /node_name
```
#### 2. rosnode list - to list all the running nodes
```
rosnode list 
```
#### 3. rosnode kill - to terminate a node
```
rosnode kill <node_name>
```
For further info on `rosnode`, look at the ros wiki page for the topic linked in the title of this section


exclamation: Read the ros wiki for in depth for other ROS related information [here](http://wiki.ros.org/)
