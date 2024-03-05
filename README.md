![Salvo](doc/salvo_header.png)

# Overview

Salvo is a library that allows you to save and visualize the log of your applications. It is compatible with ROS2, ROS (Future release), and ZMQ (Future release). It is possible to extend the logger to any communication layer.

Salvo is composed of:

- an executable, salvo, that saves to file a csv log.
- a QT gui, "Guardo", for visualizing realtime logs and saved log files.
- a bridge header class to use salvo in any cpp class that does not depends on ROS2 but has a base class based on it.

## Future release

- Add ROS1 compatibility
- Add ZMQ communcation layer
- Add option to save log files from the QT GUI
- TUI interface for a quick and fast gui to be used directly in ssh on a terminal.  

## Why Yet another log system library

The main reason for developing Salvo is that it often happens to have same software code on different applications that may have different middleware (ROS1,ROS2, etc). We want a system that uses the generic classes to be used in our application without having the need to change adapt our code to the specific communication layer( publisher, topics, etc). Imagine you have a filter.cpp class and you want to print/publish a log. For ROS1 and ROS2 you need two different software versions.

IF someone already has a ROS1 an ROS2, it is possible to visualize the log with the "Guardo" gui.

### ROS-less header files.

If someone is interested in printing logs in source code files that do not include ROS1/ROS2 headers, you can use our bridge whos goal is to abstract the log data structures and the publishing system. For more info, please refer to the apposite  [section](#class-abstraction)

# Usage

## Install using COLCON

If it is the first time you use Aitronik packages, add the registry:

```bash
git clone <repo>
```

To install, just execute the standard compiling command for ROS2:

```bash
colcon build
```

## Run Salvo in your System

To save a ros2 log to file you can execute salvo:

```bash
Usage: ros2 run salvo salvo [--help] [--version] [--path VAR] [--name VAR] [--rolling] [--maxlines VAR]

Optional arguments:
  -h, --help     shows help message and exits 
  -v, --version  prints version information and exits 
  --path         path folder where to save the log file [nargs=0..1] [default: "."]
  --name         name of the file, it will use the date as default [nargs=0..1] [default: ""]
  --rolling      name of the file, it will use the date as default 
  --maxlines     name of the file, it will use the date as default [nargs=0..1] [default: 1000]
```
 
The actual subscription topic is `/rosout`. If you want to change the subscription topic execute the following command:

```bash
ros2 run salvo salvo --ros-args -p redirect_log_to:=/rosout2
```

## Watch realtime log messages or previously saved log-files

To watch the runtime log you can execute one of the two available GUI, qt or ncurses version (future version). To execute the Qt version please run:

```bash
ros2 run salvo guardo
```

Also here, the actual subscription topic is `/rosout`. you can change the subscription topic as before.

In guardo you can either open a file log, playback a file log (simulates the execution logged in the file), and subscribe to ROS2. Other subscriptions are not yet implemented.


# Class abstraction

## How to log with Salvo if your code does not include ROS2 headers

To use Salvo in a ROS2 node you just need to include "salvo/ros2_log.h" and instantiate a Singleton Ros2Log providing the node name and the rclcpp::shared_node:

```c++
Ros2Log<rclcpp::Node,rclcpp::Publisher>::init(rclcpp::Node nh,std::string nodeName);
```

Similarly, to use Salvo in a ROS2 lifecycle node you need to instantiate the signleton and providing the rclcpp_lifecycle::LifecycleNode

```c++
Ros2Log<rclcpp_lifecycle::LifecycleNode,rclcpp_lifecycle::LifecyclePublisher>::init(rclcpp_lifecycle::LifecycleNode nh,std::string nodeName).activate();
```

Then, inside your code just call the function PRINT_LOG for printing a string, and PRINT_STREAM for using the << operator.

```c++
PRINT_LOG(salvo::Severity::INFO, "Init Complete");
PRINT_STREAM(salvo::Severity::INFO, "Publishing: "<<  message.data );

```

## How to log with Salvo if your code does not include ROS1 headers

We are planning to add the integration with ROS1 soon. Let as know if this feature is interesting to you.

## Feedback

If you have suggestions, questions, or problems, please contact us and/or open an issue. Your feedback is valuable to us! 

