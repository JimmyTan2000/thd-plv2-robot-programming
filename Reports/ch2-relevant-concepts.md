# Relevant concepts

## ROS 2 
ROS 2 stands for Robot Operating System 2. However, contrary to its name, it is not an operating system. It is a middleware framework designed to facilitate the development of robot software. It provides a set of tools and libraries for building modular and distributed robotic systems. In addition, ROS2 can run on top of various operating systems such as Ubuntu, Windows and macOS. Therefore, ROS2 is not a replacement for an operating system, but rather as a layer that runs on top of an operating system to provide robotic-specific functionality.

There are a lot of concepts to learn in order to build a complex robot in ROS2. The most essential concepts however, are Nodes and Topics. 

### Nodes 
Nodes in ROS2 are the basic building blocks. Each node should only be responsible for a single, modular purpose (for example, one node for controlling the wheel motors and another node for controlling a LiDAR sensor). They communicate with each other by publishing and subscribing to other nodes via topics, services, actions or parameters. 

Their interactions can be shown in the GIF below:  

![Interaction of nodes with each other in ROS 2](/media/gifs/Nodes-TopicandService.gif)

### Topics 
Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages. It is one of the many ways in which data is being transfered between different nodes. A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics. 

The interactions of nodes via topics are demonstrated in the GIF below:  

![Interaction of nodes via topics](/media/gifs/Topic-MultiplePublisherandMultipleSubscriber.gif)

## Turtlebot3 Waffle Pi 
It is a popular open-source robot platform designed for education, research, and hobbyist purposes. It is built around the Raspberry Pi single-board computer and features a differential drive system, a 360-degree LiDAR sensor, and a camera for visual input.

The picture below shows Turtlebot3 Waffle Pi and its components:   

![Turtlebot3 Waffle Pi](/media/turtlebot_waffle_pi.png)

Together with ROS 2 mentioned above, we can do amazing tasks such as solving a maze. However, due to some defects of the robot in the lab, we can only test our code purely based on simulation using Gazebo Simulator. 

## Gazebo Simulator 
It is an open source 3D robotics simulator with ODE (Open Dynamics Engine) physics engine built in and it supports OpenGL rendering. For this project, it is used to simulate an environment in which our virtual robot can move freely around. We can use this to build a virtual maze for our robots to solve. This is a very convenient alternative to physical mazes, which can be overwhelmingly time-consuming and difficult to build.

