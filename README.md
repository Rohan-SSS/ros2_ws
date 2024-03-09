# ROS_WS
This repo provides a basic publisher and subscriber example in ROS2 along with a turtlebot3 which navigates to set waypoints. The pub/sub package has a talker node which publishes a message and a listener node which subscribes and listens to the message. 

The turtlebot waypoint simulation uses a script to launch a gazebo instance with turtlebot world and another script to launch nav2, rviz2 and an AMCL node which initializes the turtlebot's position. With a navigator script which uses the CommanderAPI of navigation2, a node is created which sends navigation goals to the rviz2. 

## Installation

### Pre-requisites
- Install ROS2 humble from the [ROS2 installation guide.](https://docs.ros.org/en/humble/Installation.html)

- Install Gazebo sim from the [Gazebo installation guide.](https://gazebosim.org/docs/garden/ros_installation)

- Install Navigation2 binaries from the [Nav2 installation guide.](https://navigation.ros.org/getting_started/index.html)

- Install VCS Tool from [here](http://wiki.ros.org/vcstool)

- Install colcon build tool from [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon)

After installing all the pre-requisites, source the ROS2 `setup.bash` file to load all the binaries and libraries
```sh
source /opt/ros/humble/setup.bash
``` 

### Setting up your own project
Create your workspace
```sh
mkdir -p ~/ros_ws/src
cd ~/ros_ws
```

Copy your repo files in the workspace, here `turtlebot3.repos` to build the turtlebot3 into your workspace.
```sh
vcs import src < turtlebot3.repos
```

Then build and source your workspace using 
```sh
colcon build
source install/setup.bash
```
Your turtlebot is now setup as a package in your workspace.

You can create your own package using the command below, make sure you are in your src folder before creating a package.
```sh 
cd src/
ros2 pkg create --build-type ament_python <package-name> --dependencies rclpy
```
Now you can develop your own package as you wish.


## Using the repo
Make sure you have installed all the dependencies and sourced the ROS2 setup file, now clone the repo and cd into it.
```sh
git clone https://github.com/Rohan-SSS/ros2_ws
cd ros_ws
```
Build the packages and source the workspace
```sh
colcon build --symlink-install 
source install/setup.bash
```
If you see a warning after building or a stderror, you can ignore it.

### Running the hello_ros2 package
Open 2 terminals and cd into the ros_ws, also source the `install/setup.bash` and run the following command in a terminal to start the talker or publisher node.
```sh
ros2 run hello_ros2 talker 
```
and in the other terminal run the following command to start the listener or subsrciber node. 
```sh
ros2 run hello_ros2 listener
```

### Running the turtlebot waypoint simulation
Open 3 terminals and cd into the ros_ws, also source the `install/setup.bash` and run the following command in a terminal to start the turtlebot and world in gazebo.
```sh 
ros2 launch turtlebot3_sim turtlebot3_world.launch.py 
```
and in another terminal run the following command to start rviz2 which also start the nav2 and an AMCL node.
```sh
ros2 launch turtlebot3_sim nav2.launch.py
```
After both the gazebo and rviz2 successfully starts, now in another terminal run the following command to create a Node which sends goals or waypoints to the navigation stack.
```sh
ros2 run turtlebot3_sim nav_to_pose 
```
Now the turtlebot would be navigating the set waypoints.


