
# A Gentle Introduction to ROS2 and Gazebo

The goal of the session is to get a helocopter view of what [ROS2](https://docs.ros.org/en/galactic/index.html) is and how it works. To do that we are going to leverage the official [Tutorials](https://docs.ros.org/en/galactic/Tutorials.html) and the [Gazebo](https://gazebosim.org/home) simulation environment. To get get up and running quickly and to avoid installing everything locally we've created a Docker image that has everything we need. 
* [ROS2 Galactic Geochelone](https://docs.ros.org/en/galactic/index.html)
* [Gazebo Simulation Environment](https://gazebosim.org/home)
* [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

TurtleBot is a low-cost, personal robot kit with open-source software. Instead of purchasing expensive hardware to try some things out we are going to simulate a TurtleBot3 instead! The simulator is complete with LIDAR, a camera, a gyro and many other sensors and actuators.

# Setup

## 1. Clone the repository
The first thing we need to do is to clone this repository contaning the tutorial. It contains a Dockerfile that creates an image providing an HTML5 VNC interface to access a Ubuntu LXDE environment. Clone the repository using the following URL
```
git clone ros2-turtlebot3-sim
```
Now we need to use git to update the submodules containing the ROS2 tutorials
```
git submodule update --init --recursive
```

## 2. Get the docker image

It is recommended to use the prebuilt image from dockerhub. This is the quickest option and lets you get going now by running the command  
```
docker pull andrewpresland/ros2-turtlebot3-sim:latest
```
If you have you have time of want to make changes you can build it yourself by navigating to the repositiry directory and running the command  
```
docker build -t turtlebot3 .
```

## 3. Run the container
Once the image is built run the container. If you are running on Windows replace ```$PWD``` with ```%cd%``` for current directory. 
```
docker run --rm -it -v $PWD/workspace:/home/ubuntu/dev_ws -p 6080:80 andrewpresland/ros2-turtlebot3-sim
```

## 4. Access the Desktop
When the image has booted go to http://127.0.0.1:6080/ in any browser to be greeted by the VNC desktop goodness.
![](/assets/vnc-desktop.png?raw=true "VNC desktop")

## 5. Build some packages
One last thing needs to be done before we can get hands-on with the Tutorials. We need to build some base packages from the source code already installed in the Docker. We just need to open a terminal in VNC desktop (Menu -> System Tools -> LX Terminal) and build the stuff.

Navigate to the ROS2 workspace inside LX Terminal
```
cd ~/dev_ws
```
Build and install (ignore the warnings).  
```
colcon build --symlink-install
```

# Tutorials
Now that we have everything setup we are ready to look at the [ROS2 Tutorials](https://docs.ros.org/en/galactic/Tutorials.html). These will help you learn about ROS2 with some hands-on exercises. Everything you need should already be installed in the container and you can work through at your own pace. There are a lot of turorials so do not expect to complete them all. We have some resources setup we can come back on another day and continue the fun.

# Gazebo Simulation
We have also installed a Gazebo simulation of the Turtlebot3. To run it first setup the environment to run the simulation (make sure you are inside ```dev_ws```).
```
source ./install/setup.bash
source ./setenv_gazebo.bash
```
Now you can use the simulation by launching with different worlds:
* ```turtlebot3_house.launch.py```
* ```turtlebot3_world.launch.py```
* ```empty_world.launch.py```

```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
![](/assets/gazebo-sim.png?raw=true "Gazebo simulation")

Now its time to interact with the simulation so start another LX Terminal and setup the environment once again
```
source ./install/setup.bash
```
Launch keyboard teleop and use the keys (a,w,d,x,s) as instructed to send command velocities to the TurtleBot3 and make it move around its world.
```
ros2 run turtlebot3_teleop teleop_keyboard
```