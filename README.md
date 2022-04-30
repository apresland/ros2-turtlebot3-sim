
# Simulate TurtleBot3 in Gazebo

TurtleBot is a low-cost, personal robot kit with open-source software. There are 3 versions of the TurtleBot model. TurtleBot1 was developed by Tully (Platform Manager at Open Robotics) and Melonee (CEO of Fetch Robotics) for ROS deployment. In 2012, TurtleBot2 was developed by Yujin Robot based on the research robot, iClebo Kobuki. In 2017, TurtleBot3 was developed with features to supplement the lacking functions of its predecessors, and the demands of users. The TurtleBot3 adopts ROBOTIS smart actuator DYNAMIXEL for driving

Instead of purchasing expensive hardware to try some things out we are going to simulate a TurtleBot3 instead! The simulator is complete with LIDAR, a camera, a gyro and many other sensors and actuators. To get get up and running quickly and to avoid installing everything locally we've created a Docker image that has everything we need. 
* [ROS2 Galactic Geochelone](https://docs.ros.org/en/galactic/index.html)
* [Gazebo Simulation Environment](https://gazebosim.org/home)
* [TurtleBot3 Gazebo Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

## 1. Clone the repository
The first thing we need to do is to clone this repository contaning the tutorial. It contains a Dockerfile that creates an image providing an HTML5 VNC interface to access a Ubuntu LXDE environment. Clone the repository using the following URL
```
git clone ros2-turtlebot3-sim
```

## 2. Get the docker image

* Option 1: Pull from Dockerhub.  
Use the prebuilt image from dockerhub. This is the quickest option and lets you get going now by running the command  
```docker pull andrewpresland/ros2-turtlebot3-sim:latest```


* Option 2: Build it yourself.  
Enter the repository directory and build the docker image. Navigate to the repositiry directory ```ros2-turtlebot3-sim``` and run the command  
```docker build -t turtlebot3 .```

## 3. Run the container
Once the image is built run the container and when the image has booted go to http://127.0.0.1:6080/ in any browser to be greeted by the VNC desktop.
```
docker run -it -p 6080:80 turtlebot3
```
![](/assets/vnc-desktop.png?raw=true "VNC desktop")

## 4. Install the TurtleBot3 simulation
One last thing needs to be done before we can launch the simulation and that is to compile and install the TurtleBot3 Gazebo simulation packages. The source code is already intalled in the Docker image so we just need to open a terminal in VNC desktop.
```
Menu -> System Tools -> LX Terminal.
```
Navigate to the ROS2 workspace inside LX Terminal
```
cd ~/workspace
```
Build and install (ignore the warnings).  
```
colcon build --symlink-install
```

## 5. Run the Gazebo simulation
Once the install is complete make sure the environment is setup to run the simulation (make sure you are inside ```/home/ubuntu/workspace```).
```
source ./setup.bash
```
Finally launch a world in Gazebo
```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
Change the simulation by launching with different worlds:
* ```turtlebot3_house.launch.py```
* ```turtlebot3_world.launch.py```

![](/assets/gazebo-sim.png?raw=true "Gazebo simulation")

## 6. Interact with the TurtleBot3
Now start another LX Terminal and setup the environment once again
```
source ./setup.bash
```
Launch keyboard teleop and use the keys (a,w,d,x,s) as instructed to send command velocities to the TurtleBot3 and make it move around its world.
```
ros2 run turtlebot3_teleop teleop_keyboard
```
