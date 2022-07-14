### 1 Installation

The system has been tested with  Ubuntu 20.04 (ROS Noetic) and Ubuntu 18.04 (ROS Melodic). For Ubuntu 16.04 (ROS Kinetic) switch to `ubuntu_16_04_kinetic` branch.

Follow the tutorials to [install ROS Noetic for 20.04 or ROS Melodic for 18.04 (desktop-full)](http://wiki.ros.org/ROS/Installation) and to [set up catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Install dependencies. For Ubuntu 20.04 (ROS Noetic):
```
sudo apt-get install git python python3-matplotlib python3-numpy libeigen3-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev protobuf-compiler libnlopt-dev libnlopt-cxx-dev ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-msgs ros-noetic-tf-conversions ros-noetic-eigen-conversions
```
For Ubuntu 18.04 (ROS Melodic):
```
sudo apt-get install git python3-matplotlib python3-numpy libeigen3-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev protobuf-compiler libnlopt-dev ros-melodic-octomap ros-melodic-octomap-ros ros-melodic-octomap-msgs ros-melodic-tf-conversions ros-melodic-eigen-conversions
```

Navigate to the source folder of your catkin workspace, download and build the software:
```
cd ~/catkin_ws/src
git clone https://github.com/B-Bhanu-Teja/Ewok-Planner.git
```
### 2 Settings in trajectory_replanning_example_pointCloud.cpp

Add camera link name and point cloud topic name 
Appropriate comments are provided where to add
```
cd ~/catkin_ws/src/ewok_simulation/src/
gedit trajectory_replanning_example_pointCloud.cpp
cd ../..
catkin_make
```
# Ewok-Planner
Step 1: Download QGC to Control the Drone

Step 2: In PX4-Autopilot/launch/mavros_posix_sitl.launch
change sdf file of vehicle model by removing the eprevious line and adding below line
```bash
    <arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/iris_depth_camera/iris_depth_camera.sdf"/>

```

# How To Run Simulation
In terminal 1 start px4 Autopilot
### Terminal 1:-
```bash
cd PX4-Autopilot/
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch px4 mavros_posix_sitl.launch
```

In same terminal type
```bash
commander takeoff
```


### Terminal 2:-
```bash
cd catkin_ws/
source devel/setup.bash
roslaunch ewok_simulation trajectory_replanning_rishabh.launch
```

In QGC
change Flight mode to offboard mode

To change the waypoints edit this file trajectory_replanning_rishabh.launch 

```bash
 <arg name="start_x" value="0.0"/>
  <arg name="start_y" value="0.0"/>
  <arg name="start_z" value="1"/>
  <arg name="start_yaw" value="$(eval -pi/4)"/>

  <arg name="middle_x" value="0.0"/>
  <arg name="middle_y" value="0.0"/>
  <arg name="middle_z" value="1"/>
  <arg name="middle_yaw" value="$(eval -pi/4)"/>

  <arg name="stop_x" value="17.6"/>
  <arg name="stop_y" value="5.06"/>
  <arg name="stop_z" value="1.0"/>
  <arg name="stop_yaw" value="$(eval -pi/4)"/>
```
