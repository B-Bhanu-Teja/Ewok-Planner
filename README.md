# Ewok-Planner
Steip 1: Download QGC to Control the Drone

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
cd Ewok_ws/
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
