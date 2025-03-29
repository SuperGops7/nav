# Trajectory Capture and Visualization Package

## Overview
ROS2 package for capturing and visualizing robot trajectories. Records odometry data and provides visualization tools.

## Installation

### Clone the Repository
```
git clone https://github.com/SuperGops7/nav.git
```
### Install required dependencies and build the package
```bash
cd /nav/nav_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

## Directory Structure

The entire workspace consists of three packages:

### turtlebot3_simulations

Forked from the official [ROBOTIS/turtlebot3 repository](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble).
We use this as the AMR with SLAM and Navigational capabilities to save the trajectory of. This helps in testing and valdiating the trajectory capturing capability of the solution.

To get the gazebo simulation of the Turtlebot3 Waffle up and running, run the following
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

To launch rviz to set Nav2 goals:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<path-to-map>/map.yaml
```
Replace <path-to-map> with the path to the map parameters file (can be found in /nav_ws/src/trajectory_capture/maps).

### trajectory_capture
The trajectory_Capture package houses two nodes
**trajectory_capture** - to capture the trajectory information
**trajectory_publisher** - to visualize the captured trajectories.

To capture, run the following:
```bash
ros2 run trajectory_capture trajectory_capture
```

To visualize, run the following:
```bash
ros2 launch trajectory_capture map_launch.py
ros2 run trajectory_capture trajectory_publisher
```

These nodes have been written with very minimal overhead, to ensure quick and fast execution.

### trajectory_interfaces
To allow for the user to specify the name of the filename to save to, and the duration of recording, we create our custom service, which is defined in this package.

The custome service takes the following structure:
#### Request
string **filename** - name of file, is also used to determine the type of file (.csv, .json or .yaml) and parse the trajectory points as per the format

double **duration** - specify the duration of recording.

#### Response

bool **success** - specify whether any errors were met with.
string **message** - to respond with error codes, if any.

To call the service, use the following command. The user can change the filename and duration as per their need.

```bash
ros2 service call /trajectory_capture_service turtlebot_interfaces/srv/TrajectoryRequest "{filename: "test.yaml",  duration: 40.0}"
```
A custom struct has also been create to capture trajectory points, with the following structure:

```cpp
struct TrajectoryStruct {
  double x;
  double y;
  double z;
  int32_t sec;
  uint32_t nsec;
};
```

This is done to ensure only the required details to visualize are captured, avoiding storing unwanted details such as twist information, saving memory.

## Further improvements

1. Dubugging the issue where map does not load during visualization
2. Dockerizing the entire workspace in a container to facilitate better deployability
3. Add more jobs to the current Github Action workflow to impove code quality and CI/CD compliance.

## Demo
A video run of the entire workflow can be found [here](https://github.com/SuperGops7/nav/blob/master/trajectory_capture.webm)

## Contact
For any clarifications or to report any issues, please reach out to me.