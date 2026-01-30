# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS (Robot Operating System) Kinetic workspace for an autonomous wheelchair system. The project implements a complete navigation stack with custom planning algorithms, sensor integration, and motor control.

## Build System

This is a catkin workspace targeting ROS Kinetic with C++11.

### Building the workspace
```bash
catkin_make
```

### Building specific packages
```bash
catkin_make --only-pkg-with-deps <package_name>
```

### Sourcing the workspace
```bash
source devel/setup.bash
```

## System Architecture

### Core Packages

**wheelchair_navigation** - Main integration package containing:
- Motor driver interface (Roboteq motor controllers)
- Launch files for complete system startup
- Custom messages: `MotorMonitor`, `MotorReference`, `SpeedReference`
- Custom service: `ModeChange`
- Python nodes for odometry (`encoder2tf.py`), motor control (`twist2motors.py`), sensor filtering
- C++ executable: `wheelchair_node` (built from ChairNode.cpp, RoboteqDevice.cpp, RoboteqExtended.cpp, jute.cpp)

**fgm_plugin** - Fuzzy Gap Method local planner plugin:
- Implements `nav_core::BaseLocalPlanner` interface
- Plugin class: `fgm_plugin::FGMPlannerROS`
- Source files: `fgm_plugin.cpp`, `PID.cpp`
- Python support nodes: `danger_factor.py`, `fuzzy1.py`, `fuzzy2.py`, `scaled_linear_velocity.py`
- Subscribes to `/scaled_lin_vel`, `/odom`, `/inflated_pseudo_scan`, `/amcl_pose`
- Publishes to `/distance_to_goal`, `/angular_vel_output`
- Uses PID controller for heading control with configurable parameters

**rrtstar_plugin** - RRT* global planner plugin:
- Implements `nav_core::BaseGlobalPlanner` interface
- Plugin class: `RRTStarPlugin::RRTStarPlanner`
- Uses OMPL library with Dubins state space for planning
- Requires OMPL dependency (`find_package(ompl REQUIRED)`)

**perception** - Perception and sensor processing:
- Object segmentation (`object_segmenter.cpp`)
- Laser scan inflation (`laser_inflater.cpp`)
- Point cloud transformation (`point_transform_laser.cpp`)
- IMU transform publishing (`imu_static_transform_publisher.cpp`)

**ira_laser_tools** - Third-party laser scan utilities:
- `laserscan_multi_merger`: Merges multiple laser scans into one
- `laserscan_virtualizer`: Creates virtual laser scans from point clouds
- Used for combining multiple LiDAR sensors

**rplidar_ros** - Third-party RPLidar driver package

**laser_filters** - Laser scan filtering utilities

## Running the System

### Full system launch
```bash
roslaunch wheelchair_navigation wheelchair.launch
```

This master launch file starts:
- Robot state and joint state publishers with URDF model
- Map server
- Motor driver node (`wheelchair_node`)
- Odometry publisher
- RealSense camera with perception pipeline
- RPLidar sensor
- Sensor filtering (RealSense and RPLidar filters)
- Laser scan merger
- AMCL localization
- move_base with FGM local planner and RRT* global planner
- Fuzzy planner support nodes
- RViz visualization

### Navigation components only
```bash
roslaunch wheelchair_navigation move_base.launch
```

### Keyboard teleoperation
```bash
roslaunch wheelchair_navigation keyboard_teleop_chair.launch
```

## Plugin System

### Registering the planners in move_base

The system uses custom planner plugins registered via pluginlib:

**Global Planner:**
```xml
<param name="base_global_planner" value="RRTStarPlugin/RRTStarPlanner" />
```

**Local Planner:**
```xml
<param name="base_local_planner" value="fgm_plugin/FGMPlannerROS" />
```

Alternative DWA planner is available but commented out in move_base.launch.

### Plugin configuration files
- `fgm_plugin.xml`: Defines FGM local planner plugin
- `rrtstar_plugin.xml`: Defines RRT* global planner plugin

## Configuration

Parameter files are located in `wheelchair_navigation/param/`:
- `fgm_planner_params.yaml`: FGM planner configuration
- `costmap_common_params.yaml`: Shared costmap settings
- `global_costmap_params.yaml`: Global costmap configuration
- `local_costmap_params.yaml`: Local costmap configuration
- `move_base_params.yaml`: move_base behavior parameters
- `dwa_local_planner_params.yaml`: DWA planner (alternative, currently unused)
- Laser filter configs: `laser_rs.yaml`, `laser_front_rp.yaml`, `laser_back_rp.yaml`, etc.

## Sensor Setup

The wheelchair uses multiple sensors:
- **RealSense camera**: Depth sensing and IMU (launched via perception package)
- **RPLidar**: 2D laser scanning (front and possibly back units)
- Sensors are merged using `ira_laser_tools` merger node
- RealSense depth is converted to pseudo laser scan and inflated

## Important Implementation Details

### FGM Planner Architecture
- Uses look-ahead distance to select waypoints from global plan
- Implements gap-based navigation using laser scan data
- PID heading controller with configurable gains (default P=0.75)
- Receives scaled linear velocity from fuzzy system
- Goal distance tolerance parameter: `goalDistTolerance_`

### RRT* Planner Architecture
- Uses Dubins curves for non-holonomic planning (minimum turning radius: 0.9)
- Integrates with costmap for obstacle avoidance
- State validity checking against costmap

### Motor Control Chain
1. Global and local planners output `/cmd_vel` (geometry_msgs/Twist)
2. `twist2motors.py` converts twist to motor commands
3. `wheelchair_node` interfaces with Roboteq motor controllers
4. `encoder2tf.py` publishes odometry from wheel encoders

## URDF and Transforms

Robot description is loaded from `wheelchair_navigation/urdf/wheelchair.urdf`. The system maintains transforms between:
- base_link (wheelchair base frame)
- Camera frames (RealSense)
- IMU frame
- Laser scanner frames (RPLidar units)

Transform handling node: `transform_camera_imu_to_baselink.py`

## Maps

Map files are stored in `wheelchair_navigation/maps/`. Default map loaded via `wheelchair.launch` is `map.yaml`.

## RViz Configuration

Default RViz config: `wheelchair_navigation/rviz/wheelchair.rviz`
