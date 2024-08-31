
# A Star Path Planner

## Overview
This repository contains a ROS 2-based path planning system using the A* algorithm. It includes nodes for map publishing, path planning, and a service-client for requesting paths.

**Components**: 
1. map_publisher: Publishes a static map with obstacles.
2. path_planner: Handles path planning requests and publishes the planned path.
3. path_planner_client: Requests a path from the path_planner node.
4. RViz: Visualization tool for displaying the map and path.


## ROS 2 File structure
```
├── a_star_planner.rviz
├── images
│   ├── map_publisher.png
│   ├── path_planner_client.png
│   ├── path_planner_node_startup.png
│   ├── path_planner_result.png
│   └── rviz_output.png
└── path_planner
    ├── CMakeLists.txt
    ├── include
    │   └── path_planner
    │       ├── a_star_planner.hpp
    │       └── path_planner.hpp
    ├── launch
    │   └── path_planner.launch.py
    ├── package.xml
    ├── Readme.md
    └── src
        ├── a_star_planner.cpp
        ├── main.cpp
        ├── map_publisher.cpp
        ├── path_planner_client.cpp
        └── path_planner.cpp

```

## Prerequisites
1. Ubuntu 22.04
2. Ros2 Humble

## Setup

##### Open a 4 terminal 

**1. In terminal A**
  ```
  colcon build
  source install/setup.bash
  ros2 run path_planner map_publisher
  ```
![Map Publisher](/images/map_publisher.png)

**2. In terminal B**
  ```
  source install/setup.bash
  rviz2
  ```
> **Note:** Rviz should use a_star_planner.rviz file configuration.

**3. In terminal C**
  ```
  source install/setup.bash
ros2 launch path_planner path_planner.launch.py
  ```
![Path Planner Start](/images/path_planner_node_startup.png)

**4. In terminal D**
  ```
  source install/setup.bash
ros2 run path_planner path_planner_client
```
![Path Planner Client](/images/path_planner_client.png)

## Output
1. Path planner will server the request came from client and will generate the path provided request is valid.![Path Planner Result](/images/path_planner_result.png)

2. Generated path can be visualized in Rviz.    ![Rviz](/images/rviz_output.png)


#### Developer Information

- **Name:** Shailesh Pawar
- **Contact:** shaileshpawar320@gmail.com