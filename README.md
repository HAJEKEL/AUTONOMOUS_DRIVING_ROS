# Description and instructions
#### Course title: RO47003 Robot Software Practicals (2022/23 Q1)
#### Name: Henk Jekel 
#### Date: 30/10/2022
## Short description of the task and how this repository will address it: 
This software allows a virtual controllable Prius vehicle to drive autonomously through a simulated test track outlined by cones. The software uses the vehicles sensors to detect obstacles and pedestrians. It then uses these detections to generate control instructions that prevent the vehicle from hitting any cones or pedestrians. The software contains two packages called `pcl_obstacle_detector` and `control_barrel_world`.
## Short desciption of the packages their nodes and the launch file:
#### "pcl_obstacle_detector" package:
The `pcl_obstacle_detector` package contains a  node called `pcl_obstacle_detector_node` that:
1. Subscribes to the point cloud topic `/point_cloud` with a "queue_size" of 1, ensuring that the data is processed in near real time.
2. Removes the ground plane from the received point cloud with the RANSAC algorithm and uses euclidean cluster extraction to find clusters (mainly barrels) in the remaining point cloud. The clusters are extracted using the following parameters:
    -   Cluster tolerance = 0.5
    -   Minimum cluster size = 10
    -   Maximum cluster size = 25000
The trunk of the car is also detected as a cluster, but is ignored. 
3. Publishes the clusters as 3D bounding boxes by assigning the cluster center to the "center.position.center" field and using the extrema of the cluster to fill the "bbox.size" field of the message of type `vision_msgs/Detection3DArray` to the topic `/pcl_obstacle_detector_node/detections`. The detection_3d_to_markers_node subscribes to `/pcl_obstacle_detector_node/detections` and visualizes the 3D bounding boxes in Rviz. 

The node `pcl_obstacle_detector_node` is distributed over 3 files: 
-   The source file `src/pcl_obstacle_detector_node.cpp` that contains the constructor, destructor and the callback function allowing the node to subscribe and publish. 
-   The header file `include/pcl_obstacle_detector_node.h` that defines a class using function declarations of functions defined in `src/pcl_obstacle_detector_node.cpp`
-   The source file `src/pcl_obstacle_detector_main.cpp` containing the main() function which initialzes the node, creates a publisher object, creates an instance of the class defined in include/pcl_obstacle_detector_node.h and lets ROS take over control of the node. 


#### "control_barrel_world" package:
The `control_barrel_world` package contains a node called `control_barrel_world_node` that:
1. Subscribes to both `/opencv_person_detector_node/detections` and `/pcl_obstacle_detector_node/detections` topics.
2. Implements a control algorithm to:
    -   Filter the 3D obstacle detections published to the `/pcl_obstacle_detector_node/detections` topic to save only the detections in front of the car and within a radius of four meters.
    -   Drive forward when the filtered set is empty.
    -   Obtain the closest detection when the filtered set is not empty and generate steer commands as `prius_msgs/Control` messages accordingly.
    -   Manipulate the `prius_msgs/Control` message to stop permanently in case a person detection published to the `/opencv_person_detector_node/detections` topic is larger that 25000 pixels.
3. Publishes the in step 2 generated `prius_msgs/Control` message to the topic `/prius`. 
4. Contains a launch file called `solution.launch` to run the full solution. The launch file:
    -   Includes the `car_simulation/launch/simulation_barrel_world.launch` file.
    -   Runs the node `pcl_obstacle_detector_node`.
    -   Runs the node `control_barrel_world_node`.

The node `control_barrel_world_node` is distributed over 3 files: 
-   The source file `src/control_barrel_world_node.cpp` that contains the constructor, destructor and the callback function allowing the node to subscribe and publish. 
-   The header file `include/control_barrel_world_node.h` that defines a class using function declarations of functions defined in `src/pcl_obstacle_detector_node.cpp`
-   The source file `src/control_barrel_world_node_main.cpp` containing the main() function which initialzes the node, creates a publisher object, creates an instance of the class defined in `include/control_barrel_world_node.h` and lets ROS take over control of the node. 


## Build instructions: 
To build the solution run the following command lines inside the singularity container:

```
source /opt/ros/noetic/setup.sh
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone git@gitlab.ro47003.3me.tudelft.nl:students-2223/lab4/group101.git
git clone git@gitlab.ro47003.3me.tudelft.nl:students-2223/ro47003_simulator.git
cd ..
catkin_make
```

## Run instructions: 
To run the solution run the following command lines inside the singularity container:

    source devel/setup.sh
    roslaunch control_barrel_world solution.launch
##
Broke the solution just before the deadline when adding comments:( 
