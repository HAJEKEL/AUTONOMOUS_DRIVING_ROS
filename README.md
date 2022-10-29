# Description and instructions
#### Course title: RO47003 Robot Software Practicals (2022/23 Q1)
#### Name: Henk Jekel 
#### Date: 30/10/2022
## Short description of the task and how this repository will address it: 
This software allows a virtual controllable Prius vehicle to drive autonomously through a simulated test track outlined by cones. The software uses the vehicles sensors to detect obstacles and pedestrians and uses these detections to generate control instructions that prevent the vehicle from hitting any cones or pedestrians. 
## Short desciption of the packages their nodes and the launch file:
#### "opencv_person_detector" package:
The "opencv_person_detector" package has a node called "opencv_person_detector_node" that:
1. Subscrives to the image topic "/prius/front_camera/image_raw"
2. Processes the received image to find people 
3. Publishes the detections as a ROS topic

#### "pcl_obstacle_detector" package:
The "pcl_obstacle_detector" package contains a  node called "pcl_obstacle_detector_node" that:
1. Subscribes to the point cloud topic "/point_cloud" with a "queue_size" of 1, ensuring that the data is processed in near real time. 
2. Removes the ground plane from the received point cloud and uses euclidean cluster extraction to find clusters (mainly barrels) in the remaining point cloud. The clusters are extracted using the following parameters:
-   Cluster tolerance = 0.5
-   Minimum cluster size = 10
-   Maximum cluster size = 25000
3. Publishes the clusters as a 3D bounding boxes by assigning the cluster center to the "center.position.center" field and using the extrema of the cluster to fill the "bbox.size" field of the message of type "vision_msgs/Detection3DArray" to the topic "/pcl_obstacle_detector_node/detections"

#### "control_barrel_world" package:
The "control_barrel_world" package contains a node called "control_barrel_world_node" that:
1. Subscribes to both "/opencv_person_detector_node/detections" and "/pcl_obstacle_detector_node/detections"
2. Implements a control algorithm to:
-   Filter the 3D obstacle detections to save only the detections in front of the car and within a radius of four meters.
-   Drive forward when the filtered set is empty.
-   Obtain the closest detection when the filtered set is not empty and steer accordingly
-   Stop in case a person detection is larger that 25000 pixels 
3. Publishes "prius_msgs/Control" messages to the topic "/prius". 
4. Contains a launch file called "solution.launch" to run the full solution. The launch file:
-   Includes the car_simulation/launch/simulation_barrel_world.launch file
-   Runs the node "pcl_obstacle_detector_node"
-   Runs the node "control_barrel_world_node"
## Build instructions: 
To build the solution run the following command lines inside the singularity container (without $):
-   $ source /opt/ros/noetic/setup.sh
-   $ cd ~
-   $ mkdir -p catkin_ws/src
-   $ cd catkin_ws/src
-   $ catkin_init_workspace
-   $ git clone git@gitlab.ro47003.3me.tudelft.nl:students-2223/lab4/group101.git
-   $ git clone git@gitlab.ro47003.3me.tudelft.nl:students-2223/ro47003_simulator.git
-   $ cd ..
-   $ catkin_make
## Run instructions: 
-   source devel/setup.sh
-   roslaunch control_barrel_world solution.launch

