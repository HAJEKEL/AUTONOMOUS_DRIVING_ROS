#include <prius_msgs/Control.h>
#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3DArray.h>

#include <iomanip>

#include "../include/control_barrel_world.h"

// Global variables
ros::Publisher pub;
prius_msgs::Control PriusInput;
float pixels;
bool brake = false;

int main(int argc, char **argv) {
  // Initialize ROS node
  ros::init(argc, argv, "control_barrel_world_node");

  ControlBarrelWorldNode control;  // Call the class

  ros::Rate rate(10);  // Hz rate
  while (ros::ok()) {
    // Let Ros take over
    ros::spinOnce();
    rate.sleep();
  }
}
