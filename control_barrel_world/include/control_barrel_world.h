#ifndef CONTROL_BARREL_WORLD_NODE_H
#define CONTROL_BARREL_WORLD_NODE_H

#include <prius_msgs/Control.h>
#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3DArray.h>

class ControlBarrelWorldNode {
  /* This  is the class of the control barrel world node */
 private:
  ros::NodeHandle nh;
  ros::Subscriber obstacle_sub, person_sub;
  ros::Publisher pub;

 public:
  ControlBarrelWorldNode();
  ~ControlBarrelWorldNode();
  void callback1(const vision_msgs::Detection3DArray& msg1);
  void callback2(const vision_msgs::Detection2DArray& msg2);
};

// Global variables
extern ros::Publisher pub;
extern prius_msgs::Control PriusInput;
extern float pixels;
extern bool brake;
#endif  // CONTROL_BARREL_WORLD_NODE_H
