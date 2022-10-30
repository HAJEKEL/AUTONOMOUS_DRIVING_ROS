#include "../include/control_barrel_world.h"

#include <prius_msgs/Control.h>
#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3DArray.h>

#include <iomanip>
#include <vector>

ControlBarrelWorldNode::ControlBarrelWorldNode()  // Defining the constructor
                                                  // that initializes all data
                                                  // members of the class

{
  obstacle_sub = nh.subscribe("pcl_obstacle_detector_node/detections", 1,
                              &ControlBarrelWorldNode::callback1, this);
  person_sub = nh.subscribe("opencv_person_detector_node/detections", 1,
                            &ControlBarrelWorldNode::callback2, this);
  pub = nh.advertise<prius_msgs::Control>("prius", 1, this);
}
// Defining the destructor to clean up the object
ControlBarrelWorldNode::~ControlBarrelWorldNode() {
  // Printing a goodbye message when destructing the node
  std::cout << "Goodbye:)" << std::endl;
};

// Making sure the car stops when the person detection occupies more than 25000
// pixels
void ControlBarrelWorldNode::callback2(
    const vision_msgs::Detection2DArray &msg2) {
  if (brake == true) {
    PriusInput.steer = 0;
    PriusInput.throttle = 0;
    PriusInput.shift_gears = 0;
    PriusInput.brake = 1;
  } else {
    for (int i = 0; i < msg2.detections.size(); i++) {
      pixels = msg2.detections[i].bbox.size_x * msg2.detections[i].bbox.size_y;
      if (std::abs(pixels) > 25000) {
        PriusInput.steer = 0;
        PriusInput.throttle = 0;
        PriusInput.shift_gears = 0;
        PriusInput.brake = 1;
        brake = true;

        ROS_INFO("Stopping");
        return;
      }
    }
  }
}

void ControlBarrelWorldNode::callback1(
    const vision_msgs::Detection3DArray &msg1) {
  bool barrel_close =
      false;  // Define the boolean variable that is true when a barrel is
              // within 4 meter and in front of the car
  vision_msgs::Detection3DArray Barrels;
  std::vector<float> distances;

  if (brake ==
      true)  // Stop moving when a person detection exceeds 25000 pixels
  {
    PriusInput.steer = 0;
    PriusInput.throttle = 0;
    PriusInput.shift_gears = 0;
    PriusInput.brake = 1;
  }

  else  // If we did not need to stop for a pedestrian we compute the distance
        // to the closest barrel and steer accordingly
  {
    for (int i = 0; i < msg1.detections.size(); i++) {
      // Computation of the distance to the barrels
      float x = msg1.detections[i].bbox.center.position.x;
      float y = msg1.detections[i].bbox.center.position.y;
      float Distance = sqrt(pow(x, 2) + pow(y, 2));

      if (x > 0.0 && Distance < 4.0) {
        Barrels.detections.push_back(msg1.detections[i]);
        distances.push_back(Distance);  // add distance to vector
        barrel_close = true;            // set boolean to true
      }
    }

    if (barrel_close == false)  // no barrel within 4m in front of the car
    {
      // Go forward
      PriusInput.steer = 0;
      PriusInput.throttle = 1;
      PriusInput.shift_gears = 2;
    }

    else  // barrel within 4m in front of the car
    {
      int barrel_idx =
          min_element(distances.begin(), distances.end()) - distances.begin();
      if (Barrels.detections[barrel_idx].bbox.center.position.y >=
          0)  // The barrels on the left
      {
        // Go to the right
        PriusInput.steer = -1;
        PriusInput.throttle = 1;
      }

      else if (Barrels.detections[barrel_idx].bbox.center.position.y <
               0)  // The barrels on the right
      {
        // Go to the left
        PriusInput.steer = 1;
        PriusInput.throttle = 1;
      }
    }
  }
  pub.publish(PriusInput);  // publish messages to the car
}
