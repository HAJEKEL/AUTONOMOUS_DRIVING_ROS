#include <prius_msgs/Control.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;
ros::Subscriber sub;

float throttle_param;
float steer_param;

void check();

void callback(const geometry_msgs::Twist& msgIn) {
  // Initiate the outputs.
  prius_msgs::Control msgOut;

  // Put it in gear/revers if up/down arrow is pressed.
  if (msgIn.linear.x < 0) {
    msgOut.shift_gears = msgOut.REVERSE;
  } else {
    msgOut.shift_gears = msgOut.FORWARD;
  }

  // Make the car move.
  if (msgIn.linear.x != 0) {
    msgOut.throttle = throttle_param;
  } else {
    msgOut.throttle = 0;
  }

  // Steer the car wheen left/right arrows are pressed.
  if (msgIn.angular.z < 0) {
    msgOut.steer = -steer_param;
  } else if (msgIn.angular.z > 0) {
    msgOut.steer = steer_param;
  } else {
    msgOut.steer = 0;
  }

  // Publish the outputs.
  pub.publish(msgOut);
}

void translator(ros::NodeHandle nh) {
  // Check the paramters and start translate.
  check();
  pub = nh.advertise<prius_msgs::Control>("/prius", 10);
  sub = nh.subscribe("turtle1/cmd_vel", 1000, &callback);
}

void check() {
  // Get the parameters to check if they are valid.
  bool throttle_ok = ros::param::get("throttle_param", throttle_param);
  bool steer_ok = ros::param::get("steer_param", steer_param);

  // Produce a fatal error if we can not get the parameters.
  if (!throttle_ok) {
    ROS_FATAL_STREAM("Could not get throttle param");
    exit(1);
  }
  if (!steer_ok) {
    ROS_FATAL_STREAM("Could not get steer param");
    exit(1);
  }

  // Produce warnings when the parameters are outside the range [0,1].
  if (steer_param < 0 || steer_param > 1) {
    steer_param = 1;
    ROS_WARN_STREAM("Steering out of range, set to 1");
  }
  if (throttle_param < 0 || throttle_param > 1) {
    throttle_param = 1;
    ROS_WARN_STREAM("Throttle out of range, set to 1");
  }

  ROS_INFO_STREAM("Throttle: " << throttle_param << "; Steer: " << steer_param);
}
int main(int argc, char** argv) {
  // Initialize ROS node and create ROS node handle.
  ros::init(argc, argv, "simple_control_node");
  ros::NodeHandle nh;

  // Check paramters and translate keyboard input into car controls.
  translator(nh);

  // Don't exit the program.
  ros::spin();
}
