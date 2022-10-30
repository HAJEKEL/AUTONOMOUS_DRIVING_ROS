#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>

#include <Eigen/Dense>
#include <iomanip>
#include <vector>

#include "../include/pcl_obstacle_detector.h"

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node
  ros::init(argc, argv, "pcl_obstacle_detector_node");

  // Create Publisher object
  ros::Publisher obstacle_pub;

  // Create object of the class PclObstacleDetector
  PclObstacleDetector pcl;

  // Let ROS take over
  ros::spin();
}