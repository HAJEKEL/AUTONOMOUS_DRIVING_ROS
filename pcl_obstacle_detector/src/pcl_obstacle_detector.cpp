#include "../include/pcl_obstacle_detector.h"

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

// Defining the constructor that initializes all data members of the class
PclObstacleDetector::PclObstacleDetector() {
  lidar_sub =
      nh.subscribe("point_cloud", 1000, &PclObstacleDetector::callback, this);

  obstacle_pub = nh.advertise<vision_msgs::Detection3DArray>(
      "pcl_obstacle_detector_node/detections", 1);
};

// Defining the destructor to clean up the object
PclObstacleDetector::~PclObstacleDetector(){};

void PclObstacleDetector::callback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {
  // Declaring messages for the publisher obstacle_pub

  vision_msgs::Detection3DArray detections3DArray;
  vision_msgs::Detection3D detections3D;

  // Declare object for segmentation and clustering

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
      cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDWriter writer;
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;

  // Filter points cloud

  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud_filtered);

  // Declare the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // segment the plane
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.3);
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  // creating the extractor object extract.

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);

  // Get the road points
  extract.filter(*cloud_plane);
  // Remove the road keep the rest
  extract.setNegative(true);
  extract.filter(*cloud_f);
  *cloud_filtered = *cloud_f;
  // kdTree object generation for extraction search
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  // Clustering Object

  ec.setClusterTolerance(0.5);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(25000);

  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  // Separating clusters by iteration through clusters

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &idx : it->indices)
      cloud_cluster->push_back((*cloud_filtered)[idx]);
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    pcl::compute3DCentroid(*cloud_cluster, centroid);
    pcl::getMinMax3D(*cloud_cluster, min, max);
    detections3D.header = msg->header;
    detections3D.bbox.center.position.x = centroid[0];
    detections3D.bbox.center.position.y = centroid[1];
    detections3D.bbox.center.position.z = centroid[2];
    detections3D.bbox.center.orientation.x = 0;
    detections3D.bbox.center.orientation.y = 0;
    detections3D.bbox.center.orientation.w = 1.0;

    detections3D.bbox.size.x = (max[0] - min[0]);
    detections3D.bbox.size.y = (max[1] - min[1]);
    detections3D.bbox.size.z = (max[2] - min[2]);

    detections3DArray.detections.push_back(detections3D);
    detections3DArray.header = msg->header;

    j++;
  }

  obstacle_pub.publish(detections3DArray);  // Cluster published
}
