#ifndef POINTCLOUDTOOLS_H
#define POINTCLOUDTOOLS_H

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudTools{

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudT;
typedef pcl::PointXYZRGB PointT;

public:
  PointCloudTools();
  void pointCloudCb(const sensor_msgs::PointCloud2 point_cloud);
  bool segmentPlane();
private:

  pcl::PassThrough<PointT> pass;
  pcl::VoxelGrid<PointT> vg;
  CloudT::Ptr cloud_raw_ (new CloudT);

  ros::NodeHandle node_;
  tf::TransformListener listener_;
  ros::Subscriber point_cloud_sub_;
  sensor_msgs::PointCloud2 raw_cloud_msg_;

  std::string fixed_frame_;



};

#endif
