#include "ros/ros.h"
#include <point_cloud_tools/point_cloud_tools.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "object_clustering");
  PointCloudTools pct();
  // bool test = pct.segmentPlane();
  ros::spin();
  return 0;
}
