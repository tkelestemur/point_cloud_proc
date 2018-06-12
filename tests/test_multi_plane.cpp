#include <ros/ros.h>
#include <point_cloud_proc/point_cloud_proc.h>


int main(int argc, char **argv) {

  ros::init(argc, argv, "test_multi_plane_segmentation");
  ros::NodeHandle nh;
  PointCloudProc pcp(nh, true);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Duration(1.0).sleep();

  std::vector<point_cloud_proc::Plane> planes;
  pcp.segmentMultiplePlane(planes);

  ros::shutdown();
  return 0;
}