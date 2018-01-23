// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_frame_pub");
  ros::NodeHandle n("~");

  tf::TransformListener listener;
  tf::TransformBroadcaster br;

  ros::Duration(1.0).sleep();

  ros::Rate rate(200.0);
  while (n.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/base_link", "/head_pan_link", ros::Time(0), transform);
      tf::Transform cloud_frame;
      cloud_frame.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
      cloud_frame.setRotation(transform.getRotation());
      br.sendTransform(tf::StampedTransform(cloud_frame, ros::Time::now(), "base_link", "point_cloud_frame"));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    rate.sleep();
  }

  return 0;

}
