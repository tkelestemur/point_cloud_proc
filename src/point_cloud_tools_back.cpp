// #include <point_cloud_tools/point_cloud_tools.h>


PointCloudTools::PointCloudTools()
{

  std::string point_cloud2_topic = "/camera/point_cloud";
  node_.getParam("/point_cloud2_topic", point_cloud2_topic);
  point_cloud_sub_ =  node_.subscribe<sensor_msgs::PointCloud2>(point_cloud2_topic,
                                                                10,
                                                                &PointCloudTools::pointCloudCb,
                                                                this);


}

bool PointCloudTools::segmentPlane(){
// Transform point cloud to fixed frame
pcl::fromROSMsg(raw_cloud_msg_, *cloud_raw_);
fixed_frame_ = "base_link";
CloudT::Ptr cloud_transformed (new CloudT);
if (!pcl_ros::transformPointCloud(fixed_frame_, *cloud_raw_, *cloud_transformed, listener_))
{
  ROS_ERROR("Error transforming to frame %s", fixed_frame_.c_str());

}

return true;
}


void PointCloudTools::pointCloudCb(const sensor_msgs::PointCloud2 point_cloud){
  raw_cloud_msg_ = point_cloud;
}
