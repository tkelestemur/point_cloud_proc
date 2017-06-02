

class PointCloudTools{

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB PointT;

public:
  PointCloudTools(ros::NodeHandle nh){


  }
private:
  PointCloud::Ptr cloud_raw_ (new PointCloud);

};
