#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <gpd/CloudIndexed.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudT;
typedef pcl::PointXYZRGB PointT;

class PointCloudTools{
public:
  PointCloudTools(ros::NodeHandle n) : nh_(n){
    //TODO: Get parameters from ROS param servers
    leaf_size_ = 0.03;
    pass_limits_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point_cloud_topic_ = "/camera/point_cloud";
    
    nh_.getParam("/point_cloud_topic", point_cloud_topic_);
    point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 10, &PointCloudTools::pointCloudCb);
    table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/object_seg/table_cloud", 10);
    object_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/object_seg/object_cloud", 10);
    gpd_cloud_pub_ = node.advertise<gpd::CloudIndexed>("/object_seg/indexed_cloud", 10);
    object_cluster_srv_ = node.advertiseService("object_seg/cluster_objects", objectClusterService);


  }

  void pointCloudCb(const sensor_msgs::PointCloud2 point_cloud) {
    
  }

  void filterPointCloud(bool use_passthrough, bool use_voxel){

    // Remove part of the scene to leave table and objects alone
    if (use_passthrough) {
      pass_.setInputCloud (cloud_raw_);
      pass_.setFilterFieldName ("x");
      pass_.setFilterLimits (-pass_limits_[0],  pass_limits_[1]);
      pass_.filter(*cloud_filtered_);
      pass_.setInputCloud (cloud_filtered_);
      pass_.setFilterFieldName ("y");
      pass_.setFilterLimits (-pass_limits_[2],  pass_limits_[3]);
      pass_.filter(*cloud_filtered_);
      pass_.setInputCloud (cloud_filtered_);
      pass_.setFilterFieldName ("z");
      pass_.setFilterLimits (-pass_limits_[4],  pass_limits_[5]);
      pass_.filter(*cloud_filtered_);
    }

    // Downsample point cloud
    if (use_voxel) {
      vg_.setInputCloud (cloud_filtered_);
      vg_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
      vg_.filter (*cloud_filtered_2_);
    }


  }

  void segmentTable(){
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setDistanceThreshold (0.01);
    seg_.setInputCloud (cloud_filtered_2_);
    seg_.segment (*plane_inliers_, *plane_coefficients_);

    std::cerr << "Model coefficients: " << plane_coefficients_->values[0] << " "
                                        << plane_coefficients_->values[1] << " "
                                        << plane_coefficients_->values[2] << " "
                                        << plane_coefficients_->values[3] << std::endl;

    extract_.setInputCloud (cloud_filtered_2_);
    extract_.setNegative(false);
    extract_.setIndices (plane_inliers_);
    extract_.filter (*cloud_plane_);


    chull_.setInputCloud (cloud_plane_);
    chull_.setDimension(2);
    chull_.reconstruct (*plane_hull_, polygons_);

  }

  void publishPolygons(){
    // Publish boundary of the table
    // geometry_msgs::PolygonStamped plane_polygon;
    // for (int i = 0; i < plane_hull_->points.size (); i++) {
    //   geometry_msgs::Point32 pt;
    //   pt.x = plane_hull_->points[i].x;
    //   pt.y = plane_hull_->points[i].y;
    //   pt.z = plane_hull_->points[i].z;
    //
    //   plane_polygon.polygon.points.push_back(pt);
    // }
    // plane_polygon.header.frame_id = plane_hull_->header.frame_id;
    // plane_polygon.header.stamp = ros::Time::now();
    //
    // plane_polygon_pub_.publish(plane_polygon);
  }


void createGPDMsg() {
  // geometry_msgs::Point camera_viewpoint;
  // camera_viewpoint.x = 0;
  // camera_viewpoint.y = 0;
  // camera_viewpoint.z = 0;
  // toROSMsg(*cloud_voxel_, gpd_cloud_);
  // gpd::CloudIndexed gpd_cloud;
  // gpd_cloud.cloud_sources.cloud = gpd_cloud_;
  // gpd_cloud.cloud_sources.view_points.push_back(camera_viewpoint);
  // std_msgs::Int64 camera_source, index;
  // camera_source.data = 0;
  // for (int i = 0; i < transformed_cloud_.width; i++) {
  //   gpd_cloud.cloud_sources.camera_source.push_back(camera_source);
  // }
  //
  //
  // for (int i = 0; i < object_indices->indices.size(); i++) {
  //   index.data = object_indices->indices[i];
  //   gpd_cloud.indices.push_back(index);
  // }
  //
  // gpd_cloud_pub.publish(gpd_cloud);
}

bool objectClusterService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){


}

private:
  pcl::PassThrough<PointT> pass_;
  pcl::VoxelGrid<PointT> vg_;
  pcl::SACSegmentation<PointT> seg_;
  pcl::ExtractIndices<PointT> extract_;
  pcl::ConvexHull<PointT> chull_;

  pcl::PointIndices::Ptr plane_inliers_;
  pcl::ModelCoefficients::Ptr plane_coefficients_;
  std::vector<pcl::Vertices> plane_polygons_;
  std::vector<pcl::Vertices> polygons_;

  float leaf_size_;
  std::vector<float> pass_limits_;
  std::string point_cloud_topic_;
  CloudT::Ptr cloud_raw_;
  CloudT::Ptr cloud_filtered_;
  CloudT::Ptr cloud_filtered_2_;
  CloudT::Ptr cloud_plane_, plane_hull_;

  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher table_cloud_pub_, object_cloud_pub_, gpd_cloud_pub_;
  ros::ServiceServer object_cluster_srv_;
  

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_seg");
  ros::NodeHandle n("~");
  PointCloudTools pc_tools(n);

  return 0;

}
