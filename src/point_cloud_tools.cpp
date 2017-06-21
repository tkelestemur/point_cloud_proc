#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
// #include <gpd/CloudIndexed.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <boost/thread/mutex.hpp>

class PointCloudTools{

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> CloudT;

  public:
    PointCloudTools(ros::NodeHandle n) : nh_(n), debug_(true) {

      leaf_size_ = 0.01;
      pass_limits_.push_back(0.0);
      pass_limits_.push_back(4.0);
      pass_limits_.push_back(-2.0);
      pass_limits_.push_back(2.0);
      pass_limits_.push_back(0.30);
      pass_limits_.push_back(1.50);
      prism_limits_.push_back(0.0);
      prism_limits_.push_back(0.05);
      point_cloud_topic_ = "/hsrb/head_rgbd_sensor/depth_registered/points";

      nh_.param("/filters/leaf_size", leaf_size_);
      nh_.param("/filters/use_passthrough", use_pass);
      nh_.param("/filters/use_voxel", use_voxel);
      nh_.param("/filters/pass_limits", pass_limits_);
      nh_.getParam("/point_cloud_topic", point_cloud_topic_);
      point_cloud_sub_ = nh_.subscribe<CloudT> (point_cloud_topic_, 10, &PointCloudTools::pointCloudCb, this);
      table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table_cloud", 10);
      object_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("object_cloud", 10);
      plane_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("plane_polygon", 10);
      // gpd_cloud_pub_ = nh_.advertise<gpd::CloudIndexed>("indexed_cloud", 10);
      object_cluster_srv_ = nh_.advertiseService ("cluster_objects", &PointCloudTools::objectClusterServiceCb, this);

    }

    void pointCloudCb(const CloudT::ConstPtr &msg) {
      boost::mutex::scoped_lock lock(pc_mutex_);
      cloud_raw_ = msg;
      ROS_INFO("Got new point cloud!");
    }

    bool transformPointCloud() {
      CloudT::Ptr cloud_transformed(new CloudT);
      std::string fixed_frame = "/base_link"; // TODO: Make it rosparam
      bool transform_success = pcl_ros::transformPointCloud(fixed_frame, *cloud_raw_, *cloud_transformed, listener_);
      cloud_transformed_ = cloud_transformed;
      return transform_success;
    }

    void filterPointCloud(){

      CloudT::Ptr cloud_filtered (new CloudT);
      CloudT::Ptr cloud_filtered_2 (new CloudT);
      std::cout << "transformd cloud size: " << cloud_transformed_->points.size() << '\n';
      // Remove part of the scene to leave table and objects alone
      pass_.setInputCloud (cloud_transformed_);
      pass_.setFilterFieldName ("x");
      pass_.setFilterLimits (pass_limits_[0],  pass_limits_[1]);
      pass_.filter(*cloud_filtered);
      pass_.setInputCloud (cloud_filtered);
      pass_.setFilterFieldName ("y");
      pass_.setFilterLimits (pass_limits_[2],  pass_limits_[3]);
      pass_.filter(*cloud_filtered);
      pass_.setInputCloud (cloud_filtered);
      pass_.setFilterFieldName ("z");
      pass_.setFilterLimits (pass_limits_[4],  pass_limits_[5]);
      pass_.filter(*cloud_filtered);
      //
      // Downsample point cloud
      vg_.setInputCloud (cloud_filtered);
      vg_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
      vg_.filter (*cloud_filtered_2);

      cloud_filtered_ = cloud_filtered_2;

    }

    void segmentSinglePlane(){

      CloudT::Ptr cloud_plane (new CloudT);
      CloudT::Ptr cloud_hull (new CloudT);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      pcl::SACSegmentation<PointT> seg_;
      pcl::ExtractIndices<PointT> extract_;

      // IMPORTANT TODO: Switch to segmentation from normals to make it segment only horizantal planes.

      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PLANE);
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setDistanceThreshold (0.01);
      seg_.setInputCloud (cloud_filtered_);
      seg_.segment (*inliers, *coefficients);

      std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " "
                                          << coefficients->values[3] << std::endl;

      extract_.setInputCloud (cloud_filtered_);
      extract_.setNegative(false);
      extract_.setIndices (inliers);
      extract_.filter (*cloud_plane);

      if (debug_) {
        table_cloud_pub_.publish(cloud_plane);
      }

      chull_.setInputCloud (cloud_plane);
      chull_.setDimension(2);
      chull_.reconstruct (*cloud_hull);

      cloud_hull_ = cloud_hull;

      // Publish boundary of the table
      geometry_msgs::PolygonStamped plane_polygon;
      for (int i = 0; i < cloud_hull->points.size (); i++) {
        geometry_msgs::Point32 pt;
        pt.x = cloud_hull->points[i].x;
        pt.y = cloud_hull->points[i].y;
        pt.z = cloud_hull->points[i].z;

        plane_polygon.polygon.points.push_back(pt);
      }
      plane_polygon.header.frame_id = cloud_hull->header.frame_id;
      plane_polygon.header.stamp = ros::Time::now(); //TODO: should this be point cloud time?

      plane_polygon_pub_.publish(plane_polygon);

    }


    void segmentMultiplePlane(/* arguments */) {
      /* code */
    }

    void extractObjects(/* arguments */) {
      CloudT::Ptr cloud_objects (new CloudT);
      pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);
      prism_.setInputCloud(cloud_filtered_);
      prism_.setInputPlanarHull(cloud_hull_);
      prism_.setHeightLimits(prism_limits_[0], prism_limits_[1]); // Height limits
      prism_.segment(*object_indices);

      extract_.setInputCloud (cloud_filtered_);
      extract_.setIndices(object_indices);
      extract_.filter(*cloud_objects);

      if (debug_) {
        object_cloud_pub_.publish(cloud_objects);
      }

      // TODO: Do we need post-processing?

    }

  // void createGPDMsg() {
  //   // geometry_msgs::Point camera_viewpoint;
  //   // camera_viewpoint.x = 0;
  //   // camera_viewpoint.y = 0;
  //   // camera_viewpoint.z = 0;
  //   // toROSMsg(*cloud_voxel_, gpd_cloud_);
  //   // gpd::CloudIndexed gpd_cloud;
  //   // gpd_cloud.cloud_sources.cloud = gpd_cloud_;
  //   // gpd_cloud.cloud_sources.view_points.push_back(camera_viewpoint);
  //   // std_msgs::Int64 camera_source, index;
  //   // camera_source.data = 0;
  //   // for (int i = 0; i < transformed_cloud_.width; i++) {
  //   //   gpd_cloud.cloud_sources.camera_source.push_back(camera_source);
  //   // }
  //   //
  //   //
  //   // for (int i = 0; i < object_indices->indices.size(); i++) {
  //   //   index.data = object_indices->indices[i];
  //   //   gpd_cloud.indices.push_back(index);
  //   // }
  //   //
  //   // gpd_cloud_pub.publish(gpd_cloud);
  // }

  bool objectClusterServiceCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if (!PointCloudTools::transformPointCloud()){
      return false;
      ROS_INFO("Couldn't transform point cloud!");
    }

    PointCloudTools::filterPointCloud();
    PointCloudTools::segmentSinglePlane();

    return true;
  }

  private:
    pcl::PassThrough<PointT> pass_;
    pcl::VoxelGrid<PointT> vg_;
    pcl::SACSegmentation<PointT> seg_;
    pcl::ExtractIndices<PointT> extract_;
    pcl::ConvexHull<PointT> chull_;
    pcl::ExtractPolygonalPrismData<PointT> prism_;

    bool debug_;
    int use_pass, use_voxel;
    float leaf_size_;
    std::vector<float> pass_limits_, prism_limits_;
    std::string point_cloud_topic_;
    CloudT::ConstPtr cloud_raw_;
    CloudT::Ptr cloud_transformed_, cloud_filtered_, cloud_hull_;

    boost::mutex pc_mutex_;

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher table_cloud_pub_, object_cloud_pub_, gpd_cloud_pub_;
    ros::Publisher plane_polygon_pub_;
    ros::ServiceServer object_cluster_srv_;
    tf::TransformListener listener_;


};

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_seg");
  ros::NodeHandle n("~");
  PointCloudTools pc_tools(n);
  ros::spin();

  return 0;

}
