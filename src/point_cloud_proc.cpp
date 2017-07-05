// ROS
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <gpd/CloudIndexed.h>
#include <shape_msgs/Plane.h>
#include <shape_msgs/Mesh.h>
#include <object_msgs/PlaneObjects.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

// Other
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Dense>

class PointCloudProc{

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> CloudT;

  public:
    PointCloudProc(ros::NodeHandle n) : nh_(n), debug_(true) {

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

      nh_.getParam("/filters/leaf_size", leaf_size_);
      nh_.getParam("/filters/use_passthrough", use_pass);
      nh_.getParam("/filters/use_voxel", use_voxel);
      nh_.getParam("/filters/pass_limits", pass_limits_);
      nh_.getParam("/segmentation/prism_limits", prism_limits_);
      nh_.getParam("/point_cloud_topic", point_cloud_topic_);
      point_cloud_sub_ = nh_.subscribe<CloudT> (point_cloud_topic_, 10, &PointCloudProc::pointCloudCb, this);
      table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table_cloud", 10);
      object_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("object_cloud", 10);
      plane_object_pub_ = nh_.advertise<object_msgs::PlaneObjects>("plane_objects", 10);
      gpd_cloud_pub_ = nh_.advertise<gpd::CloudIndexed>("indexed_cloud", 10);
      object_cluster_srv_ = nh_.advertiseService ("extract_objects", &PointCloudProc::objectClusterServiceCb, this);
      segment_multiple_plane_srv_ = nh_.advertiseService ("segment_multiple_plane", &PointCloudProc::segmentMultiplePlaneServiceCb, this);
      create_gpd_msg_srv_ = nh_.advertiseService ("create_gpd_msg", &PointCloudProc::createGPDMsgServiceCb, this);
      find_table_srv_ = nh_.advertiseService ("find_table", &PointCloudProc::findTableServiceCb, this);

    }

    void pointCloudCb(const CloudT::ConstPtr &msg) {
      boost::mutex::scoped_lock lock(pc_mutex_);
      cloud_raw_ = msg;
      // ROS_INFO("Got new point cloud!");
    }

    bool transformPointCloud() {
      CloudT::Ptr cloud_transformed(new CloudT);
      std::string fixed_frame = "/base_link"; // TODO: Make it rosparam
      bool transform_success = pcl_ros::transformPointCloud(fixed_frame, *cloud_raw_, *cloud_transformed, listener_);
      cloud_transformed_ = cloud_transformed;
      return transform_success;
    }

    bool filterPointCloud(bool use_voxel=true){

      CloudT::Ptr cloud_filtered (new CloudT);
      CloudT::Ptr cloud_filtered_2 (new CloudT);

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

      // Downsample point cloud
      if (use_voxel) {
        vg_.setInputCloud (cloud_filtered);
        vg_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
        vg_.filter (*cloud_filtered_2);
        cloud_filtered_ = cloud_filtered_2;
      } else cloud_filtered_ = cloud_filtered;


      ROS_INFO("Point cloud is filtered!");
      if (cloud_filtered_->points.size() == 0) {
        ROS_WARN("Point cloud is empty after filtering!");
        return false;
      }

      return true;
    }

    bool segmentSinglePlane(){

      CloudT::Ptr cloud_plane (new CloudT);
      CloudT::Ptr cloud_hull (new CloudT);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //z axis
      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setAxis(axis);
      seg_.setEpsAngle(20.0f * (M_PI/180.0f));
      seg_.setDistanceThreshold (0.01);
      seg_.setInputCloud (cloud_filtered_);
      seg_.segment (*inliers, *coefficients);

      if (inliers->indices.size() == 0) {
        ROS_INFO("No plane found!");
        return false;
      }

      ROS_INFO("Single plane is segmented!");
      ROS_INFO_STREAM("Coefficients: " << coefficients->values[0] << " "
                                       << coefficients->values[1] << " "
                                       << coefficients->values[2] << " "
                                       << coefficients->values[3]);

      extract_.setInputCloud (cloud_filtered_);
      extract_.setNegative(false);
      extract_.setIndices (inliers);
      extract_.filter (*cloud_plane);
      ROS_INFO_STREAM("# of points in plane: " << cloud_plane->points.size());

      if (debug_) {
        table_cloud_pub_.publish(cloud_plane);
      }

      chull_.setInputCloud (cloud_plane);
      chull_.setDimension(2);
      chull_.reconstruct (*cloud_hull);

      cloud_hull_ = cloud_hull;

      Eigen::Vector4f center;
      pcl::compute3DCentroid(*cloud_plane, center);


      // Construct plane object msg
      object_msgs::PlaneObjects plane_object_msgs;
      object_msgs::PlaneObject plane_object_msg;

      pcl_conversions::fromPCL(cloud_plane->header, plane_object_msg.header);

      // Get plane center
      plane_object_msg.center.x = center[0];
      plane_object_msg.center.y = center[1];
      plane_object_msg.center.z = center[2];

      // Get plane polygon
      for (int i = 0; i < cloud_hull->points.size (); i++) {
        geometry_msgs::Point pt;
        pt.x = cloud_hull->points[i].x;
        pt.y = cloud_hull->points[i].y;
        pt.z = cloud_hull->points[i].z;

        plane_object_msg.polygon.push_back(pt);
      }

      // Get plane coefficients
      plane_object_msg.coef[0] = coefficients->values[0];
      plane_object_msg.coef[1] = coefficients->values[1];
      plane_object_msg.coef[2] = coefficients->values[2];
      plane_object_msg.coef[3] = coefficients->values[3];

      plane_object_msg.size.data = cloud_plane->points.size();

      plane_object_msgs.plane_objects.push_back(plane_object_msg);
      plane_object_pub_.publish(plane_object_msgs);

      return true;
    }


    void segmentMultiplePlane() {

      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
      std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
      std::vector<pcl::ModelCoefficients> plane_coefficients;
      std::vector<pcl::PointIndices> plane_indices;
      std::vector<pcl::PointIndices> label_indices;
      std::vector<pcl::PointIndices> boundary_indices;
      std::vector<Eigen::Vector4f, Eigen::aligned_allocator< Eigen::Vector4f > > centroids;
      std::vector<Eigen::Matrix3f, Eigen::aligned_allocator< Eigen::Matrix3f > > covariances;
      pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);

      pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
      ne.setMaxDepthChangeFactor (0.03f);
      ne.setNormalSmoothingSize (20.0f);
      ne.setInputCloud (cloud_transformed_);
      ne.compute (*normal_cloud);

      pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
      mps.setMinInliers (700);
      mps.setAngularThreshold ((M_PI/180) * 3.0); //3 degrees
      mps.setDistanceThreshold (0.02); //2cm
      mps.setInputNormals (normal_cloud);
      mps.setInputCloud (cloud_transformed_);
      mps.segmentAndRefine (regions, plane_coefficients, plane_indices, labels, label_indices, boundary_indices);

      if (regions.size() != 0) {
        ROS_INFO_STREAM("Number of planes: " << regions.size());
      } else {
        ROS_INFO("No plane found!");
        return;
      }

      // Construct plane object msg
      object_msgs::PlaneObjects plane_object_msgs;
      object_msgs::PlaneObject plane_object_msg;

      for (int i = 0; i < regions.size (); i++){

        pcl_conversions::fromPCL(cloud_transformed_->header, plane_object_msg.header);

        // Get plane coefficients
        Eigen::Vector4f model = regions[i].getCoefficients();
        plane_object_msg.coef[0] = model[0];
        plane_object_msg.coef[1] = model[1];
        plane_object_msg.coef[2] = model[2];
        plane_object_msg.coef[3] = model[3];

        // Get plane center
        Eigen::Vector3f centroid = regions[i].getCentroid();
        plane_object_msg.center.x = centroid[0];
        plane_object_msg.center.y = centroid[1];
        plane_object_msg.center.z = centroid[2];

        // Get plane polygon
        pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
        contour->points = regions[i].getContour();

        // std::cout << "number of points in contour: " << contour->points.size() << '\n';
        // std::cout << "number of points in plane_indices: " << plane_indices[i].indices.size() << '\n';
        // std::cout << "number of points in boundary_indices: " << boundary_indices[i].indices.size() << '\n';
        // std::cout << "number of points in label_indices: " << label_indices[i].indices.size() << '\n';

        for (int j = 0; j < contour->points.size (); j++) { // TODO: Find a function that does this job.
          geometry_msgs::Point pt;
          pt.x = contour->points[j].x;
          pt.y = contour->points[j].y;
          pt.z = contour->points[j].z;

          plane_object_msg.polygon.push_back(pt);
        }

        // Get point cloud size
        plane_object_msg.size.data = plane_indices[i].indices.size();

        plane_object_msgs.plane_objects.push_back(plane_object_msg);
        plane_object_pub_.publish(plane_object_msgs);

      }


    }

    void extractObjects() {

      CloudT::Ptr cloud_objects (new CloudT);
      pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);
      prism_.setInputCloud(cloud_filtered_);
      prism_.setInputPlanarHull(cloud_hull_);
      prism_.setHeightLimits(prism_limits_[0], prism_limits_[1]); // Height limits
      prism_.segment(*object_indices);

      object_indices_ = object_indices;

      extract_.setInputCloud (cloud_filtered_);
      extract_.setIndices(object_indices);
      extract_.filter(*cloud_objects);

      if (debug_) {
        object_cloud_pub_.publish(cloud_objects);
      }

      ROS_INFO("Objects are segmented!");
      // TODO: Do we need post-processing?

    }

    // TODO:
    void clusterObjects(/* arguments */) {
      /* code */
    }

  void createGPDMsg() {
    geometry_msgs::Point camera_viewpoint;
    camera_viewpoint.x = 0;
    camera_viewpoint.y = 0;
    camera_viewpoint.z = 0;
    toROSMsg(*cloud_filtered_, gpd_cloud_);
    gpd::CloudIndexed gpd_cloud;
    gpd_cloud.cloud_sources.cloud = gpd_cloud_;
    gpd_cloud.cloud_sources.view_points.push_back(camera_viewpoint);
    std_msgs::Int64 camera_source, index;
    camera_source.data = 0;
    for (int i = 0; i < gpd_cloud_.width; i++) {
      gpd_cloud.cloud_sources.camera_source.push_back(camera_source);
    }

    for (int i = 0; i < object_indices_->indices.size(); i++) {
      index.data = object_indices_->indices[i];
      gpd_cloud.indices.push_back(index);
    }

    gpd_cloud_pub_.publish(gpd_cloud);
  }

  bool objectClusterServiceCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if (!PointCloudProc::transformPointCloud()){
      ROS_INFO("Couldn't transform point cloud!");
      return false;
    }

    if (!PointCloudProc::filterPointCloud()){
      ROS_INFO("Couldn't filter point cloud!");
      return false;
    }

    PointCloudProc::segmentSinglePlane();
    PointCloudProc::extractObjects();

    return true;
  }

  bool createGPDMsgServiceCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if (!PointCloudProc::transformPointCloud()){
      ROS_INFO("Couldn't transform point cloud!");
      return false;
    }

    if (!PointCloudProc::filterPointCloud()){
      ROS_INFO("Couldn't filter point cloud!");
      return false;
    }

    PointCloudProc::segmentSinglePlane();
    PointCloudProc::extractObjects();
    PointCloudProc::createGPDMsg();
    return true;

  }

  bool segmentMultiplePlaneServiceCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if (!PointCloudProc::transformPointCloud()){
      ROS_INFO("Couldn't transform point cloud!");
      return false;
    }

    PointCloudProc::segmentMultiplePlane();
    return true;

  }

  bool findTableServiceCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if (!PointCloudProc::transformPointCloud()){
      ROS_INFO("Couldn't transform point cloud!");
      return false;
    }

    if (!PointCloudProc::filterPointCloud(false)){
      ROS_INFO("Couldn't filter point cloud!");
      return false;
    }

    if (!PointCloudProc::segmentSinglePlane()){
      ROS_INFO("Couldn't segment a plane!");
      return false;
    }

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
    sensor_msgs::PointCloud2 gpd_cloud_;
    pcl::PointIndices::Ptr object_indices_;

    boost::mutex pc_mutex_;

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher table_cloud_pub_, object_cloud_pub_, gpd_cloud_pub_;
    ros::Publisher plane_object_pub_;
    ros::ServiceServer object_cluster_srv_, create_gpd_msg_srv_;
    ros::ServiceServer segment_multiple_plane_srv_, find_table_srv_;
    tf::TransformListener listener_;


};

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_seg");
  ros::NodeHandle n("~");
  PointCloudProc pc_tools(n);
  ros::spin();

  return 0;

}
