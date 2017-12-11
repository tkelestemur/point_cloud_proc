// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <point_cloud_proc/Planes.h>
#include <point_cloud_proc/Objects.h>
#include <point_cloud_proc/SinglePlaneSegmentation.h>
#include <point_cloud_proc/MultiPlaneSegmentation.h>
#include <point_cloud_proc/TabletopExtraction.h>
#include <point_cloud_proc/TabletopClustering.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
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
    PointCloudProc(ros::NodeHandle n) : nh_(n), debug_(false) {

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
      fixed_frame_ = "/base_link";

      nh_.getParam("/filters/leaf_size", leaf_size_);
      nh_.getParam("/filters/use_passthrough", use_pass);
      nh_.getParam("/filters/use_voxel", use_voxel);
      nh_.getParam("/filters/pass_limits", pass_limits_);
      nh_.getParam("/segmentation/prism_limits", prism_limits_);
      nh_.getParam("/point_cloud_topic", point_cloud_topic_);
      nh_.getParam("/point_cloud_debug", debug_);
      nh_.getParam("/fixed_frame", fixed_frame_);

      point_cloud_sub_ = nh_.subscribe<CloudT> (point_cloud_topic_, 10, &PointCloudProc::pointCloudCb, this);
      table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table_cloud", 10);
      multi_object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tabletop_cloud", 10);
      single_object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("object_cloud", 10);
      single_object_image_pub_ = nh_.advertise<sensor_msgs::Image>("object_image", 10);
      plane_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("plane_polygon", 10);

      segment_multiple_plane_srv_ = nh_.advertiseService ("segment_multi_plane", &PointCloudProc::segmentMultiplePlaneServiceCb, this);
      segment_single_plane_srv_ = nh_.advertiseService ("segment_single_plane", &PointCloudProc::segmentSinglePlaneServiceCb, this);
      extract_tabletop_srv_ = nh_.advertiseService ("extract_tabletop", &PointCloudProc::extractTabletopServiceCb, this);
      cluster_tabletop_srv_ = nh_.advertiseService ("cluster_tabletop", &PointCloudProc::clusterTabletopObjectsServiceCb, this);


    }

    void pointCloudCb(const CloudT::ConstPtr &msg) {
      boost::mutex::scoped_lock lock(pc_mutex_);
      cloud_raw_ = msg;

    }

    bool transformPointCloud() {
      CloudT::Ptr cloud_transformed(new CloudT);
      std::string fixed_frame = "/base_link"; // TODO: Make it rosparam
      bool transform_success = pcl_ros::transformPointCloud(fixed_frame_, *cloud_raw_, *cloud_transformed, listener_);
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

    void segmentSinglePlane(bool create_srv_res){

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

      if (inliers->indices.size() == 0) { //TODO: return false if this doesn't hold
        ROS_INFO("No plane found!");
        // res.success = false;
        return;
      } else ROS_INFO("Single plane is segmented!");

      // ROS_INFO_STREAM("Coefficients: " << coefficients->values[0] << " "
      //                                  << coefficients->values[1] << " "
      //                                  << coefficients->values[2] << " "
      //                                  << coefficients->values[3]);

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


      if (create_srv_res) {
        // Construct plane object msg
        point_cloud_proc::Plane plane_object_msg;

        pcl_conversions::fromPCL(cloud_plane->header, plane_object_msg.header);

        // Get plane center
        plane_object_msg.center.x = center[0];
        plane_object_msg.center.y = center[1];
        plane_object_msg.center.z = center[2];

        // Get plane polygon
        for (int i = 0; i < cloud_hull->points.size (); i++) {
          geometry_msgs::Point32 p;
          p.x = cloud_hull->points[i].x;
          p.y = cloud_hull->points[i].y;
          p.z = cloud_hull->points[i].z;

          plane_object_msg.polygon.push_back(p);
        }

        // Get plane coefficients
        plane_object_msg.coef[0] = coefficients->values[0];
        plane_object_msg.coef[1] = coefficients->values[1];
        plane_object_msg.coef[2] = coefficients->values[2];
        plane_object_msg.coef[3] = coefficients->values[3];

        plane_object_msg.size.data = cloud_plane->points.size();
        plane_object_msg.is_horizontal = true;
        plane_object_ = plane_object_msg;
        // res.success = true;
        // res.plane_object = plane_object_msg;
      }



    }


    void segmentMultiplePlane(bool create_srv_res) {
      //   // Construct plane object msg
        point_cloud_proc::Planes plane_object_msgs;
        point_cloud_proc::Plane plane_object_msg;
        int MIN_PLANE_SIZE = 1000;
        int no_planes = 0;
        CloudT::Ptr cloud_plane (new CloudT);
        CloudT::Ptr cloud_hull (new CloudT);


        Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //z axis
        seg_.setOptimizeCoefficients (true);
        seg_.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg_.setMethodType (pcl::SAC_RANSAC);
        seg_.setAxis(axis);
        seg_.setEpsAngle(15.0f * (M_PI/180.0f));
        seg_.setDistanceThreshold (0.01);



        while(true) {

          pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
          pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

          seg_.setInputCloud (cloud_filtered_);
          seg_.segment (*inliers, *coefficients);

          if (inliers->indices.size() == 0 and no_planes == 0) {
            ROS_INFO("No plane found!"); // TODO: return false;
            break;
          }
          else if (inliers->indices.size() < MIN_PLANE_SIZE) {
            break;
          }
          else {
            ROS_INFO_STREAM(no_planes << ". plane segmented!");
            ROS_INFO_STREAM("Plane size : " << inliers->indices.size());
            no_planes++;
          }

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

          Eigen::Vector4f center;
          pcl::compute3DCentroid(*cloud_hull, center); // TODO: Compare with cloud_plane center


          if (create_srv_res) {
            // Construct plane object msg
            pcl_conversions::fromPCL(cloud_plane->header, plane_object_msg.header);

            // Get plane center
            plane_object_msg.center.x = center[0];
            plane_object_msg.center.y = center[1];
            plane_object_msg.center.z = center[2];

            // Get plane polygon
            for (int i = 0; i < cloud_hull->points.size (); i++) {
              geometry_msgs::Point32 p;
              p.x = cloud_hull->points[i].x;
              p.y = cloud_hull->points[i].y;
              p.z = cloud_hull->points[i].z;

              plane_object_msg.polygon.push_back(p);
            }

            // Get plane coefficients
            plane_object_msg.coef[0] = coefficients->values[0];
            plane_object_msg.coef[1] = coefficients->values[1];
            plane_object_msg.coef[2] = coefficients->values[2];
            plane_object_msg.coef[3] = coefficients->values[3];

            plane_object_msg.size.data = cloud_plane->points.size();
            plane_object_msg.is_horizontal = true;

            plane_object_msgs.objects.push_back(plane_object_msg);
            extract_.setNegative(true);
            extract_.filter(*cloud_filtered_);
          }

          if (create_srv_res) {
            plane_objects_ = plane_object_msgs;
          }

        }
    }



    void extractTabletop() {

      CloudT::Ptr cloud_objects (new CloudT);
      // sensor_msgs::PointCloud2 tabletop_objects;
      pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);
      prism_.setInputCloud(cloud_filtered_);
      prism_.setInputPlanarHull(cloud_hull_);
      prism_.setHeightLimits(prism_limits_[0], prism_limits_[1]); // Height limits
      prism_.segment(*object_indices);

      object_indices_ = object_indices;

      extract_.setInputCloud (cloud_filtered_);
      extract_.setIndices(object_indices);
      extract_.filter(*cloud_objects);

      cloud_tabletop_ = cloud_objects;
      if (debug_) {
        multi_object_pub_.publish(cloud_objects);
      }


      ROS_INFO("Objects are segmented!");
      // TODO: Do we need post-processing?

      toROSMsg(*cloud_objects, tabletop_cloud_);
      // res.object_cluster = tabletop_objects;
    }


    void clusterObjects() {
      tabletop_objects_.objects.clear();
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud (cloud_tabletop_);
      std::vector<pcl::PointIndices> cluster_indices;

      ec_.setClusterTolerance (0.02);
      ec_.setMinClusterSize (300);
      ec_.setMaxClusterSize (25000);
      ec_.setSearchMethod (tree);
      ec_.setInputCloud (cloud_tabletop_);
      ec_.extract (cluster_indices);

      int j = 0;
      ROS_INFO_STREAM("Number of objects: " << cluster_indices.size());
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

        CloudT::Ptr cloud_cluster (new CloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          cloud_cluster->points.push_back (cloud_tabletop_->points[*pit]);
          std::cout << "points" << cloud_tabletop_->points[*pit] <<'\n';
        }

        // extract_.setInputCloud (cloud_tabletop_);
        // extract_.setNegative(false);
        // extract_.setIndices (it-);
        // extract_.filter (*cloud_cluster);

        cloud_cluster->header.frame_id = cloud_tabletop_->header.frame_id;
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // get object point cloud
        point_cloud_proc::Object object;
        pcl_conversions::fromPCL(cloud_cluster->header, object.header);
        toROSMsg(*cloud_cluster, object.cloud);

        // get object center
        Eigen::Vector4f center;
        pcl::compute3DCentroid(*cloud_cluster, center);
        object.center.x = center[0];
        object.center.y = center[1];
        object.center.z = center[2];

        // get rgb image
        sensor_msgs::Image rgb_image;


        j++;
        ROS_INFO_STREAM("# of points in object " << j << " : " << cloud_cluster->points.size());

        tabletop_objects_.objects.push_back(object);

        if (debug_) {
          single_object_pub_.publish(object.cloud);
        }

      }
    }


  bool segmentSinglePlaneServiceCb(point_cloud_proc::SinglePlaneSegmentation::Request &req,
                                   point_cloud_proc::SinglePlaneSegmentation::Response &res){
    if (!PointCloudProc::transformPointCloud()){
      ROS_INFO("Couldn't transform point cloud!");
      res.success = false;
      return true;
    }

    if (!PointCloudProc::filterPointCloud(false)){
      ROS_INFO("Couldn't filter point cloud!");
      res.success = false;
      return true;
    }

    PointCloudProc::segmentSinglePlane(true);
    res.plane_object = plane_object_;
    res.success = true;

    return true;
  }


  bool segmentMultiplePlaneServiceCb(point_cloud_proc::MultiPlaneSegmentation::Request &req,
                                     point_cloud_proc::MultiPlaneSegmentation::Response &res){
    if (!PointCloudProc::transformPointCloud()){
      ROS_INFO("Couldn't transform point cloud!");
      res.success = false;
      return true;
    }

    if (!PointCloudProc::filterPointCloud(true)){
      ROS_INFO("Couldn't filter point cloud!");
      res.success = false;
      return true;
    }

    PointCloudProc::segmentMultiplePlane(true);
    res.plane_objects = plane_objects_;
    res.success = true;
    return true;

  }

  bool extractTabletopServiceCb(point_cloud_proc::TabletopExtraction::Request &req,
                                point_cloud_proc::TabletopExtraction::Response &res){
    if (!PointCloudProc::transformPointCloud()){
      ROS_INFO("Couldn't transform point cloud!");
      res.success = false;
      return true;
    }

    if (!PointCloudProc::filterPointCloud(false)){
      ROS_INFO("Couldn't filter point cloud!");
      res.success = false;
      return true;
    }
    PointCloudProc::segmentSinglePlane(false);
    PointCloudProc::extractTabletop();
    res.object_cluster = tabletop_cloud_;
    res.success = true;
    return true;

  }

  bool clusterTabletopObjectsServiceCb(point_cloud_proc::TabletopClustering::Request &req,
                                       point_cloud_proc::TabletopClustering::Response &res){

     if (!PointCloudProc::transformPointCloud()){
       ROS_INFO("Couldn't transform point cloud!");
       res.success = false;
       return false;
     }

     if (!PointCloudProc::filterPointCloud(false)){
       ROS_INFO("Couldn't filter point cloud!");
       res.success = false;
       return true;
     }

     PointCloudProc::segmentSinglePlane(false);
     PointCloudProc::extractTabletop();
     PointCloudProc::clusterObjects();
     res.tabletop_objects = tabletop_objects_;
     res.success = true;

     return true;
  }

  private:
    pcl::PassThrough<PointT> pass_;
    pcl::VoxelGrid<PointT> vg_;
    pcl::SACSegmentation<PointT> seg_;
    pcl::ExtractIndices<PointT> extract_;
    pcl::ConvexHull<PointT> chull_;
    pcl::ExtractPolygonalPrismData<PointT> prism_;
    pcl::EuclideanClusterExtraction<PointT> ec_;

    bool debug_;
    int use_pass, use_voxel;
    float leaf_size_;
    std::vector<float> pass_limits_, prism_limits_;
    std::string point_cloud_topic_, fixed_frame_;
    CloudT::ConstPtr cloud_raw_;
    CloudT::Ptr cloud_transformed_, cloud_filtered_, cloud_hull_, cloud_tabletop_;
    sensor_msgs::PointCloud2 gpd_cloud_, tabletop_cloud_;
    pcl::PointIndices::Ptr object_indices_;

    boost::mutex pc_mutex_;

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher table_cloud_pub_, plane_polygon_pub_;
    ros::Publisher multi_object_pub_, single_object_pub_, single_object_image_pub_;
    ros::ServiceServer extract_tabletop_srv_, cluster_tabletop_srv_;
    ros::ServiceServer segment_multiple_plane_srv_, segment_single_plane_srv_;
    tf::TransformListener listener_;

    point_cloud_proc::Plane plane_object_;
    point_cloud_proc::Planes plane_objects_;
    point_cloud_proc::Object tabletop_object_;
    point_cloud_proc::Objects tabletop_objects_;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_proc");
  ros::NodeHandle n("~");
  PointCloudProc pc_tools(n);
  ros::spin();

  return 0;

}
