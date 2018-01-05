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
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
// #include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/model_outlier_removal.h>
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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

// Other
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Dense>

class PointCloudProc{

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointNormal PointNT;
  typedef pcl::FPFHSignature33 FeatureT;
  typedef pcl::PointCloud<PointT> CloudT;
  typedef pcl::PointCloud<PointNT> CloudNT;
  typedef pcl::PointCloud<FeatureT> FeatureCloudT;


  public:
    PointCloudProc(ros::NodeHandle n) : nh_(n), debug_(false) {

      leaf_size_ = 0.01;
      pass_limits_.push_back(-2.0);
      pass_limits_.push_back(2.0);
      pass_limits_.push_back(-2.0);
      pass_limits_.push_back(2.0);
      pass_limits_.push_back(-2.0);
      pass_limits_.push_back(2.0);
      prism_limits_.push_back(0.0);
      prism_limits_.push_back(0.05);
      point_cloud_topic_ = "/hsrb/head_rgbd_sensor/depth_registered/points";
      fixed_frame_ = "/base_link";

      nh_.getParam("/filters/leaf_size", leaf_size_);
      nh_.getParam("/filters/pass_limits", pass_limits_);
      nh_.getParam("/segmentation/prism_limits", prism_limits_);
      nh_.getParam("/point_cloud_topic", point_cloud_topic_);
      nh_.getParam("/point_cloud_debug", debug_);
      nh_.getParam("/fixed_frame", fixed_frame_);

      point_cloud_sub_ = nh_.subscribe<CloudT> (point_cloud_topic_, 10, &PointCloudProc::pointCloudCb, this);

      if (debug_) {
        table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table_cloud", 10);
        multi_object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tabletop_cloud", 10);
        single_object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("recognized_object", 10);
        single_object_image_pub_ = nh_.advertise<sensor_msgs::Image>("object_image", 10);
        plane_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("plane_polygon", 10);
      }

      segment_multiple_plane_srv_ = nh_.advertiseService ("segment_multi_plane", &PointCloudProc::segmentMultiplePlaneServiceCb, this);
      segment_single_plane_srv_ = nh_.advertiseService ("segment_single_plane", &PointCloudProc::segmentSinglePlaneServiceCb, this);
      extract_tabletop_srv_ = nh_.advertiseService ("extract_tabletop", &PointCloudProc::extractTabletopServiceCb, this);
      cluster_tabletop_srv_ = nh_.advertiseService ("cluster_tabletop", &PointCloudProc::clusterTabletopObjectsServiceCb, this);
      recognize_object_srv_ = nh_.advertiseService ("recognize_object", &PointCloudProc::recognizeObjectServiceCb, this);


    }

    void pointCloudCb(const CloudT::ConstPtr &msg) {
      boost::mutex::scoped_lock lock(pc_mutex_);
      cloud_raw_ = msg;

    }

    bool transformPointCloud() {
      CloudT::Ptr cloud_transformed(new CloudT);
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
      CloudT::Ptr cloud_plane_proj (new CloudT);
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

      extract_.setInputCloud (cloud_filtered_);
      extract_.setNegative(false);
      extract_.setIndices (inliers);
      extract_.filter (*cloud_plane);
      ROS_INFO_STREAM("# of points in plane: " << cloud_plane->points.size());


      // model_filter_.setModelCoefficients (*coefficients);
      // model_filter_.setThreshold (0.01);
      // model_filter_.setModelType (pcl::SACMODEL_PLANE);
      // model_filter_.setInputCloud (cloud_plane);
      // model_filter_.filter (*cloud_plane_proj);

      // pcl::StatisticalOutlierRemoval<PointT> sor;
      // sor.setInputCloud (cloud_plane);
      // sor.setMeanK (30);
      // sor.setStddevMulThresh (1.0);
      // sor.filter (*cloud_plane_proj);
      //

      if (debug_) {
        table_cloud_pub_.publish(cloud_plane);
      }

      chull_.setInputCloud (cloud_plane);
      chull_.setDimension(2);
      chull_.reconstruct (*cloud_hull);

      cloud_hull_ = cloud_hull;

      Eigen::Vector4f center;
      pcl::compute3DCentroid(*cloud_plane, center);

      Eigen::Vector4f min_vals, max_vals;
      pcl::getMinMax3D(*cloud_plane, min_vals, max_vals);
      // std::cout << "min values: " << min_vals << '\n';
      // std::cout << "max values: " << max_vals << '\n';

      if (create_srv_res) {
        // Construct plane object msg
        point_cloud_proc::Plane plane_object_msg;

        pcl_conversions::fromPCL(cloud_plane->header, plane_object_msg.header);

        // Get plane center
        plane_object_msg.center.x = center[0];
        plane_object_msg.center.y = center[1];
        plane_object_msg.center.z = center[2];

        // Get plane min and max values
        plane_object_msg.min.x = min_vals[0];
        plane_object_msg.min.y = min_vals[1];
        plane_object_msg.min.z = min_vals[2];


        plane_object_msg.max.x = max_vals[0];
        plane_object_msg.max.y = max_vals[1];
        plane_object_msg.max.z = max_vals[2];

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
        seg_.setMaxIterations(200);
        seg_.setMethodType (pcl::SAC_RANSAC);
        seg_.setAxis(axis);
        seg_.setEpsAngle(25.0f * (M_PI/180.0f));
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
            ROS_INFO_STREAM(no_planes+1 << ". plane segmented!");
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

          Eigen::Vector4f min_vals, max_vals;
          pcl::getMinMax3D(*cloud_plane, min_vals, max_vals);


          if (create_srv_res) {
            // Construct plane object msg
            pcl_conversions::fromPCL(cloud_plane->header, plane_object_msg.header);

            // Get plane center
            plane_object_msg.center.x = center[0];
            plane_object_msg.center.y = center[1];
            plane_object_msg.center.z = center[2];

            // Get plane min and max values
            plane_object_msg.min.x = min_vals[0];
            plane_object_msg.min.y = min_vals[1];
            plane_object_msg.min.z = min_vals[2];


            plane_object_msg.max.x = max_vals[0];
            plane_object_msg.max.y = max_vals[1];
            plane_object_msg.max.z = max_vals[2];

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

      extract_.setInputCloud (cloud_filtered_);
      extract_.setIndices(object_indices);
      extract_.filter(*cloud_objects);


      // pcl::io::savePCDFileASCII ("/home/tarik/Desktop/objects/scene.pcd", *cloud_objects);

      if (debug_) {
        multi_object_pub_.publish(cloud_objects);
      }

      cloud_tabletop_ = cloud_objects;

      ROS_INFO("Objects are segmented!");

      toROSMsg(*cloud_objects, tabletop_cloud_);
    }


    void clusterObjects() {
      tabletop_objects_.objects.clear();
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud (cloud_tabletop_);
      std::vector<pcl::PointIndices> cluster_indices;

      ec_.setClusterTolerance (0.05);
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
        }

        cloud_cluster->header = cloud_tabletop_->header;
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters_.push_back(cloud_cluster);
        // if (save_object_pcd_) {
        //   pcl::io::savePCDFileASCII ("/home/tarik/Desktop/objects/object.pcd", *cloud_cluster);
        // }

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

        // get min max points coords
        Eigen::Vector4f min_vals, max_vals;
        pcl::getMinMax3D(*cloud_cluster, min_vals, max_vals);

        object.min.x = min_vals[0];
        object.min.y = min_vals[1];
        object.min.z = min_vals[2];
        object.max.x = max_vals[0];
        object.max.y = max_vals[1];
        object.max.z = max_vals[2];

        j++;
        ROS_INFO_STREAM("# of points in object " << j << " : " << cloud_cluster->points.size());

        tabletop_objects_.objects.push_back(object);

      }
    }


    void getICPScores(/* arguments */) {

      CloudT::Ptr object (new CloudT);
      CloudT::Ptr result (new CloudT);
      CloudT::Ptr recognized_object (new CloudT);
      std::string object_path = "/home/tarik/Desktop/objects/object.pcd";
      std::vector<double> scores;

      if (pcl::io::loadPCDFile<PointT> (object_path, *object) < 0){
        ROS_ERROR("Error loading object/scene file!");
      }

      pcl::IterativeClosestPoint<PointT, PointT> icp;

      double max_score = std::numeric_limits<double>::min();

      for (size_t i = 0; i < clusters_.size(); i++) {
        icp.setInputSource(clusters_[i]);
        icp.setInputTarget(object);
        icp.align(*result);
        ROS_INFO_STREAM("score: " << icp.getFitnessScore());
        if (icp.hasConverged()) {
          if (icp.getFitnessScore() > max_score) {
            max_score = icp.getFitnessScore();
            ROS_INFO_STREAM("max score: " << max_score);
            recognized_object = result;
          }
        }

      }


      if (debug_) {
        single_object_pub_.publish(recognized_object);
      }

    }

    void recognizeObject() {
      CloudNT::Ptr scene (new CloudNT);
      CloudNT::Ptr object (new CloudNT);
      CloudNT::Ptr object_aligned (new CloudNT);
      CloudNT::Ptr object_aligned_transformed (new CloudNT);
      FeatureCloudT::Ptr object_features (new FeatureCloudT);
      FeatureCloudT::Ptr scene_features (new FeatureCloudT);

      std::string object_path = "/home/tarik/Desktop/objects/object.pcd";

      if (pcl::io::loadPCDFile<PointNT> (object_path, *object) < 0){
        ROS_ERROR("Error loading object/scene file!");
      }

      pcl::copyPointCloud(*cloud_tabletop_, *scene);

      ROS_INFO_STREAM("scene size: " << scene->points.size());
      ROS_INFO_STREAM("object size: " << object->points.size());


      pcl::VoxelGrid<PointNT> grid;
      const float leaf = 0.005f;
      grid.setLeafSize (leaf, leaf, leaf);
      grid.setInputCloud (object);
      grid.filter (*object);
      grid.setInputCloud (scene);
      grid.filter (*scene);

      pcl::NormalEstimationOMP<PointNT,PointNT> nest;

      nest.setRadiusSearch (0.01);
      nest.setInputCloud (scene);
      nest.compute (*scene);
      nest.setInputCloud (object);
      nest.compute (*object);

      pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> fest;
      fest.setRadiusSearch (0.01);
      fest.setInputCloud (object);
      fest.setInputNormals (object);
      fest.compute (*object_features);
      fest.setInputCloud (scene);
      fest.setInputNormals (scene);
      fest.compute (*scene_features);

      pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
      align.setInputSource (object);
      align.setSourceFeatures (object_features);
      align.setInputTarget (scene);
      align.setTargetFeatures (scene_features);
      align.setMaximumIterations (50000); // Number of RANSAC iterations
      align.setNumberOfSamples (10); // Number of points to sample for generating/prerejecting a pose
      align.setCorrespondenceRandomness (5); // Number of nearest features to use
      align.setSimilarityThreshold (0.5f); // Polygonal edge length similarity threshold
      align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
      align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
      {
        // pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);
      }

      if (align.hasConverged()){
        ROS_INFO("Object aligment is successfull!");
        std::vector<int> inliers = align.getInliers();
        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        pcl::transformPointCloud(*object_aligned, *object_aligned_transformed, transformation);
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
        object_aligned_transformed->header = cloud_tabletop_->header;
        single_object_pub_.publish(object_aligned_transformed);
      }
      else{
        ROS_WARN("Object aligment is not successfull!");
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

  bool recognizeObjectServiceCb(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res){
    if (!PointCloudProc::transformPointCloud()){
      ROS_INFO("Couldn't transform point cloud!");
      return true;
    }

    if (!PointCloudProc::filterPointCloud(false)){
      ROS_INFO("Couldn't filter point cloud!");
      return true;
    }
    PointCloudProc::segmentSinglePlane(false);
    PointCloudProc::extractTabletop();
    PointCloudProc::clusterObjects();
    PointCloudProc::getICPScores();
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
    // pcl::ModelOutlierRemoval<PointT> model_filter_;

    bool debug_, use_voxel;
    float leaf_size_;
    std::vector<float> pass_limits_, prism_limits_;
    std::string point_cloud_topic_, fixed_frame_;
    CloudT::ConstPtr cloud_raw_;
    CloudT::Ptr cloud_transformed_, cloud_filtered_, cloud_hull_, cloud_tabletop_;
    sensor_msgs::PointCloud2 gpd_cloud_, tabletop_cloud_;
    pcl::PointIndices::Ptr object_indices_;
    std::vector<CloudT::Ptr> clusters_;

    boost::mutex pc_mutex_;

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher table_cloud_pub_, plane_polygon_pub_;
    ros::Publisher multi_object_pub_, single_object_pub_, single_object_image_pub_;
    ros::ServiceServer extract_tabletop_srv_, cluster_tabletop_srv_, recognize_object_srv_;
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
