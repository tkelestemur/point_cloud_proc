#include <point_cloud_proc/point_cloud_proc.h>

PointCloudProc::PointCloudProc(ros::NodeHandle n) :
        nh_(n), cloud_transformed_(new CloudT), cloud_filtered_(new CloudT),
        cloud_hull_(new CloudT), cloud_tabletop_(new CloudT){

      debug_ = false;
      k_search_ = 50;
      leaf_size_ = 0.01;
      cluster_tol_ = 0.03;
      normal_radius_ = 0.03;
      pass_limits_ = {-2.0, 2.0, -2.0, 2.0, -2.0, 2.0};
      prism_limits_ = {0.0, 0.05};
      point_cloud_topic_ = "/hsrb/head_rgbd_sensor/depth_registered/points";
      fixed_frame_ = "/base_link";

      nh_.getParam("/point_cloud_debug", debug_);
      nh_.getParam("/filters/leaf_size", leaf_size_);
      nh_.getParam("/filters/pass_limits", pass_limits_);
      nh_.getParam("/filters/normal_radius", normal_radius_);
      nh_.getParam("/filters/k_search", k_search_);
      nh_.getParam("/segmentation/prism_limits", prism_limits_);
      nh_.getParam("/segmentation/cluster_tolerance", cluster_tol_);
      nh_.getParam("/point_cloud_topic", point_cloud_topic_);

      nh_.getParam("/fixed_frame", fixed_frame_);

      point_cloud_sub_ = nh_.subscribe<CloudT> (point_cloud_topic_, 10, &PointCloudProc::pointCloudCb, this);

      if (debug_) {
        table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table_cloud", 10);
        multi_object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tabletop_cloud", 10);
        // single_object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("recognized_object", 10);
        // single_object_image_pub_ = nh_.advertise<sensor_msgs::Image>("object_image", 10);
        plane_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("plane_polygon", 10);
      }

      segment_multiple_plane_srv_ = nh_.advertiseService ("segment_multi_plane", &PointCloudProc::segmentMultiplePlaneServiceCb, this);
      segment_single_plane_srv_ = nh_.advertiseService ("segment_single_plane", &PointCloudProc::segmentSinglePlaneServiceCb, this);
      extract_tabletop_srv_ = nh_.advertiseService ("extract_tabletop", &PointCloudProc::extractTabletopServiceCb, this);
      cluster_tabletop_srv_ = nh_.advertiseService ("cluster_tabletop", &PointCloudProc::clusterTabletopObjectsServiceCb, this);

    }

void PointCloudProc::pointCloudCb(const CloudT::ConstPtr &msg) {
      boost::mutex::scoped_lock lock(pc_mutex_);
      cloud_raw_ = msg;

    }

bool PointCloudProc::transformPointCloud() {
      bool transform_success = pcl_ros::transformPointCloud(fixed_frame_, *cloud_raw_, *cloud_transformed_, listener_);
      return transform_success;
    }

bool PointCloudProc::filterPointCloud(bool use_voxel=true){

      // Remove part of the scene to leave table and objects alone
      pass_.setInputCloud (cloud_transformed_);
      pass_.setFilterFieldName ("x");
      pass_.setFilterLimits (pass_limits_[0],  pass_limits_[1]);
      pass_.filter(*cloud_filtered_);
      pass_.setInputCloud (cloud_filtered_);
      pass_.setFilterFieldName ("y");
      pass_.setFilterLimits (pass_limits_[2],  pass_limits_[3]);
      pass_.filter(*cloud_filtered_);
      pass_.setInputCloud (cloud_filtered_);
      pass_.setFilterFieldName ("z");
      pass_.setFilterLimits (pass_limits_[4],  pass_limits_[5]);
      pass_.filter(*cloud_filtered_);

      // Downsample point cloud
      // if (use_voxel) {
      //   vg_.setInputCloud (cloud_filtered_);
      //   vg_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
      //   vg_.filter (*cloud_filtered_);
      // }



      ROS_INFO("Point cloud is filtered!");
      if (cloud_filtered_->points.size() == 0) {
        ROS_WARN("Point cloud is empty after filtering!");
        return false;
      }

      return true;
    }

bool PointCloudProc::segmentSinglePlane(bool create_srv_res){

      CloudT::Ptr cloud_plane (new CloudT);
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
        return false;
      }

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
      chull_.reconstruct (*cloud_hull_);


      if (create_srv_res) {
        // Construct plane object msg
        point_cloud_proc::Plane plane_object_msg;

        pcl_conversions::fromPCL(cloud_plane->header, plane_object_msg.header);


        // Get plane center
        Eigen::Vector4f center;
        pcl::compute3DCentroid(*cloud_plane, center);
        plane_object_msg.center.x = center[0];
        plane_object_msg.center.y = center[1];
        plane_object_msg.center.z = center[2];

        // Get plane min and max values
        Eigen::Vector4f min_vals, max_vals;
        pcl::getMinMax3D(*cloud_plane, min_vals, max_vals);

        plane_object_msg.min.x = min_vals[0];
        plane_object_msg.min.y = min_vals[1];
        plane_object_msg.min.z = min_vals[2];


        plane_object_msg.max.x = max_vals[0];
        plane_object_msg.max.y = max_vals[1];
        plane_object_msg.max.z = max_vals[2];

        // Get plane polygon
        for (int i = 0; i < cloud_hull_->points.size (); i++) {
          geometry_msgs::Point32 p;
          p.x = cloud_hull_->points[i].x;
          p.y = cloud_hull_->points[i].y;
          p.z = cloud_hull_->points[i].z;

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

      return true;

    }

void PointCloudProc::segmentMultiplePlane(bool create_srv_res) {
    //   // Construct plane object msg
    point_cloud_proc::Planes plane_object_msgs;
    point_cloud_proc::Plane plane_object_msg;
    int MIN_PLANE_SIZE = 10000;
    int no_planes = 0;
    CloudT::Ptr cloud_plane (new CloudT);
    CloudT::Ptr cloud_hull (new CloudT);


    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //z axis
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg_.setMaxIterations(200);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setAxis(axis);
    seg_.setEpsAngle(5.0f * (M_PI/180.0f));
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

bool PointCloudProc::extractTabletop() {

      pcl::PointIndices::Ptr tabletop_indices(new pcl::PointIndices);
      prism_.setInputCloud(cloud_filtered_);
      prism_.setInputPlanarHull(cloud_hull_);
      prism_.setHeightLimits(prism_limits_[0], prism_limits_[1]); // Height limits
      prism_.segment(*tabletop_indices);

      extract_.setInputCloud (cloud_filtered_);
      extract_.setIndices(tabletop_indices);
      extract_.filter(*cloud_tabletop_);

      if (cloud_tabletop_->points.size() == 0) {
        return false;
      } else {

        if (debug_) {
          multi_object_pub_.publish(cloud_tabletop_);
        }
        toROSMsg(*cloud_tabletop_, tabletop_cloud_);
        return true;
      }

    }

bool PointCloudProc::clusterObjects() {
      tabletop_objects_.objects.clear();
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);


      tree->setInputCloud (cloud_tabletop_);
      std::vector<pcl::PointIndices> cluster_indices;

      ec_.setClusterTolerance (cluster_tol_);
      ec_.setMinClusterSize (50);
      ec_.setMaxClusterSize (25000);
      ec_.setSearchMethod (tree);
      ec_.setInputCloud (cloud_tabletop_);
      ec_.extract (cluster_indices);

      // pcl::MomentOfInertiaEstimation <PointT> intertia_est_;
      pcl::PCA<PointT> pca_ = new pcl::PCA<PointT>;
      pcl::NormalEstimationOMP<PointT, PointNT> ne(4);


      int k = 0;
      if (cluster_indices.size() == 0) {
        return false;
      } else{
          ROS_INFO_STREAM("Number of objects: " << cluster_indices.size());
      }

      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

        CloudT::Ptr cluster (new CloudT);
        CloudNT::Ptr cluster_normals (new CloudNT);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          cluster->points.push_back (cloud_tabletop_->points[*pit]);
        }


        pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);
        object_indices->indices = it->indices;
        Eigen::Matrix3f eigen_vectors;
        Eigen::Vector3f eigen_values;
        pca_.setInputCloud(cloud_tabletop_);
        pca_.setIndices(object_indices);
        eigen_vectors = pca_.getEigenVectors();
        eigen_values = pca_.getEigenValues();


        cluster->header = cloud_tabletop_->header;
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        // compute point normals
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        ne.setInputCloud(cluster);
        ne.setSearchMethod (tree);
        // ne.setRadiusSearch (normal_radius_);
        ne.setKSearch(k_search_);
        ne.compute (*cluster_normals);


        point_cloud_proc::Object object;

        // get object point cloud
        pcl_conversions::fromPCL(cluster->header, object.header);
        // toROSMsg(*cluster, object.cloud);

        // get point normals
        for (int i = 0; i < cluster_normals->points.size(); i++) {
          geometry_msgs::Vector3 normal;
          normal.x = cluster_normals->points[i].normal_x;
          normal.y = cluster_normals->points[i].normal_y;
          normal.z = cluster_normals->points[i].normal_z;
          object.normals.push_back(normal);
        }

        // get point coordinates
        for (int j = 0; j < cluster->points.size(); j++) {
          geometry_msgs::Vector3 p;
          p.x = cluster->points[j].x;
          p.y = cluster->points[j].y;
          p.z = cluster->points[j].z;
          object.points.push_back(p);
        }

        // get object center
        Eigen::Vector4f center;
        pcl::compute3DCentroid(*cluster, center);
        object.center.x = center[0];
        object.center.y = center[1];
        object.center.z = center[2];

        // geometry_msgs::Pose cluster_pose;
        object.pose.position.x = center[0];
        object.pose.position.y = center[1];
        object.pose.position.z = center[2];
        Eigen::Quaternionf quat (eigen_vectors);
        quat.normalize();

        object.pose.orientation.x = quat.x();
        object.pose.orientation.y = quat.y();
        object.pose.orientation.z = quat.z();
        object.pose.orientation.w = quat.w();
        // get min max points coords
        Eigen::Vector4f min_vals, max_vals;
        pcl::getMinMax3D(*cluster, min_vals, max_vals);

        object.min.x = min_vals[0];
        object.min.y = min_vals[1];
        object.min.z = min_vals[2];
        object.max.x = max_vals[0];
        object.max.y = max_vals[1];
        object.max.z = max_vals[2];

        k++;
        ROS_INFO_STREAM("# of points in object " << k << " : " << cluster->points.size());

        tabletop_objects_.objects.push_back(object);

      }
      return true;
    }

bool PointCloudProc::segmentSinglePlaneServiceCb(point_cloud_proc::SinglePlaneSegmentation::Request &req,
                               point_cloud_proc::SinglePlaneSegmentation::Response &res){
    if (!this->transformPointCloud()){
      ROS_WARN("Couldn't transform point cloud!");
      res.success = false;
      return true;
    }

    if (!this->filterPointCloud(false)){
      ROS_WARN("Couldn't filter point cloud!");
      res.success = false;
      return true;
    }


    if (!this->segmentSinglePlane(true)) {
      ROS_INFO("No plane found!");
      res.success = false;
      return true;
    }

    ROS_INFO("Single plane is segmented!");
    res.plane_object = plane_object_;
    res.success = true;

    return true;
  }

bool PointCloudProc::segmentMultiplePlaneServiceCb(point_cloud_proc::MultiPlaneSegmentation::Request &req,
                                     point_cloud_proc::MultiPlaneSegmentation::Response &res){
    if (!this->transformPointCloud()){
      ROS_WARN("Couldn't transform point cloud!");
      res.success = false;
      return true;
    }

    if (!this->filterPointCloud(false)){
      ROS_WARN("Couldn't filter point cloud!");
      res.success = false;
      return true;
    }

    this->segmentMultiplePlane(true);
    res.planes = plane_objects_.objects;
    res.success = true;
    return true;

  }

bool PointCloudProc::extractTabletopServiceCb(point_cloud_proc::TabletopExtraction::Request &req,
                                point_cloud_proc::TabletopExtraction::Response &res){
    if (!this->transformPointCloud()){
      ROS_WARN("Couldn't transform point cloud!");
      res.success = false;
      return true;
    }

    if (!this->filterPointCloud(false)){
      ROS_WARN("Couldn't filter point cloud!");
      res.success = false;
      return true;
    }
    if (!this->segmentSinglePlane(true)) {
      ROS_INFO("No plane found!");
      res.success = false;
      return true;
    }

    if (!this->extractTabletop()){
      ROS_WARN("No tabletop object found!");
      res.success = false;
      return true;
    }

    ROS_INFO("Tabletop cloud is extracted!");
    res.object_cluster = tabletop_cloud_;
    res.success = true;
    return true;

  }

bool PointCloudProc::clusterTabletopObjectsServiceCb(point_cloud_proc::TabletopClustering::Request &req,
                                       point_cloud_proc::TabletopClustering::Response &res){

     if (!this->transformPointCloud()){
       ROS_WARN("Couldn't transform point cloud!");
       res.success = false;
       return false;
     }

     if (!this->filterPointCloud(false)){
       ROS_WARN("Couldn't filter point cloud!");
       res.success = false;
       return true;
     }

     if (!this->segmentSinglePlane(true)) {
       ROS_INFO("No plane found!");
       res.success = false;
       return true;
     }
     if (!this->extractTabletop()){
       ROS_WARN("No tabletop object found!");
       res.success = false;
       return true;
     }

     if (!this->clusterObjects()){
       ROS_WARN("No object found!");
       res.success = false;
       return true;
     }
     res.objects = tabletop_objects_.objects;
     res.success = true;

     return true;
  }
