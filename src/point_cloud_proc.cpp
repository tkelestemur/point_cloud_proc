#include <point_cloud_proc/point_cloud_proc.h>

PointCloudProc::PointCloudProc(ros::NodeHandle n, bool debug) :
        nh_(n), debug_(debug), cloud_transformed_(new CloudT), cloud_filtered_(new CloudT),
        cloud_hull_(new CloudT), cloud_tabletop_(new CloudT) {

    leaf_size_ = 0.01;
    k_search_ = 50;
    cluster_tol_ = 0.03;
    pass_limits_ = {-0.2, 1.5, -1.5, 1.5, 0.2, 2.0};
    prism_limits_ = {-0.25, -0.02};
    point_cloud_topic_ = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
    fixed_frame_ = "base_link";

//    nh_.getParam("/point_cloud_debug", debug_);
//    nh_.getParam("/filters/pass_limits", pass_limits_);
//    nh_.getParam("/filters/k_search", k_search_);
//    nh_.getParam("/segmentation/prism_limits", prism_limits_);
//    nh_.getParam("/segmentation/cluster_tolerance", cluster_tol_);
//    nh_.getParam("/point_cloud_topic_name", point_cloud_topic_);
//    nh_.getParam("/point_cloud_frame", fixed_frame_);

    point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 10, &PointCloudProc::pointCloudCb, this);


    if (debug_) {
      plane_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_cloud", 10);
      debug_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud", 10);
      tabletop_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tabletop_cloud", 10);


    }
}

//void PointCloudProc::pointCloudCb(const CloudT::ConstPtr &msg) {
//    boost::mutex::scoped_lock lock(pc_mutex_);
//    cloud_raw_ = msg;
//}

void PointCloudProc::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
  boost::mutex::scoped_lock lock(pc_mutex_);
  cloud_raw_ros_ = *msg;
//  sensor_msgs::Image img;
//  pcl::toROSMsg(*msg, img);
//  test_img_pub_.publish(img);
}

//void PointCloudProc::transformBroadcasterThread() {
//    tf::TransformListener listener;
//    tf::TransformBroadcaster br;
//
//    listener.waitForTransform("base_link", "head_pan_link", ros::Time::now(), ros::Duration(2.0));
//
//    ros::Rate rate(100.0);
//    while (nh_.ok()){
//      tf::StampedTransform transform;
//      try{
//        listener.lookupTransform("base_link", "head_pan_link", ros::Time(0), transform);
////        tf::Transform cloud_frame;
//        cloud_frame_.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
//        cloud_frame_.setRotation(transform.getRotation());
////        br.sendTransform(tf::StampedTransform(cloud_frame, ros::Time::now(), "base_link", "point_cloud_frame"));
//      }
//      catch (tf::TransformException ex){
//        ROS_ERROR("%s",ex.what());
//        ros::Duration(1.0).sleep();
//      }
//      rate.sleep();
//    }
//
//}
//
//
//void PointCloudProc::startThreads() {
//  transform_br_thread_ = boost::thread(&PointCloudProc::transformBroadcasterThread, this);
//
//}

bool PointCloudProc::transformPointCloud() {
    boost::mutex::scoped_lock lock(pc_mutex_);

    cloud_transformed_->clear();

    tf::TransformListener listener;
    std::string target_frame = cloud_raw_ros_.header.frame_id; //

    listener.waitForTransform(fixed_frame_, target_frame, ros::Time(0), ros::Duration(2.0));
    tf::StampedTransform transform;
    tf::Transform cloud_transform;

    try{
      listener.lookupTransform(fixed_frame_, target_frame, ros::Time(0), transform);
      cloud_transform.setOrigin(transform.getOrigin());
      cloud_transform.setRotation(transform.getRotation());

      sensor_msgs::PointCloud2 cloud_transformed;
      pcl_ros::transformPointCloud(fixed_frame_, cloud_transform, cloud_raw_ros_, cloud_transformed);

      pcl::fromROSMsg(cloud_transformed, *cloud_transformed_);

      std::cout << "PCP: point cloud is transformed!" << std::endl;
      return true;

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }


}

bool PointCloudProc::filterPointCloud() {

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

    std::cout << "PCP: point cloud is filtered!" << std::endl;
    if (cloud_filtered_->points.size() == 0) {
        std::cout <<  "PCP: point cloud is empty after filtering!" << std::endl;
        return false;
    }

  // Downsample point cloud
//   vg_.setInputCloud (cloud_filtered_);
//   vg_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
//   vg_.filter (*cloud_filtered_);

    return true;
}

bool PointCloudProc::removeOutliers(CloudT::Ptr in, CloudT::Ptr out) {

  outrem_.setInputCloud(in);
  outrem_.setRadiusSearch(0.01);
  outrem_.setMinNeighborsInRadius (50);
  outrem_.filter (*out);

}

bool PointCloudProc::segmentSinglePlane(point_cloud_proc::Plane& plane) {
//    boost::mutex::scoped_lock lock(pc_mutex_);
    std::cout << "PCP: segmenting single plane..." << std::endl;

    if(!transformPointCloud()){
      std::cout << "PCP: couldn't transform point cloud!" << std::endl;
      return false;
    }

    if(!filterPointCloud()){
      std::cout << "PCP: couldn't filter point cloud!" << std::endl;
      return false;
    }



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


    if (inliers->indices.size() == 0){
      std::cout << "PCP: plane is empty!" << std::endl;
      return false;
    }


    extract_.setInputCloud (cloud_filtered_);
    extract_.setNegative(false);
    extract_.setIndices (inliers);
    extract_.filter (*cloud_plane);

    if (debug_) {
      std::cout << "PCP: # of points in plane: " << cloud_plane->points.size() << std::endl;
      plane_cloud_pub_.publish(cloud_plane);
    }

    cloud_hull_->clear();
    chull_.setInputCloud (cloud_plane);
    chull_.setDimension(2);
    chull_.reconstruct (*cloud_hull_);


    // Construct plane object msg
    pcl_conversions::fromPCL(cloud_plane->header, plane.header);

    // Get plane center
    Eigen::Vector4f center;
    pcl::compute3DCentroid(*cloud_plane, center);
    plane.center.x = center[0];
    plane.center.y = center[1];
    plane.center.z = center[2];

    // Get plane min and max values
    Eigen::Vector4f min_vals, max_vals;
    pcl::getMinMax3D(*cloud_plane, min_vals, max_vals);

    plane.min.x = min_vals[0];
    plane.min.y = min_vals[1];
    plane.min.z = min_vals[2];


    plane.max.x = max_vals[0];
    plane.max.y = max_vals[1];
    plane.max.z = max_vals[2];

    // Get plane polygon
    for (int i = 0; i < cloud_hull_->points.size (); i++) {
        geometry_msgs::Point32 p;
        p.x = cloud_hull_->points[i].x;
        p.y = cloud_hull_->points[i].y;
        p.z = cloud_hull_->points[i].z;

        plane.polygon.push_back(p);
    }

    // Get plane coefficients
    plane.coef[0] = coefficients->values[0];
    plane.coef[1] = coefficients->values[1];
    plane.coef[2] = coefficients->values[2];
    plane.coef[3] = coefficients->values[3];

    plane.size.data = cloud_plane->points.size();
    plane.is_horizontal = true;

    return true;
}

bool PointCloudProc::segmentMultiplePlane(std::vector<point_cloud_proc::Plane>& planes) {

//    boost::mutex::scoped_lock lock(pc_mutex_);

    if(!transformPointCloud()){
      std::cout << "PCP: couldn't transform point cloud!" << std::endl;
      return false;
    }

    if(!filterPointCloud()){
      std::cout << "PCP: couldn't filter point cloud!" << std::endl;
      return false;
    }

    CloudT plane_clouds;
    plane_clouds.header.frame_id = cloud_transformed_->header.frame_id;
    point_cloud_proc::Plane plane_object_msg;
    int MIN_PLANE_SIZE = 5000;
    int no_planes = 0;
    CloudT::Ptr cloud_plane (new CloudT);
    CloudT::Ptr cloud_hull (new CloudT);

    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //z axis
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg_.setMaxIterations(200);
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
            std::cout <<  "PCP: no plane found!!!" << std::endl; // TODO: return false;
            return false;
        }

        else if (inliers->indices.size() < MIN_PLANE_SIZE) {
            break;
        }
        else {
            std::cout << "PCP: " <<no_planes+1 << ". plane segmented! # of points: " << inliers->indices.size() << std::endl;
            no_planes++;
        }

        extract_.setInputCloud (cloud_filtered_);
        extract_.setNegative(false);
        extract_.setIndices (inliers);
        extract_.filter (*cloud_plane);

        plane_clouds +=*cloud_plane;

        chull_.setInputCloud (cloud_plane);
        chull_.setDimension(2);
        chull_.reconstruct (*cloud_hull);

        Eigen::Vector4f center;
        pcl::compute3DCentroid(*cloud_hull, center); // TODO: Compare with cloud_plane center

        Eigen::Vector4f min_vals, max_vals;
        pcl::getMinMax3D(*cloud_plane, min_vals, max_vals);


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

        planes.push_back(plane_object_msg);
        extract_.setNegative(true);
        extract_.filter(*cloud_filtered_);

    }

    if (debug_) {
      plane_cloud_pub_.publish(plane_clouds);
    }
    return true;
}

bool PointCloudProc::extractTabletop() {

    pcl::PointIndices::Ptr tabletop_indices(new pcl::PointIndices);
    prism_.setInputCloud(cloud_filtered_);
    prism_.setInputPlanarHull(cloud_hull_);
    prism_.setHeightLimits(prism_limits_[0], prism_limits_[1]);
    prism_.segment(*tabletop_indices);

    extract_.setInputCloud (cloud_filtered_);
    extract_.setIndices(tabletop_indices);
    extract_.filter(*cloud_tabletop_);

    if (cloud_tabletop_->points.size() == 0) {
        return false;
    }
    else {
        if (debug_) {
          tabletop_pub_.publish(cloud_tabletop_);
        }
        return true;
    }
}

bool PointCloudProc::clusterObjects(std::vector<point_cloud_proc::Object>& objects) {

    if (!extractTabletop()){
      return false;
    }

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    tree->setInputCloud (cloud_tabletop_);
    std::vector<pcl::PointIndices> cluster_indices;

    ec_.setClusterTolerance (cluster_tol_);
    ec_.setMinClusterSize (50);
    ec_.setMaxClusterSize (25000);
    ec_.setSearchMethod (tree);
    ec_.setInputCloud (cloud_tabletop_);
    ec_.extract (cluster_indices);

    pcl::PCA<PointT> pca_ = new pcl::PCA<PointT>;
    pcl::NormalEstimationOMP<PointT, PointNT> ne(4);

    int k = 0;
    if (cluster_indices.size() == 0)
        return false;
    else
      std::cout << "PCP: number of objects: " << cluster_indices.size() << std::endl;

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

        // Compute point normals
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        ne.setInputCloud(cluster);
        ne.setSearchMethod (tree);
        ne.setKSearch(k_search_);
        ne.compute (*cluster_normals);

        point_cloud_proc::Object object;

        // Get object point cloud
        pcl_conversions::fromPCL(cluster->header, object.header);

        // Get point normals
        for (int i = 0; i < cluster_normals->points.size(); i++) {
            geometry_msgs::Vector3 normal;
            normal.x = cluster_normals->points[i].normal_x;
            normal.y = cluster_normals->points[i].normal_y;
            normal.z = cluster_normals->points[i].normal_z;
            object.normals.push_back(normal);
        }

        // Get point coordinates
        for (int j = 0; j < cluster->points.size(); j++) {
            geometry_msgs::Vector3 p;
            p.x = cluster->points[j].x;
            p.y = cluster->points[j].y;
            p.z = cluster->points[j].z;
            object.points.push_back(p);
        }

        // Get object center
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
        // Get min max points coords
        Eigen::Vector4f min_vals, max_vals;
        pcl::getMinMax3D(*cluster, min_vals, max_vals);

        object.min.x = min_vals[0];
        object.min.y = min_vals[1];
        object.min.z = min_vals[2];
        object.max.x = max_vals[0];
        object.max.y = max_vals[1];
        object.max.z = max_vals[2];

        k++;
        if(debug_){
          std::cout << "PCP: # of points in object " << k << " : " << cluster->points.size() << std::endl;
        }


        objects.push_back(object);
    }
    return true;
}

bool PointCloudProc::get3DPoint(int col, int row, geometry_msgs::PointStamped &point) {

  if(!transformPointCloud()){
    std::cout << "PCP: couldn't transform point cloud!" << std::endl;
    return false;
  }

  pcl_conversions::fromPCL(cloud_transformed_->header, point.header);

  if (pcl::isFinite(cloud_transformed_->at(col, row))) {
    point.point.x = cloud_transformed_->at(col, row).x;
    point.point.y = cloud_transformed_->at(col, row).y;
    point.point.z = cloud_transformed_->at(col, row).z;
    return true;
  }else{
    std::cout << "PCP: The 3D point is not valid!" << std::endl;
    return false;
  }

}

bool PointCloudProc::getObjectFromBBox(int *bbox, point_cloud_proc::Object& object) {

  if(!transformPointCloud()){
    std::cout << "PCP: couldn't transform point cloud!" << std::endl;
    return false;
  }
  sensor_msgs::PointCloud2 object_cloud_ros;
  pcl_conversions::fromPCL(cloud_transformed_->header, object.header);

  CloudT::Ptr object_cloud(new CloudT);
  CloudT::Ptr object_cloud_filtered(new CloudT);
  object_cloud->header = cloud_transformed_->header;

  for (int i = bbox[0]; i < bbox[2]; i++){
    for (int j = bbox[1]; j < bbox[3]; j++){
      if (pcl::isFinite(cloud_transformed_->at(i, j))) {

        object_cloud->push_back(cloud_transformed_->at(i, j));
//        std::cout << cloud_transformed_->at(i, j).x << " " << cloud_transformed_->at(i, j).y << " " << cloud_transformed_->at(i, j).z <<std::endl;
      }
    }

  }

  removeOutliers(object_cloud, object_cloud_filtered);
  if(object_cloud_filtered->empty()){
    std::cout << "PCP: object cloud is empty after removing outliers!" << std::endl;
    return false;
  }

  Eigen::Vector4f min_vals, max_vals;

  pcl::getMinMax3D(*object_cloud_filtered, min_vals, max_vals);

  object.min.x = min_vals[0];
  object.min.y = min_vals[1];
  object.min.z = min_vals[2];
  object.max.x = max_vals[0];
  object.max.y = max_vals[1];
  object.max.z = max_vals[2];

  Eigen::Vector4f center;
  pcl::compute3DCentroid(*object_cloud_filtered, center);
  object.center.x = center[0];
  object.center.y = center[1];
  object.center.z = center[2];

  debug_cloud_pub_.publish(object_cloud_filtered);
  return true;

}