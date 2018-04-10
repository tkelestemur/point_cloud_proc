#ifndef POINT_CLOUD_PROC_H
#define POINT_CLOUD_PROC_H

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
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
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
#include <pcl/features/moment_of_inertia_estimation.h>

// Other
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class PointCloudProc{
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::Normal PointNT;
    typedef pcl::PointCloud<PointT> CloudT;
    typedef pcl::PointCloud<PointNT> CloudNT;

private:
    pcl::PassThrough<PointT> pass_;
    pcl::VoxelGrid<PointT> vg_;
    pcl::SACSegmentation<PointT> seg_;
    pcl::ExtractIndices<PointT> extract_;
    pcl::ConvexHull<PointT> chull_;
    pcl::ExtractPolygonalPrismData<PointT> prism_;
    pcl::EuclideanClusterExtraction<PointT> ec_;

    int k_search_;
    bool debug_, use_voxel;
    float leaf_size_, cluster_tol_, normal_radius_;
    std::vector<float> pass_limits_, prism_limits_;
    std::string point_cloud_topic_, fixed_frame_;
    CloudT::ConstPtr cloud_raw_;
    CloudT::Ptr cloud_transformed_, cloud_filtered_, cloud_hull_, cloud_tabletop_;
    sensor_msgs::PointCloud2 tabletop_cloud_;
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
    point_cloud_proc::Objects tabletop_objects_;

public:
    PointCloudProc(ros::NodeHandle n);
    void pointCloudCb(const CloudT::ConstPtr &msg);
    bool transformPointCloud();
    bool filterPointCloud(bool use_voxel);
    bool segmentSinglePlane(bool create_srv_res);
    void segmentMultiplePlane(bool create_srv_res);
    bool extractTabletop();
    bool clusterObjects();
};

#endif //POINT_CLOUD_PROC_H
