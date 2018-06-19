#ifndef POINT_CLOUD_PROC_H
#define POINT_CLOUD_PROC_H

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl_msgs/PolygonMesh.h>
#include <point_cloud_proc/Mesh.h>
#include <point_cloud_proc/Planes.h>
#include <point_cloud_proc/Object.h>
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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
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
#include <pcl/surface/gp3.h>
//#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

// Other
#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

enum AXIS{
    XAXIS,
    YAXIS,
    ZAXIS
};

class PointCloudProc{
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::Normal PointNT;
    typedef pcl::PointCloud<PointT> CloudT;
    typedef pcl::PointCloud<PointNT> CloudNT;


public:
    PointCloudProc(ros::NodeHandle n, bool debug=false);
    void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &msg);

    bool transformPointCloud();
    bool filterPointCloud(int pass=0);
    bool removeOutliers(CloudT::Ptr in, CloudT::Ptr out);
    bool segmentSinglePlane(point_cloud_proc::Plane& plane, char axis='z', int pass=0);
    bool segmentMultiplePlane(std::vector<point_cloud_proc::Plane>& planes, int pass=0);
    bool clusterObjects(std::vector<point_cloud_proc::Object>& objects);
    bool extractTabletop();
    bool get3DPoint(int col, int row, geometry_msgs::PointStamped& point);
    bool getObjectFromBBox(int *bbox, point_cloud_proc::Object& object);
    bool trianglePointCloud(sensor_msgs::PointCloud2& cloud, pcl_msgs::PolygonMesh& mesh);
    void getRemainingCloud(sensor_msgs::PointCloud2& cloud);
    void getFilteredCloud(sensor_msgs::PointCloud2& cloud);
    sensor_msgs::PointCloud2::Ptr getTabletopCloud();
    CloudT::Ptr getFilteredCloud();
    pcl::PointIndices::Ptr getTabletopIndicies();


private:
    pcl::PassThrough<PointT> pass_;
    pcl::VoxelGrid<PointT> vg_;
    pcl::SACSegmentation<PointT> seg_;
    pcl::ExtractIndices<PointT> extract_;
    pcl::ConvexHull<PointT> chull_;
    pcl::ExtractPolygonalPrismData<PointT> prism_;
    pcl::EuclideanClusterExtraction<PointT> ec_;
    pcl::RadiusOutlierRemoval<PointT> outrem_;
    pcl::ProjectInliers<PointT> plane_proj_;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3_;

    bool debug_;
    int k_search_, min_plane_size_, max_iter_, min_cluster_size_, max_cluster_size_, min_neighbors_;
    float cluster_tol_, leaf_size_, eps_angle_, single_dist_thresh_, multi_dist_thresh_, radius_search_;

    std::vector<float> pass_limits_, prism_limits_, pass_limits_shelf_, pass_limits_table_;
    std::string point_cloud_topic_, fixed_frame_;

    CloudT::Ptr cloud_transformed_, cloud_filtered_, cloud_hull_, cloud_tabletop_;
    pcl::PointIndices::Ptr tabletop_indicies_;
    sensor_msgs::PointCloud2 cloud_raw_ros_;

    boost::mutex pc_mutex_;

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher plane_cloud_pub_, tabletop_pub_, debug_cloud_pub_;

    int PASS_SHELF = 1;
    int PASS_TABLE = 2;
    int PASS = 0;
};

#endif //POINT_CLOUD_PROC_H
