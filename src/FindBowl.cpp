#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_reader/Ping.h>
#include <pcl_reader/BowlValues.h>

//cloud > cloud filter > cloud cluster > cylinder

typedef pcl::PointXYZ PointT;

enum Status {SEARCH, FINISHED};

class FindBowl
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer bowl_service, bowl_values_service;
    ros::Subscriber depth_sub;
    Status stage;
    int b_x, b_y, b_z;

public:
    FindBowl();
    void begin_detection();
    bool bowl_cb(pcl_reader::Ping::Request&, pcl_reader::Ping::Response&);
    bool bowl_values_cb(pcl_reader::BowlValues::Request&, pcl_reader::BowlValues::Response&);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
};

FindBowl::FindBowl()
{
    bowl_service = nh.advertiseService("find_bowl_service", &FindBowl::bowl_cb, this); 
    bowl_values_service = nh.advertiseService("bowl_values_service", &FindBowl::bowl_values_cb, this); 
    depth_sub = nh.subscribe ("/camera/depth_registered/points", 1, &FindBowl::cloud_cb, this);
}

void
FindBowl::begin_detection()
{
    ros::spin();
}

bool
FindBowl::bowl_cb(pcl_reader::Ping::Request &req, pcl_reader::Ping::Response &res)
{
    stage = SEARCH;

    b_x = -1337;
    b_y = -1337;
    b_z = -1337;

    return true;
}

void
FindBowl::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    if(stage == FINISHED)
        return;

    // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Read in the cloud data
    pcl::fromROSMsg(*input, *cloud);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1);
    pass.filter (*cloud_filtered);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    
    if (cloud_cylinder->points.empty ()) 
        return;

    b_x = cloud_cylinder->points[0].x;
    b_y = cloud_cylinder->points[0].y;
    b_z = cloud_cylinder->points[0].z;
    
    stage = FINISHED;

/*   Uncomment to enable filtered PC publishing
    pcl::fromROSMsg(*input, *cloud);

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud_cylinder, output);

    output.header.stamp = ros::Time::now();
    output.header.frame_id = "camera_rgb_optical_frame";
*/
}

bool FindBowl::bowl_values_cb(pcl_reader::BowlValues::Request &req, pcl_reader::BowlValues::Response &res) 
{
    res.x = b_x;
    res.y = b_y;
    res.z = b_z;
    return true;
}
