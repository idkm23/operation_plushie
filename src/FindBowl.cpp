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

#include <operation_plushie/Ping.h>
#include <operation_plushie/BowlValues.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

//cloud > cloud filter > cloud cluster > cylinder

typedef pcl::PointXYZ PointT;

enum Status {SEARCH, FINISHED};

class FindBowl
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer bowl_service, bowl_values_service;
    ros::Publisher output_pub;
    ros::Subscriber depth_sub;
    tf::TransformListener listener;
    Status stage;
    double b_x, b_y;

public:
    FindBowl();
    void begin_detection();
    bool bowl_cb(operation_plushie::Ping::Request&, operation_plushie::Ping::Response&);
    bool bowl_values_cb(operation_plushie::BowlValues::Request&, operation_plushie::BowlValues::Response&);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
   
    static PointT calcCentroid(pcl::PointCloud<PointT>::Ptr);
};

FindBowl::FindBowl()
{
    bowl_service = nh.advertiseService("bowl_service", &FindBowl::bowl_cb, this); 
    bowl_values_service = nh.advertiseService("bowl_values_service", &FindBowl::bowl_values_cb, this); 
    depth_sub = nh.subscribe ("/camera/depth_registered/points", 1, &FindBowl::cloud_cb, this);
    output_pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1000);
}

void
FindBowl::begin_detection()
{
    ros::spin();
}

bool
FindBowl::bowl_cb(operation_plushie::Ping::Request &req, operation_plushie::Ping::Response &res)
{
    stage = SEARCH;

    b_x = -1337;
    b_y = -1337;

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

    PointT cylinder_centroid = calcCentroid(cloud_cylinder);

    geometry_msgs::PointStamped pt, pt_transformed;

    pt.header.stamp = ros::Time(0);
    pt.header.frame_id = "camera_rgb_optical_frame";

    pt.point.x = cylinder_centroid.x;
    pt.point.y = cylinder_centroid.y;
    pt.point.z = cylinder_centroid.z; 

    listener.transformPoint("base", pt, pt_transformed);

    b_x = pt_transformed.point.x;
    b_y = pt_transformed.point.y;
   
    ROS_INFO("Finished x: %f, y: %f", b_x, b_y);
 
    stage = FINISHED;
/*
  // Uncomment to enable filtered PC publishing
    pcl::PointCloud<PointT>::Ptr centroid_line(new pcl::PointCloud<PointT>());
    
    centroid_line->width = 15;
    centroid_line->height = 1;
    centroid_line->points.resize(centroid_line->width * centroid_line->height);
        
    for(int i = 0; i < centroid_line->points.size(); i++)
    {
        centroid_line->points[i].x = b_x;
        centroid_line->points[i].y = b_y;
        centroid_line->points[i].z = pt_transformed.point.z + .01 * i;
    }
    
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*centroid_line, output);

    output.header.stamp = ros::Time::now();
    output.header.frame_id = "base";

    while(ros::ok) {
        output_pub.publish(output);
        ros::spinOnce();
    }*/
}

PointT FindBowl::calcCentroid(pcl::PointCloud<PointT>::Ptr cloud) {
    size_t size = cloud->points.size();
    double x = 0, y = 0, z = 0;
    
    //for(std::vector<PointT>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); ++it)
    for(size_t i = 0; i < size; i++)    
    {
        x += cloud->points[i].x;
        y += cloud->points[i].y;
        z += cloud->points[i].z;
    }
    
    return PointT(x / size, y / size, z / size);
}

bool FindBowl::bowl_values_cb(operation_plushie::BowlValues::Request &req, operation_plushie::BowlValues::Response &res) 
{
    res.x = b_x;
    res.y = b_y;
    return true;
}
