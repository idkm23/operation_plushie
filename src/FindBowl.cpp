#include "FindBowl.h"

FindBowl::FindBowl()
{
    bowl_service = nh.advertiseService("bowl_service", &FindBowl::bowl_cb, this); 
    bowl_values_service = nh.advertiseService("bowl_values_service", &FindBowl::bowl_values_cb, this); 
    output_pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1000);
    stage = FINISHED;
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
    depth_sub = nh.subscribe ("/camera/depth_registered/points", 1, &FindBowl::cloud_cb, this);
    
    b_x = -1337;
    b_y = -1337;

    return true;
}

void
FindBowl::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    if(stage == FINISHED)
    {
        depth_sub.shutdown();
        return;
    }

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
    seg.setDistanceThreshold (0.12);
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
    
    if (cloud_cylinder->points.size() < 800) 
        return;
    ROS_INFO("size: %ld", cloud_cylinder->points.size());
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

}

PointT 
FindBowl::calcCentroid(pcl::PointCloud<PointT>::Ptr cloud) {
    size_t size = cloud->points.size();
    double x = 0, y = 0, z = 0;
    
    for(size_t i = 0; i < size; i++)    
    {
        x += cloud->points[i].x;
        y += cloud->points[i].y;
        z += cloud->points[i].z;
    }
    
    return PointT(x / size, y / size, z / size);
}

bool 
FindBowl::bowl_values_cb(operation_plushie::BowlValues::Request &req, operation_plushie::BowlValues::Response &res) 
{
    res.x = b_x;
    res.y = b_y;
    return true;
}
