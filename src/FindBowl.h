/* This header file provides function headers for FindBowl.cpp. */

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
    bool bowl_cb(operation_plushie::Ping::Request&, operation_plushie::Ping::Response&);
    bool bowl_values_cb(operation_plushie::BowlValues::Request&, operation_plushie::BowlValues::Response&);
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

    static PointT calcCentroid(pcl::PointCloud<PointT>::Ptr);
};

