#include <ros/ros.h>
#include <operation_plushie/isComplete.h>
#include <operation_plushie/PositionJoints.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <string.h>

class PositionJoints 
{
private:
    ros::NodeHandle n;
    ros::ServiceServer position_joints_progress, position_joints_service;
    ros::Subscriber joint_position_sub;
    ros::Publisher armPose_pub;
    baxter_core_msgs::JointCommand orders;
    std::vector<double> current_positions;
    bool isComplete, isLeft;
public:
    PositionJoints();
    void begin_detection();    
    bool position_joints_callback(operation_plushie::PositionJoints::Request&, operation_plushie::PositionJoints::Response&);
    void update_joint_positions(sensor_msgs::JointState);
    bool isPositioned();
    bool hasArrived(operation_plushie::isComplete::Request&, operation_plushie::isComplete::Response&);
};

