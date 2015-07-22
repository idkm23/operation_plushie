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

PositionJoints::PositionJoints()
{
    position_joints_service = n.advertiseService("position_joints_service", &PositionJoints::position_joints_callback, this);
    position_joints_progress = n.advertiseService("position_joints_progress", &PositionJoints::hasArrived, this);
    joint_position_sub = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1000, &PositionJoints::update_joint_positions, this);
    isComplete = true;
}

void PositionJoints::begin_detection()
{
    ros::spin();
}

//get moving boys!
bool PositionJoints::position_joints_callback(operation_plushie::PositionJoints::Request &req, operation_plushie::PositionJoints::Response &res)
{
    isComplete = false;
    isLeft = (req.names[0].find("left") != std::string::npos);
    armPose_pub = n.advertise<baxter_core_msgs::JointCommand>(std::string("/robot/limb/") + (isLeft?"left":"right") + "/joint_command", 1000);
    
    orders.mode = 1;
    orders.names = req.names;
    orders.command = req.command;
    return true;
}

//figures out if we are there yet
void PositionJoints::update_joint_positions(sensor_msgs::JointState msg)
{
    if(isComplete)
        return;
    
    current_positions.clear();
    for(size_t i = 0; i < msg.position.size(); i++)
    {
        if(msg.name[i].find((isLeft?"left":"right")) != std::string::npos)
            current_positions.push_back(msg.position[i]);
    }    

    if(isPositioned()) 
    {
        isComplete = true;
    }
    else
    {
        armPose_pub.publish(orders);
    }
}

bool PositionJoints::isPositioned()
{
    for(size_t i = 0; i < orders.command.size(); i++)
    {
        if(fabs(orders.command[i] - current_positions[i]) > .03)
            return false;
    }
    return true;
}

bool PositionJoints::hasArrived(operation_plushie::isComplete::Request &req, operation_plushie::isComplete::Response &res) 
{
    res.isComplete = isComplete;    
    return true;
}
