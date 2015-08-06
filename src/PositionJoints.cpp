/* This will take a vector of positions for joints and move each joint until they are all at their desired position. */

#include "PositionJoints.h"

PositionJoints::PositionJoints()
{
    position_joints_service = n.advertiseService("position_joints_service", &PositionJoints::position_joints_callback, this);
    position_joints_progress = n.advertiseService("position_joints_progress", &PositionJoints::hasArrived, this);
    joint_position_sub = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1000, &PositionJoints::update_joint_positions, this);
    isComplete = true;
}

/* Looks at the request to see which joints should move where. */
bool 
PositionJoints::position_joints_callback(operation_plushie::PositionJoints::Request &req, operation_plushie::PositionJoints::Response &res)
{
    isComplete = false;
    isLeft = (req.names[0].find("left") != std::string::npos);
    armPose_pub = n.advertise<baxter_core_msgs::JointCommand>(std::string("/robot/limb/") + (isLeft?"left":"right") + "/joint_command", 1000);
    
    orders.mode = 1;
    orders.names = req.names;
    orders.command = req.command;
    return true;
}

/* Updates the arm's position and decides if it has reached its destination. */
void 
PositionJoints::update_joint_positions(sensor_msgs::JointState msg)
{
    if(isComplete)
        return;
    
    current_positions.clear();
    for(size_t i = 0; i < msg.position.size(); i++)
    {
        if(msg.name[i].find((isLeft?"left":"right")) != std::string::npos)
            current_positions.push_back(msg.position[i]);
    }    

    // Keep publishing commands until we reach the correct position.
    if(isPositioned()) 
    {
        isComplete = true;
    }
    else
    {
        armPose_pub.publish(orders);
    }
}

/* Returns false if the current position is not within range of where the arm was told to go, Returns true otherwise. */
bool 
PositionJoints::isPositioned()
{
    for(size_t i = 0; i < orders.command.size(); i++)
    {
        if(fabs(orders.command[i] - current_positions[i]) > .03)
            return false;
    }
    return true;
}

/* Sets the response value to isComplete and returns true to show that it contacted the service. isComplete checks if the service worked correctly. */
bool 
PositionJoints::hasArrived(operation_plushie::isComplete::Request &req, operation_plushie::isComplete::Response &res) 
{
    res.isComplete = isComplete;    
    return true;
}
