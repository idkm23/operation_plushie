/* This header file provides function headers for RepositionHand.cpp. */

#include <ros/ros.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include "operation_plushie/RepositionHand.h"
#include "operation_plushie/isComplete.h"
#include <baxter_core_msgs/EndEffectorCommand.h>
#include "sensor_msgs/JointState.h"
#include <cmath>
#include <cstdlib>
#include <pthread.h>

class RepositionHand
{
private:
    ros::NodeHandle n;
    ros::ServiceServer reposition_hand_service, reposition_progress_service;
    ros::ServiceClient ik_solver;
    ros::Publisher joint_pub[2];
    ros::Subscriber endstate_sub[2], torque_sub;
    baxter_core_msgs::EndpointState eps;
    double s1_torque, s1_torque_original;
    bool isLeft, isMoving, isStuck, needsConsistency;

    //position to go to
    geometry_msgs::PoseStamped ps; 
    baxter_core_msgs::JointCommand msg;

    //consistent torque
    double consistent_torque, c_x, c_y, c_z, c_orix, c_oriy, c_oriz, c_oriw;
    int consistent_torque_count, c_pose_count;

public:
    RepositionHand();
    bool callback(operation_plushie::RepositionHand::Request&, operation_plushie::RepositionHand::Response&);
    void updateLeftEndpoint(baxter_core_msgs::EndpointState);
    void updateRightEndpoint(baxter_core_msgs::EndpointState);
    void updateEndpoint(baxter_core_msgs::EndpointState);
    bool isPositioned(baxter_core_msgs::EndpointState);
    void updateEffort(sensor_msgs::JointState js);
    bool progressCallback(operation_plushie::isComplete::Request&, operation_plushie::isComplete::Response&);
   
    static const int RIGHT, LEFT;
};

