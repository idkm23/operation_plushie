#include <ros/ros.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include "operation_plushie/RepositionHand.h"
#include <baxter_core_msgs/EndEffectorCommand.h>
#include "sensor_msgs/JointState.h"
#include <cmath>
#include <cstdlib>
#include <pthread.h>

class RepositionHand
{
private:
    ros::NodeHandle n;
    ros::ServiceServer reposition_hand_service;
    ros::ServiceClient ik_solver;
    ros::Publisher joint_pub;
    ros::Subscriber endstate_sub, torque_sub;
    baxter_core_msgs::EndpointState eps;
    double s1_torque_avg, s1_torque_total;
    int s1_torque_count;
    bool isLeft;

public:
    RepositionHand();
    void begin_detection();
    bool callback(operation_plushie::RepositionHand::Request&, operation_plushie::RepositionHand::Response&);
    void updateEndpoint(baxter_core_msgs::EndpointState);
    bool isPositioned(double, double, double, double, double, double, double);
    void updateEffort(sensor_msgs::JointState js);
};

void RepositionHand::begin_detection()
{
    ros::spin();
}

RepositionHand::RepositionHand()
{
    reposition_hand_service = n.advertiseService("reposition_hand_service", &RepositionHand::callback, this);
}

bool RepositionHand::callback(operation_plushie::RepositionHand::Request &req, operation_plushie::RepositionHand::Response &res)
{
    isLeft = req.isLeft;

    ik_solver = n.serviceClient<baxter_core_msgs::SolvePositionIK>(
        std::string("/ExternalTools/") + (isLeft?"left":"right") + "/PositionKinematicsNode/IKService");
    
    joint_pub = n.advertise<baxter_core_msgs::JointCommand>(
        std::string("/robot/limb/") + (isLeft?"left":"right") + "/joint_command", 200);

    endstate_sub = n.subscribe<baxter_core_msgs::EndpointState>(
        std::string("/robot/limb/") + (isLeft ? "left" : "right") + "/endpoint_state", 1000, &RepositionHand::updateEndpoint, this);

    torque_sub = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1000, &RepositionHand::updateEffort, this);
    baxter_core_msgs::SolvePositionIK srv;

    const double ROLL = 0, PITCH = 3.14, YAW = 0;

    double mathc1 = cos(PITCH),
        maths1 = sin(PITCH),
        mathc2 = cos(YAW),
        maths2 = sin(YAW),
        mathc3 = cos(ROLL),
        maths3 = sin(ROLL),
        oriw, oriw4, orix, oriy, oriz;
        
    oriw = sqrt(1.0 + mathc1 * mathc2 + mathc1 * mathc3 
        - maths1 * maths2 * maths3 + mathc2 * mathc3) / 2.0;
    oriw4 = (4.0 * oriw);
    orix = (mathc2 * maths3 + mathc1 * maths3 + maths1 * maths2 * mathc3) / oriw4;
    oriy = (maths1 * mathc2 + maths1 * mathc3 + mathc1 * maths2 * maths3) / oriw4;
    oriz = (-maths1 * maths3 + mathc1 * maths2 * mathc3 + maths2) / oriw4;

    geometry_msgs::PoseStamped ps;

    ps.header.frame_id = "base"; 
    ps.header.stamp = ros::Time::now();

    ps.pose.position.x = req.x;
    ps.pose.position.y = req.y;
    ps.pose.position.z = req.z;

    ps.pose.orientation.x = orix;
    ps.pose.orientation.y = oriy;
    ps.pose.orientation.z = oriz;
    ps.pose.orientation.w = oriw;

    ROS_INFO("x:%f y:%f z:%f orix:%f oriy:%f oriz:%f oriw:%f", req.x, req.y, req.z, orix, oriy, oriz, oriw);
    
    srv.request.pose_stamp.push_back(ps);

    if(!ik_solver.call(srv))
    {
        ROS_ERROR("Failed to call service IKSolver");
        return false;
    }

    if(!srv.response.isValid[0])
    {
        ROS_ERROR("Not a valid position, :(");
        return false;
    }

    baxter_core_msgs::JointCommand msg;
    
    msg.mode = 1;
    
    for(int i = 0; i < srv.response.joints[0].name.size(); i++)
    {
        msg.names.push_back(srv.response.joints[0].name[i]);
        msg.command.push_back(srv.response.joints[0].position[i]);
    }

    double s1_torque_original = s1_torque_avg;
    ROS_INFO("orig: %f", s1_torque_original); 
    
    ros::Rate loop_rate(10);
//    while(ros::ok() && !isPositioned(req.x, req.y, req.z, orix, oriy, oriz, oriw))
//    {
        joint_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        
        if((s1_torque_avg - s1_torque_original) > 3 && req.z < eps.pose.position.z)
        {
            ROS_INFO("final torque: %f", s1_torque_avg);
            res.isStuck = true;
            break;
        }
//    }
    
    return true;
}

void RepositionHand::updateEndpoint(baxter_core_msgs::EndpointState eps__)
{
    eps = eps__;
}

bool RepositionHand::isPositioned(double x, double y, double z, double orix, double oriy, double oriz, double oriw)
{
    const double P_WIG = .007f, O_WIG = .05f;
    
    return (fabs(eps.pose.position.x - x) < P_WIG && fabs(eps.pose.position.y - y) < P_WIG && fabs(eps.pose.position.z - z) < P_WIG 
        && fabs(eps.pose.orientation.x - orix) < O_WIG && fabs(eps.pose.orientation.y - oriy) < O_WIG && fabs(eps.pose.orientation.z - oriz) < O_WIG && fabs(eps.pose.orientation.w - oriw) < O_WIG);
}

void RepositionHand::updateEffort(sensor_msgs::JointState js)
{
    s1_torque_total += js.effort[(isLeft ? 5 : 12)];
    
    if(s1_torque_count++ >= 4)
    {
        s1_torque_count = 0;
        s1_torque_avg = s1_torque_total / 5.0;
        s1_torque_total = 0;
    }
}
