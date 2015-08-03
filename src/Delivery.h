#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/DigitalIOState.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include <baxter_core_msgs/EndEffectorState.h>

#include "operation_plushie/Deliver.h"
#include "operation_plushie/isComplete.h"

#include <cstdlib>
#include <string>
#include <pthread.h>

enum Stage {STRETCHING, RELEASING, RETURNING, FINISHED};

class Delivery {

private:
    ros::NodeHandle n;
    ros::Publisher armPose_pub, gripper_pub;
    ros::Subscriber armState, button_sub, is_holding_sub;
    ros::ServiceServer deliveryService, isComplete_service;
    ros::Rate loop_rate;        
        
    Stage state;
    baxter_core_msgs::JointCommand stretchPose, origPose;
    double e0, e1, s0, s1, w0, w1, w2; 
    bool isRight, isPressed, isHolding, origStored;

public:
    Delivery();
    void callback(sensor_msgs::JointState);
    bool deliver(operation_plushie::Deliver::Request &req, operation_plushie::Deliver::Response &res);
    void beginDetection();
    bool isCorrectPosition(baxter_core_msgs::JointCommand);
    void updateButtonState(baxter_core_msgs::DigitalIOState);
    void updateEndEffectorState(baxter_core_msgs::EndEffectorState);
        
    bool isComplete(operation_plushie::isComplete::Request&, operation_plushie::isComplete::Response&);

    void selectState();
    //Stages
    void stretch();
    void release();
    void returning();
        
    static double getArmPos(double);

};
