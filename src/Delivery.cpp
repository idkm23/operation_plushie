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
    bool isRight, isPressed, origStored;

public:
    Delivery();
    void callback(sensor_msgs::JointState);
    bool deliver(operation_plushie::Deliver::Request &req, operation_plushie::Deliver::Response &res);
    void beginDetection();
    bool isCorrectPosition(baxter_core_msgs::JointCommand);
    void updateButtonState(baxter_core_msgs::DigitalIOState);
    bool isComplete(operation_plushie::isComplete::Request&, operation_plushie::isComplete::Response&);

    void selectState();
    //Stages
    void stretch();
    void release();
    void returning();
    
    static double getArmPos(double);

};

Delivery::Delivery() : loop_rate(10) {
    armState = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1000, &Delivery::callback, this);
    deliveryService = n.advertiseService("delivery_service", &Delivery::deliver, this);
    isComplete_service = n.advertiseService("delivery_isComplete_service", &Delivery::isComplete, this);
    state = FINISHED;
}

void Delivery::beginDetection() {
    ros::spin();
}

void Delivery::callback(sensor_msgs::JointState msg)
{
    if(state == FINISHED)
        return;

    int left_e0Index = -999;
    for(int i = 0; i < msg.name.size(); i++)
    {
        if(strcmp(msg.name[i].c_str(), "left_e0") == 0)
        {
            left_e0Index = i;
            break;
        }
    }

    if(left_e0Index == -999)
    {
        ROS_ERROR("left_e0 not found.\n");
        exit(1);
    }

    int index_space = (isRight ? 7 : 0);
    e0 = msg.position[left_e0Index + index_space];
    e1 = msg.position[left_e0Index + index_space + 1];
    s0 = msg.position[left_e0Index + index_space + 2];
    s1 = msg.position[left_e0Index + index_space + 3];
    w0 = msg.position[left_e0Index + index_space + 4];
    w1 = msg.position[left_e0Index + index_space + 5];
    w2 = msg.position[left_e0Index + index_space + 6];

    if(!origStored)
    {
        origPose.command.push_back(e0);
        origPose.command.push_back(e1);
        origPose.command.push_back(s0);
        origPose.command.push_back(s1);
        origPose.command.push_back(w0);
        origPose.command.push_back(w1);
        origPose.command.push_back(w2);
    }

    selectState();
}

bool Delivery::deliver(operation_plushie::Deliver::Request &req, operation_plushie::Deliver::Response &res)
{
    //isRight = req.headPos >= 0;
    isRight = false;
    
    gripper_pub = n.advertise<baxter_core_msgs::EndEffectorCommand>(
        std::string("/robot/end_effector/") + (isRight?"right":"left") + "_gripper/command", 1000);

    button_sub = n.subscribe<baxter_core_msgs::DigitalIOState>(
        std::string("/robot/digital_io/") + (isRight ? "right" : "left") + "_itb_button0/state", 10, &Delivery::updateButtonState, this);

    std::string names[7];   
    
    stretchPose.names.clear();
    origPose.names.clear();
    stretchPose.command.clear();
    origPose.command.clear();
 
    if(isRight)
    {
        armPose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);

        names[0] = "right_e0";
        names[1] = "right_e1";
        names[2] = "right_s0";
        names[3] = "right_s1";
        names[4] = "right_w0";
        names[5] = "right_w1";
        names[6] = "right_w2";
    }
    else 
    {
        armPose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);

        names[0] = "left_e0";
        names[1] = "left_e1";
        names[2] = "left_s0";
        names[3] = "left_s1";
        names[4] = "left_w0";
        names[5] = "left_w1";
        names[6] = "left_w2";
    }    
    
    for(int i = 0; i < 7; i++)
    {
        stretchPose.names.push_back(names[i]);
        origPose.names.push_back(names[i]);
    }    

    //Move the arm into a outstretched position
    stretchPose.command.push_back(-3.028);
    stretchPose.command.push_back(-0.25);
    stretchPose.command.push_back(getArmPos(req.headPos));
    stretchPose.command.push_back(0.25);
    stretchPose.command.push_back(3.05);
    stretchPose.command.push_back(0.1);
    stretchPose.command.push_back(0);

    stretchPose.mode = 1; //Set it in position mode
    origPose.mode = 1;
   
    origStored = false;
    isPressed = false;
    state = STRETCHING;

    return true;
}

double Delivery::getArmPos(double headPos)
{
    if(headPos < 0)
        return -0.8;
    
    return headPos + (headPos >= 0 ? -0.8 : 0.8);
}

void Delivery::selectState()
{    

    switch(state)
    {
    case STRETCHING:
        stretch();
        break;
    case RELEASING:
        release();
        break;
    case RETURNING:
        returning();
        break;
    }
}

void Delivery::stretch()
{
    if(isCorrectPosition(stretchPose))
        state = RELEASING;
    else
        armPose_pub.publish(stretchPose);
}

void Delivery::release()
{
    if(isPressed)
    {
        baxter_core_msgs::EndEffectorCommand hand_command;
        hand_command.id = 65538;
        // hand_command.command = "calibrate";
        // gripper_pub.publish(hand_command);
        hand_command.command = "release";
        gripper_pub.publish(hand_command);

        usleep(1000000);

        state = RETURNING;
    }
}

void Delivery::returning()
{
    if(isCorrectPosition(origPose))
        state = FINISHED;
    else
        armPose_pub.publish(origPose);
}

bool Delivery::isCorrectPosition(baxter_core_msgs::JointCommand msg)
{
    bool isPoseCorrect;
    if(e0 <= msg.command[0] - 0.1 || e0 >= msg.command[0] + 0.1 || e1 <= msg.command[1] - 0.1 || e1 >= msg.command[1] + 0.1 || s0 <= msg.command[2] - 0.1 
        || s0 >= msg.command[2] + 0.1 || s1 <= msg.command[3] - 0.1 || s1 >= msg.command[3] + 0.1 || w0 <= msg.command[4] - 0.1 || w0 >= msg.command[4] + 0.1 
        || w1 <= msg.command[5] - 0.1 || w1 >= msg.command[5] + 0.1 || w2 <= msg.command[6] - 0.1 || w2 >= msg.command[6] + 0.1)
    {    
        isPoseCorrect = false;
    }
    else
    {
        isPoseCorrect = true;
    }

    return isPoseCorrect;
}

void Delivery::updateButtonState(baxter_core_msgs::DigitalIOState ok)
{
    if(ok.state)
        isPressed = true;
}

bool Delivery::isComplete(operation_plushie::isComplete::Request &req, operation_plushie::isComplete::Response &res) 
{
    res.isComplete = (state == FINISHED);
    return true;
}

