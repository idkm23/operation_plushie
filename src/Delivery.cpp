#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "image_reader/Wave.h"

#include <cstdlib>
#include <string>

class Delivery {

private:
    ros::NodeHandle n;
    ros::Publisher armPose_pub;
    ros::Subscriber armState;
    ros::ServiceServer waveService;
    ros::Rate loop_rate;    

    double e0, e1, s0, s1, w0, w1, w2;
    bool isRight;

public:
    Delivery();
    void callback(sensor_msgs::JointState);
    bool wave(image_reader::Wave::Request &req, image_reader::Wave::Response &res);
    void beginDetection();
    static double getArmPos(double);

};

Delivery::Delivery() : loop_rate(10) {
    armState = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1000, &Delivery::callback, this);
    waveService = n.advertiseService("wave_server", &Delivery::wave, this);
}

void Delivery::beginDetection() {
    ros::spin();
}

void Delivery::callback(sensor_msgs::JointState msg)
{
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
}
 
bool Delivery::wave(image_reader::Wave::Request &req, image_reader::Wave::Response &res)
{
    double head = req.headPos;

    double positions[7];
    std::string names[7];
    
    isRight = head >= 0;
    
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

    //Move the arm into a waving position
    positions[0] = (isRight ? 1 : -1) * 3.028;
    positions[1] = (M_PI/2);
    positions[2] = 0;
    positions[3] = 0;
    positions[4] = -3.059;
    positions[5] = 0;
    positions[6] = (M_PI/2);
    
    baxter_core_msgs::JointCommand wavePose;
    
    for(int i = 0; i < 7; i++)
    {
        wavePose.names.push_back(names[i]);
        wavePose.command.push_back(positions[i]);
    }
    wavePose.mode = 1; //Set it in position mode
    //STATE 0: GET IN WAVE POSITION
    while( ros::ok() && (e0 <= (positions[0] - 0.1) || e0 >= (positions[0] + 0.1) || e1 <= ((M_PI/2) - 0.1) || e1 >= ((M_PI/2) + 0.1) || s0 <= -0.1 || s0 >= 0.1 || s1 <= -0.1 || s1 >= 0.1 || w0 >= -3.045 || w1 <= -0.1 || w1 >= 0.1 || w2 <= ((M_PI/2) - 0.1) || w2 >= ((M_PI/2) + 0.1)) )
    {
        armPose_pub.publish(wavePose);
        loop_rate.sleep();
        ros::spinOnce();
    }

    baxter_core_msgs::JointCommand moveArm;
    
    //Set up the arm command
    moveArm.mode = 1;
    moveArm.names.push_back((isRight ? "right_s0" : "left_s0"));

    double armCommand = getArmPos(head);
    moveArm.command.push_back(armCommand);
   
    //STATE 1: MOVE s0 
    while(ros::ok() && (s0 <= armCommand - 0.1 || s0 >= armCommand + 0.1 ))
    {
        armPose_pub.publish(moveArm);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    baxter_core_msgs::JointCommand waveMove1, waveMove2, waveMove3;
    
    //Move the hand in positive-radian direction
    waveMove1.mode = 1; //Set it in position mode
    waveMove1.names.push_back(names[5]);
    waveMove1.command.push_back(1);
    
    //Move the hand in negative-radian direction
    waveMove2.mode = 1; //Set it in position mode
    waveMove2.names.push_back(names[5]);
    waveMove2.command.push_back(-1);
    
    //Move the hand straight up 
    waveMove3.mode = 1; //Set it in position mode
    waveMove3.names.push_back(names[5]);
    waveMove3.command.push_back(0);

    //STAGE 2: WAVE HAND
    int count = 0;
    while(ros::ok && count < 4) 
    {
        if((count % 2) == 0)
        {
            if(w1 <= .9)
            {
                armPose_pub.publish(waveMove1);
            }
            else
            {
                count++;
            }   
        }
        else
        {
            if(w1 >= -.9)
            {
                armPose_pub.publish(waveMove2);
            }
            else
            {
                count++;
            }
        }
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    
    //STAGE 3: RETURN HAND TO DEFAULT
    while(ros::ok && w1 <= -0.1 || w1 >= 0.1)
    {
        armPose_pub.publish(waveMove3);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    baxter_core_msgs::JointCommand wavePose2;
    wavePose2.mode = 1;
    wavePose2.names = wavePose.names;
    
    for(int i = 0; i < wavePose2.names.size(); i++)
        wavePose2.command.push_back(0);
    
    wavePose2.command[1] = M_PI/2;    

    //STAGE 4: RETURN ARM TO DEFAULT
    while(ros::ok && (e0 <= -0.1 || e0 >= 0.1 || e1 <= (M_PI/2) - 0.1 || e1 >= (M_PI)/2 + 0.1 || s0 <= -0.1 || s0 >= 0.1 || s1 <= -0.1 || s1 >= 0.1 || w0 <= -0.1 || w0 >= 0.1 || w1 <= -0.1 || w1 >= 0.1 || w2 <= -0.1 || w2 >= 0.1))
    {
        armPose_pub.publish(wavePose2);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return true;
}

double Delivery::getArmPos(double headPos)
{
    return headPos + (headPos >= 0 ? -0.8 : 0.8);
}






