#include "ros/ros.h"

#include "operation_plushie/Pickup.h"
#include "operation_plushie/RepositionHand.h"
#include "operation_plushie/isComplete.h"
#include "operation_plushie/Ping.h"
#include "operation_plushie/BowlValues.h"

//inverse kinematics
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/EndpointState.h>

//baxter movement
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <sensor_msgs/JointState.h>

//Image processing
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <pthread.h>

//for IR sensor
#include <sensor_msgs/Range.h>

//for button
#include "baxter_core_msgs/DigitalIOState.h"

enum Stage {TOBOWL, INITIALIZING, CENTERING, LOWERING, RETURNING, FINISHED};

class Pickup 
{
private:
    ros::NodeHandle n;
    ros::ServiceServer pickup_service, isComplete_service;
    ros::ServiceClient reposition_hand_client, reposition_progress_client, bowl_client, bowl_values_client;
    ros::Publisher arm_pub, xdisplay_pub, gripper_pub;
    ros::Subscriber raw_image, endstate_sub, ir_sensor_sub, is_holding_sub, ok_button_sub;
    bool isCentered, isLeft, isHolding, isPressed, missedLast;
    double x, y, z;
    double ir_sensor;
    double lowering_x, lowering_y;
    Stage stage;
    int yaw_index, no_sign_of_plushies;

    static const int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    static const double yawDictionary[];

public:
    Pickup();
    void begin_detection();
    
    bool isComplete(operation_plushie::isComplete::Request&, operation_plushie::isComplete::Response&);

    //main service call function
    bool grabPlushie(operation_plushie::Pickup::Request&, operation_plushie::Pickup::Response&);
    
    void updateEndpoint(baxter_core_msgs::EndpointState);
    void updateIrSensor(sensor_msgs::Range);
    void updateEndEffectorState(baxter_core_msgs::EndEffectorState);
    void updateOKButtonState(baxter_core_msgs::DigitalIOState);

    void chooseStage(const sensor_msgs::ImageConstPtr&);
    
    //Choose stage calls these three main functions
    void getHandImage(const sensor_msgs::ImageConstPtr&);
    void lowerArm();
    void fetchNRaise();
    void moveAboveBowl();
    
    //worker functions the stages use
    void setupHand();
    void moveArm(int, int);
    bool stepDown(double, double);
    bool sleepUntilDone();
    void moveOutOfDepthCloud();
};

const double Pickup::yawDictionary[] = {0, 3.14 / 4, 3.14 / 2, 3 * 3.14 / 4};

const int Pickup::iLowH = 80, Pickup::iHighH = 179, Pickup::iLowS = 0, 
          Pickup::iHighS = 255, Pickup::iLowV = 240, Pickup::iHighV = 255;

Pickup::Pickup() 
{
    pickup_service = n.advertiseService("pickup_service", &Pickup::grabPlushie, this);
    isComplete_service = n.advertiseService("pickup_isComplete_service", &Pickup::isComplete, this);
    reposition_hand_client = n.serviceClient<operation_plushie::RepositionHand>("reposition_hand_service");
    xdisplay_pub = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1000);
    reposition_progress_client = n.serviceClient<operation_plushie::isComplete>("reposition_progress_service");
    bowl_client = n.serviceClient<operation_plushie::Ping>("bowl_service");
    bowl_values_client = n.serviceClient<operation_plushie::BowlValues>("bowl_values_service");
 
    x = 0.6;
    y = 0.5;
    z = 0.15;
}

void Pickup::begin_detection()
{
    stage = FINISHED;
    ros::spin();
}

bool Pickup::grabPlushie(operation_plushie::Pickup::Request &req, operation_plushie::Pickup::Response &res)
{
    isLeft = req.isLeft;
    
    arm_pub = n.advertise<baxter_core_msgs::JointCommand>(
        std::string("/robot/limb/") + (isLeft?"left":"right") + "/joint_command", 1000);
    
    gripper_pub = n.advertise<baxter_core_msgs::EndEffectorCommand>(
        std::string("/robot/end_effector/") + (isLeft?"left":"right") + "_gripper/command", 1000);
    
    raw_image = n.subscribe<sensor_msgs::Image>(
        std::string("/cameras/"/*"/republished/"*/) + (isLeft?"left":"right") + "_hand_camera/image", 1, &Pickup::chooseStage, this);
   
    endstate_sub = n.subscribe<baxter_core_msgs::EndpointState>(
        std::string("/robot/limb/") + (isLeft ? "left" : "right") + "/endpoint_state", 10, &Pickup::updateEndpoint, this);

    ir_sensor_sub = n.subscribe<sensor_msgs::Range>(
        std::string("/robot/range/") + (isLeft ? "left" : "right") + "_hand_range/state", 10, &Pickup::updateIrSensor, this);
    
    is_holding_sub = n.subscribe<baxter_core_msgs::EndEffectorState>(
        std::string("/robot/end_effector/") + (isLeft ? "left" : "right") + "_gripper/state", 10, &Pickup::updateEndEffectorState, this);

    ok_button_sub = n.subscribe<baxter_core_msgs::DigitalIOState>(
        std::string("/robot/digital_io/") + (isLeft ? "left" : "right") + "_itb_button0/state", 10, &Pickup::updateOKButtonState, this);
    
    yaw_index = -1;
    no_sign_of_plushies = 0;
    stage = (req.isFirst?TOBOWL:INITIALIZING);

    ROS_INFO("isFirst %d", req.isFirst);
        
    return true;
}   

void Pickup::chooseStage(const sensor_msgs::ImageConstPtr& msg)
{
    switch(stage)
    {
    case TOBOWL:
        moveAboveBowl();
        break;
    case INITIALIZING:
        setupHand();
        break;
    case CENTERING:
        getHandImage(msg);
        break;
    case LOWERING:
        lowerArm();
        break;
    case RETURNING:
        fetchNRaise(); 
        break;
    case FINISHED:
        return;    
    }
}

void Pickup::getHandImage(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr_cam;

    try 
    {   
        cv_ptr_cam = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }   
    catch (cv_bridge::Exception& e)
    {   
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }  
 
    cv::Mat imgHSV;

    cv::cvtColor(cv_ptr_cam->image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::Mat imgThresholded;

    //Threshold the image
    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

    //morphological opening (remove small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );


    cv::imshow("Original", cv_ptr_cam->image);    
    cv::waitKey(5);    

    cv::Moments oMoments = moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    if (dArea > 10000)
    {
        no_sign_of_plushies = 0;
        const int XTRANS = 60, YTRANS = -70;

        //calculate the position of the ball
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;        

        cv::cvtColor(imgThresholded, imgThresholded, cv::COLOR_GRAY2BGR);
        
        //Center of screen and center of proper color
        cv::ellipse(imgThresholded, cv::Point(imgThresholded.cols/2 + XTRANS, imgThresholded.rows/2 + YTRANS), cv::Size(10, 10), 360, 0, 360, cv::Scalar(255, 0, 0), 3, 8);
        cv::ellipse( imgThresholded, cv::Point(posX, posY), cv::Size(10, 10), 360, 0, 360, cv::Scalar(100, 100, 255), 5, 8);

       // sensor_msgs::ImagePtr xdisplay_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgThresholded).toImageMsg();
       // xdisplay_pub.publish(xdisplay_img);
        
        cv::imshow("Thresh", imgThresholded);
        
        //move arm towards center of proper color    
        moveArm(posX - imgThresholded.cols/2 - XTRANS, posY - imgThresholded.rows/2 - YTRANS);
    }
    else
    {
        if(++no_sign_of_plushies > 40)
        {
            moveAboveBowl();
        }
    }
}

void Pickup::moveAboveBowl() 
{
    ROS_INFO("Moving above bowl");
    moveOutOfDepthCloud();

    operation_plushie::BowlValues srv_values;
    operation_plushie::Ping srv_ping;
   
    do { 

        if(!bowl_client.call(srv_ping))
        {
            ROS_ERROR("Cannot contact bowl_service");           
        }
        
        int timeout = 0;
        do {

            if(!bowl_values_client.call(srv_values))
            {
                ROS_ERROR("Cannot contact bowl_values_service");
            } 
        
            timeout++;
        } while(srv_values.response.x == -1337 && timeout < 2500);

        if(timeout >= 2500) 
        {
            ROS_INFO("Bowl_values_service timed out");    
            return;
        }
        
        operation_plushie::RepositionHand srv;

        srv.request.x = srv_values.response.x; 
        srv.request.y = srv_values.response.y;
        
        srv.request.z = .15;
        srv.request.isLeft = isLeft;
        srv.request.frame = "base";
        
        ROS_INFO("Pickup x: %f, y: %f", x, y);

        if(!reposition_hand_client.call(srv))
        {
            ROS_ERROR("Failed to call reposition_hand_service");
            return;
        } 

    } while(sleepUntilDone());

    stage = INITIALIZING;
}

void Pickup::moveOutOfDepthCloud()
{
    operation_plushie::RepositionHand srv;
    srv.request.x = (isLeft?1:-1)*.22;
    srv.request.y = .74;
    srv.request.z = .24;
    srv.request.isLeft = isLeft;
    srv.request.frame = "base";
    
    if(!reposition_hand_client.call(srv))
    {
        ROS_ERROR("Failed to call reposition_hand_service");
        return;
    } 

    sleepUntilDone();
}

void Pickup::moveArm(int y_shift, int x_shift)
{
    const int CENT_VAL = 10;   

    if(abs(x_shift) < CENT_VAL && abs(y_shift) < CENT_VAL)
    {
        ROS_INFO("CENTERED");
        stage = LOWERING;
        lowering_x = x;
        lowering_y = y;
        
        if(missedLast)
        {
            if(++yaw_index > 3)
                yaw_index = 0;
        }
        else
        {
            yaw_index = 0;
        }

        return;
    }
   
    //If you have gotten this far, we are going to make a movement
    operation_plushie::RepositionHand srv;

    //if the x_shift is outside of the center rect
    if(abs(x_shift) > CENT_VAL)
        srv.request.x = (x + (x_shift < 0 ? .01 : -.01));
    else
        srv.request.x = x;

    //if the y_shift is outside of the center rect
    if(abs(y_shift) > CENT_VAL)
        srv.request.y = (y + (y_shift < 0 ? .01 : -.01));
    else
        srv.request.y = y;
    
    //keeps z the same
    srv.request.z = z;
    srv.request.isLeft = isLeft;
    srv.request.frame = "base";   
 
    if(!reposition_hand_client.call(srv))
    {
        ROS_ERROR("Failed to call reposition_hand_service");
        return;
    } 

    sleepUntilDone();
}

bool Pickup::stepDown(double __x, double __y)
{
    operation_plushie::RepositionHand srv;
    srv.request.x = __x;
    srv.request.y = __y;
    srv.request.z = z - 0.01f;
    srv.request.yaw = yawDictionary[yaw_index];
    srv.request.isLeft = isLeft;
    srv.request.frame = "base";   
    srv.request.needsConsistency = true;
 
    if(!reposition_hand_client.call(srv))
    {
        ROS_ERROR("Failed to call reposition_hand_service in stepDown");
        return false;
    }
    
    return sleepUntilDone(); 
}

bool Pickup::sleepUntilDone()
{
    operation_plushie::isComplete srv;
    while(!srv.response.isComplete)
    {
        reposition_progress_client.call(srv);
    }
    ROS_INFO("Slept until done!\n");
    
    return srv.response.isStuck;
}

void Pickup::lowerArm()
{
    if(stepDown(lowering_x, lowering_y))// && ir_sensor > 0.1175f);
    {
        stage = RETURNING;
    }
}

void Pickup::fetchNRaise()
{
    baxter_core_msgs::EndEffectorCommand hand_command;
    hand_command.id = 65538;
    hand_command.command = "grip";
    gripper_pub.publish(hand_command);
    
    usleep(400000);

    operation_plushie::RepositionHand srv;

    srv.request.isLeft = isLeft;    
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = .15;
    z = .15;
    srv.request.yaw = yawDictionary[yaw_index];
    srv.request.frame = "base";

    if(!reposition_hand_client.call(srv))
    {
        ROS_ERROR("Failed to call reposition_hand_service");
        return;
    }
   
    sleepUntilDone();

    usleep(400000);

    stage = INITIALIZING;
}

void Pickup::setupHand()
{
    if(missedLast = !isHolding)
    {
        baxter_core_msgs::EndEffectorCommand hand_command;
        hand_command.id = 65538;
        hand_command.command = "calibrate";
        gripper_pub.publish(hand_command);

        hand_command.command = "release";
        gripper_pub.publish(hand_command);
   
        stage = CENTERING;
    }
    else 
    {
        stage = FINISHED;
    }
}

void Pickup::updateEndpoint(baxter_core_msgs::EndpointState eps)
{
    x = eps.pose.position.x;
    y = eps.pose.position.y;
    z = eps.pose.position.z;
}

void Pickup::updateIrSensor(sensor_msgs::Range ir_sensor__)
{
    ir_sensor = ir_sensor__.range;
}

void Pickup::updateEndEffectorState(baxter_core_msgs::EndEffectorState ees)
{
    isHolding = ees.gripping;
}

void Pickup::updateOKButtonState(baxter_core_msgs::DigitalIOState ok)
{
    isPressed = ok.state;
}

bool Pickup::isComplete(operation_plushie::isComplete::Request &req, operation_plushie::isComplete::Response &res) 
{
    res.isComplete = (stage == FINISHED);
    return true;
}
