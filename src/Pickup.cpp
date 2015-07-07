#include "ros/ros.h"

#include "operation_plushie/Pickup.h"
#include "operation_plushie/RepositionHand.h"
#include "operation_plushie/RepositionProgress.h"

//inverse kinematics
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/EndpointState.h>

//baxter movement
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
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

class Pickup 
{
private:
    ros::NodeHandle n;
    ros::ServiceServer pickup_service;
    ros::ServiceClient reposition_hand_client, reposition_progress_client;
    ros::Publisher arm_pub, xdisplay_pub, gripper_pub;
    ros::Subscriber raw_image, endstate_sub, ir_sensor_sub;
    bool isCentered, isMoving, isLeft;
    double x, y, z;
    double ir_sensor;

    static const int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;

public:
    Pickup();
    void begin_detection();
    bool grabPlushie(operation_plushie::Pickup::Request &req, operation_plushie::Pickup::Response &res);
    void getHandImage(const sensor_msgs::ImageConstPtr&);
    void moveArm(int, int);
    void updateEndpoint(baxter_core_msgs::EndpointState);
    void updateIrSensor(sensor_msgs::Range);
    bool stepDown(double, double);
    bool sleepUntilDone();
};

const int Pickup::iLowH = 99, Pickup::iHighH = 122, Pickup::iLowS = 85, 
          Pickup::iHighS = 150, Pickup::iLowV = 130, Pickup::iHighV = 226;

Pickup::Pickup() 
{
    pickup_service = n.advertiseService("pickup_service", &Pickup::grabPlushie, this);
    reposition_hand_client = n.serviceClient<operation_plushie::RepositionHand>("reposition_hand_service");
    xdisplay_pub = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1000);
    reposition_progress_client = n.serviceClient<operation_plushie::RepositionProgress>("reposition_progress_service");
    x = 0.6;
    y = 0.5;
    z = 0.15;
}

void Pickup::begin_detection()
{
    isCentered = false;
    isMoving = false;
    ros::spin();
}

bool Pickup::grabPlushie(operation_plushie::Pickup::Request &req, operation_plushie::Pickup::Response &res)
{
    ROS_INFO("Entered pickup");
    isLeft = req.isLeft;
    
    arm_pub = n.advertise<baxter_core_msgs::JointCommand>(
        std::string("/robot/limb/")         + (isLeft?"left":"right") + "/joint_command", 1000);
    
    gripper_pub = n.advertise<baxter_core_msgs::EndEffectorCommand>(
        std::string("/robot/end_effector/") + (isLeft?"left":"right") + "_gripper/command", 1000);
    
    raw_image = n.subscribe<sensor_msgs::Image>(
        std::string("cameras/") + (isLeft?"left":"right") + "_hand_camera/image", 1, &Pickup::getHandImage, this);
   
    endstate_sub = n.subscribe<baxter_core_msgs::EndpointState>(
        std::string("/robot/limb/") + (isLeft ? "left" : "right") + "/endpoint_state", 10, &Pickup::updateEndpoint, this);

    ir_sensor_sub = n.subscribe<sensor_msgs::Range>(
        std::string("/robot/range/") + (isLeft ? "left" : "right") + "_hand_range/state", 10, &Pickup::updateIrSensor, this);
    
    ros::Rate loop_rate(10);
    while(1)
    {
        //Center above color 
        while(ros::ok() && !isCentered)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        
        ROS_INFO("Centered above color");
        double cent_x = x, cent_y = y;

        ROS_INFO("going down");
        
        
        //go down
        do {
            ros::spinOnce();
            loop_rate.sleep(); 
        } while(ros::ok() && !stepDown(cent_x, cent_y));// && ir_sensor > 0.1175f);

        baxter_core_msgs::EndEffectorCommand hand_command;
        hand_command.id = 65538;
        hand_command.command = "calibrate";
        gripper_pub.publish(hand_command);

        hand_command.command = "grip";
        gripper_pub.publish(hand_command);
        
        usleep(400000);

        ROS_INFO("Going Up");
              
        //go up
        operation_plushie::RepositionHand srv;

        srv.request.isLeft = isLeft;    
        srv.request.x = x;
        srv.request.y = y;
        srv.request.z = .15;
        z = .15;
        usleep(400000);

        if(!reposition_hand_client.call(srv))
        {
            ROS_ERROR("Failed to call reposition_hand_service");
            isMoving = false;
            return false;
        }
       
        sleepUntilDone();
 
        //open
        hand_command.command = "release";
        gripper_pub.publish(hand_command);
        usleep(400000);        
     
        isCentered = false; 
    }

    return true;
}

void Pickup::getHandImage(const sensor_msgs::ImageConstPtr& msg)
{
    if(isCentered)
        return;

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
        const int XTRANS = 60, YTRANS = -70;

        //calculate the position of the ball
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;        

        cv::cvtColor(imgThresholded, imgThresholded, cv::COLOR_GRAY2BGR);
        
        //Center of screen and center of proper color
        cv::ellipse(imgThresholded, cv::Point(imgThresholded.cols/2 + XTRANS, imgThresholded.rows/2 + YTRANS), cv::Size(10, 10), 360, 0, 360, cv::Scalar(255, 0, 0), 3, 8);
        cv::ellipse( imgThresholded, cv::Point(posX, posY), cv::Size(10, 10), 360, 0, 360, cv::Scalar(100, 100, 255), 5, 8);

        sensor_msgs::ImagePtr xdisplay_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgThresholded).toImageMsg();
        xdisplay_pub.publish(xdisplay_img);
        
        cv::imshow("Thresh", imgThresholded);
        
        //move arm towards center of proper color    
        moveArm(posX - imgThresholded.cols/2 - XTRANS, posY - imgThresholded.rows/2 - YTRANS);
    }
}

void Pickup::moveArm(int y_shift, int x_shift)
{
    if(isMoving)
        return;
    
    isMoving = true;    
    
    const int CENT_VAL = 10;   

    if(abs(x_shift) < CENT_VAL && abs(y_shift) < CENT_VAL)
    {
        ROS_INFO("CENTERED");
        isCentered = true;
        isMoving = false;
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
    
    if(!reposition_hand_client.call(srv))
    {
        ROS_ERROR("Failed to call reposition_hand_service");
        isMoving = false;
        return;
    } 

    sleepUntilDone();

    isMoving = false;
}

bool Pickup::stepDown(double __x, double __y)
{
    operation_plushie::RepositionHand srv;
    srv.request.x = __x;
    srv.request.y = __y;
    srv.request.z = z - 0.01f;
    srv.request.isLeft = isLeft;
    
    if(!reposition_hand_client.call(srv))
    {
        ROS_ERROR("Failed to call reposition_hand_service in stepDown");
        return false;
    }
    
    return sleepUntilDone(); 
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

bool Pickup::sleepUntilDone()
{
    ROS_INFO("Entered sleepUntilDone!\n");
    operation_plushie::RepositionProgress srv;
    while(!srv.response.isComplete)
    {
        reposition_progress_client.call(srv);
    }
    ROS_INFO("Slept until done!\n");
    
    return srv.response.isStuck;
}
