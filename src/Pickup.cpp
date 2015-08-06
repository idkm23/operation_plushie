#include "Pickup.h"

//the various rotations the gripper will use to grab at different angles
const double Pickup::yawDictionary[] = {0, 3.14 / 4, 3.14 / 2, 3 * 3.14 / 4};

//HSV values to find a plush robot
const int Pickup::iLowH = 80, Pickup::iHighH = 179, Pickup::iLowS = 0, 
          Pickup::iHighS = 255, Pickup::iLowV = 240, Pickup::iHighV = 255;

/* Initializes ros objects and happy/sad images */
Pickup::Pickup() 
{
    pickup_service = n.advertiseService("pickup_service", &Pickup::grabPlushie, this);
    isComplete_service = n.advertiseService("pickup_isComplete_service", &Pickup::isComplete, this);
    reposition_hand_client = n.serviceClient<operation_plushie::RepositionHand>("reposition_hand_service");
    xdisplay_pub = n.advertise<sensor_msgs::Image>("/robot/xdisplay", 1000);
    reposition_progress_client = n.serviceClient<operation_plushie::isComplete>("reposition_progress_service");
    bowl_client = n.serviceClient<operation_plushie::Ping>("bowl_service");
    bowl_values_client = n.serviceClient<operation_plushie::BowlValues>("bowl_values_service");
    position_joints_progress = n.serviceClient<operation_plushie::isComplete>("position_joints_progress");
    position_joints_client = n.serviceClient<operation_plushie::PositionJoints>("position_joints_service");

    cv::Mat happy_mat = cv::imread("../../../src/operation_plushie/res/happy.jpg", CV_LOAD_IMAGE_COLOR),
            sad_mat = cv::imread("../../../src/operation_plushie/res/sad.jpg", CV_LOAD_IMAGE_COLOR);
 
    happy_face = cv_bridge::CvImage(std_msgs::Header(), "bgr8", happy_mat).toImageMsg();
    sad_face = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sad_mat).toImageMsg();
}

void 
Pickup::begin_detection()
{
    stage = FINISHED;
    ros::spin();
}

/* Main service call-back, starts the pickup process */
bool 
Pickup::grabPlushie(operation_plushie::Pickup::Request &req, operation_plushie::Pickup::Response &res)
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

    is_holding_sub = n.subscribe<baxter_core_msgs::EndEffectorState>(
        std::string("/robot/end_effector/") + (isLeft ? "left" : "right") + "_gripper/state", 10, &Pickup::updateEndEffectorState, this);

    ok_button_sub = n.subscribe<baxter_core_msgs::DigitalIOState>(
        std::string("/robot/digital_io/") + (isLeft ? "left" : "right") + "_itb_button0/state", 10, &Pickup::updateOKButtonState, this);
    
    yaw_index = -1;
    no_sign_of_plushies = 0;
    stage = TOBOWL;
        
    return true;
}   

/* Dictates what function is called depending on which stage */
void 
Pickup::chooseStage(const sensor_msgs::ImageConstPtr& msg)
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

/* Using the hand camera, image thresholding is applied for visual servoing */
void 
Pickup::getHandImage(const sensor_msgs::ImageConstPtr& msg)
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
        
        cv::imshow("Thresh", imgThresholded);
        
        //move arm towards center of proper color    
        moveArm(posX - imgThresholded.cols/2 - XTRANS, posY - imgThresholded.rows/2 - YTRANS);
    }
    else
    {
        if(++no_sign_of_plushies > 20)
        {
            no_sign_of_plushies = 0;
            moveAboveBowl();
        }
    }
}

/* Moves Baxter's arm above the centroid of the bowl with the help of the FindBowl service */
void 
Pickup::moveAboveBowl() 
{
    if(isHolding) {
        stage = FINISHED;
        return;
    }
    moveOutOfDepthCloud();

    operation_plushie::BowlValues srv_values;
    operation_plushie::Ping srv_ping;

    bool cant_reach = false;   
    do { 

        if(cant_reach)
            xdisplay_pub.publish(sad_face);

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

        if(!reposition_hand_client.call(srv))
        {
            ROS_ERROR("Failed to call reposition_hand_service");
            return;
        } 

        cant_reach = true;

    } while(sleepUntilDone());

    xdisplay_pub.publish(happy_face);

    stage = INITIALIZING;
}

/* Moves arm into a specific pose with the position_joints service to move it out of the point cloud */
//TODO: make this functional for both arms
void 
Pickup::moveOutOfDepthCloud()
{
    operation_plushie::PositionJoints srv;

    srv.request.names.push_back("left_e0"); 
    srv.request.names.push_back("left_e1");
    srv.request.names.push_back("left_s0");
    srv.request.names.push_back("left_s1");
    srv.request.names.push_back("left_w0");
    srv.request.names.push_back("left_w1");
    srv.request.names.push_back("left_w2");

    srv.request.command.push_back(0.141);
    srv.request.command.push_back(1.998);
    srv.request.command.push_back(0.361);
    srv.request.command.push_back(-1.366);
    srv.request.command.push_back(0);
    srv.request.command.push_back(0.938);
    srv.request.command.push_back(1.308);

    if (!position_joints_client.call(srv))
    {
        ROS_ERROR("Failed to call position_joints_service");
        return;
    }

    operation_plushie::isComplete srv_progress;

    do {
        if(!position_joints_progress.call(srv_progress))
        {
            ROS_ERROR("Failed to call position_joints_progress");
            return;
        }
    } while(!srv_progress.response.isComplete);
}

/* Moves arm based off of information gathered from visual servoing */
void 
Pickup::moveArm(int y_shift, int x_shift)
{
    const int CENT_VAL = 10;   

    if(abs(x_shift) < CENT_VAL && abs(y_shift) < CENT_VAL)
    {
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

/* Moves arm by a small amount downwards from a specific x and y coordinate */
bool 
Pickup::stepDown(double __x, double __y)
{

    //We do not use srv.request.x = x for a good reason
    //Because Baxter's movements are not precise, his x and y change if he moves down
    //Before stepping down, you must save the desired x and y and reuse those coordinates rather than grabbing the location of Baxter's hand after
    //every iteration, otherwise he will go downwards diagonally/randomly
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

/* Function that suspends the thread until the reposition service has finished moving the arm into place */
bool 
Pickup::sleepUntilDone()
{
    operation_plushie::isComplete srv;
    while(!srv.response.isComplete)
    {
        reposition_progress_client.call(srv);
    }
    
    return srv.response.isStuck;
}

/* Uses the stepDown function to move down, if stepDown returns true then the process has completed */
void 
Pickup::lowerArm()
{
    if(stepDown(lowering_x, lowering_y))
    {
        stage = RETURNING;
    }
}

/* Assumes to be at the bottom of a bowl, closes grippers, and raises to z=.15 */
void 
Pickup::fetchNRaise()
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

/* Calibrates and releases the grippers if they were closed */
void 
Pickup::setupHand()
{
    //yes, this is supposed to be the assignment operator
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
        moveOutOfDepthCloud();
        stage = FINISHED;
    }
}

/* Fetches data from the end effector to find the position of the gripper */
void 
Pickup::updateEndpoint(baxter_core_msgs::EndpointState eps)
{
    x = eps.pose.position.x;
    y = eps.pose.position.y;
    z = eps.pose.position.z;
}

/* Fetches data for isHolding */
void 
Pickup::updateEndEffectorState(baxter_core_msgs::EndEffectorState ees)
{
    isHolding = ees.gripping;
}

/* Fetches button data */
void 
Pickup::updateOKButtonState(baxter_core_msgs::DigitalIOState ok)
{
    isPressed = ok.state;
}

/* Informs ServiceClients of the Pickup node's progress */
bool 
Pickup::isComplete(operation_plushie::isComplete::Request &req, operation_plushie::isComplete::Response &res) 
{
    res.isComplete = (stage == FINISHED);
    return true;
}
