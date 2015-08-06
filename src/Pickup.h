/* This header file provides function headers for Pickup.cpp. */

#include "ros/ros.h"

#include "operation_plushie/Pickup.h"
#include "operation_plushie/RepositionHand.h"
#include "operation_plushie/isComplete.h"
#include "operation_plushie/Ping.h"
#include "operation_plushie/BowlValues.h"
#include "operation_plushie/PositionJoints.h"

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

//for button
#include "baxter_core_msgs/DigitalIOState.h"

#include <pthread.h>

enum Stage {TOBOWL, INITIALIZING, CENTERING, LOWERING, RETURNING, FINISHED};

class Pickup
{
private:
    ros::NodeHandle n;
    ros::ServiceServer pickup_service, isComplete_service;
    ros::ServiceClient reposition_hand_client, reposition_progress_client, bowl_client, bowl_values_client, position_joints_client, position_joints_progress;
    ros::Publisher arm_pub, xdisplay_pub, gripper_pub;
    ros::Subscriber raw_image, endstate_sub, is_holding_sub, ok_button_sub;
    bool isCentered, isLeft, isHolding, isPressed, missedLast;
    double x, y, z;
    double lowering_x, lowering_y;
    Stage stage;
    int yaw_index, no_sign_of_plushies;
    sensor_msgs::ImagePtr happy_face, sad_face;

    static const int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    static const double yawDictionary[];

public:
    Pickup();
    void begin_detection();

    bool isComplete(operation_plushie::isComplete::Request&, operation_plushie::isComplete::Response&);

    //main service call function
    bool grabPlushie(operation_plushie::Pickup::Request&, operation_plushie::Pickup::Response&);

    void updateEndpoint(baxter_core_msgs::EndpointState);
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
