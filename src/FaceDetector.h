#include <ros/ros.h>
#include "operation_plushie/Pickup.h"
#include "operation_plushie/Delivery.h"
#include "operation_plushie/isComplete.h"

#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/HeadState.h>

//For the camera's image format conversion to Mat
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//For face detection
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string>
#include <pthread.h>

enum Stage {PICKUP, DELIVER};

typedef struct {
    double rating;
    cv::Rect rect;
} ConsistentRect;

class FaceDetector 
{
private:
    ros::NodeHandle n;
    ros::Publisher monitor_pub, xdisplay_pub;
    ros::Subscriber raw_image, monitor_sub;
    ros::ServiceClient pickup_client, pickup_isComplete_client, delivery_client, delivery_isComplete_client;   
    
    //For initial face detection 
    cv::CascadeClassifier face_cascade;
    cv::RNG rng;

    baxter_core_msgs::HeadState head_state;
    sensor_msgs::ImagePtr happy_face, unsure_face, lemon_face;

    Stage state;
    int no_face_count, skippedFrames;
    std::vector<ConsistentRect> consistent_rects;

public:
    FaceDetector();
    void begin_detection();
    void call_back(const sensor_msgs::ImageConstPtr&);
    void detectAndDisplay(cv::Mat); 
    void updateHead(const baxter_core_msgs::HeadState::ConstPtr&);
    bool properColor(cv::Mat);
    void addConsistent(cv::Rect);
    bool isOverlapping(cv::Rect, cv::Rect);
    void decrementConsistentRects();
    int findBestIndex(cv::Mat);
    void tickFaceCount(int, int, cv::Mat);
    std::vector<cv::Rect> findConfirmedFaces(std::vector<cv::Rect>, cv::Mat);

    void chooseStage();
    void pickup();
    void deliver();
};
