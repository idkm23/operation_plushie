#include "FaceDetector.cpp"

/*This program starts everything else by calling begin_detection 
    in FaceDetector.cpp*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "operation_plushie_node");
    FaceDetector fd;

    ros::spin();
 
    return 0;
}
