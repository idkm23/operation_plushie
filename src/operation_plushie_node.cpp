#include "FaceDetector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_reader_node");
    
    FaceDetector fd;
    fd.begin_detection();
 
    return 0;
}
