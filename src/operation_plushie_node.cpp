#include "FaceDetector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "operation_plushie_node");
    FaceDetector fd;
    fd.begin_detection();   
 
//    Pickup p;
    
/*    ros::ServiceClient pickup_client = n.serviceClient<operation_plushie::Pickup>("pickup_service");

//    operation_plushie::Pickup srv;

//    p.begin_detection();

    srv.request.isLeft = true;

    if(!pickup_client.call(srv))
    {
        ROS_ERROR("Failed to call pickup_service.");
    }

//    ROS_INFO("Done.");
*/
    return 0;
}
