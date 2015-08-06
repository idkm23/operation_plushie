/* Launched from operation_plushie.launch. Starts the PositionJoints service. */

#include "PositionJoints.cpp"

int 
main (int argc, char** argv)
{
    ros::init (argc, argv, "position_joints_node");
    PositionJoints pj;
    pj.begin_detection();   
    return 0;
}

