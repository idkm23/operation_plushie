/* Launched from operation_plushie.launch. Starts the RepositionHand service. */

#include "RepositionHand.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reposition_hand_node");

    RepositionHand rh;
    rh.begin_detection();

    return 0;
}
