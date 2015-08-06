/* Launched from operation_plushie.launch. Starts the Pickup service. */

#include "Pickup.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delivery_node");

    Pickup p;
    p.begin_detection();

    return 0;
}
