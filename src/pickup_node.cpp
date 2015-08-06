/* Launched from operation_plushie.launch. Starts the Pickup service. */

#include "Pickup.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delivery_node");
    Pickup p;

    ros::spin();

    return 0;
}
