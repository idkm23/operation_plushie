/* This is launched from operation_plushie.launch to start up the Delivery service. */

#include "Delivery.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delivery_node");
    Delivery d;

    ros::spin();

    return 0;
}
