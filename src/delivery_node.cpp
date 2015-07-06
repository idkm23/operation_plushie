#include "Delivery.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delivery_node");

    Delivery d;
    d.beginDetection();

    return 0;
}
