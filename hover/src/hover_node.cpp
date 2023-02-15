#include"hover/hover.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    bool input_setpoint = true; // if true run offboard control else initialize parameters and receive messages

    OffboardControl *hover = new OffboardControl(nh, nh_private, input_setpoint);
    ros::spin();
        
    return 0;
}