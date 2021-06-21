#include<target_detection/sim_detect.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_fsm_node");
    // ros::MultiThreadedSpinner spinner(16);
    ros::NodeHandle nh_priv("~");
    Sim_detect sim_detection(nh_priv);
    // spinner.spin();
    ros::spin();
    return 0;
}