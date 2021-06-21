#include<plan_manage/plan_manage.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_fsm_node");
    // ros::MultiThreadedSpinner spinner(16);
    ros::NodeHandle nh_priv("~");
    plan_manage tracking_fsm(nh_priv);
    // spinner.spin();
    ros::spin();
    return 0;
}