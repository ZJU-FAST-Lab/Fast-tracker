#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <car_planner/car_search.h>
using namespace car_planner;
std::string mesh_resource;
ros::Subscriber waypoints_sub;
ros::Publisher viscar_pub,state_pub;
ros::Timer state_timer,state_update_timer;
Eigen::Vector3d current_state,target_pt;
Car_KinoSearch* kinosearch;
bool update_flag = false;
double update_time;
double car_l,car_w,car_h;
void rcvWaypointsCallback(const geometry_msgs::PoseStamped& msg);
void vis_carmodel(Eigen::Vector3d cur_state);
void pub_carstate(const ros::TimerEvent& event);
void state_update(const ros::TimerEvent& event);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_search_node");
    ros::NodeHandle nh_priv("~");
    nh_priv.param("car/init_x",current_state[0],-16.0);
    nh_priv.param("car/init_y",current_state[1],4.0);
    nh_priv.param("car/init_yaw",current_state[2],0.0);
    nh_priv.param("car/mesh_resource", mesh_resource, std::string("package://car_planner/param/car.dae"));
    nh_priv.param("car_search/car_l",car_l,0.6);
    nh_priv.param("car_search/car_w",car_w,0.4);
    nh_priv.param("car_search/car_h",car_h,0.3);
    ros::MultiThreadedSpinner spinner(8);
    waypoints_sub= nh_priv.subscribe( "waypoints", 1, rcvWaypointsCallback );
    state_timer = nh_priv.createTimer(ros::Duration(0.05),pub_carstate);
    state_update_timer = nh_priv.createTimer(ros::Duration(0.01),state_update);
    viscar_pub =  nh_priv.advertise<visualization_msgs::Marker>("/visualization/car_model", 100,true);
    state_pub = nh_priv.advertise<nav_msgs::Odometry>("car_state",1,true);
    kinosearch = new Car_KinoSearch(nh_priv);
    spinner.spin();
    return 0;
}
void rcvWaypointsCallback(const geometry_msgs::PoseStamped& msg)                                                                                      
{  
    Eigen::Quaterniond tq; 
    tq.x() = msg.pose.orientation.x;
    tq.y() = msg.pose.orientation.y;
    tq.z() = msg.pose.orientation.z;
    tq.w() = msg.pose.orientation.w;
    Eigen::Matrix3d trot(tq);
    double tyaw  = atan2(trot.col(0)[1],trot.col(0)[0]);
    target_pt<<msg.pose.position.x,msg.pose.position.y,tyaw;
    ROS_INFO_STREAM("TARGET="<<target_pt);
    ROS_INFO("[node] receive the planning target");
    kinosearch->reset();
    kinosearch->car_search(current_state,target_pt);
    kinosearch->visualize(0.1);
    update_flag = true;
    update_time = ros::Time::now().toSec();
}
void state_update(const ros::TimerEvent& event){
    if(update_flag){
            double now_time = ros::Time::now().toSec();
            double delta_time = now_time - update_time;
            if(delta_time<=kinosearch->get_totalT()){
                current_state = kinosearch->evaluate_state(delta_time);
            }
            else{
                update_flag = false;
            }
    }
    vis_carmodel(current_state);
}
void vis_carmodel(Eigen::Vector3d cur_state){
    Eigen::Vector3d pos;
    Eigen::Matrix3d rota_M;
    double x = cur_state[0],y=cur_state[1],z=0;
    double yaw = cur_state[2];
    rota_M  <<  cos(yaw),-sin(yaw),0,
                sin(yaw),cos(yaw),0,
                0       ,0       ,1; 
    pos = Eigen::Vector3d(x,y,z)+rota_M*(Eigen::Vector3d(car_l/2,0,0));
    visualization_msgs::Marker WpMarker;
    WpMarker.id = 0;
    WpMarker.header.stamp = ros::Time::now();
    WpMarker.header.frame_id = "world";
    WpMarker.action = visualization_msgs::Marker::ADD;
    WpMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    WpMarker.ns = "car_mesh";
    WpMarker.mesh_use_embedded_materials = true;
    WpMarker.color.r = 0.0;
    WpMarker.color.g = 0.0;
    WpMarker.color.b = 0.0;
    WpMarker.color.a = 0.0;
    WpMarker.scale.x = car_l/4.5;
    WpMarker.scale.y = car_l/4.5;
    WpMarker.scale.z = car_l/4.5;
    Eigen::Matrix3d rot_c,rot_f;
    rot_c << 0,-1,0, 
             1,0,0,
             0,0,1;
    rot_f = rota_M*rot_c;
    Eigen::Quaterniond q(rot_f);
    WpMarker.pose.orientation.w = q.w();
    WpMarker.pose.orientation.x = q.x();
    WpMarker.pose.orientation.y = q.y();
    WpMarker.pose.orientation.z = q.z();
    WpMarker.pose.position.x = pos[0];
    WpMarker.pose.position.y = pos[1];
    WpMarker.pose.position.z = pos[2];
    WpMarker.mesh_resource = mesh_resource;
    viscar_pub.publish(WpMarker);
}
void pub_carstate(const ros::TimerEvent& event){
    // state_pub
    nav_msgs::Odometry car_odom;
    car_odom.header.frame_id = "world";
    car_odom.header.stamp  =ros::Time::now();
    car_odom.pose.pose.position.x = current_state[0];
    car_odom.pose.pose.position.y = current_state[1];
    car_odom.pose.pose.position.z = 0;
    Eigen::Matrix3d rota_M;
    double yaw = current_state[2];
    rota_M  <<  cos(yaw),-sin(yaw),0,
                sin(yaw),cos(yaw),0,
                0       ,0       ,1;
    Eigen::Quaterniond q(rota_M); 
    car_odom.pose.pose.orientation.x = q.x();
    car_odom.pose.pose.orientation.y = q.y();
    car_odom.pose.pose.orientation.z = q.z();
    car_odom.pose.pose.orientation.w = q.w();
    state_pub.publish(car_odom);
}