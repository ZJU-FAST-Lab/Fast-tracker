#include <target_detection/sim_detect.h>
Sim_detect::Sim_detect(ros::NodeHandle& nh){
    //     <param name="grid_map/resolution"      value="0.1" /> 
    // <param name="grid_map/map_size_x"   value="$(arg map_size_x_)" /> 
    // <param name="grid_map/map_size_y"   value="$(arg map_size_y_)" /> 
    // <param name="grid_map/map_size_z"   value="$(arg map_size_z_)" /> 
    nh.param("grid_map/resolution",resolution,0.1);
    nh.param("grid_map/map_size_x",map_size_3d_[0],100.0);
    nh.param("grid_map/map_size_y",map_size_3d_[1],100.0);
    nh.param("grid_map/map_size_z",map_size_3d_[2],3.0);
    origin_[0] = -map_size_3d_(0)/2;
    origin_[1] = -map_size_3d_(1)/2;
    origin_[2] = -0.1;
    dim_(0) = map_size_3d_(0)/resolution;
    dim_(1) = map_size_3d_(1)/resolution;
    dim_(2) = map_size_3d_(2)/resolution;
    buffer_size = dim_(0) * dim_(1) * dim_(2);
    map_buffer.resize(buffer_size);
    map_sub = nh.subscribe("global_map", 10, &Sim_detect::GlobalMapBuild, this);
    target_sub = nh.subscribe("car_state_detect",1,&Sim_detect::car_state_cb,this);
    drone_sub = nh.subscribe("drone_odom",1,&Sim_detect::drone_odom_cb,this);
    detection_pub = nh.advertise<nav_msgs::Odometry>("target",1,true);
}
void Sim_detect::GlobalMapBuild(const sensor_msgs::PointCloud2 & pointcloud_map){
    if(has_map) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pointcloud_map, cloud);
    if( (int)cloud.points.size() == 0 ) return;
    pcl::PointXYZ pt;
    ROS_INFO("cloud points size=%d\n",(int)cloud.points.size());
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {
        pt = cloud.points[idx];
        setObs(Eigen::Vector3d(pt.x,pt.y,pt.z));
    }
    has_map = true;
    // ROS_WARN("Sim Detection Node1111: Finish gridmap built");
}
void Sim_detect::setObs(Eigen::Vector3d pt){
    int expand_size = 0;
    Eigen::Vector3i index = FloatToint(pt);
    if(isOutside(index))
        return;
    for (int i=-expand_size;i<=expand_size;i++)
    for (int j=-expand_size;j<=expand_size;j++)
    for (int k=-expand_size;k<=expand_size;k++)
        {
            Eigen::Vector3i temp_index;
            temp_index(0) = index(0)+i;
            temp_index(1) = index(1)+j;
            temp_index(2) = index(2)+k;
            if(isOutside(temp_index)) continue;
            map_buffer[getIndex(temp_index)] = 1;
        }
}
bool Sim_detect::isOutside(Eigen::Vector3i idx){
    for(int i = 0; i <3; i++)
          if (idx(i) < 0 || idx(i) >= dim_(i))
            return true;
        return false;
}
int Sim_detect::getIndex(Eigen::Vector3i idx){
    return idx(0) + dim_(0) * idx(1) + dim_(0) * dim_(1) * idx(2);
}
bool Sim_detect::isOccupied(Eigen::Vector3i idx){
    if(isOutside(idx)) {
    // ROS_INFO_STREAM("IDX: "<<idx );
    return true;
    }
    return map_buffer[getIndex(idx)];
}
Eigen::Vector3d Sim_detect::intToFloat(const Eigen::Vector3i idx){
    return (idx.template cast<double>() + Eigen::Vector3d::Constant(0.5)) * resolution + origin_;
}
Eigen::Vector3i Sim_detect::FloatToint(const Eigen::Vector3d pt){
    Eigen::Vector3i pn;
    for(int i = 0; i < 3; i++)
          pn(i) = std::round((pt(i) - origin_(i)) / resolution - 0.5);
    return pn;
}
bool Sim_detect::is_block(Eigen::Vector3d pos1,Eigen::Vector3d pos2){
    double t;
    for(t=0.0;t<=1;t+=0.01){
        Eigen::Vector3d tmp_pos;
        tmp_pos = t*pos1+(1-t)*pos2;
        if(isOccupied(FloatToint(tmp_pos))){
            return true;
        }
    }
    return false;
}
void Sim_detect::car_state_cb(const nav_msgs::Odometry& car_state){
    if(!has_map) return;
    Eigen::Vector3d car_pos;
    car_pos<<car_state.pose.pose.position.x,car_state.pose.pose.position.y,car_state.pose.pose.position.z;
    // if((car_pos-drone_pos).norm()>3.0) return;
    if(!is_block(car_pos,drone_pos)){
        nav_msgs::Odometry state_pub = car_state;
        state_pub.header.stamp = ros::Time::now();    
        detection_pub.publish(state_pub);
    }

}
void Sim_detect::drone_odom_cb(const nav_msgs::Odometry& odom){
    drone_pos << odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z;
}