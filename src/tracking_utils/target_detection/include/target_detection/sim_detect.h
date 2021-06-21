#ifndef _SIM_DETECT_H
#define _SIM_DETECT_H
#include <ros/console.h>
#include <ros/ros.h>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tuple>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>

class Sim_detect{
    /*map*/
    private:
        Eigen::Vector3d origin_, map_size_3d_,dim_;
        std::vector<signed char> map_buffer;
        double resolution;
        int buffer_size;
        bool has_map = false;
        ros::Publisher detection_pub;
        ros::Subscriber map_sub, target_sub,drone_sub;
        Eigen::Vector3d drone_pos;
    public:
        Sim_detect(ros::NodeHandle& nh);
        ~Sim_detect(){};
        /*grid map*/
        void GlobalMapBuild(const sensor_msgs::PointCloud2 & pointcloud_map);
        void setObs(Eigen::Vector3d pt);
        bool isOutside(Eigen::Vector3i idx);
        int getIndex(Eigen::Vector3i idx);
        bool isOccupied(Eigen::Vector3i idx);
        Eigen::Vector3d intToFloat(const Eigen::Vector3i idx);
        Eigen::Vector3i FloatToint(const Eigen::Vector3d pt);
        /* main */
        bool is_block(Eigen::Vector3d pos1,Eigen::Vector3d pos2);
        void car_state_cb(const nav_msgs::Odometry& car_state);
        void drone_odom_cb(const nav_msgs::Odometry& odom);

};
#endif