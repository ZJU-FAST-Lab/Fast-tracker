#ifndef _PLANMANAGE_H
#define _PLANMANAGE_H
#include<grid_path_searcher/hybridAstar_searcher.h>
#include<bezier_prediction/bezier_predict.h>
#include <sfc_generation/sfc.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
// #include "lbfgs.hpp"
// #include "traj.hpp"
// #include "traj3.hpp"
#include "Optraj.h"
#define BUDGET_TIME 0.06


enum STATE_TYPE{
        EMERGENCY,
        RELOCATING,
        TRACKING
};
class plan_manage{
    private:
        /*tracking utils*/
        hybridAstar_searcher kinosearch;
        Bezierpredict tgpredict;
        FlightCorridor sfc;
        GridMap::Ptr env;
        /*system variable*/
        Eigen::Vector3d position;
        Eigen::Quaterniond q;
        double yaw;
        std::vector<Eigen::Vector4d> target_detect_list; //px py pz t
        quadrotor_msgs::PolynomialTrajectory traj_msg;
        OpTrajectory traj;
        OpTrajectory last_traj;
        double last_rcvtime;
        /*state machine*/
        bool relocate_init = false;
        bool init_flag = false;
        STATE_TYPE fsm;
        /*parameters*/
        double v_max;
        double a_max;
        double replan_frequency = 12.0;
        /*ros*/
        // ros::Timer 
        ros::Timer replan_timer;
        ros::Subscriber detect_sub,odom_sub;
        ros::Publisher kino_search_vispub,traj_vispub,pretraj_vispub,cor_vispub;
        ros::Publisher TrackTrajPub;

    public:
        plan_manage(ros::NodeHandle& nh);
        ~plan_manage(){}
        /*back end trajectory generate*/
        /*callback function*/
        void fsm_timer_cb(const ros::TimerEvent& event);
        void tg_list_cb(const nav_msgs::Odometry& car_state);
        void odom_cb(const nav_msgs::Odometry& odom);
        /*APIS*/
        Eigen::Vector3d evaluteP(double t);
        Eigen::Vector3d evaluteV(double t);
        Eigen::Vector3d evaluteA(double t);
        quadrotor_msgs::PolynomialTrajectory traj2msg(OpTrajectory traj);
        /*visualization*/
        void visualize_pre(std::vector<Eigen::Vector3d> poslist);
        void visualize_traj(OpTrajectory traj);
        void visualize_relocate(OpTrajectory traj);
};
#endif