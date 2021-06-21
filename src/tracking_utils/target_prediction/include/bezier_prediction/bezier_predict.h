#ifndef _BEZIER_H
#define _BEZIER_H
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <Eigen/Eigen>
#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseMa27.h>
#include <std_msgs/Float64.h>
#define _MAX_SEG 30         
#define _PREDICT_SEG  30    
#define _TIME_INTERVAL 0.05 
#define Lambda_ACC  1.5     
#define SAMPLE_INTERVALS 0.001



class Bezierpredict{
  private:
        double obj;
        Eigen::MatrixXd PolyCoeff;
        Eigen::VectorXd PolyTime;
        Eigen::MatrixXd M;
        Eigen::MatrixXd _Q,_M;
        vector<int> C_;
        int segs;
        int traj_order;
        double history_time_total;
        double history_time_init;
        vector<double> time_each_seg;
    public:


      int factorial(int n){
        int fact = 1;
        for(int i = n; i > 0 ; i--)
        fact *= i;
        return fact;    
      }
      int  combinatorial(int n, int k) {
        return factorial(n) / (factorial(k) * factorial(n - k));}
      Bezierpredict(){
        traj_order = 5;            
        M = Eigen::MatrixXd::Zero(6,6);
        M << 1,   0,   0,   0,  0,  0,
            -5,   5,   0,   0,  0,  0,
            10, -20,  10,   0,  0,  0,
            -10,  30, -30,  10,  0,  0,
            5, -20,  30, -20,  5,  0,
            -1,   5, -10,  10, -5,  1;
          for(int i=0;i<=5;i++){
            C_.push_back(combinatorial(5,i));
          }
        for(int i = 0; i < _MAX_SEG; i++){
          time_each_seg.push_back(i * _TIME_INTERVAL);
        }    
        }

        ~Bezierpredict(){}


        int TrackingGeneration(
        const double max_vel,
        const double max_acc,
        vector<Eigen::Vector4d> predict_list_complete
        );       
        Eigen::MatrixXd getQ(const int vars_number, const vector<double> Time, const int seg_index);
        Eigen::MatrixXd getM(const int vars_number, const vector<double> Time, const int seg_index);
        Eigen::MatrixXd getPolyCoeff()
        {
            return PolyCoeff;
        };

        Eigen::VectorXd getPolyTime()
        {
            return PolyTime;
        };

        double getObjective()
        {
            return obj;
        };
        inline Eigen::Vector3d getPosFromBezier(const double & t_now, const int & seg_now ){
		      Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);
          double T = PolyTime(seg_now);
          // T=1;
          // cout<<PolyCoeff;
		      for(int i = 0; i < 3; i++)
		        for(int j = 0; j < traj_order+1; j++)
		          ret(i) += C_[j] * PolyCoeff(seg_now, i * (traj_order+1) + j) * pow(t_now/T, j) * pow((1 - t_now/T), (traj_order - j) ); 
              //ROS_INFO_STREAM("asdf" << t_now/T);
              // ROS_INFO("ret=%f  %f  %f",ret(0),ret(1),ret(2));
		      return ret;  
		    };
        inline vector<Eigen::Vector3d> getPosListFromBezier(const int & predict_seg){
          int seg = 1;
          vector<Eigen::Vector3d> PosList;
          
          for(int i = 0; i < seg; i++){
            // for (double t = _TIME_INTERVAL * _MAX_SEG; t < _TIME_INTERVAL * (_MAX_SEG + predict_seg); t += _TIME_INTERVAL){
            for (double t = history_time_total; t < _TIME_INTERVAL * predict_seg + history_time_total; t += _TIME_INTERVAL){
              //ROS_INFO_STREAM("index: " << t);
              Eigen::Vector3d pos = getPosFromBezier(t,i);
              PosList.push_back(pos);
            }
          }
          //ROS_INFO_STREAM("asdffsdf   " << PosList.size());
          return PosList;
        }
        inline vector<Eigen::Vector3d> getPosListFromBezier(int a, const int & predict_seg){
          int seg = 1;
          vector<Eigen::Vector3d> PosList;
         
          for(int i = 0; i < seg; i++){
            // for (double t = _TIME_INTERVAL * _MAX_SEG; t < _TIME_INTERVAL * (_MAX_SEG + predict_seg); t += _TIME_INTERVAL){
            for (double t = 0; t < history_time_total; t += _TIME_INTERVAL){
              //ROS_INFO_STREAM("index: " << t);
              Eigen::Vector3d pos = getPosFromBezier(t,i);
              PosList.push_back(pos);
            }
          }
          //ROS_INFO_STREAM("asdffsdf   " << PosList.size());
          return PosList;
        }
        inline vector<Eigen::Vector3d> SamplePoslist_bezier(const int & predict_seg){
          int seg = 1;
          vector<Eigen::Vector3d> PosList;
         
          for(int i = 0; i < seg; i++){
            // for (double t = _TIME_INTERVAL * _MAX_SEG; t < _TIME_INTERVAL * (_MAX_SEG + predict_seg); t += _TIME_INTERVAL){
            for (double t = history_time_total; t < _TIME_INTERVAL * predict_seg + history_time_total; t += SAMPLE_INTERVALS){
            //for (double t = 0; t < _TIME_INTERVAL * _MAX_SEG; t += SAMPLE_INTERVALS){
              Eigen::Vector3d pos = getPosFromBezier(t,i);
              PosList.push_back(pos);
            }
          }
          //ROS_INFO_STREAM("asdffsdf   " << PosList.size());
          return PosList;
        }
        inline Eigen::Vector3d getVelFromBezier(const double & t_now, const int & seg_now ){
		      Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);
          double T = PolyTime(seg_now);
          // T=1;
          // cout<<PolyCoeff;
		      for(int i = 0; i < 3; i++)
		        for(int j = 0; j < traj_order; j++)
            {
              double c_n=traj_order*(PolyCoeff(seg_now, i * (traj_order+1) + j + 1)-PolyCoeff(seg_now, i * (traj_order+1) + j))/T;
              ret(i)+=combinatorial(traj_order-1,j)*c_n*pow(t_now/T, j) * pow((1 - t_now/T), (traj_order-1 - j) );
            }
		          // ret(i) += C_[j] * PolyCoeff(seg_now, i * (traj_order+1) + j) * pow(t_now/T, j) * pow((1 - t_now/T), (traj_order - j) ); 
              // ROS_INFO("ret=%f  %f  %f",ret(0),ret(1),ret(2));
		      return ret;  
		};
        inline vector<Eigen::Matrix<double, 6, 1>> getStateListFromBezier(const int & predict_seg){
          int seg = 1;
          vector<Eigen::Matrix<double,6,1>> predict_traj_state;
       
          for(int i = 0; i < seg; i++){
            for (double t = history_time_total; t < _TIME_INTERVAL * predict_seg + history_time_total; t += _TIME_INTERVAL){
              Eigen::Vector3d pos = getPosFromBezier(t,i);
              Eigen::Vector3d vel = getVelFromBezier(t,i);
              Eigen::Matrix<double,6,1> tmp_state;
              tmp_state.head(3) = pos;
              tmp_state.tail(3) = vel;
              predict_traj_state.push_back(tmp_state);
            }
          }
          return predict_traj_state;
        }
        inline Eigen::Vector3d getAccFromBezier(const double & t_now, const int & seg_now ){
		      Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);
          double T = PolyTime(seg_now);
          // T=1;
          // cout<<PolyCoeff;
		      for(int i = 0; i < 3; i++)
		        for(int j = 0; j < traj_order-1; j++)
            {
              double c_n=traj_order*(traj_order-1)*(PolyCoeff(seg_now, i * (traj_order+1) + j + 2)-2*PolyCoeff(seg_now, i * (traj_order+1) + j + 1)+PolyCoeff(seg_now, i * (traj_order+1) + j))/pow(T,2);
              ret(i)+=combinatorial(traj_order-2,j)*c_n*pow(t_now/T, j) * pow((1 - t_now/T), (traj_order-2 - j) );
            }
		          // ret(i) += C_[j] * PolyCoeff(seg_now, i * (traj_order+1) + j) * pow(t_now/T, j) * pow((1 - t_now/T), (traj_order - j) ); 
              // ROS_INFO("ret=%f  %f  %f",ret(0),ret(1),ret(2));
		      return ret;  
		};
        inline Eigen::Vector3d getJerkFromBezier(const double & t_now, const int & seg_now ){
		      Eigen::Vector3d ret = Eigen::VectorXd::Zero(3);
          double T = PolyTime(seg_now);
          // T=1;
          // cout<<PolyCoeff;
		      for(int i = 0; i < 3; i++)
		        for(int j = 0; j < traj_order-2; j++)
            {
              double c_n=traj_order*(traj_order-1)*(traj_order-2)*(PolyCoeff(seg_now, i * (traj_order+1) + j + 3)-3*PolyCoeff(seg_now, i * (traj_order+1) + j + 2)+3*PolyCoeff(seg_now, i * (traj_order+1) + j + 1)-PolyCoeff(seg_now, i * (traj_order+1) + j))/pow(T,3);
              ret(i)+=combinatorial(traj_order-3,j)*c_n*pow(t_now/T, j) * pow((1 - t_now/T), (traj_order-3 - j) );
            }
		          // ret(i) += C_[j] * PolyCoeff(seg_now, i * (traj_order+1) + j) * pow(t_now/T, j) * pow((1 - t_now/T), (traj_order - j) ); 
              // ROS_INFO("ret=%f  %f  %f",ret(0),ret(1),ret(2));
		      return ret;  
		};
        inline Eigen::MatrixXd getCt(double curr_time,Eigen::Vector3d curr_pos){
  
          Eigen::MatrixXd Ct = Eigen::MatrixXd::Zero(1, 3*(traj_order + 1));
          for(int i =0;i<3;i++){
            for(int j= 0;j<traj_order+1;j++){
              //ROS_INFO_STREAM("Ct: "<<t/time_interval);
              Ct(0,j+i*(1+traj_order)) = (-2)*curr_pos[i]*pow(curr_time,j);//X Y Z
            }
          } 
          //ROS_INFO_STREAM("Ct: "<<Ct);
          return Ct; 
        }
        inline Eigen::MatrixXd getdistance_Q(double curr_time){
      
          Eigen::MatrixXd distance_Q = Eigen::MatrixXd::Zero(3*(traj_order + 1),3*(traj_order + 1));
          for(int i = 0;i<3;i++){
            Eigen::MatrixXd tmp_Q = Eigen::MatrixXd::Zero(traj_order+1,traj_order+1);
            for(int j=0;j<traj_order+1;j++){
              for(int k=0;k<traj_order+1;k++){
                tmp_Q(j,k) = pow(curr_time,j+k);
               // ROS_INFO_STREAM("asdf" << tmp_Q);
              }
            }
            //          ROS_INFO_STREAM("Q: "<<tmp_Q);
            distance_Q.block(i*(traj_order+1),i*(traj_order+1),traj_order+1,traj_order+1) = tmp_Q;               
          }

          return distance_Q;
        };
        inline Eigen::Vector3d getinitial_vel(){
          return getVelFromBezier(history_time_total,0);
        } 
};
#endif
