#include <grid_path_searcher/hybridAstar_searcher.h>
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
#include <algorithm>
using namespace std;
using namespace Eigen;



double hybridAstar_searcher::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                                           double& optimal_time) {
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);
  // derive of C*
  double c1 = -36 * dp.dot(dp);                             //-4 order
  double c2 = 24 * (v0 + v1).dot(dp);                       //-3 order 
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));  //-2 order
  double c4 = 0;                                            //0
  double c5 = w_time_;                                      //0 order

  // find the optimal t of C*
  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d  = t_bar;

  for (auto t : ts) {
    if (t < t_bar) continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost) {
      cost = c;
      t_d  = t;
    }
  }
  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}
bool hybridAstar_searcher::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                                       double time_to_goal) {
  /* ---------- get coefficient ---------- */
  const Vector3d p0  = state1.head(3);
  const Vector3d dp  = state2.head(3) - p0;
  const Vector3d v0  = state1.segment(3, 3);
  const Vector3d v1  = state2.segment(3, 3);
  const Vector3d dv  = v1 - v0;
  double         t_d = time_to_goal;
  MatrixXd       coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */


  vector<Vector3d> pos_list;
  double time;
  for (time = 0; time <= t_d; time += 0.01) {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++) t(j) = pow(time, j);
    for (int dim = 0; dim < 3; dim++) {
      poly1d     = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim)   = (Tm * poly1d).dot(t);
      acc(dim)   = (Tm * Tm * poly1d).dot(t);

      if (fabs(acc(dim)) > max_acc_) {
        return false;
      }
    }
    pos_list.push_back(coord);
    // if (coord(0) < gl_xl || coord(0) >= gl_xu || coord(1) < gl_yl ||
    //     coord(1) >= gl_yu || coord(2) < gl_zl || coord(2) >= gl_zu) {
    //   return false;
    // }

    // if (!isFree(coord2gridIndex(coord))) {
    //   return false;
    // }
    if(gridmapPtr_->getInflateOccupancy(coord))
      return false;
  }
  if(time!=t_d){
    time = t_d;
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++) t(j) = pow(time, j);
    for (int dim = 0; dim < 3; dim++) {
      poly1d     = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim)   = (Tm * poly1d).dot(t);
      acc(dim)   = (Tm * Tm * poly1d).dot(t);

      if (fabs(acc(dim)) > max_acc_) {
        return false;
      }
    }
    pos_list.push_back(coord);
    // if (coord(0) < gl_xl || coord(0) >= gl_xu || coord(1) < gl_yl ||
    //     coord(1) >= gl_yu || coord(2) < gl_zl || coord(2) >= gl_zu) {
    //   return false;
    // }

    // if (!isFree(coord2gridIndex(coord))) {
    //   return false;
    // }
    if(gridmapPtr_->getInflateOccupancy(coord))
      return false;

  }
  vector<Vector3d> gridpath;
  gridpath = traj2grids(pos_list);
  if(!corridor_has(gridpath))
    return false;
  coef_shot_    = coef;
  t_shot_       = t_d;
  is_shot_succ_ = true;
  return true;
  }

vector<double> hybridAstar_searcher::cubic(double a, double b, double c, double d) {
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> hybridAstar_searcher::quartic(double a, double b, double c, double d, double e) {
  vector<double> dts;
  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double         y1 = ys.front();
  double         r  = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}
void hybridAstar_searcher::stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                                    double tau) {
  for (int i = 0; i < 3; ++i) phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}


vector<Eigen::Vector3d> hybridAstar_searcher::getKinoTraj(double delta_t){
  PathNodePtr node = terminatePtr;
  /* ---------- get traj of searching ---------- */
  vector<Vector3d> state_list;
  Matrix<double, 6, 1> x0, xt;
  while (node->parent != NULL) {
    Vector3d ut       = node->input;
    double   duration = node->duration;
    x0                = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  state_list.push_back(node->state.head(3));
  
  reverse(state_list.begin(), state_list.end());


  if(is_shot_succ_){

    Vector3d coord;
    VectorXd poly1d, time(4);
    double t;
    for ( t = delta_t; t <= t_shot_; t += delta_t) {
      for (int j = 0; j < 4; j++) time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d     = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
    if(t!=t_shot_){
      t = t_shot_;
      for (int j = 0; j < 4; j++) time(j) = pow(t, j);
      for (int dim = 0; dim < 3; dim++) {
        poly1d     = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }
  return state_list;
}

void hybridAstar_searcher::setParam(ros::NodeHandle& nh) {
  nh.param("search/max_tau", max_tau_, 0.6);//0.6
  nh.param("search/init_max_tau", init_max_tau_, 0.8);//0.8
  nh.param("search/max_vel", max_vel_, 5.0);//5
  nh.param("search/max_acc", max_acc_, 5.0);//5
  nh.param("search/w_time", w_time_, 10.0);//10
  nh.param("search/lambda_heu", lambda_heu_, 5.0);//5
  nh.param("search/check_num", check_num_, 5);//5
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new PathNode;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
  GraphSearch = new Astar();
  GraphSearch->setParam(nh);
}
void hybridAstar_searcher::setEnvironment(const GridMap::Ptr& env){
  this->gridmapPtr_ = env;
  has_map_ = true;
  GraphSearch->setEnvironment(env);
}
int hybridAstar_searcher::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v,vector<Eigen::Matrix<double,6,1>> predict_list,
        double time_interval){
  ros::Time time_1 = ros::Time::now();
  start_vel_ = start_v;
  Vector3i start_idx = gridmapPtr_->posToIndex(start_pt);
  if(gridmapPtr_->getInflateOccupancy(start_pt)){
    ROS_ERROR("hybrid a star crash into obstacles");
    return 0;
  }
  Eigen::Matrix<double,6,1> target_state = predict_list[0];
  double time_to_goal;
  is_shot_succ_ = false;
  Eigen::VectorXd start_state(6);
  start_state.head(3) = start_pt;
  start_state.tail(3) = start_v;
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = start_idx;
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_*estimateHeuristic(start_state,target_state,time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  cur_node->time = 0;
  open_set_.push(cur_node);
  use_node_num_+=1;
  expanded_nodes_.insert(cur_node->index, cur_node);
  PathNodePtr neighbor       = NULL;
  PathNodePtr terminate_node = NULL;
  const int   tolerance      = ceil(1 / resolution_);
  int i = 0;
  int num = 0;
  while ( !open_set_.empty() ){
      cur_node = open_set_.top();
      if(gridmapPtr_->getInflateOccupancy(cur_node->state.head(3))){
        ROS_ERROR("hybrid astar failed, expanded node is not free!");
        return NO_PATH;
      }
      open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      ros::Time time_now = ros::Time::now();
      if((time_now-time_1).toSec()>0.05){
        ROS_WARN("unoptimal solver by kinodynamic search"); 
        has_path_ = true;
        terminatePtr = cur_node;
        return 1;
      }
      iter_num_++;
      Eigen::Matrix<double, 6, 1> predict_state;
      bool end_flag = false;
      if(cur_node->time/time_interval>=predict_list.size()){
        predict_state = predict_list[predict_list.size()-1];
        end_flag = true;
      }
      else{
        predict_state = predict_list[cur_node->time/time_interval];
        end_flag = false;
      }
      // end_flag = true;
      Eigen::Matrix<double, 6, 1> track_state2 = (1 - w_predict) * target_state + w_predict*predict_state;
      if (end_flag == false)
      {
        if(((cur_node->time >= 5.0) && cur_node->time > 0) || (((cur_node->state.head(3) - track_state2.head(3)).norm() < 0.5) && cur_node->time > 0))
        {
          terminatePtr     = cur_node;
          ros::Time time_2 = ros::Time::now();
          return 1;
        }
      }
      else
      {
      //ROS_WARN("one shot");
        double time_to_goal;
        estimateHeuristic(cur_node->state,track_state2,time_to_goal);
        computeShotTraj(cur_node->state,track_state2,time_to_goal);
        if(((cur_node->time>= 5.0)||((cur_node->state.head(3)-track_state2.head(3)).norm()<0.2)||is_shot_succ_)
        &&cur_node->time>0){
          has_path_ = true;
          terminatePtr = cur_node;
          return 1;
        }
      }
      Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
      Eigen::Matrix<double, 6, 1> pro_state;
      Eigen::Vector3d             um;
      std::vector<Eigen::Vector3d> inputs;
      std::vector<double>          durations;
      std::vector<PathNodePtr>         tmp_expand_nodes;
      double pro_t;
      double res = 1 / 2.0, time_res = 1 / 4.0;
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
            for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
              for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res) {
                um << ax, ay, az;
                inputs.push_back(um);
      }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
          durations.push_back(tau);
      for(int i= 0;i<inputs.size();i++){
        for(int j=0;j<durations.size();j++){
          um = inputs[i];
          double tau = durations[j];
          stateTransit(cur_state,pro_state,um,tau);
          pro_t = cur_node->time+tau;
          if(gridmapPtr_->getInflateOccupancy(pro_state.head(3))) continue;
          //not in closelist
          Eigen::Vector3i pro_id   = gridmapPtr_->posToIndex(pro_state.head(3));
          PathNodePtr pro_node = expanded_nodes_.find(pro_id);
          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
            continue;
          }
          //collision feasible
          Eigen::Matrix<double,6,1> xt;
          bool is_occ = false;
          Eigen::Vector3d pos;
          Vector3i grid1;
          Vector3i grid2;
          grid1 = gridmapPtr_->posToIndex(cur_state.head(3));
          vector<Vector3d> gridpath;
          gridpath.push_back(gridmapPtr_->indexToPos(grid1));
          for(double t=0;t<=tau;t+=0.01){
            stateTransit(cur_state,xt,um,t);
            pos = xt.head(3);
            grid2 = gridmapPtr_->posToIndex(pos);
            if(grid2==grid1) continue;
            else{
              grid1 = grid2;
              gridpath.push_back(gridmapPtr_->indexToPos(grid1));
            }
          }
          stateTransit(cur_state,xt,um,tau);
          pos = xt.head(3);
          grid2 = gridmapPtr_->posToIndex(pos);
          if(grid2!=grid1){
            grid1 = grid2;
            gridpath.push_back(gridmapPtr_->indexToPos(grid1));
          }
          is_occ =  !corridor_has(gridpath);
          if(is_occ) continue;   
          //vel feasiable
          Eigen::Vector3d pro_v = pro_state.tail(3);
          if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_) {
            continue;
          }
          //not in same voxel
          Eigen::Vector3i pro_index = gridmapPtr_->posToIndex(pro_state.head(3));
          Eigen::Vector3i diff = pro_index - cur_node->index;
          if (diff.norm() == 0) {
            continue;
          }
          //compute cost
          double tmp_g_score,tmp_f_score,tmp_h_score,time_to_goal;
          Eigen::Matrix<double, 6, 1> predict_state;
          if(pro_t/time_interval>=predict_list.size()){
            predict_state = predict_list[predict_list.size()-1];
          }
          else{
            predict_state = predict_list[pro_t/time_interval];
          }
          Eigen::Matrix<double, 6, 1> track_state = (1 - w_predict) * target_state + w_predict*predict_state;
          double e_dis = 0;
          Eigen::Matrix<double,6,1> x0 = cur_node->state;
          tmp_g_score = (um.squaredNorm()+w_time_)*tau+cur_node->g_score;
          tmp_h_score = lambda_heu_*estimateHeuristic(pro_state,track_state,time_to_goal) + 1000 * (1.5 - pro_t);
          tmp_f_score = tmp_h_score+tmp_g_score;
          bool prune = false;
          for (int j = 0; j < tmp_expand_nodes.size(); ++j) {
            PathNodePtr expand_node = tmp_expand_nodes[j];
            if ((pro_id - expand_node->index).norm() == 0) {
              prune = true;
              if (tmp_f_score < expand_node->f_score) {
                expand_node->f_score  = tmp_f_score;
                expand_node->g_score  = tmp_g_score;
                expand_node->state    = pro_state;
                expand_node->input    = um;
                expand_node->duration = tau;
                expand_node->time =  pro_t;
              }
              break;
            }
          }

          /* ---------- new neighbor in this loop ---------- */
          if (!prune) {
            if (pro_node == NULL) {
              pro_node             = path_node_pool_[use_node_num_];
              pro_node->index      = pro_id;
              pro_node->state      = pro_state;
              pro_node->f_score    = tmp_f_score;
              pro_node->g_score    = tmp_g_score;
              pro_node->input      = um;
              pro_node->duration   = tau;
              pro_node->parent     = cur_node;
              pro_node->node_state = IN_OPEN_SET;
              pro_node->time = pro_t;

              open_set_.push(pro_node);

              expanded_nodes_.insert(pro_id, pro_node);//hash

              tmp_expand_nodes.push_back(pro_node);

              use_node_num_ += 1;
              if (use_node_num_ == allocate_num_) {
                cout << "run out of memory." << endl;
                return NO_PATH;
              }
            } else if (pro_node->node_state == IN_OPEN_SET) {
              if (tmp_g_score < pro_node->g_score) {
                // pro_node->index = pro_id;
                pro_node->state    = pro_state;
                pro_node->f_score  = tmp_f_score;
                pro_node->g_score  = tmp_g_score;
                pro_node->input    = um;
                pro_node->duration = tau;
                pro_node->parent   = cur_node;
                pro_node->time =  pro_t;
              }
            } else {
              cout << "error type in searching: " << pro_node->node_state << endl;
            }
          }
        }
      }
    }
      ros::Time time_2 = ros::Time::now();
      return -2;
}
std::vector<Eigen::Vector3d> hybridAstar_searcher::traj2grids(std::vector<Eigen::Vector3d> pos_xyz){
  Eigen::Vector3i grid1;
  Eigen::Vector3i grid2;
  std::vector<Eigen::Vector3d> grid_path;
  grid1 = gridmapPtr_->posToIndex(pos_xyz[0]);
  grid_path.push_back(gridmapPtr_->indexToPos(grid1));
for(int i=1;i<pos_xyz.size();i++){
    grid2 = gridmapPtr_->posToIndex(pos_xyz[i]);
    if(grid2==grid1) continue;
    else{
      grid1 = grid2;
      grid_path.push_back(gridmapPtr_->indexToPos(grid1));
    }
  }

  return grid_path;
}
vector<Eigen::Vector3d> hybridAstar_searcher::poslist2freegrid(vector<Eigen::Vector3d> pos_list){
  vector<Vector3d> gridpath = traj2grids(pos_list);
  int i = 0;
  int j = 0;
  vector<Vector3d> freepath;
  //Vector3i 
  while(i<gridpath.size()){
    if(gridmapPtr_->getInflateOccupancy(gridpath[i])){
      i++;
    }
    else{
      break;
    }
  }

  bool init_flag=false;
  int last=i;
  while(i<gridpath.size()){
    if(!init_flag) {freepath.push_back(gridpath[i]);
    i++;
    init_flag = 1;
    continue;
    }
    if(!gridmapPtr_->getInflateOccupancy(gridpath[i])){
      Vector3i dir = gridmapPtr_->posToIndex(gridpath[i])-gridmapPtr_->posToIndex(gridpath[last]);
      int dir_x = dir[0];
      int dir_y = dir[1];
      int dir_z = dir[2]; 
      int flag_x0=dir_x==0;
      int flag_y0=dir_y==0;
      int flag_z0=dir_z==0;
      if(flag_x0+flag_z0+flag_y0==2){
        freepath.push_back(gridpath[i]);
        last = i;
        i++;
      }
      else{
          GraphSearch->search(gridpath[last],gridpath[i]);
          vector<Eigen::Vector3d> Astar_path = GraphSearch->getPath();
          for(int k = 0;k<Astar_path.size();k++){
            freepath.push_back(Astar_path[k]);
          }
          GraphSearch->reset();
          last = i;
          i++; 
      }
    }
    else{
      Vector3d start_pt = gridpath[i-1];
       j = i;
      while(j<gridpath.size()){
        if(gridmapPtr_->getInflateOccupancy(gridpath[j])){
          j++;
        }
        else break;
      }
      if(j==gridpath.size()){
        return freepath;
      }
      Vector3d end_pt  = gridpath[j];
      GraphSearch->search(start_pt,end_pt);
      vector<Vector3d> Astar_path = GraphSearch->getPath();
      for(int k = 1;k<Astar_path.size();k++){
        freepath.push_back(Astar_path[k]);
      }
      GraphSearch->reset();
      last = j;
      i = j+1;

    }
  }

  return freepath;
}


void hybridAstar_searcher::reset(){
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}
vector<Eigen::Vector3d> hybridAstar_searcher::occ_gridpath(vector<Eigen::Vector3d> predict_gridpath,Eigen::Vector3d start){
    vector<Eigen::Vector3d> traj_gridpath;
    if(predict_gridpath.size()<1){
        ROS_ERROR("predict grid path is emtpy!");
    }
    GraphSearch->search(start,predict_gridpath[0]);
    vector<Vector3d> gridpath1 = GraphSearch->getPath();
    GraphSearch->reset();
    for(int i = 0;i<gridpath1.size();i++){
        traj_gridpath.push_back(gridpath1[i]);
    }
    for(int i = 1;i<predict_gridpath.size();i++){
        traj_gridpath.push_back(predict_gridpath[i]);
    }
    return traj_gridpath;
}

bool hybridAstar_searcher::corridor_has(vector<Eigen::Vector3d> gridpath){
    Vector3d Start_point = gridpath[0];
    Vector3d End_point = gridpath[gridpath.size()-1];
    if(gridmapPtr_->getInflateOccupancy(Start_point)||gridmapPtr_->getInflateOccupancy(End_point))
      return 0;
    Vector3d cube_point1,cube_point2;
    int index1,index2;
    index1 = index2 = 0;
    int path_len = gridpath.size();
    bool suc_flag=0;
    while(true)
    {   
      int i;
      for(i = index2;i<path_len;i++){
        cube_point1 = gridpath[index2];
        cube_point2 = gridpath[i];
        Vector3i grid1 = gridmapPtr_->posToIndex(cube_point1);
        Vector3i grid2 = gridmapPtr_->posToIndex(cube_point2);  
        int x_min,x_max,y_min,y_max,z_min,z_max;
        x_min  = min(grid1[0],grid2[0]);
        x_max = max(grid1[0],grid2[0]);
        y_min = min(grid1[1],grid2[1]);
        y_max = max(grid1[1],grid2[1]);
        z_min = min(grid1[2],grid2[2]);
        z_max = max(grid1[2],grid2[2]);
        int safe_flag = 1;
        for(int j=x_min;j<=x_max;j++){
          for(int k=y_min;k<=y_max;k++){
            for(int l=z_min;l<=z_max;l++){
              if(gridmapPtr_->getInflateOccupancy(gridmapPtr_->indexToPos(Eigen::Vector3i(j,k,l)))){
                safe_flag=0;
                break;
              }
            }
          }
        } 
       if(!safe_flag)
        break;                      
      }
      index1 = index2;
      index2 = i-1;
      if(index2>=path_len-1) return 1;
      else if(index1==index2){
        return 0;
      }
    }
}