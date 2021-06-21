#ifndef _KINOASTAR_H
#define _KINOASTAR_H
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
#include <plan_env/grid_map.h>
#include <grid_path_searcher/astar.h>

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;
class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class hybridAstar_searcher
{
    private:
        /*record data*/
        int use_node_num_, iter_num_;
        Eigen::Vector3d start_vel_, end_vel_, start_acc_;
        Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
        bool is_shot_succ_ = false;
        Eigen::MatrixXd coef_shot_;
        double t_shot_;
        bool has_path_ = false;

        /*main data structure*/
        std_msgs::Float64 total_time; 
        std::vector<PathNodePtr> path_node_pool_;
        NodeHashTable expanded_nodes_;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
        open_set_;
        std::vector<PathNodePtr> path_nodes_;
        PathNodePtr terminatePtr;

        /*main param*/
        double max_tau_ = 0.25;
        double init_max_tau_ = 0.8;
        double max_vel_ = 3.0;
        double max_acc_ = 3.0;
        double w_time_ = 10.0;
        double w_sim = 1;
        double lambda_heu_ = 10;
        int check_num_ = 10;
        double w_predict = 0.5;
        double tie_breaker_ = 1.0 + 1.0 / 10000;
        int allocate_num_ = 100000;

        /*map*/
        GridMap::Ptr gridmapPtr_;
        bool has_map_ = false;
        double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
        Eigen::Vector3d origin_, map_size_3d_;
        double time_origin_;
        /*a star search*/
        Astar* GraphSearch;


        /*calculation function*/
        vector<double> cubic(double a, double b, double c, double d);
        vector<double> quartic(double a, double b, double c, double d, double e);
        bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
        double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
        /* state propagation */
        void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                    Eigen::Vector3d um, double tau);
    public:
        hybridAstar_searcher(){
            phi_          = Eigen::MatrixXd::Identity(6, 6);
            use_node_num_ = 0;
            iter_num_     = 0;
            srand((unsigned)time(NULL));
            GraphSearch = new Astar(); 
        };
        ~hybridAstar_searcher(){
          delete GraphSearch;
        };
        /* shot trajectory */
        vector<Eigen::Vector3d> getKinoTraj(double delta_t); 
        /*API*/
        void setParam(ros::NodeHandle& nh);
        int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v,vector<Eigen::Matrix<double,6,1>> predict_list,
                    double time_interval); 
        vector<Eigen::Vector3d> traj2grids(vector<Eigen::Vector3d> pos_xyz);
        vector<Eigen::Vector3d> poslist2freegrid(vector<Eigen::Vector3d> pos_list);
        vector<Eigen::Vector3d> occ_gridpath(vector<Eigen::Vector3d> predict_gridpath,Eigen::Vector3d start);
        bool corridor_has(vector<Eigen::Vector3d> gridpath);
        void setEnvironment(const GridMap::Ptr& env);
        void reset();
        std_msgs::Float64 get_time(){
            std_msgs::Float64 time;
            time.data = terminatePtr->time;
            return time;    
        }
        vector<Eigen::Vector3d> get_endva(){
            vector<Eigen::Vector3d> endva;
            endva.push_back(terminatePtr->state.tail(3));
            endva.push_back(terminatePtr->input);
            return endva;
        }
        
};
#endif

