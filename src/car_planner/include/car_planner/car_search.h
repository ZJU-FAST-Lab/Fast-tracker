#ifndef _CAR_SEARCH_H
#define _CAR_SEARCH_H
#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
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

namespace car_planner {
enum { REACH_HORIZON = 1, NO_PATH = 2,REACH_END = 3};
enum SUBPATH_TYPE{
        LEFT_FORWORD,
        STRAIGHT_FORWORD,
        RIGHT_FORWORD,
        LEFT_BACKWARD,
        STRAIGHT_BACKWORD,
        RIGHT_BACKWARD
};
static constexpr char IN_CLOSE_SET = 'a';
static constexpr char IN_OPEN_SET = 'b';
static constexpr char NOT_EXPAND = 'c';
static constexpr double inf = 1 >> 30;

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Vector3d state;//px py yaw
  double g_score, f_score;
  SUBPATH_TYPE input;
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

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
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

class Car_KinoSearch {
 private:
  /* ---------- main data structure ---------- */
  std::vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  bool is_shot_succ_ = false;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double tau,vel;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double check_dt;
  double tie_breaker_;
  bool optimistic_;
  ompl::base::StateSpacePtr shotptr;
  Eigen::Vector3d goal_state;
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  double time_origin_;
  double yaw_resolution;
  double total_T;
  double shot_T;
  /* car param*/
  double car_l,car_w,car_h;
  double max_steer;
  double wheelbase;
  double MinTurnRadius;
  /* penaly param*/
  double turn_penalty_factor = 1.5;
  double backward_penalty_factor = 4.5;
  /* ros relate*/
  ros::Publisher kinosearchPub;
  std::string frame;
  ros::Subscriber point_cloud_sub_;
  /* map */
  Eigen::Vector3d origin_, map_size_3d_,dim_;
  std::vector<signed char> map_buffer;
  int buffer_size;
  bool has_map = false;

  void retrievePath(PathNodePtr end_node);
  double estimateHeuristic(Eigen::Vector3d x1,
                           Eigen::Vector3d x2);
  /* state propagation */
  void stateTransit(Eigen::Vector3d& state0,
                    Eigen::Vector3d& state1,
                    double vel,
                    double dt,
                    double steer);
  void stateTransit(Eigen::Vector3d& state0,
                    Eigen::Vector3d& state1,
                    SUBPATH_TYPE  type,
                    double dt);
  Eigen::Vector3d ExpandNeighbor(Eigen::Vector3d& state,double vel,double dt,SUBPATH_TYPE type, double& cost);
  Eigen::Vector3i stateToindex(Eigen::Vector3d state);
  double normalizeTheta(double yaw);
  /*grid map*/
  void GlobalMapBuild(const sensor_msgs::PointCloud2 & pointcloud_map);
  void setObs(Eigen::Vector3d pt);
  bool isOutside(Eigen::Vector3i idx);
  int getIndex(Eigen::Vector3i idx);
  bool isOccupied(Eigen::Vector3i idx);
  Eigen::Vector3d intToFloat(const Eigen::Vector3i idx);
  Eigen::Vector3i FloatToint(const Eigen::Vector3d pt);

 public:
  Car_KinoSearch(ros::NodeHandle& nh);
  ~Car_KinoSearch() {
    for (int i = 0; i < allocate_num_; i++) {
      delete path_node_pool_[i];
    }
  }

  /* main API */
  void reset();
  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
  double computeShotTraj(Eigen::Vector3d state1, Eigen::Vector3d state2,std::vector<Eigen::Vector3d>& path_list,
                                       double& len,double dt);
  bool is_shot_sucess(Eigen::Vector3d state1,Eigen::Vector3d state2);
  int car_search(Eigen::Vector3d start_state,Eigen::Vector3d end_state);
  void visualize(double dt);
  Eigen::Vector3d evaluate_state(double time);
  inline bool is_collision(Eigen::Vector3d state);
  typedef std::shared_ptr<Car_KinoSearch> Ptr;
  double get_totalT();
};
};
#endif
