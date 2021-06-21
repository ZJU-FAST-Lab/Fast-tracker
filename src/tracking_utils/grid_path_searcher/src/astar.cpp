

#include <grid_path_searcher/astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;


Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic, double time_start) {
  /* ---------- initialize ---------- */
  if(!has_map){
    ROS_ERROR("Astar: no map!");
  }
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->index = posToIndex(start_pt);
  cur_node->position = IndexTopos(cur_node->index);
  cur_node->g_score = 0.0;
  Eigen::Vector3i end_index;
  double time_to_goal;
  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic) {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  } else
    expanded_nodes_.insert(cur_node->index, cur_node);

  NodePtr neighbor = NULL;
  NodePtr terminate_node = NULL;
  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    /* ---------- get lowest f_score node ---------- */
    cur_node = open_set_.top();
    // cout << "pos: " << cur_node->state.head(3).transpose() << endl;
    // cout << "time: " << cur_node->time << endl;
    // cout << "dist: " <<
    // edt_environment_->evaluateCoarseEDT(cur_node->state.head(3),
    // cur_node->time) <<
    // endl;

    /* ---------- determine termination ---------- */

    // bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
    //     abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
    bool reach_end = (cur_node->index==end_index);
    if (reach_end) {
      // cout << "[Astar]:---------------------- " << use_node_num_ << endl;
      // cout << "use node num: " << use_node_num_ << endl;
      // cout << "iter num: " << iter_num_ << endl;
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;

      return REACH_END;
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init neighbor expansion ---------- */

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d pro_pos;
    double pro_t;

    vector<Eigen::Vector3d> inputs;
    Eigen::Vector3d d_pos;

    /* ---------- expansion loop ---------- */
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          d_pos << dx, dy, dz;
          if (d_pos.norm() < 1e-3) continue;
          if(d_pos.norm()>resolution_) continue;
          

          pro_pos = cur_pos + d_pos;

          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_3d_(0) || pro_pos(1) <= origin_(1) ||
              pro_pos(1) >= map_size_3d_(1) || pro_pos(2) <= origin_(2) ||
              pro_pos(2) >= map_size_3d_(2)) {
            // cout << "outside map" << endl;
            continue;
          }

          /* not in close set */
          Eigen::Vector3i pro_id = posToIndex(pro_pos);
          // int pro_t_id = timeToIndex(pro_t);
          // ROS_INFO("HHHHHHHHHHHHHHHHHHHHHHHHH");

          // NodePtr pro_node =
          //     dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
          NodePtr pro_node = expanded_nodes_.find(pro_id);
          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {           
            continue;
          }

          /* collision free */
          // double dist = dynamic ?
          // edt_environment_->evaluateCoarseEDT(pro_pos, cur_node->time + dt) :
          //                         edt_environment_->evaluateCoarseEDT(pro_pos,
          //                         -1.0);
        //   double dist = edt_environment_->evaluateCoarseEDT(pro_pos, -1.0);
        //   if (dist <= margin_) {
        //     continue;
        //   }
        if(edt_environment_->getInflateOccupancy(pro_pos))
            continue;

          /* ---------- compute cost ---------- */
          double time_to_goal, tmp_g_score, tmp_f_score;
          tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
          tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);
          // ROS_INFO("IIIIIIIIIIIIIIIIIIIIIIIIII");
          if (pro_node == NULL) {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->position = pro_pos;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic) {
              pro_node->time = cur_node->time + 1.0;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              std::cout << "run out of memory." << std::endl;
              return NO_PATH;
            }
          } else if (pro_node->node_state == IN_OPEN_SET) {
            if (tmp_g_score < pro_node->g_score) {
              // pro_node->index = pro_id;
              pro_node->position = pro_pos;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->parent = cur_node;
              if (dynamic) pro_node->time = cur_node->time + 1.0;
            }
          } else {
            std::cout << "error type in searching: " << pro_node->node_state << endl;
          }

          /* ----------  ---------- */
        }
  }

  /* ---------- open set empty, no path ---------- */
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void Astar::setParam(ros::NodeHandle& nh) {
  nh.param("astar/resolution_astar", resolution_, 0.1);
  nh.param("astar/time_resolution", time_resolution_, 1.0);
  nh.param("astar/lambda_heu", lambda_heu_, 1.0);
  nh.param("astar/allocate_num", allocate_num_, 100000);
  tie_breaker_ = 1.0 + 1.0 / 10000;
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

void Astar::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath() {
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < path_nodes_.size(); ++i) {
    path.push_back(path_nodes_[i]->position);
  }
  return path;
}

double Astar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  double h;
  int diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double Astar::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  return tie_breaker_ * (x2 - x1).norm();
}



void Astar::setEnvironment(const GridMap::Ptr& env) {
  this->edt_environment_ = env;
  has_map = true;
  this->edt_environment_->getRegion(origin_, map_size_3d_);
  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;
}

void Astar::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++) {
    NodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

std::vector<NodePtr> Astar::getVisitedNodes() {
  vector<NodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt) {
  Vector3i idx = edt_environment_->posToIndex(pt);
  return idx;
}
Eigen::Vector3d Astar::IndexTopos(Eigen::Vector3i index){
  Vector3d pt = edt_environment_->indexToPos(index);
  return pt;
}

int Astar::timeToIndex(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);
}


