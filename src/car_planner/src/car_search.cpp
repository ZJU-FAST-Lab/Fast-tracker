#include <car_planner/car_search.h>
using namespace car_planner;
Car_KinoSearch::Car_KinoSearch(ros::NodeHandle& nh){
    nh.param("car_search/tau",tau,0.1);
    nh.param("car_seach/velocity",vel,2.0);
    nh.param("car_search/lambda_heu",lambda_heu_,1.0);//1.0 is the equal number level
    nh.param("car_search/allocate_num", allocate_num_, 100000);
    nh.param("car_search/check_num",check_num_,10);
    nh.param("car_search/car_l",car_l,0.6);
    nh.param("car_search/car_w",car_w,0.4);
    nh.param("car_search/car_h",car_h,0.3);
    nh.param("car_search/max_steer",max_steer,0.7);
    nh.param("car_search/wheelbase",wheelbase,0.6);
    nh.param("car_search/yaw_resolution",yaw_resolution,0.17453);
    nh.param("car_search/check_dt",check_dt,0.05);
    nh.param("car_search/frame",frame,std::string("world"));
    nh.param("car_search/resolution",resolution_,0.1);
    nh.param("map/z_size",map_size_3d_(2),2.0);
    nh.param("map/y_size",map_size_3d_(1),100.0);
    nh.param("map/x_size",map_size_3d_(0),100.0);
    origin_[0] = -map_size_3d_(0)/2;
    origin_[1] = -map_size_3d_(1)/2;
    origin_[2] = -0.1;
    dim_(0) = map_size_3d_(0)/resolution_;
    dim_(1) = map_size_3d_(1)/resolution_;
    dim_(2) = map_size_3d_(2)/resolution_;
    buffer_size = dim_(0) * dim_(1) * dim_(2);
    map_buffer.resize(buffer_size);
    //
    /*
    car model:
    O----O
    |----|
    |----|                                                                                                                                  
    O----O
    */
    kinosearchPub = nh.advertise<visualization_msgs::MarkerArray>("/visualization/car_kinosearch", 1);
    point_cloud_sub_ = nh.subscribe("global_map", 10, &Car_KinoSearch::GlobalMapBuild, this);
    MinTurnRadius = wheelbase/std::tan(max_steer);
    tie_breaker_ = 1.0+1.0/10000;
    // gridMapPtr_ = env;
    inv_resolution_ = 1.0 / resolution_;
    inv_time_resolution_ = 1.0 / time_resolution_;
    // gridMapPtr_->getRegion(origin_, map_size_3d_);
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++) {
      path_node_pool_[i] = new PathNode;
    }
    shotptr =std::make_shared<ompl::base::ReedsSheppStateSpace>(MinTurnRadius);
    use_node_num_ = 0;
    iter_num_ = 0;
}
                     
// void setObs(Eigen::Vector3d pt);
//   bool isOutside(Eigen::Vector3i idx);
//   int getindex(Eigen::Vector3i idx);
//   bool isOccupied(Eigen::Vector3i idx);
//   Eigen::Vector3d intToFloat(const Eigen::Vector3i idx);
//   Eigen::Vector3i FloatToint(const Eigen::Vector3d pt);
void Car_KinoSearch::GlobalMapBuild(const sensor_msgs::PointCloud2 & pointcloud_map){
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
    // ROS_WARN("Sim Detection Node: Finish gridmap built");
}
void Car_KinoSearch::setObs(Eigen::Vector3d pt){
    int expand_size = 1;
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
bool Car_KinoSearch::isOutside(Eigen::Vector3i idx){
    for(int i = 0; i <3; i++)
          if (idx(i) < 0 || idx(i) >= dim_(i))
            return true;
        return false;
}
int Car_KinoSearch::getIndex(Eigen::Vector3i idx){
    return idx(0) + dim_(0) * idx(1) + dim_(0) * dim_(1) * idx(2);
}
bool Car_KinoSearch::isOccupied(Eigen::Vector3i idx){
    if(isOutside(idx)) {
    // ROS_INFO_STREAM("IDX: "<<idx );
    return true;
    }
    return map_buffer[getIndex(idx)];
}
Eigen::Vector3d Car_KinoSearch::intToFloat(const Eigen::Vector3i idx){
    return (idx.template cast<double>() + Eigen::Vector3d::Constant(0.5)) * resolution_ + origin_;
}
Eigen::Vector3i Car_KinoSearch::FloatToint(const Eigen::Vector3d pt){
    Eigen::Vector3i pn;
    for(int i = 0; i < 3; i++)
          pn(i) = std::round((pt(i) - origin_(i)) / resolution_ - 0.5);
    return pn;
}

double Car_KinoSearch::estimateHeuristic(Eigen::Vector3d x1,Eigen::Vector3d x2){
    double dx = x1[0]-x2[0];
    double dy = x1[1]-x2[1];
    Eigen::Vector3d d(dx,dy,0); 
    return 1.0 * (1 + tie_breaker_)*d.norm();

}
Eigen::Vector3d Car_KinoSearch::ExpandNeighbor(Eigen::Vector3d& state,double vel,double dt,SUBPATH_TYPE type,double& cost){
    Eigen::Vector3d TranState;
    switch (type)
    {
        case STRAIGHT_FORWORD:{
            stateTransit(state,TranState,vel,dt,0);
            cost = vel*dt;
            break;
        }
        case STRAIGHT_BACKWORD:{
            stateTransit(state,TranState,-vel,dt,0);
            cost = backward_penalty_factor*vel*dt;
            break;
        }
        case LEFT_FORWORD:{
            stateTransit(state,TranState,vel,dt,max_steer);
            cost = turn_penalty_factor*vel*dt;
            break;
        }
        case LEFT_BACKWARD:{
            stateTransit(state,TranState,-vel,dt,max_steer);
            cost = backward_penalty_factor*turn_penalty_factor*vel*dt;
            break;
        }
        case RIGHT_FORWORD:{
            stateTransit(state,TranState,vel,dt,-max_steer);
            cost = turn_penalty_factor*vel*dt;
            break;
        }
        case RIGHT_BACKWARD:{
            stateTransit(state,TranState,-vel,dt,-max_steer);
            cost = backward_penalty_factor*turn_penalty_factor*vel*dt;
            break;
        }
        default:{
            ROS_ERROR("Wrong Car Search Type!");
            break;
        }
    }
    return TranState;
}
void Car_KinoSearch::stateTransit(Eigen::Vector3d& state0,
                    Eigen::Vector3d& state1,
                    double  vel,
                    double dt,
                    double steer){
    double yaw = state0[2];
    if(steer!=0){
        double k = vel/wheelbase*tan(steer);//k=0?
        state1[0] = state0[0]+vel*sin(yaw+k*dt)/k-vel*sin(yaw)/k;
        state1[1] = state0[1]+vel*cos(yaw)/k-vel*cos(yaw+k*dt)/k;
        state1[2] = state0[2]+k*dt;
        state1[2] = normalizeTheta(state1[2]);
    }
    else{
        state1[0] = state0[0]+vel*dt*cos(yaw);
        state1[1] = state0[1]+vel*dt*sin(yaw);
        state1[2] = yaw;
        state1[2] = normalizeTheta(state1[2]);
    }
}
void Car_KinoSearch::stateTransit(Eigen::Vector3d& state0,
                    Eigen::Vector3d& state1,
                    SUBPATH_TYPE  type,
                    double dt){
    switch (type)
    {
        case STRAIGHT_FORWORD:{
            stateTransit(state0,state1,vel,dt,0);
            // ROS_INFO_STREAM("STRAIGHT_FORWORD "<<state0[2]);
            break;
        }
        case STRAIGHT_BACKWORD:{
            stateTransit(state0,state1,-vel,dt,0);
            // ROS_INFO_STREAM("STRAIGHT_BACKWORD "<<state0[2]);
            break;
        }
        case LEFT_FORWORD:{
            stateTransit(state0,state1,vel,dt,max_steer);
            // ROS_INFO_STREAM("LEFT_FORWORD "<<state0[2]);
            break;
        }
        case LEFT_BACKWARD:{
            stateTransit(state0,state1,-vel,dt,max_steer);
            // ROS_INFO_STREAM("LEFT_BACKWARD "<<state0[2]);
            break;
        }
        case RIGHT_FORWORD:{
            stateTransit(state0,state1,vel,dt,-max_steer);
            // ROS_INFO_STREAM("RIGHT_FORWORD "<<state0[2]);
            break;
        }
        case RIGHT_BACKWARD:{
            stateTransit(state0,state1,-vel,dt,-max_steer);
            // ROS_INFO_STREAM("RIGHT_BACKWARD "<<state0[2]);
            break;
        }
        default:{
            ROS_ERROR("Wrong Car Search Type!");
            break;
        }
    }
}
bool Car_KinoSearch::is_collision(Eigen::Vector3d state){
    Eigen::Vector3d p = Eigen::Vector3d(state[0],state[1],0);
    Eigen::Matrix3d rota_M;
    double yaw = state[2];
    rota_M  <<  cos(yaw),-sin(yaw),0,
                sin(yaw),cos(yaw),0,
                0       ,0       ,1; 
    for(double w = -car_w/2;w<=car_w/2;w+=resolution_/2){
        for(double l = 0;l<=car_l;l+=resolution_/2){
            for(double h = 0;h<=car_h;h+=resolution_/2){
                Eigen::Vector3d cur_p = p +rota_M*Eigen::Vector3d(l,w,h);
                // ROS_INFO_STREAM("CUR_P: "<<cur_p);
                if(isOccupied(FloatToint(cur_p)))
                    return true; 
            }
        }
    }
    return false;
}
void Car_KinoSearch::reset() {
    expanded_nodes_.clear();
    path_nodes_.clear();
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
        empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++) {
      PathNodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
    has_path_ = false;
  }
bool Car_KinoSearch::is_shot_sucess(Eigen::Vector3d state1,Eigen::Vector3d state2){
    std::vector<Eigen::Vector3d> path_list;
    double len,st;
    st = computeShotTraj(state1,state2,path_list,len,check_dt);
    for(int i=0;i<path_list.size();i++){
        if(is_collision(path_list[i]))
            return false;
    }
    is_shot_succ_ = true;
    shot_T = st;
    return true;
}
double Car_KinoSearch::computeShotTraj(Eigen::Vector3d state1, Eigen::Vector3d state2,std::vector<Eigen::Vector3d> &path_list,
                                       double& len,double dt){
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    ob::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
    from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
    to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
    std::vector<double> reals;
    len = shotptr->distance(from(), to());
    double sum_T = len/vel;    
    double i;
    for (i=0; i<=sum_T; i+=dt){
        shotptr->interpolate(from(), to(), i/sum_T, s());
        reals = s.reals();
        path_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
    }
    if(i!=sum_T){
        shotptr->interpolate(from(), to(), 1.0, s());
        reals = s.reals();
        path_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
    }
    return sum_T;
}
Eigen::Vector3i Car_KinoSearch::stateToindex(Eigen::Vector3d state){
    double px,py,yaw;
    px = state[0];
    py = state[1];
    yaw = normalizeTheta(yaw);
    Eigen::Vector3d dex = Eigen::Vector3d((px-origin_[0])* inv_resolution_,(py-origin_[1])* inv_resolution_,
    (yaw + M_PI)/yaw_resolution);
    return dex.array().floor().cast<int>();
}
double Car_KinoSearch::normalizeTheta(double yaw){
    if (yaw >= -M_PI && yaw < M_PI)
        return yaw;
    double multiplier = std::floor(yaw / (2*M_PI));
    yaw = yaw - multiplier*2*M_PI;
    if (yaw >= M_PI)
        yaw -= 2*M_PI;
    if (yaw < -M_PI)
        yaw += 2*M_PI;
    return yaw;
}
void Car_KinoSearch::retrievePath(PathNodePtr end_node) {
    PathNodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL) {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
  }

std::vector<Eigen::Vector3d> Car_KinoSearch::getKinoTraj(double delta_t){
    // stateTransit
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    std::vector<Eigen::Vector3d> state_list;
    /* ---------- get traj of searching ---------- */
    PathNodePtr node = path_nodes_.back();
    Eigen::Vector3d x0, xt;
    while (node->parent != NULL) {
      SUBPATH_TYPE ut = node->input;
      double duration = node->duration;
      x0 = node->parent->state;

      for (double t = duration; t >= -1e-5; t -= delta_t) {
        stateTransit(x0, xt, ut,t);
        state_list.push_back(xt);
      }
      node = node->parent;
    }
    state_list.push_back(path_nodes_.front()->state);
    reverse(state_list.begin(), state_list.end());
    /* ---------- get traj of one shot ---------- */
    if (is_shot_succ_) {
        namespace ob = ompl::base;
        namespace og = ompl::geometric;
        ob::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
        Eigen::Vector3d state1,state2;
        state1 = path_nodes_.back()->state;
        state2 = goal_state;
        from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
        to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
        std::vector<double> reals;
        double len = shotptr->distance(from(), to());
        double sum_T = len/vel;    
        double i;
        for (i=delta_t; i<=sum_T; i+=delta_t){
            shotptr->interpolate(from(), to(), i/sum_T, s());
            reals = s.reals();
            state_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
        }
        if(i!=sum_T){
            shotptr->interpolate(from(), to(), 1.0, s());
            reals = s.reals();
            state_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
        }

    }
    return state_list;
}
Eigen::Vector3d Car_KinoSearch::evaluate_state(double time){
    PathNodePtr node = path_nodes_.back();
    if(time<0){
        ROS_ERROR("Wrong time!");
        return Eigen::Vector3d(-1,-1,-1);
    }
    if(node->time<=time){
        //shot
        double delta_time = time - node->time;
        if(!is_shot_succ_)
            return goal_state;
        else{
            namespace ob = ompl::base;
            namespace og = ompl::geometric;
            ob::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
            Eigen::Vector3d state1,state2;
            state1 = node->state;
            state2 = goal_state;
            from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
            to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
            std::vector<double> reals;
            double len = shotptr->distance(from(), to());
            double sum_T = len/vel;    
            if(delta_time>sum_T) return goal_state;
            else{
                shotptr->interpolate(from(), to(), delta_time/sum_T, s());
                reals = s.reals();
                return Eigen::Vector3d(reals[0], reals[1], reals[2]);        
            }
        }
    }
    else{
        //locate primitive
        Eigen::Vector3d x0, xt;
        while (node->parent != NULL) {
            double time1 = node->parent->time;
            double time2 = node->time;
            if(time<time2&&time>=time1){
                //return
                double delta_t = time-time1;
                x0 = node->parent->state;
                // ROS_INFO("EVALUATE!");
                // ROS_INFO_STREAM("dt: "<<delta_t);
                stateTransit(x0,xt,node->input,delta_t);
                // ROS_INFO_STREAM("X0: "<<x0.transpose());
                // ROS_INFO_STREAM("X1: "<<xt.transpose());
                // ROS_INFO("-------------------------------");
                return xt;
            }
            else{
                node = node->parent;        
            }
        }
    }
}
void Car_KinoSearch::visualize(double dt){
    std::vector<Eigen::Vector3d> path_list = getKinoTraj(dt);
    visualization_msgs::MarkerArray car_traj;
    Eigen::Vector3d pos;
    Eigen::Matrix3d rota_M;
    for (int i = 0; i < path_list.size(); i++)
    {
        double yaw = path_list[i][2];
        rota_M  <<  cos(yaw),-sin(yaw),0,
                sin(yaw),cos(yaw),0,
                0       ,0       ,1; 
        double x = path_list[i][0],y=path_list[i][1],z=0;
        pos = Eigen::Vector3d(x,y,z)+rota_M*(Eigen::Vector3d(car_l/2,0,car_h/2));
        Eigen::Quaterniond q(rota_M);
        visualization_msgs::Marker WpMarker;
        WpMarker.id = i;
        WpMarker.header.stamp = ros::Time::now();
        WpMarker.header.frame_id = frame;
        WpMarker.action = visualization_msgs::Marker::ADD;
        WpMarker.type = visualization_msgs::Marker::CUBE;
        WpMarker.ns = "car_kinoseach";
        WpMarker.color.r = 1.0;
        WpMarker.color.g = 0;
        WpMarker.color.b = 0;
        WpMarker.color.a = 0.5;
        WpMarker.scale.x = car_l;
        WpMarker.scale.y = car_w;
        WpMarker.scale.z = car_h;
        WpMarker.pose.orientation.w = q.w();
        WpMarker.pose.orientation.x = q.x();
        WpMarker.pose.orientation.y = q.y();
        WpMarker.pose.orientation.z = q.z();
        WpMarker.pose.position.x = pos[0];
        WpMarker.pose.position.y = pos[1];
        WpMarker.pose.position.z = pos[2];
        car_traj.markers.push_back(WpMarker);
    }

    kinosearchPub.publish(car_traj);
}
double Car_KinoSearch::get_totalT(){
    if(!has_path_)
        ROS_ERROR("no path!");
    if(!is_shot_succ_)
        total_T = path_nodes_.back()->time;
    else
        total_T = path_nodes_.back()->time+shot_T;
    return total_T;
}
int Car_KinoSearch::car_search(Eigen::Vector3d start_state,Eigen::Vector3d end_state){
    /* ---------- initialize ---------- */
    if(!has_map) ROS_ERROR("map is not built for car!");
    start_state[2] = normalizeTheta(start_state[2]);
    end_state[2] = normalizeTheta(end_state[2]);
    goal_state = end_state;
    PathNodePtr cur_node    = path_node_pool_[0];
    cur_node->parent        = NULL;
    cur_node->state = start_state;
    cur_node->index         = stateToindex(start_state);
    cur_node->g_score       = 0.0;
    cur_node->f_score    = lambda_heu_ * estimateHeuristic(start_state,end_state);
    cur_node->node_state = IN_OPEN_SET;
    cur_node->time = 0;

    Eigen::Vector3i end_index            = stateToindex(end_state);

    open_set_.push(cur_node);
    use_node_num_ += 1;

    expanded_nodes_.insert(cur_node->index, cur_node);
    PathNodePtr neighbor       = NULL;
    PathNodePtr terminate_node = NULL;
    const int   tolerance      = 4*ceil(1 / resolution_);
    /* ---------- search loop ---------- */
    while (!open_set_.empty()) {
        /* ---------- get lowest f_score node ---------- */
        cur_node = open_set_.top();
        if(is_collision(cur_node->state)){
            ROS_WARN("Car kinodynamic search failed, start is not free!");
        return NO_PATH;
        } 
        /* ---------- determine termination ---------- */

        bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
            abs(cur_node->index(1) - end_index(1)) <= tolerance;
            // &&
            // (abs(cur_node->state[2] - end_state[2]) <= 5*yaw_resolution 
            // || 2*M_PI-abs(cur_node->state[2] - end_state[2])<=5*yaw_resolution);
        if (near_end) {
        std::cout << "[Car Kino Astar]:---------------------- " << use_node_num_ << std::endl;
        std::cout << "use node num: " << use_node_num_ << std::endl;
        std::cout << "iter num: " << iter_num_ << std::endl;
        terminate_node = cur_node;
        retrievePath(terminate_node);
        has_path_ = true;
        std::cout << "[Kino Astar]: near end." << std::endl;
        /* one shot trajectory */
        estimateHeuristic(cur_node->state, end_state);
        is_shot_sucess(cur_node->state,end_state);
        if (!is_shot_succ_)
            return NO_PATH;
        else{

            return REACH_END;
        }
        }
        // else{
        //     is_shot_sucess(cur_node->state,end_state);
        //     if(is_shot_succ_){
        //         terminate_node = cur_node;
        //         retrievePath(terminate_node);
        //         has_path_ = true;
        //         cout << "[Kino Astar]: near end." << endl;
        //         return REACH_END;
        //     }
        // }
        /* ---------- pop node and add to close set ---------- */
        open_set_.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num_ += 1;
        /* ---------- state propagation ---------- */
        Eigen::Vector3d cur_state = cur_node->state;
        Eigen::Vector3d pro_state;
        std::vector<PathNodePtr>         tmp_expand_nodes;
        double                      pro_t;
        double                      pro_cost;
        std::vector<SUBPATH_TYPE> types = {LEFT_FORWORD, STRAIGHT_FORWORD, RIGHT_FORWORD,
                 LEFT_BACKWARD, STRAIGHT_BACKWORD, RIGHT_BACKWARD};
        for (const auto& type: types) {
            pro_state = ExpandNeighbor(cur_state,vel,tau,type,pro_cost);
            pro_t = cur_node->time + tau;
            /* ---------- check if in free space ---------- */
            /* inside map range */
            if(is_collision(pro_state)){
                continue;
            }
            /* not in close set */
            Eigen::Vector3i pro_id   = stateToindex(pro_state);
            PathNodePtr pro_node = expanded_nodes_.find(pro_id);
            if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
            continue;
            }
            // /* not in the same voxel */
            // Eigen::Vector3i diff      = pro_id - cur_node->index;
            // // int             diff_time = pro_t_id - cur_node->time_idx;
            // if (diff.norm() == 0) {
            // // cout<<"in the same voxel";
            // continue;
            // }
            /* collision free */
            Eigen::Vector3d             xt;
            bool                        is_occ = false;

            for (int k = 0; k <= check_num_; ++k) {
            double dt = tau * double(k) / double(check_num_);
            double tmp_cost;
            xt = ExpandNeighbor(cur_state,vel,dt,type,tmp_cost);
            if (is_collision(xt)) {
                is_occ = true;
                break;
            }
            }

            if (is_occ) {
            continue;
            }
            /* ---------- compute cost ---------- */
            double time_to_goal, tmp_g_score, tmp_f_score;
            tmp_g_score = pro_cost + cur_node->g_score;
            tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state);
            /* ---------- compare expanded node in this loop ---------- */

            bool prune = false;
            for (int j = 0; j < tmp_expand_nodes.size(); ++j) {
            PathNodePtr expand_node = tmp_expand_nodes[j];
            if ((pro_id - expand_node->index).norm() == 0) {

                prune = true;

                if (tmp_f_score < expand_node->f_score) {
                expand_node->f_score  = tmp_f_score;
                expand_node->g_score  = tmp_g_score;
                expand_node->state    = pro_state;
                expand_node->input    = type;
                expand_node->duration = tau;
                expand_node->time = pro_t;
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
                pro_node->input      = type;
                pro_node->duration   = tau;
                pro_node->parent     = cur_node;
                pro_node->node_state = IN_OPEN_SET;
                pro_node->time = pro_t;

                open_set_.push(pro_node);
                expanded_nodes_.insert(pro_id, pro_node);//hash

                tmp_expand_nodes.push_back(pro_node);

                use_node_num_ += 1;
                if (use_node_num_ == allocate_num_) {
                std::cout << "run out of memory." <<std::endl;
                return NO_PATH;
                }
            }     
            else if (pro_node->node_state == IN_OPEN_SET) {
                if (tmp_g_score < pro_node->g_score) {
                // pro_node->index = pro_id;
                pro_node->state    = pro_state;
                pro_node->f_score  = tmp_f_score;
                pro_node->g_score  = tmp_g_score;
                pro_node->input    = type;
                pro_node->duration = tau;
                pro_node->parent   = cur_node;
                pro_node->time = pro_t;
                }
            } else {
                std::cout << "error type in searching: " << pro_node->node_state << std::endl;
            }
            }
            /* ----------  ---------- */
        }
    }
    /* ---------- open set empty, no path ---------- */
    std::cout << "open set empty, no path!" << std::endl;
    std::cout << "use node num: " << use_node_num_ << std::endl;
    std::cout << "iter num: " << iter_num_ << std::endl;
    return NO_PATH;
}
