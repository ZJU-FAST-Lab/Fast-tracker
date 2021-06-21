#include <plan_manage/plan_manage.h>
plan_manage::plan_manage(ros::NodeHandle& nh){
    env.reset(new GridMap);
    env->initMap(nh);
    kinosearch.setParam(nh);
    kinosearch.setEnvironment(env);
    sfc.set_map(env);   
    nh.param("traj/vmax", v_max, 2.0);//0.6
    nh.param("traj/amax", a_max, 3.0);
    /*
    ros::Subscriber detect_sub,odom_sub;
        ros::Publisher kino_search_pub,traj_vis,pretraj_pub,cor_pub;
    */
   detect_sub = nh.subscribe("target",1,&plan_manage::tg_list_cb,this);
   odom_sub = nh.subscribe("odom",1,&plan_manage::odom_cb,this);
//    kino_search_vispub = nh.advertise<visualization_msgs::Marker>("visualization/vis_hybridAstar_traj", 1);
   traj_vispub = nh.advertise<visualization_msgs::Marker>("visualization/vis_smooth_traj", 1);
   pretraj_vispub = nh.advertise<visualization_msgs::Marker>("visualization/vis_pre_traj", 1);
//    cor_vispub = nh.advertise<visualization_msgs::Marker>("visualization/vis_corridor",1);
   TrackTrajPub =  nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
   replan_timer = nh.createTimer(ros::Duration(1/replan_frequency), &plan_manage::fsm_timer_cb, this);
}
void plan_manage::odom_cb(const nav_msgs::Odometry& odom){
    position<<odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z;
    q.w() = odom.pose.pose.orientation.w;
    q.x() = odom.pose.pose.orientation.x;
    q.y() = odom.pose.pose.orientation.y;
    q.z() = odom.pose.pose.orientation.z;
    q = q.normalized();
    yaw=atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    
}
void plan_manage::tg_list_cb(const nav_msgs::Odometry& car_state){
    static bool initialize = false;
    Eigen::Vector4d state(car_state.pose.pose.position.x,car_state.pose.pose.position.y,1.2,car_state.header.stamp.toSec());
    if(!initialize){
        target_detect_list.push_back(state);        
        if(target_detect_list.size()>=_MAX_SEG){
            initialize=1;
        }            
    }
    else{
        target_detect_list.erase(target_detect_list.begin());
        target_detect_list.push_back(state);        
    }
    last_rcvtime = ros::Time::now().toSec();
    
    
}
// Trajectory plan_manage::optimaltraj_generate(const Eigen::MatrixXd &iniState,
//                    const Eigen::MatrixXd &finState,
//                    const Eigen::MatrixXd &cuboidsParams,
//                    double vmax,
//                    double amax,
//                    const double rh)
//                    {
      
//     AUTO nonlinOpt(rh,vmax,amax);
//     Trajectory traj;
//     static int count=0;
//     double cost;
//     nonlinOpt.setup(iniState,finState,cuboidsParams);//3*3(p,v,a),3*3,double,6*n
//     traj=nonlinOpt.optimize(256,cost);
//     double total_time = traj.getTotalDuration();
//     return traj;
// }
Eigen::Vector3d plan_manage::evaluteP(double t){
    if(t>=last_traj.getTotalDuration())
        t = last_traj.getTotalDuration();
    return last_traj.getPos(t);
}
Eigen::Vector3d plan_manage::evaluteV(double t){
    if(t>=last_traj.getTotalDuration())
        t = last_traj.getTotalDuration();
    return last_traj.getVel(t);
}
Eigen::Vector3d plan_manage::evaluteA(double t){
    if(t>=last_traj.getTotalDuration())
        t = last_traj.getTotalDuration();
    return last_traj.getAcc(t);
}
void plan_manage::visualize_traj(OpTrajectory traj){
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "world";
    _traj_vis.ns = "/tracking_traj";
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.1;
    _traj_vis.scale.y = 0.1;
    _traj_vis.scale.z = 0.1;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;//black
    _traj_vis.id = 0;
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;
    for(double t=0;t<traj.getTotalDuration();t+=0.01){
        pos  = traj.getPos(t);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _traj_vis.points.push_back(pt);
    }
    traj_vispub.publish(_traj_vis);
}
void plan_manage::visualize_relocate(OpTrajectory traj){
    visualization_msgs::Marker _traj_vis;
    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "world";
    _traj_vis.ns = "/tracking_traj";
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.1;
    _traj_vis.scale.y = 0.1;
    _traj_vis.scale.z = 0.1;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;
    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;//red
    _traj_vis.id = 0;
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;
    for(double t=0;t<traj.getTotalDuration();t+=0.01){
        pos  = traj.getPos(t);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _traj_vis.points.push_back(pt);
    }
    traj_vispub.publish(_traj_vis);
}

void plan_manage::visualize_pre(std::vector<Eigen::Vector3d> poslist){
    visualization_msgs::Marker _pred_vis;
    _pred_vis.header.stamp       = ros::Time::now();
    _pred_vis.header.frame_id    = "world";
    _pred_vis.ns = "/tracking_pred";
    _pred_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _pred_vis.action = visualization_msgs::Marker::ADD;
    _pred_vis.scale.x = 0.1;
    _pred_vis.scale.y = 0.1;
    _pred_vis.scale.z = 0.1;
    _pred_vis.pose.orientation.x = 0.0;
    _pred_vis.pose.orientation.y = 0.0;
    _pred_vis.pose.orientation.z = 0.0;
    _pred_vis.pose.orientation.w = 1.0;
    _pred_vis.color.a = 1.0;
    _pred_vis.color.r = 0.0;
    _pred_vis.color.g = 1.0;
    _pred_vis.color.b = 0.0;//black
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;
    for(int i=0;i<poslist.size();i++){
        pos  = poslist[i];
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        _pred_vis.points.push_back(pt);
    }
    pretraj_vispub.publish(_pred_vis);

}
quadrotor_msgs::PolynomialTrajectory plan_manage::traj2msg(OpTrajectory traj){
    static int count=0;
    quadrotor_msgs::PolynomialTrajectory traj_msg;
    traj_msg.header.seq = count;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = std::string("world");
    traj_msg.trajectory_id = count;
    traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj_msg.num_order = traj[0].getOrder(); // the order of polynomial
    traj_msg.num_segment = traj.getPieceNum();
    traj_msg.start_yaw = 0;
    traj_msg.final_yaw = 0;
    // cout << "p_order:" << poly_number << endl;
    // cout << "traj_msg.num_order:" << traj_msg.num_order << endl;
    // cout << "traj_msg.num_segment:" << traj_msg.num_segment << endl;
    for(unsigned int i=0; i<traj_msg.num_segment; i++)
    {
        for (unsigned int j = 0; j <= traj[i].getOrder(); j++)
        {
          Eigen::Matrix<double, 3, 6>  coemat = traj[i].normalizedCoeffMat();
          traj_msg.coef_x.push_back(coemat(0,j));
          traj_msg.coef_y.push_back(coemat(1,j));
          traj_msg.coef_z.push_back(coemat(2,j));
        }
        traj_msg.time.push_back(traj[i].getDuration());
        traj_msg.order.push_back(traj[i].getOrder());
    }
    traj_msg.mag_coeff = 1;
    count++;
    return traj_msg;
}
void plan_manage::fsm_timer_cb(const ros::TimerEvent& event){
    /*record data define*/
    static std::vector<Eigen::Matrix<double,6,1>> predict_state_list;
    static std::vector<Eigen::Vector3d> Sample_list;
    static std::vector<Eigen::Vector3d> Freegridpath;
    static double publish_time;
    static double last_occtime;
    double start_time;
    double rt;
    /*state*/
    Eigen::Matrix3d initState,finState;
    Eigen::Vector3d start_pt,start_vel,start_acc;
    if(target_detect_list.size()<_MAX_SEG) return;
    if(!init_flag){
        initState<<position,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero();
        start_pt = position;
        start_vel = Eigen::Vector3d::Zero();
        start_acc = Eigen::Vector3d::Zero();
    }
    else{
        start_time = ros::Time::now().toSec();
        rt = start_time - publish_time+BUDGET_TIME;
        start_pt = evaluteP(rt);
        start_vel = evaluteV(rt);
        start_acc = evaluteA(rt);
        initState<<start_pt,start_vel,start_acc;
    }
    //FSM TrackTrajPub
    if(env->getInflateOccupancy(start_pt)){
        //emergency!
        fsm = EMERGENCY;
        ROS_INFO("EME");
    }
    else if(ros::Time::now().toSec()-last_rcvtime>0.4){
        //lost target!
        fsm = RELOCATING;
        ROS_INFO("RELOCATE");
    }
    else{
        //normal tracking!
        fsm = TRACKING;
        ROS_INFO("TRACKING");
    }
    switch (fsm)
    {
        case EMERGENCY:{
            init_flag = false;
            relocate_init = false;
            traj_msg.action = quadrotor_msgs::PositionCommand::ACTION_STOP;
            TrackTrajPub.publish(traj_msg);
            return;
        }
        case RELOCATING:{
            if(!relocate_init){
                if(Sample_list.size()<=1){
                    ROS_ERROR("OCC Sample list is nearly empty!");
                    return;
                }
                Freegridpath = kinosearch.poslist2freegrid(Sample_list);
                if(Freegridpath.size()<=1) {
                    ROS_ERROR("OCC failed");
                    return;
                }
                std::vector<Eigen::Vector3d> traj_gridpath = kinosearch.occ_gridpath(Freegridpath,start_pt);
                finState<<traj_gridpath[traj_gridpath.size()-1],Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero();
                if(!sfc.generate(traj_gridpath)){
                    ROS_ERROR("corridor failed, RELOCATE1");
                    return;
                }
                traj = optimaltraj_generate(initState,finState,sfc.corridor2mat(),v_max,a_max);
                visualize_relocate(traj);
                if(!init_flag){
                    traj_msg = traj2msg(traj);
                    TrackTrajPub.publish(traj_msg);
                    publish_time = ros::Time::now().toSec();
                    last_traj = traj;
                    relocate_init = true;
                    init_flag=true;
                }
                else{
                    double now_time;
                    now_time = ros::Time::now().toSec();
                    if(now_time-start_time>=BUDGET_TIME) 
                    {
                        ROS_WARN("this traj is not time feasible relocate");
                        return;
                    }//abort this replan
                    else{
                        while(1){
                            now_time = ros::Time::now().toSec();
                            if(now_time-start_time>=BUDGET_TIME)
                                break;
                    }
                    relocate_init = true;
                    traj_msg = traj2msg(traj);
                    TrackTrajPub.publish(traj_msg);
                    publish_time = ros::Time::now().toSec();
                    last_traj = traj;
                    }    
                }
                sfc.cubes.clear();
            }
            else{
                vector<Eigen::Vector3d> pos_list;
                for(double t  = rt;t<last_traj.getTotalDuration();t+=0.01){
                    pos_list.push_back(last_traj.getPos(t));
                }
                if(rt>=last_traj.getTotalDuration())
                    return;
                vector<Eigen::Vector3d> traj_gridpath = kinosearch.poslist2freegrid(pos_list);//起点是FREE的
                if(traj_gridpath.size()<=1) return;                
                finState<<traj_gridpath[traj_gridpath.size()-1],Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero();
                if(!sfc.generate(traj_gridpath)){
                    ROS_ERROR("corridor failed, RELOCATE2");
                    return;
                }
                traj = optimaltraj_generate(initState,finState,sfc.corridor2mat(),v_max,a_max);
                visualize_relocate(traj);
                if(!init_flag){
                    traj_msg = traj2msg(traj);
                    TrackTrajPub.publish(traj_msg);
                    publish_time =  ros::Time::now().toSec();
                    last_traj = traj;
                    init_flag=true;
                }
                else{
                    double now_time;
                    now_time = ros::Time::now().toSec();
                    if(now_time-start_time>=BUDGET_TIME) 
                    {
                        ROS_WARN("this traj is not time feasible relocate");
                        return;
                    }//abort this replan
                    else{
                        while(1){
                            now_time = ros::Time::now().toSec();
                            if(now_time-start_time>=BUDGET_TIME)
                                break;
                    }
                    relocate_init = true;
                    traj_msg = traj2msg(traj);
                    TrackTrajPub.publish(traj_msg);
                    publish_time = ros::Time::now().toSec();
                    last_traj = traj;
                    }    
                }
                sfc.cubes.clear();
            }
            return;
        }
        case TRACKING:{
            relocate_init = false;
            int bezier_flag = tgpredict.TrackingGeneration(5,5,target_detect_list);
            if(bezier_flag==0){
                predict_state_list = tgpredict.getStateListFromBezier(_PREDICT_SEG);
                Sample_list = tgpredict.SamplePoslist_bezier(_PREDICT_SEG);
            }
            else{
                ROS_WARN("bezier predict error");
            }
            if(predict_state_list.size() < 1) 
            {
                ROS_ERROR("Bezier predict failed");    
                return;
            }
            visualize_pre(Sample_list);
            int flag_pp = 0;
            Eigen::Vector3d begin_point = predict_state_list[0].head(3);
            flag_pp = kinosearch.search(start_pt,start_vel,predict_state_list,_TIME_INTERVAL);   
            if(flag_pp==-2)
            {
                //NO PATH
                flag_pp = 0;
            }
            if((start_pt-begin_point).norm()<0.6)
            {
                flag_pp=0;
            }
            if(flag_pp){
                vector<Eigen::Vector3d> pos_xyz = kinosearch.getKinoTraj(0.01);
                vector<Eigen::Vector3d> gridpath = kinosearch.traj2grids(pos_xyz);
                finState<<gridpath[gridpath.size()-1],Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero();
                if(!sfc.generate(gridpath)){
                    ROS_ERROR("corridor failed, TRACKING!");
                    return;
                }
                traj = optimaltraj_generate(initState,finState,sfc.corridor2mat(),v_max,a_max);
                traj_msg = traj2msg(traj);
                visualize_traj(traj);
                if(!init_flag){
                    publish_time = ros::Time::now().toSec();
                    traj_msg = traj2msg(traj);
                    TrackTrajPub.publish(traj_msg);
                    last_traj = traj;
                    init_flag=true;
                }
                else{
                    double now_time;
                    now_time = ros::Time::now().toSec();
                    if(now_time-start_time>=BUDGET_TIME) 
                    {   ROS_WARN("this traj is not time feasible");
                        return;}
                    else{
                        while(1){
                            now_time = ros::Time::now().toSec();
                            if(now_time-start_time>=BUDGET_TIME){
                                break;
                            }
                        }
                        publish_time =ros::Time::now().toSec();
                        //ROS_WARN("total time consume: %f ms",1000*(publish_time-start_time));
                        traj_msg = traj2msg(traj);
                        TrackTrajPub.publish(traj_msg);
                        last_traj = traj;
                    }    
                }
                sfc.cubes.clear();
            }
            kinosearch.reset();
            return;
        }
    }
}