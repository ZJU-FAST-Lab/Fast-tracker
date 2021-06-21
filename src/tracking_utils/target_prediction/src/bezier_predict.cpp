#include <bezier_prediction/bezier_predict.h>
#include <stdio.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
using namespace std;    
using namespace Eigen;


Eigen::MatrixXd Bezierpredict::getQ(const int vars_number, const vector<double> Time, const int seg_index){
    // calculate Matrix Q_k of the seg_index-th segment
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(vars_number, vars_number);
    int d_order = (traj_order+1)/2 - 1;//for us, is 3 here 
    for (int i = 0; i < vars_number; i++)
    {
        for (int j = 0; j < vars_number; j++)
        {
            if (i - d_order >= 0 && j - d_order >= 0)
            { 
                Q_k(i, j) = (factorial(i) / factorial(i - d_order)) * ((factorial(j) / factorial(j - d_order))) /
                            (i + j - 2 * d_order + 1) * pow(Time[seg_index], (i + j - 2 * d_order + 1)); // Q of one segment
            }
        }
    }
   //get Q for calculation of cost
    return Q_k;
}
Eigen::MatrixXd Bezierpredict::getM(const int vars_number, const vector<double> Time, const int seg_index){
    MatrixXd M_k = MatrixXd::Zero(vars_number, vars_number);
    VectorXd t_pow = VectorXd::Zero(vars_number);
    for(int i = 0; i < vars_number; i++)
    {
        t_pow(i) = pow(Time[seg_index],i);
    }
    M_k = M;
    for(int i=0;i<vars_number;i++){
        M_k.row(i) = M_k.row(i)/t_pow(i);    
    }
    return M_k;
} 

int Bezierpredict::TrackingGeneration(
        const double max_vel,
        const double max_acc,
        vector<Eigen::Vector4d> predict_list_complete){
    ros::Time time_1 = ros::Time::now();

    segs = 1;
    vector<double>  time_intervals;
    vector<double>  total_time_intervals;
    vector<Vector3d> predict_list;
    vector<double> predict_list_time;



    vector<double> history_weight_list; 
    history_time_total = 0;
    int init_flag = 0;
    for(int i = 0; i < _MAX_SEG; i++){
        predict_list.push_back(predict_list_complete[i].head(3));
        if(!init_flag){
            history_time_init = predict_list_complete[i][3];
            init_flag = 1;
        }
        history_time_total = predict_list_complete[i][3] - history_time_init;
        predict_list_time.push_back(history_time_total);
        //ROS_INFO_STREAM("pre_time: " << predict_list_time[i]);
    }
    //ROS_INFO_STREAM("tanh weight: " );
    for(int i = 0; i < _MAX_SEG; i++){
        double tanh_input = predict_list_time[_MAX_SEG - 1] - predict_list_time[i];
        if(!tanh_input){
            history_weight_list.push_back(1);
        }
        else{
            tanh_input = 1.0 / tanh_input;
            history_weight_list.push_back(tanh(1.2 * tanh_input));
            //ROS_INFO_STREAM("round: " <<i << " " << tanh(6 * tanh_input));
        }
    }
   
    for(int i=0;i<segs;i++){
        //time_intervals.push_back(time(i));
        time_intervals.push_back(history_time_total); //0.1秒
        total_time_intervals.push_back(time_intervals[i] + (_TIME_INTERVAL* _PREDICT_SEG));
    }
    
 
    Vector3d end_p = predict_list[_MAX_SEG - 1];

    int constrain_flag = 0;
    // double time_diff = (ros::Time::now().toSec() - history_time_init)- predict_list_time[_MAX_SEG-1];
    // //ROS_INFO_STREAM("time diff: " << time_diff);
    // if(time_diff >= 0.25){
    //     constrain_flag = 1;
    // }

    int vars_number = traj_order+1;//6
    int all_vars_number = 3*vars_number;//XYZ 18
    int nx = segs*3*vars_number;
    double c[nx];
    double  xupp[nx];    
    char   ixupp[nx];
    double  xlow[nx];
    char   ixlow[nx];
    for(int i=0;i<nx;i++){
        c[i] = 0.0;
        xlow[i] = 0.0;
        ixlow[i] = 0;
        xupp[i] = 0.0;
        ixupp[i] = 0;
    }



    int my = 3;                             
    double b[3];                            
    b[0] = end_p[0];   //end_p
    b[1] = end_p[1];
    b[2] = end_p[2];
    int nnzA = vars_number * 3;             
    int irowA[nnzA];                       
    int jcolA[nnzA];
    double dA[nnzA];
    int nn_idx = 0;
    int row_idx = 0;

    double curr_t = time_intervals[0] ;
    double total_t =  total_time_intervals[0];
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < vars_number; j++){
            dA[nn_idx] = C_[j] * pow(curr_t/total_t, j) * pow((1 - curr_t/total_t), (traj_order - j));
            irowA[nn_idx]  = row_idx;
            jcolA[nn_idx]  = i * vars_number + j;
            nn_idx ++;
        }
        row_idx ++;
    }

    

    int high_order_con_num = 3*(vars_number-1)*segs+3*(vars_number-2)*segs;
    const int mz  = high_order_con_num;
    char iclow[mz];
    char icupp[mz];
    double clow[mz];
    double cupp[mz];

    int m_idx = 0;

    for(int i = 0; i < 3*(vars_number-1)*segs; i++)
    {
        iclow[m_idx] = 1;
        icupp[m_idx] = 1;
        clow[m_idx]  = -max_vel;
        cupp[m_idx]  = max_vel;
        m_idx++;
    }

    for(int i=0;i<3*(vars_number-2)*segs;i++){
        iclow[m_idx]=1;
        icupp[m_idx]=1;
        clow[m_idx]=-max_acc;
        cupp[m_idx]= max_acc;
        m_idx++;
    }
   
   
    int nnzC =segs * (vars_number-1)*2 *3 + segs * (vars_number-2)*3 *3;
    int irowC[nnzC];
    int jcolC[nnzC];
    double dC[nnzC];
    nn_idx  = 0;
    row_idx = 0;

    //velocity
    for(int k = 0; k < segs ; k ++ )
    {   
            for(int i = 0; i < 3; i++)
            {  // for x, y, z loop
                for(int j = 0;j < traj_order;j++){
                    dC[nn_idx]     = -1.0 * traj_order/total_time_intervals[k];
                    dC[nn_idx + 1] =  1.0 * traj_order/total_time_intervals[k];
                    irowC[nn_idx]     = row_idx;
                    irowC[nn_idx + 1] = row_idx;
                    jcolC[nn_idx]     = k * all_vars_number + i * vars_number+j;    
                    jcolC[nn_idx + 1] = k * all_vars_number + i * vars_number + j + 1;    
                    row_idx ++;
                    nn_idx += 2;
                }
            }
    }
    //acceleration
    for(int k = 0; k < segs ; k ++ )
    {   
        double scale_k = pow(total_time_intervals[k],2);
        for(int i = 0; i < 3; i++)
        { 
            for(int j = 0; j < traj_order - 1; j++)
                {    
                    dC[nn_idx]     =  1.0 * traj_order * (traj_order - 1) / scale_k;
                    dC[nn_idx + 1] = -2.0 * traj_order * (traj_order - 1) / scale_k;
                    dC[nn_idx + 2] =  1.0 * traj_order * (traj_order - 1) / scale_k;

                    irowC[nn_idx]     = row_idx;
                    irowC[nn_idx + 1] = row_idx;
                    irowC[nn_idx + 2] = row_idx;

                    jcolC[nn_idx]     = k * all_vars_number + i * vars_number + j;    
                    jcolC[nn_idx + 1] = k * all_vars_number + i * vars_number + j + 1;    
                    jcolC[nn_idx + 2] = k * all_vars_number + i * vars_number + j + 2;    
                    
                    row_idx ++;
                    nn_idx += 3;
                }
            }
    }
    



    
    _Q = MatrixXd::Zero(vars_number * segs * 3, vars_number * segs * 3);
    _M = MatrixXd::Zero(vars_number * segs * 3, vars_number * segs * 3);
    for(int i=0; i<segs; i++){
        for(int j = 0; j < 3; j++){
            // calculate Matrix Q
            _Q.block(i*all_vars_number+j*vars_number, i*all_vars_number+j*vars_number, vars_number, vars_number) 
            = getQ(vars_number, total_time_intervals,i);
            // calculate Matrix M   
            _M.block(i*all_vars_number+j*vars_number, i*all_vars_number+j*vars_number, vars_number, vars_number) 
            = getM(vars_number, total_time_intervals, i);
        }
    }


    MatrixXd Ct = MatrixXd::Zero(1,segs*3*(traj_order+1));
    MatrixXd distance_Q = MatrixXd::Zero(vars_number * segs * 3, vars_number * segs * 3); 
    double t_base = 0;
    for(int i= 0;i<segs;i++){
        if(i>=1)
            t_base  += time_intervals[i-1]; 
        for(double j = 0;j<_MAX_SEG;j+=1){
            Ct.block(0,i*3*(traj_order+1),1,3*(traj_order+1)) 
                += getCt(predict_list_time[j], predict_list[j]) * history_weight_list[j];
            distance_Q.block(all_vars_number*i,all_vars_number*i,all_vars_number,all_vars_number)
                 += getdistance_Q(predict_list_time[j]) * history_weight_list[j];        
        }
    }
    //ROS_INFO_STREAM("asdf" << Ct);
    Ct =  Ct*_M;
    /*
    */
    //MatrixXd snapQ = _Q;
    //double sim_weight = 0.0;
    
    for(int i = 0; i < nx; i++)
        c[i] = Ct(0,i);
    
    _Q = 2 * (Lambda_ACC * _MAX_SEG * _Q  + distance_Q);
    //ROS_INFO_STREAM("asdf" << _Q);

    MatrixXd M_QM;
    M_QM = MatrixXd::Zero(_M.rows(),_M.cols());
    //ROS_INFO_STREAM  ("M:rows"<<_M.rows()<<"col"<<_M.cols()<<"Q:rows"<<_Q.rows()<<"col:"<<_Q.cols());
    M_QM = _M.transpose()*_Q*_M;



    
    const int nnzQ = 3 * segs * (traj_order + 1) * (traj_order + 2) / 2; //n(n+1)/2
    int    irowQ[nnzQ]; 
    int    jcolQ[nnzQ];
    double    dQ[nnzQ];

    int sub_shift = 0;
    int Q_idx = 0;

    for(int k = 0; k < segs; k ++){
        for(int p = 0; p < 3; p ++ )
            for( int i = 0; i < vars_number; i ++ )
                for( int j = 0; j < vars_number; j ++ )
                    if( i >= j ){
                        irowQ[Q_idx] = sub_shift + p * vars_number + i;   
                        jcolQ[Q_idx] = sub_shift + p * vars_number + j;  
                        dQ[Q_idx] = M_QM(sub_shift + p * vars_number + i,sub_shift + p * vars_number + j);
                        //dQ[Q_idx] = 1;
                        Q_idx ++ ;
                    }
        sub_shift += all_vars_number;
    }
    //my=0;
    //nnzA=0;
    QpGenSparseMa27 * qp;
    QpGenData * prob;
    QpGenVars      * vars;
    QpGenResiduals * resid;
    GondzioSolver  * s;
    if(constrain_flag){
           
        int my = 0;               
        double *b = 0;          
        int nnzA = 0;             
        int *irowA = 0;           
        int *jcolA = 0;
        double *dA = 0;

        int nn_idx  = 0;
        int row_idx = 0;
        qp 
        = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );
        //cout<<"irowQ: "<<irowQ[nnzQ-1]<<"jcolQ: "<<jcolQ[nnzQ-1];
        //cout<<"nx: "<<nx<<" my: "<<my<<" mz: "<<mz<<" nnzQ: "<<nnzQ<<" nnzA: "<<nnzA<<" nnzC: "<<nnzC<<" size: "<<M_QM.cols()<<" "<<M_QM.rows();
        prob = (QpGenData * ) qp->copyDataFromSparseTriple(
            c,      irowQ,  nnzQ,   jcolQ,  dQ,
            xlow,   ixlow,  xupp,   ixupp,
            irowA,  nnzA,   jcolA,  dA,     b,
            irowC,  nnzC,   jcolC,  dC,
            clow,   iclow,  cupp,   icupp );

        vars  = (QpGenVars *) qp->makeVariables( prob );
        resid = (QpGenResiduals *) qp->makeResiduals( prob );
        s     = new GondzioSolver( qp, prob );
    }
    else{
        qp 
        = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );
        //cout<<"irowQ: "<<irowQ[nnzQ-1]<<"jcolQ: "<<jcolQ[nnzQ-1];
        //cout<<"nx: "<<nx<<" my: "<<my<<" mz: "<<mz<<" nnzQ: "<<nnzQ<<" nnzA: "<<nnzA<<" nnzC: "<<nnzC<<" size: "<<M_QM.cols()<<" "<<M_QM.rows();
        prob = (QpGenData * ) qp->copyDataFromSparseTriple(
            c,      irowQ,  nnzQ,   jcolQ,  dQ,
            xlow,   ixlow,  xupp,   ixupp,
            irowA,  nnzA,   jcolA,  dA,     b,
            irowC,  nnzC,   jcolC,  dC,
            clow,   iclow,  cupp,   icupp );

        vars  = (QpGenVars *) qp->makeVariables( prob );
        resid = (QpGenResiduals *) qp->makeResiduals( prob );
        s     = new GondzioSolver( qp, prob );
    }
    
    // Turn Off/On the print of the solving process
    //s->monitorSelf();
    int ierr = s->solve(prob, vars, resid);
    if( ierr == 0 ) 
    {
        double d_var[nx];
        vars->x->copyIntoArray(d_var);
        // cout<<"d_var="<<d_var;
        // int temp_count=0;
        // for(int kk=0;kk<nx;kk++)
        // {
        //     cout<<"d_var="<<d_var[kk];
        //     temp_count++;
        //     if(temp_count%10==0)
        //         cout<<"    count="<<temp_count<<endl;
        // }

        // cout.precision(4);
        // cout << "Solution: \n";
        // vars->x->writefToStream(cout, "x[%{index}] = %{value}");
        vars->x->copyIntoArray(d_var);

        PolyCoeff = MatrixXd::Zero(segs, all_vars_number);
        PolyTime  = VectorXd::Zero(segs);
        obj = 0.0;
        
        int var_shift = 0;

        MatrixXd Q_o(vars_number,vars_number);
        //    int s1d1CtrlP_num = traj_order + 1;
        //    int s1CtrlP_num   = 3 * s1d1CtrlP_num;
        //int min_order_l = floor(minimize_order);
        //int min_order_u = ceil (minimize_order);

        for(int i = 0; i < segs; i++ )
        {   
            PolyTime(i) = total_time_intervals[i];

            for(int j = 0; j < all_vars_number; j++)
                {
                    PolyCoeff(i , j) = d_var[j + var_shift];
                    // cout<<"coeff in is  "<<PolyCoeff(i , j)<<"i="<<i<<"  j="<<j<<endl;
                }
            var_shift += all_vars_number;     
        } 
        MatrixXd flat_poly = MatrixXd::Zero(1,segs*all_vars_number);
        for(int i=0;i<segs;i++){
            flat_poly.block(0,i*all_vars_number,1,all_vars_number) = PolyCoeff.block(i,0,1,all_vars_number);
        }
        /*double sum_time;
        for(int i=0;i<time_intervals.size();i++)
            sum_time+=time_intervals[i];
        ROS_INFO_STREAM("CT: "<<Ct*flat_poly.transpose());
        ROS_INFO_STREAM("distance: "<<flat_poly* distance_Q*flat_poly.transpose());
        ROS_INFO_STREAM("snap Q: "<<flat_poly*snapQ*flat_poly.transpose());
        ROS_INFO_STREAM("time : "<<sum_time);*/

    } 
    else if( ierr == 3)
        ROS_ERROR("Front Bezier Predict: The program is provably infeasible, check the formulation");
    else if (ierr == 4)
        ROS_ERROR("Front Bezier Predict: The program is very slow in convergence, may have numerical issue");
    else
        ROS_ERROR("Front Bezier Predict: Solver numerical error");
    
    ros::Time time_2 = ros::Time::now();
    // ROS_INFO_STREAM("Bezier time consumed:" << (time_2 - time_1).toSec()*1000<<" ms");

    return ierr;
}

