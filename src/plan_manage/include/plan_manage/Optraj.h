#ifndef _OPTRAJ_H
#define _OPTRAJ_H
#include<grid_path_searcher/hybridAstar_searcher.h>
#include<bezier_prediction/bezier_predict.h>
#include <sfc_generation/sfc.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
class OpPiece{
    public:
        OpPiece(){}
        ~OpPiece(){}
        double duration;
        Eigen::Matrix<double, 3, 6> nCoeffMat;
        inline int getOrder() {return 5;}
        inline double getDuration(){return duration;}
        inline Eigen::Matrix<double, 3, 6> normalizedCoeffMat(){
            return nCoeffMat;
        }
        Eigen::Vector3d getPos(double t);
        Eigen::Vector3d getVel(double t);
        Eigen::Vector3d getAcc(double t);
};
class OpTrajectory{
    public:
    std::vector<OpPiece> pieces;  
    OpTrajectory(){}
    ~OpTrajectory(){}
    int getPieceNum();
    Eigen::VectorXd getDurations();
    double getTotalDuration();
    int locatePieceIdx(double &t) ;
    Eigen::Vector3d getPos(double t);
    Eigen::Vector3d getVel(double t);
    Eigen::Vector3d getAcc(double t);
    inline  OpPiece &operator[](int i) { return pieces[i];}
    
};
OpTrajectory optimaltraj_generate(const Eigen::MatrixXd &iniState,
                   const Eigen::MatrixXd &finState,
                   const Eigen::MatrixXd &cuboidsParams,
                   double vmax,
                   double amax,
                   const double rh=200);



// optimaltraj_generate

#endif