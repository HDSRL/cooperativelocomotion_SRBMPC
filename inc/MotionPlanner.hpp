#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "global_loco_structs.hpp"
#include "MPC_global_params.hpp"
#include "ContactEst.hpp"
#include "Transforms.hpp"
#include "Bezier.h"
#include "EigenUtils.hpp"

#define MAX_SL_F_X 0.16     // Max forward step length (magnitude)
#define MAX_SL_R_X 0.16     // Max backward step length (magnitude)
#define MAX_SL_Y 0.12       // Max lateral step length (magnitude)  

using MP = Settings::Motion_params;

class MotionPlanner
{
public:
    MotionPlanner();
    virtual ~MotionPlanner(){};

    void planTraj(const StateInfo *state, const KinematicsInfo *kin, ContactEst *con_obj, size_t gait, double phase, size_t ctrlTick, MP *params);
    void updateStandVars(const Eigen::Matrix<double,3,1> &com, double timeToStand);
    void updatePoseType(size_t type);
    void updateVel(Eigen::Matrix<double,3,1> &desVel, Eigen::Matrix<double, 3, 1> &desOmega, MP *params);
    void setDesiredCom(Eigen::Matrix<double, 12, 1> comDes){traj.comDes = comDes;};
    const TrajInfo* getTrajInfoPointer(){return &traj;};

protected:
    inline void setStepLen(double x, double y, double z){
        traj.stepLen[0] = x; traj.stepLen[1] = y; traj.stepLen[2] = z;
    }

private:
    double x0;
    double xf;
    double y0;
    double yf;
    double z0;
    double standTime;
    TrajInfo traj;
    size_t poseType = POSE_Z;
};

#endif
