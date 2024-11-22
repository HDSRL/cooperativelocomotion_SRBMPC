#ifndef LOCO_WRAPPER
#define LOCO_WRAPPER

#include "global_loco_structs.hpp"
#include "Parameters.hpp"
#include "RobotModel.hpp"
#include "LowLevelCtrl.hpp"
#include "VirtualConstraints.hpp"
#include "ContactEst.hpp"
#include "MotionPlanner.hpp"
#include "DataLog.hpp"
// #include "DeePC_MPC.hpp"
#include "Filters.h"

#include <memory>

class LocoWrapper : public Parameters
{
public:
    LocoWrapper(int argc, char *argv[]);
    virtual ~LocoWrapper();

    void calcTau(const double q[18], const double dq[18], const double R[9], const int force[4], size_t gait, size_t ctrlTick);
    double* getTorque(){return LL->getTorque();};
    const int* getConDes(){return con->des;};
    Eigen::Matrix<double, 18, 1> getJointPosCmd(){return ll->q;};
    Eigen::Matrix<double, 18, 1> getJointVelCmd(){return ll->dq;};
    void initStandVars(Eigen::Matrix<double,3,1> com, double standTime, size_t type){ PP->updateStandVars(com,standTime); PP->updatePoseType(type);};
    // void initStandVarsMPC(Eigen::Matrix<double,3,1> com){ deepc->updateStandVars(com);};
    void updateDesiredForce(Eigen::Matrix<double, 12, 1> fDes){VC->setDesiredForce(fDes);};
    void setDesired(Eigen::Matrix<double, 12, 1> comDes){HL_com=comDes;};
    void setDesired(Eigen::Matrix<double, 12, 1> fDes, size_t forceindex){HL_force=fDes;};
    void setDesired(Eigen::Matrix<double, 12, 1> comDes, Eigen::Matrix<double, 12, 1> fDes){HL_com=comDes; HL_force=fDes;};
    
    // Vec12d getNextMPCTraj(){return deepc->getNextOutput();};
    // Vec12d getFinalMPCTraj(){return deepc->getFinalOutput();};
    double getPhaseVar(){return phaseVar;};
    Eigen::Matrix<double, NRS*HORIZ,1> getDesTraj(){return traj->redDes;};
    double getDomLen(){return traj->domLen;};

    void setAgnum(size_t agnum){eachagnum = agnum;};


    // Pointers to structs
    const StateInfo *state;
    const DynamicsInfo *dyn;
    const KinematicsInfo *kin;
    const ContactInfo *con;
    const TrajInfo *traj;
    const VCInfo *vcon;
    const LLInfo *ll;

private:
    // size_t newDom = 0;
    size_t locoTick = 0;
    double phaseVar = 0;
    double maxPhase = 1.05;
    size_t gaitTemp = STAND;
    size_t forceDomainChange = 0;

    // Pointers to class objects
    std::unique_ptr<DataLog> data0;
    std::unique_ptr<DataLog> data1;
    size_t eachagnum = 0;
    // std::unique_ptr<DeePC> deepc;
    RobotModel *quad;
    LowLevelCtrl *LL;
    VirtualConstraints *VC;
    ContactEst *conEst;
    MotionPlanner *PP;

    FiltStruct_d* trajFilt  = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));

    bool useHL = true;

    Eigen::Matrix<double, 12, 1> HL_com;
    Eigen::Matrix<double, 12, 1> HL_force;
};

inline double getPhase(double time, double time_0, double time_f){
    return (1.0*time-1.0*time_0)/(1.0*time_f-1.0*time_0);
};

#endif
