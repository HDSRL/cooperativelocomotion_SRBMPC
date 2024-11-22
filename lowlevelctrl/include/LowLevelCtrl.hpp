#ifndef LOWLEVELCONTROLLER
#define LOWLEVELCONTROLLER

#include "global_loco_structs.hpp"
#include "EigenUtils.hpp"
#include "Transforms.hpp"
#if(USE_QPSWIFT)
#include "iswift_qp.h"
#else
#include "OSQP_wrapper.hpp"
#endif

using DynInf = DynamicsInfo;
using KinInf = KinematicsInfo;
using ConInf = ContactInfo;
using LLP = Settings::LL_params;

class LowLevelCtrl
{
public:
    LowLevelCtrl();
    virtual ~LowLevelCtrl();

    void calcTorque(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params);
    const LLInfo* getllPointer(){return &ll;};
    double* getTorque(){return tau;};

private:

    #if(USE_QPSWIFT)
    void solve_qpSWIFT(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    void cost(LLP *params, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    void constraints(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    Eigen::Matrix<double, 31, 31> P_QP; // Each QP matrix is set to the max size for any domain
    Eigen::Matrix<double, 31,  1> c_QP;
    Eigen::Matrix<double, 18, 31> A_QP;
    Eigen::Matrix<double, 18,  1> b_QP;
    Eigen::Matrix<double, 45, 31> G_QP;
    Eigen::Matrix<double, 45,  1> h_QP;
    double *optimOut;
    #else
    void solve_OSQP(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    void initMatrices();
    void initVectors();
    void updateMatrices(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    void updateVectors(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF);
    Eigen::Matrix<double, 30, 30> P_QP; // Each QP matrix is set to the max size for any domain
    Eigen::Matrix<double, 30,  1> c_QP; // OSQP DOES NOT support the CLF at this time
    Eigen::Matrix<double, 50, 30> A_QP;
    Eigen::Matrix<double, 50, 30> A_sparsity;
    Eigen::Matrix<double, 18,  1> b_QP;

    Eigen::MatrixXd lb_QP, ub_QP, q_QP;

    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));
    #endif

    void swingInvKin(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params);
    void saturateTorque();

    LLInfo ll;

    Eigen::Matrix<double, 3, 1> sat = {22,50,50};

    double V;
    double Veps;
    double LfV;
    Eigen::MatrixXd LgV;
    double dV;

    double tau[6+TOTAL_IN] = {0};

};

#endif
