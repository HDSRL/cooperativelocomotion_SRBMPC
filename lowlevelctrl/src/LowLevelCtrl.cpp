//
// Authror: Randy Fawcett on 12/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "LowLevelCtrl.hpp"

LowLevelCtrl::LowLevelCtrl(){
    #if(USE_QPSWIFT)
    P_QP.setZero();
    c_QP.setZero();
    A_QP.setZero();
    b_QP.setZero();
    G_QP.setZero();
    h_QP.setZero();
    #else
    P_QP.setZero();
    c_QP.setZero();
    A_QP.setZero();
    b_QP.setZero();

    data->A = NULL;
    data->P = NULL;
    work = NULL;

    initMatrices();
    initVectors();

    // Define solver settings as default
    osqp_set_default_settings(settings);
    // settings->alpha      = 1;
    settings->verbose    = 0;
    settings->warm_start = 0;
    // settings->max_iter   = 50;
    // settings->time_limit = 0.035;

    c_int exitflag = osqp_setup(&work, data, settings);
    #endif
}

LowLevelCtrl::~LowLevelCtrl(){
    #if (USE_QPSWIFT)
    #else
    if (work) osqp_cleanup(work);
    freeData(data);
    c_free(settings);
    #endif
};

// Main function
void LowLevelCtrl::calcTorque(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params){
    // ====================================================================== //
    // =============================== Setup ================================ //
    // ====================================================================== //
    size_t useCLF = params->useCLF;
    size_t conDim = 3*con->cnt;
    size_t outDim = 6+3*(4-con->cnt);
    size_t numDec = conDim+TOTAL_IN+outDim+useCLF;

    // ====================================================================== //
    // ============================ Solve the QP ============================ //
    // ====================================================================== //
    #if (USE_QPSWIFT)
    solve_qpSWIFT(params, dyn, kin, vc, con, outDim, conDim, numDec, useCLF);
    #else
    solve_OSQP(params, dyn, kin, vc, con, outDim, conDim, numDec, useCLF);
    #endif
    // saturateTorque();
    Eigen::Map<Eigen::Matrix<double, 18, 1>> tau_eig(tau,18,1);

    // ====================================================================== //
    // ============================ Swing Leg PD ============================ //
    // ====================================================================== //
    if (conDim<12){
        Eigen::MatrixXd Delta_temp = (kin->Js*dyn->Dinv*kin->Js.transpose());
        Eigen::MatrixXd Delta = Delta_temp.inverse();
        Eigen::MatrixXd p_d = Eigen::MatrixXd::Zero(12-conDim,1);
        Eigen::MatrixXd v_d = Eigen::MatrixXd::Zero(12-conDim,1);
        Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(12-conDim,12-conDim);
        double Kd = 40;
        double wd = 40;
        size_t cnts = 0;
        for (size_t i=0; i<4; i++){
            if(con->ind[i]==0){
                p_d.block(cnts,0,3,1) = (vc->hd.block(6+cnts,0,3,1)-kin->toePos.block(0,i,3,1));
                v_d.block(cnts,0,3,1) = (vc->dhd.block(6+cnts,0,3,1)-kin->Jtoe.block(3*i,0,3,TOTAL_DOF)*state->dq);
                Kp.block(cnts,cnts,3,3).diagonal() = wd*wd*Delta.block(cnts,cnts,3,3).diagonal();
                cnts+=3;
            }
        }
        tau_eig += kin->Js.transpose()*(Kp*p_d + Kd*v_d);
    }

    // ====================================================================== //
    // ======================= Calculate Joint Angles ======================= //
    // ====================================================================== //
    ll.ddq = dyn->Dinv*(dyn->B*tau_eig.block(6,0,12,1)+kin->Jtoe.transpose()*ll.QP_force - dyn->H);
    ll.dq = state->dq+ll.ddq/LL_Hz;
    ll.q  = state->q+ll.dq/LL_Hz+0.5/(LL_Hz*LL_Hz)*ll.ddq;
    if(conDim!=12){
        swingInvKin(state, dyn, kin, vc, con, params);
        for(int i=0; i<4; ++i){
            if (con->ind[i]==0){
                // tau_eig.block(6+3*i,0,3,1) += 20*(ll.q.block(6+3*i,0,3,1)-state->q.block(6+3*i,0,3,1))
                //                            + 1*(ll.dq.block(6+3*i,0,3,1)-state->dq.block(6+3*i,0,3,1));
                // std::cout<<state->q.block(6+3*i,0,3,1).transpose()<<std::endl;
                // std::cout<<ll.q.block(6+3*i,0,3,1).transpose()<<"\n\n"; 
           }
       }
    }

}

#if (USE_QPSWIFT)
// qpSwift functions
void LowLevelCtrl::solve_qpSWIFT(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){
    // ====================================================================== //
    // ============================ Solve the QP ============================ //
    // ====================================================================== //
    cost(params, vc, con, outDim, conDim, numDec, useCLF);
    constraints(params, dyn, kin, vc, con, outDim, conDim, numDec, useCLF);
    optimOut = new double[numDec];

    iswiftQp_e(P_QP.block(0,0,numDec,numDec), c_QP.block(0,0,numDec,1),
               A_QP.block(0,0,conDim+outDim,numDec), b_QP.block(0,0,conDim+outDim,1),
               G_QP.block(0,0,5*con->cnt+2*TOTAL_IN+useCLF,numDec), h_QP.block(0,0,5*con->cnt+2*TOTAL_IN+useCLF,1),
               optimOut);
    
    // ====================================================================== //
    // ======================== Parse the QP solution ======================= //
    // ====================================================================== //
    int cnt = 0;
    ll.QP_force.setZero();
    for(int i=0; i<4; ++i){
        if(con->ind[i]==1){
            for(int j=0; j<3; ++j){
                ll.QP_force(3*i+j) = optimOut[cnt];
                cnt++;
            }
        }
    }
    for(int i=0; i<TOTAL_IN; ++i){
        tau[6+i] = optimOut[cnt];
        ll.tau[6+i] = optimOut[cnt];
        cnt++;
    }
    if(useCLF){
        dV = LfV+Veps;
        for(int i=0; i<outDim; ++i){
            dV += LgV(i)*optimOut[cnt];
            cnt++;
        }
    }
    ll.dV = dV;
    delete[] optimOut;
}

void LowLevelCtrl::cost(LLP *params, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){
    // ====================================================================== //
    // ============================ Cost Function =========================== //
    // ====================================================================== //
    P_QP.block(0,0,conDim,conDim) = params->dfPen*Eigen::MatrixXd::Identity(conDim, conDim);
    P_QP.block(conDim,conDim,TOTAL_IN,TOTAL_IN) = params->tauPen*Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
    P_QP.block(conDim+TOTAL_IN,conDim+TOTAL_IN,outDim,outDim) = params->auxPen*Eigen::MatrixXd::Identity(outDim,outDim);
    if (useCLF){
        P_QP(numDec-1, numDec-1) = params->clfPen;
    }

    c_QP.setZero();
    Eigen::MatrixXd Fd(conDim,1);
    int cnt = 0;
    for (int i=0; i<4; ++i){
    	if (con->ind[i]==1){
    		Fd.block(cnt,0,3,1) = vc->fDes.block(3*i,0,3,1);
    		cnt+=3;
    	}
    }	
    
    c_QP.block(0,0,conDim,1) = -Fd*params->dfPen;
}

void LowLevelCtrl::constraints(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){  
    double mu = params->mu;
    double kpGain = params->kp;
    double kdGain = params->kd;
    
    // ====================================================================== //
    // ======================== Equality Constraints ======================== //
    // ====================================================================== //
    A_QP.block(0,0,conDim+outDim,numDec-useCLF) <<  
            (kin->Jc)*(dyn->Dinv)*(kin->Jc.transpose()), (kin->Jc)*(dyn->Dinv)*(dyn->B), Eigen::MatrixXd::Zero(conDim,outDim),
            (vc->H0)*(dyn->Dinv)*(kin->Jc.transpose()), (vc->H0)*(dyn->Dinv)*(dyn->B), Eigen::MatrixXd::Identity(outDim,outDim);
    b_QP.block(0,0,conDim+outDim,1) << (kin->Jc)*(dyn->Dinv)*(dyn->H) - (kin->dJc),
                                     (-kpGain*(vc->y)-kdGain*(vc->dy)) + (vc->H0)*(dyn->Dinv)*(dyn->H) - (vc->dH0);

    // ====================================================================== //
    // ======================= Inequality Constraints ======================= //
    // ====================================================================== //
    // Friction Cone
    Eigen::Matrix<double, 5, 3> gc;
    G_QP.setZero();
    h_QP.setZero();
    gc <<  1,  0, -mu/sqrt(2),
          -1,  0, -mu/sqrt(2),
           0,  1, -mu/sqrt(2),
           0, -1, -mu/sqrt(2),
           0,  0,          -1;
    repdiag(gc,G_QP,con->cnt);

    // Bounds
    G_QP.block(5*con->cnt,conDim,TOTAL_IN,TOTAL_IN) <<  Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
    G_QP.block(5*con->cnt+TOTAL_IN,conDim,TOTAL_IN,TOTAL_IN) << -Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
    h_QP.block(5*con->cnt,0,TOTAL_IN,1) << sat,sat,sat,sat;
    h_QP.block(5*con->cnt+TOTAL_IN,0,TOTAL_IN,1) << sat,sat,sat,sat;

    if(useCLF == 1){
        Eigen::MatrixXd PP = Eigen::MatrixXd::Zero(2*outDim,2*outDim);
        Eigen::MatrixXd P_temp(2*outDim,2*outDim);
        Eigen::MatrixXd tuneMat(2*outDim,2*outDim);
        Eigen::MatrixXd FF(2*outDim, 2*outDim);
        Eigen::MatrixXd GG(2*outDim, outDim);
        Eigen::MatrixXd eta(2*outDim,1);
        Eigen::Matrix<double, 1, 1> V_;
        Eigen::Matrix<double, 1, 1> LfV_;
        LgV.setZero(1,outDim);


        // Solve Algebraic Lyapunov Equation (MATLAB Check: lyap(FF',I) )
        // NOTE THE TRANSPOSE IN THE MATLAB CHECK!!!
        // This is a "special" solution given the particular of the matrices form used in this code
        double P1, Pd, P2;
        P1 = (kdGain*kdGain+kpGain*kpGain+kpGain)/(2*kpGain*kdGain);
        Pd = 1/(2*kpGain);
        P2 = (kpGain+1)/(2*kdGain*kpGain);
        PP.block(0,0,outDim,outDim) = P1*Eigen::MatrixXd::Identity(outDim,outDim);
        PP.block(0,outDim,outDim,outDim) = Pd*Eigen::MatrixXd::Identity(outDim,outDim);
        PP.block(outDim,0,outDim,outDim) = Pd*Eigen::MatrixXd::Identity(outDim,outDim);
        PP.block(outDim,outDim,outDim,outDim) = P2*Eigen::MatrixXd::Identity(outDim,outDim);
        double cc = 1.0/( 0.5*( P1 + P2 + sqrt(P1*P1-2*P1*P2+P2*P2+4*Pd*Pd) ) );

        double eps = params->clfEps;

        tuneMat.setIdentity();
        tuneMat.block(0,0,outDim,outDim) *= (1.0/eps);
        P_temp = tuneMat*PP*tuneMat;
        PP = P_temp;

        FF.setZero();
        GG.setZero();

        FF.block(0,outDim,outDim,outDim).setIdentity();
        FF.block(outDim,0,outDim,outDim) = -kpGain*Eigen::MatrixXd::Identity(outDim,outDim);
        FF.block(outDim,outDim,outDim,outDim) = -kdGain*Eigen::MatrixXd::Identity(outDim,outDim);

        GG.block(outDim,0,outDim,outDim).setIdentity();
        eta << vc->y, vc->dy;

        V_   = (eta.transpose()*PP*eta);
        LfV_ = eta.transpose()*(FF.transpose()*PP+PP*FF)*eta;
        LgV  = 2*eta.transpose()*PP*GG;

        G_QP.block(2*TOTAL_IN+5*con->cnt, conDim+TOTAL_IN, 1, outDim) = LgV;
        G_QP(2*TOTAL_IN+5*con->cnt, numDec-1) = -1.0; // defect variable
        h_QP(2*TOTAL_IN+5*con->cnt, 0) = -LfV_(0)-cc/eps*V_(0);

        V = V_(0);
        Veps = cc/eps*V_(0);
        LfV = LfV_(0);
        ll.V = V;
    }
}
#else
// OSQP functions
void LowLevelCtrl::solve_OSQP(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){
    // ====================================================================== //
    // ============================ Solve the QP ============================ //
    // ====================================================================== //
    updateMatrices(params, dyn, kin, vc, con, outDim, conDim, numDec, useCLF);
    updateVectors(params, dyn, kin, vc, con, outDim, conDim, numDec, useCLF);
    c_int OSQP_exit = osqp_solve(work);
    // std::cout<<work->info->solve_time+work->info->update_time<<std::endl;
    
    // ====================================================================== //
    // ======================== Parse the QP solution ======================= //
    // ====================================================================== //
    int cnt = 0;
    ll.QP_force.setZero();
    for(int i=0; i<4; ++i){
        if(con->ind[i]==1){
            for(int j=0; j<3; ++j){
                ll.QP_force(3*i+j) = work->solution->x[cnt];
                cnt++;
            }
        }
    }
    for(int i=0; i<TOTAL_IN; ++i){
        tau[6+i] = work->solution->x[cnt];
        ll.tau[6+i] = work->solution->x[cnt];
        cnt++;
    }
    if(useCLF){
        dV = LfV+Veps;
        for(int i=0; i<outDim; ++i){
            dV += LgV(i)*work->solution->x[cnt];
            cnt++;
        }
    }
    ll.dV = dV;
}

void LowLevelCtrl::initMatrices(){
    // Sparsity structure of the cost
    P_QP.setIdentity();
    
    // Sparsity structure of the constraints
    A_QP.block(0,0,18,30).setOnes();
    A_QP.block(0,24,12,6).setZero();
    A_QP.block(12,24,6,6).setIdentity();

    Eigen::Matrix<double, 5, 3> gc;
    Eigen::Matrix<double, 20, 12> G;
    gc <<  1, 0, 1,
           1, 0, 1,
           0, 1, 1,
           0, 1, 1,
           0, 0, 1;
    repdiag(gc,G,4);
    A_QP.block(18,  0, 20, 12) = G;
    A_QP.block(38,  6, 12, 12) += Eigen::Matrix<double,12,12>::Identity();
    A_QP.block(38, 12, 12, 12) += Eigen::Matrix<double,12,12>::Identity();
    A_sparsity = A_QP;

    data->n = P_QP.cols();
    data->m = A_QP.rows();
    data->P = csc_matrix_eig(P_QP);
    data->A = csc_matrix_eig(A_QP);
}

void LowLevelCtrl::initVectors(){
    q_QP.setZero(data->n,1);
    lb_QP.setZero(data->m,1);
    ub_QP.setZero(data->m,1);
    data->q = q_QP.data();
    data->l = lb_QP.data();
    data->u = ub_QP.data();

}

void LowLevelCtrl::updateMatrices(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){
    // ====================================================================== //
    // ============================ Cost Function =========================== //
    // ====================================================================== //
    P_QP.block(0,0,conDim,conDim) = params->dfPen*Eigen::MatrixXd::Identity(conDim, conDim);
    P_QP.block(conDim,conDim,TOTAL_IN,TOTAL_IN) = params->tauPen*Eigen::MatrixXd::Identity(TOTAL_IN,TOTAL_IN);
    P_QP.block(conDim+TOTAL_IN,conDim+TOTAL_IN,outDim,outDim) = params->auxPen*Eigen::MatrixXd::Identity(outDim,outDim);
    if (useCLF){
        P_QP(numDec-1, numDec-1) = params->clfPen;
    }

    // ====================================================================== //
    // ======================== Equality Constraints ======================== //
    // ====================================================================== //
    A_QP.setZero();
    A_QP.block(0,0,conDim+outDim,numDec-useCLF) <<  
            (kin->Jc)*(dyn->Dinv)*(kin->Jc.transpose()), (kin->Jc)*(dyn->Dinv)*(dyn->B), Eigen::MatrixXd::Zero(conDim,outDim),
            (vc->H0)*(dyn->Dinv)*(kin->Jc.transpose()), (vc->H0)*(dyn->Dinv)*(dyn->B), Eigen::MatrixXd::Identity(outDim,outDim);

    // ====================================================================== //
    // ======================= Inequality Constraints ======================= //
    // ====================================================================== //
    // Friction Cone
    double mu = params->mu;
    Eigen::Matrix<double, 5, 3> gc;
    Eigen::Matrix<double, 20, 12> G;
    gc <<  1,  0, -mu/sqrt(2),
          -1,  0, -mu/sqrt(2),
           0,  1, -mu/sqrt(2),
           0, -1, -mu/sqrt(2),
           0,  0,          -1;
    repdiag(gc,G,con->cnt);
    A_QP.block(18, 0, 20, 12) = G;

    // Torque Bounds
    A_QP.block(38,  3*con->cnt, 12, 12) = Eigen::Matrix<double,12,12>::Identity();

    // ====================================================================== //
    // ============================= Update OSQP ============================ //
    // ====================================================================== //
    double A[data->A->nzmax] = {0};
    int cntn = 0;
    for (int i=0; i<data->A->nzmax; ++i){
        while(A_sparsity(cntn)==0){
            cntn++;
        }
        A[i] = A_QP(cntn);
        // std::cout<<A[i]<<std::endl;
        cntn++;
    }
    double P[data->P->nzmax] = {0};
    for (int i=0; i<data->P->nzmax; ++i){
        P[i] = P_QP(i,i);
    }

    c_int exit = osqp_update_P_A(work, P, OSQP_NULL, data->P->nzmax,
                                       A, OSQP_NULL, data->A->nzmax);

    // std::cout<<"P"<<std::endl;
    // std::cout<<P_QP<<std::endl;
    // std::cout<<"A"<<std::endl;
    // std::cout<<A_QP<<std::endl;
}

void LowLevelCtrl::updateVectors(LLP *params, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, size_t &outDim, size_t &conDim, size_t &numDec, size_t &useCLF){
    // Cost function
    q_QP.setZero();
    Eigen::MatrixXd Fd(conDim,1);
    int cnt = 0;
    for (int i=0; i<4; ++i){
    	if (con->ind[i]==1){
    		Fd.block(cnt,0,3,1) = vc->fDes.block(3*i,0,3,1);
    		cnt+=3;
    	}
    }	
    q_QP.block(0,0,conDim,1) = -Fd*params->dfPen;

    // Equality constraints
    double kpGain = params->kp;
    double kdGain = params->kd;
    b_QP.block(0,0,conDim+outDim,1) << (kin->Jc)*(dyn->Dinv)*(dyn->H) - (kin->dJc),
                                     (-kpGain*(vc->y)-kdGain*(vc->dy)) + (vc->H0)*(dyn->Dinv)*(dyn->H) - (vc->dH0);

    // Inequality constraints
    Eigen::Matrix<double, 32, 1> lb, ub;
    ub.setZero();
    lb.setZero();
    ub.block(20, 0, TOTAL_IN, 1) << sat,sat,sat,sat;
    lb.block( 0, 0,       20, 1) << -OSQP_INFTY*Eigen::Matrix<double,20,1>::Ones();
    lb.block(20, 0, TOTAL_IN, 1) << -sat,-sat,-sat,-sat;

    ub_QP << b_QP, ub;
    lb_QP << b_QP, lb;

    // ====================================================================== //
    // ============================= Update OSQP ============================ //
    // ====================================================================== //
    c_int q_exit = osqp_update_lin_cost(work, q_QP.data());
    c_int b_exit = osqp_update_bounds(work, lb_QP.data(), ub_QP.data());


    // std::cout<<"c"<<std::endl;
    // std::cout<<q_QP<<std::endl;
    // std::cout<<"ub"<<std::endl;
    // std::cout<<ub_QP<<std::endl;
    // std::cout<<"lb"<<std::endl;
    // std::cout<<lb_QP<<std::endl<<std::endl;
}
#endif

// General functions
void LowLevelCtrl::saturateTorque(){
    // Joint Torque Saturation
    for(size_t i=0; i<4; i++){
        tau[6+3*i]   =   tau[6+3*i]>sat[0] ? sat[0] : ( (  tau[6+3*i]<-1*sat[0]) ? -1*sat[0] :   tau[6+3*i] );
        tau[6+3*i+1] = tau[6+3*i+1]>sat[1] ? sat[1] : ( (tau[6+3*i+1]<-1*sat[1]) ? -1*sat[1] : tau[6+3*i+1] );
        tau[6+3*i+2] = tau[6+3*i+2]>sat[2] ? sat[2] : ( (tau[6+3*i+2]<-1*sat[2]) ? -1*sat[2] : tau[6+3*i+2] );
    }
}

void LowLevelCtrl::swingInvKin(const StateInfo *state, const DynInf *dyn, const KinInf *kin, const VCInfo *vc, const ConInf *con, LLP *params){
    // ===== General Equations ===== //
    // dxde_h = dxde - dxh      \dot{x}_des w.r.t hip where \dot{x}_des is desired foot vel (all in global)
    //  xde_h =  xde -  xh      x_des w.r.t hip where x_des is desired foot pos (all in global)
    //   xe_h =   xe -  xh      x w.r.t hip where x is the current foor pos (all in global)

    size_t swDim = 3*(4-con->cnt);
    size_t conDim = 12-swDim;
    Eigen::MatrixXd dxde_h = Eigen::MatrixXd::Zero(swDim,1);
    Eigen::MatrixXd xde_h  = Eigen::MatrixXd::Zero(swDim,1);
    Eigen::MatrixXd xe_h   = Eigen::MatrixXd::Zero(swDim,1);
    Eigen::MatrixXd Jtemp  = Eigen::MatrixXd::Zero(3,TOTAL_DOF);
    Eigen::MatrixXd Jtheta = Eigen::MatrixXd::Zero(swDim,3);
    Eigen::MatrixXd Jq     = Eigen::MatrixXd::Zero(swDim,swDim);
    Eigen::MatrixXd dq     = Eigen::MatrixXd::Zero(swDim,1);
    size_t cnts = 0;
    for (int i=0;  i<4; ++i){
        if(con->ind[i]==0){
            dxde_h.block(cnts,0,3,1) = vc->dhd.block(6+cnts,0,3,1) - (kin->Jhip.block(cnts,0,3,TOTAL_DOF)*state->dq);
            xde_h.block(cnts,0,3,1)  = vc->hd.block(6+cnts,0,3,1) - kin->hipPos.block(0,i,3,1);
            xe_h.block(cnts,0,3,1)   = kin->toePos.block(0,i,3,1) - kin->hipPos.block(0,i,3,1);
            Jtemp.block(0,0,3,TOTAL_DOF) = kin->Jtoe.block(3*i,0,3,TOTAL_DOF) - kin->Jhip.block(3*i,0,3,TOTAL_DOF);
            Jtheta.block(cnts,0,3,3) = Jtemp.block(0,3,3,3);
            Jq.block(cnts,cnts,3,3)  = Jtemp.block(0,6+3*i,3,3);
            cnts+=3;
        }
    }
    dq = Jq.inverse()*( dxde_h + 20*(xde_h - xe_h )- Jtheta*state->dq.block(3,0,3,1)); //

    cnts = 0;
    for (int i=0; i<4; ++i){
        if(con->ind[i]==0){
            ll.dq.block(6+3*i,0,3,1) = dq.block(cnts,0,3,1);
            ll.q.block(6+3*i,0,3,1)  = state->q.block(6+3*i,0,3,1)+dq.block(cnts,0,3,1)/LL_Hz;
            cnts+=3;
        }
    }
}

