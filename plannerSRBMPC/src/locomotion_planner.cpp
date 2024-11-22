#include "a1_controller/locomotion_planner.h"

LocomotionPlanner::LocomotionPlanner(): FILE_RW(false), compositesrbs_(false), lambda_bias_(false){
    if(FILE_RW){
        for (size_t i=0; i< FILE_CNT; i++){
            file[i].open(FILE_NAMES[i].c_str(), std::ios_base::out);
        }
    }
    controlTick_ = 0;
    locomotionTick_ = 0;
    mpcTick_ = 0;

    init_x_ = 0.0;
    init_y_ = -1.0;

    // holonomic constraint applied position wrt COM of robot
    d1_ << 0.0, 0.0, 0.15;
    d2_ << 0.0, 0.0, 0.15;

    mu_ = 0.45; // friciton coefficient    
    dt_ = 0.005;//0.005; // sim time step
    g_ = 9.81; // acc due to gravity

    n_hor_ = 5; // planning horizon
    distributed_hor_ = 5;//n_hor_; //5 shows the best results
    if(n_hor_ < distributed_hor_){
        std::cout << "Control Horizon of centralized MPC is shorter than Distributed MPC!!!!"<<std::endl; 
        std::cout << "This makes data parsing error at the early stage of generating desired trajectories etc" << std::endl;
    }

    // MPC linear system number of states and control (centralized)
    n_state_ = 24; // 6 + 6 + 6 + 6 : compos(3)x2 + comvel(3)x2 + rot(3)x2 + omega(3)x2
    n_holonomic_ = 1; // number of lambda
    n_inputs_ = 24; //12 + 12: contactforce(3)x4legs + contactforce(3)x4legs 
    n_decision_vars_ = n_inputs_ + n_state_ + n_holonomic_;

    // MPC linear system number of states and control (distributed)
    reduced_state_ = n_state_  * 0.5;
    reduced_holonomic_ = n_holonomic_;
    reduced_inputs_ = n_inputs_ * 0.5;
    reduced_decision_vars_ = reduced_inputs_ + reduced_state_ + reduced_holonomic_;

    Tst_ = 0.2;//0.15; // swing time

    // =========================== A1 robot properties ========================== //
    mass_ = 12.4530;
    J_ << 0.01683993,    8.3902e-5,     0.000597679 ,    
          8.3902e-5,     0.056579028,   2.5134e-5 ,     
          0.000597679,   2.5134e-5,     0.064713601;

    phiprf_local_ <<  0.183,  -0.1321,  0.0;
    phiplf_local_ <<  0.183,   0.1321,  0.0;
    phiprh_local_ << -0.183,  -0.1321,  0.0;
    phiplh_local_ << -0.183,   0.1321,  0.0;

    zheight_ = 0.26;

    // ================ operation points ==================== //
    state_op_ << 0.0, 0.0, zheight_,    init_x_, init_y_, zheight_,   
                 0, 0, 0,    0, 0, 0,   
                 1,0,0, 0,1,0, 0,0,1,   1,0,0, 0,1,0, 0,0,1,    
                 0, 0, 0,    0, 0, 0;
    // state_op_ << 0.0, 0.0, 0.05, init_x_, init_y_, 0.05,   
    //              0,0,0,0,0,0,   1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1,    0,0,0,0,0,0;
    state_op_variational_.block(0,0,12,1) = state_op_.block(0,0,12,1);
    state_op_variational_.block(12,0,12,1) = Eigen::MatrixXd::Zero(12,1);
    rot1_op_.setIdentity(3,3);
    rot2_op_.setIdentity(3,3);
    lambda_op_ = 0.0;
    lambda_op0_ = 0.0;
    lambda_op1_ = 0.0;

    p_foot_global_ <<  0.183, -0.1321, 0.0,                  0.183,0.1321, 0.0,                  
               -0.183, -0.1321, 0.0,                 -0.183,0.1321, 0.0,   
                0.183+init_x_, -0.1321+init_y_, 0.0,    0.183+init_x_, 0.1321+init_y_, 0.0,   
               -0.183+init_x_, -0.1321+init_y_, 0.0,   -0.183+init_x_, 0.1321+init_y_, 0.0;
    p_foot_nominal_ <<  0.183, -0.1321, 0.0,    0.183,0.1321, 0.0,                  
                       -0.183, -0.1321, 0.0,   -0.183,0.1321, 0.0,   
                        0.183, -0.1321, 0.0,    0.183, 0.1321, 0.0,   
                       -0.183, -0.1321, 0.0,   -0.183, 0.1321, 0.0;
    p_foot_ = p_foot_nominal_;
    p_foot1_ = p_foot_global_.block(0,0,12,1);
    p_foot2_ = p_foot_global_.block(12,0,12,1);
    
    fop_nominal_ << 0,0,mass_*g_*0.25,    0,0,mass_*g_*0.25,    0,0,mass_*g_*0.25,    0,0,mass_*g_*0.25,    
                    0,0,mass_*g_*0.25,    0,0,mass_*g_*0.25,    0,0,mass_*g_*0.25,    0,0,mass_*g_*0.25;
    //fop_nominal_ = 2.0*fop_nominal_;
    if(compositesrbs_){
        fop_nominal_ = 2.0*fop_nominal_;
    }
    //fop_ = fop_nominal_;
    //fop_ = 2 * fop_;
    // ========= foot state =========== //
    foot_state1_ << 0,1,1,0;//1,0,0,1;//1, 0, 0, 1; // compositetrot: 0,1,0,1;
    foot_state2_ << 0,1,1,0;//1,0,0,1;//1, 0, 0, 1; //compositetrot:  1,0,1,0;
    if(compositesrbs_){
        foot_state1_ << 0,1,0,1;
        foot_state2_ << 1,0,1,0;
    }
    fop_.block(0,0,3,1) = fop_nominal_.block(0,0,3,1) * foot_state1_(0);
    fop_.block(3,0,3,1) = fop_nominal_.block(3,0,3,1) * foot_state1_(1);
    fop_.block(6,0,3,1) = fop_nominal_.block(6,0,3,1) * foot_state1_(2);
    fop_.block(9,0,3,1) = fop_nominal_.block(9,0,3,1) * foot_state1_(3);
    fop_.block(12,0,3,1) = fop_nominal_.block(12,0,3,1) * foot_state2_(0);
    fop_.block(15,0,3,1) = fop_nominal_.block(15,0,3,1) * foot_state2_(1);
    fop_.block(18,0,3,1) = fop_nominal_.block(18,0,3,1) * foot_state2_(2);
    fop_.block(21,0,3,1) = fop_nominal_.block(21,0,3,1) * foot_state2_(3);

    // ========= srb state =========== //
    body1pos_.setZero();
    body1rot_.setZero(3,3);
    body1vel_.setZero();
    body1omega_.setZero();

    body2pos_.setZero();
    body2rot_.setZero(3,3);
    body2vel_.setZero();
    body2omega_.setZero();

    // ========= desired state design portion ========= //
    desired_comvel_ << 0.15, 0.0, 0.0;
    desired_comvel0_ = desired_comvel_;
    desired_comvel1_ = desired_comvel_;
    desired_state_.setZero(36, n_hor_);
    desired_input_.setZero(n_inputs_, n_hor_);
    desired_lambda_ = 0.0;

    // ========= agent Identification ========== //
    agentID_ = 0;
    totalagentnumber_ = 2;

    opt_sol_e_.setZero(n_hor_* n_decision_vars_, 1);
    opt_u_.setZero(n_inputs_,1);
    kktmultiplier_e_.setZero(n_hor_*(n_state_ + n_holonomic_),1);

    dist_opt_sol_e_.setZero(distributed_hor_ * reduced_decision_vars_, 1);
    dist_opt_sol0_e_ = dist_opt_sol_e_;
    dist_opt_sol1_e_ = dist_opt_sol_e_;
    dist_opt_u_.setZero(reduced_inputs_,1);
    kktmultiplier0dist_.setZero(distributed_hor_ * (reduced_state_ + reduced_holonomic_),1);
    kktmultiplier1dist_.setZero(distributed_hor_ * (reduced_state_ + reduced_holonomic_),1);

    indxupdatefromfullorder_ = false;
}

void LocomotionPlanner::locotickReset(){
    locomotionTick_ =0;
}

void LocomotionPlanner::mpctickReset(){
    mpcTick_ = 0;
}

void LocomotionPlanner::timeSetup(size_t standing_duration, size_t loco_start){
    standingDuration_ = standing_duration;
    locoStart_ = loco_start;
}

void LocomotionPlanner::agentidsetup(const size_t numberofagent, const size_t agentID){
    agentID_ = agentID;
    totalagentnumber_ = numberofagent;
}

void LocomotionPlanner::getRobotState(const Eigen::VectorXd& pos1, const Eigen::VectorXd& pos2, 
                                      const Eigen::VectorXd& vel1, const Eigen::VectorXd& vel2,
                                      const Eigen::MatrixXd& rot1, const Eigen::MatrixXd& rot2,
                                      const Eigen::VectorXd& omega1, const Eigen::VectorXd& omega2, const size_t ctrlTick){
    body1pos_ = pos1; body2pos_ = pos2;
    body1vel_ = vel1; body2vel_ = vel2;
    body1rot_ = rot1; body2rot_ = rot2;
    body1omega_ = omega1; body2omega_ = omega2;
    controlTick_ = ctrlTick;
    //std::cout << body1pos_.transpose() << "      " <<body2pos_.transpose()<< std::endl;

    Eigen::Vector3d xi1, xi2;
    veemap(rot1_op_.transpose() * body1rot_ - Eigen::MatrixXd::Identity(3,3), xi1);
    veemap(rot2_op_.transpose() * body2rot_ - Eigen::MatrixXd::Identity(3,3), xi2);
    state_.block(0,0,3,1) = body1pos_;
    state_.block(3,0,3,1) = body2pos_;
    state_.block(6,0,3,1) = body1vel_;
    state_.block(9,0,3,1) = body2vel_;
    state_.block(12,0,3,1) = xi1;
    state_.block(15,0,3,1) = xi2;
    state_.block(18,0,3,1) = body1omega_;
    state_.block(21,0,3,1) = body2omega_;
    //std::cout << xi1.transpose() <<"      "<<xi2.transpose()<<std::endl;
}

void LocomotionPlanner::getRobotStatewindx(const Eigen::VectorXd& pos1, const Eigen::VectorXd& pos2, 
                                      const Eigen::VectorXd& vel1, const Eigen::VectorXd& vel2,
                                      const Eigen::MatrixXd& rot1, const Eigen::MatrixXd& rot2,
                                      const Eigen::VectorXd& omega1, const Eigen::VectorXd& omega2, const size_t ctrlTick,
                                      int* condIndx0, int* condIndx1){
    body1pos_ = pos1; body2pos_ = pos2;
    body1vel_ = vel1; body2vel_ = vel2;
    body1rot_ = rot1; body2rot_ = rot2;
    body1omega_ = omega1; body2omega_ = omega2;
    controlTick_ = ctrlTick;
    //std::cout << body1pos_.transpose() << "      " <<body2pos_.transpose()<< std::endl;

    Eigen::Vector3d xi1, xi2;
    veemap(rot1_op_.transpose() * body1rot_ - Eigen::MatrixXd::Identity(3,3), xi1);
    veemap(rot2_op_.transpose() * body2rot_ - Eigen::MatrixXd::Identity(3,3), xi2);
    state_.block(0,0,3,1) = body1pos_;
    state_.block(3,0,3,1) = body2pos_;
    state_.block(6,0,3,1) = body1vel_;
    state_.block(9,0,3,1) = body2vel_;
    state_.block(12,0,3,1) = xi1;
    state_.block(15,0,3,1) = xi2;
    state_.block(18,0,3,1) = body1omega_;
    state_.block(21,0,3,1) = body2omega_;
    //std::cout << xi1.transpose() <<"      "<<xi2.transpose()<<std::endl;

    for(size_t i =0; i<4; i++){
        foot_state1_(i) = condIndx0[i];
        foot_state2_(i) = condIndx1[i];
    }
    indxupdatefromfullorder_ = true;
}

void LocomotionPlanner::updateStateop(){
    Eigen::VectorXd rot1vec(Eigen::Map<Eigen::VectorXd>(body1rot_.data(), body1rot_.cols()* body1rot_.rows())); //column major
    Eigen::VectorXd rot2vec(Eigen::Map<Eigen::VectorXd>(body2rot_.data(), body2rot_.cols()* body2rot_.rows())); //column major
    state_op_.block(0,0,3,1) = body1pos_;
    state_op_.block(3,0,3,1) = body2pos_;
    state_op_.block(6,0,3,1) = body1vel_;
    state_op_.block(9,0,3,1) = body2vel_;
    state_op_.block(12,0,9,1) = rot1vec;
    state_op_.block(21,0,9,1) = rot2vec;
    state_op_.block(30,0,3,1) = body1omega_;
    state_op_.block(33,0,3,1) = body2omega_;
    rot1_op_ = body1rot_;
    rot2_op_ = body2rot_;

    Eigen::Vector3d xi1_op, xi2_op;
    veemap(rot1_op_.transpose() * body1rot_ - Eigen::MatrixXd::Identity(3,3), xi1_op);
    veemap(rot2_op_.transpose() * body2rot_ - Eigen::MatrixXd::Identity(3,3), xi2_op);
    // veemap(body1rot_ - Eigen::MatrixXd::Identity(3,3), xi1_op);
    // veemap(body2rot_ - Eigen::MatrixXd::Identity(3,3), xi2_op);
    state_op_variational_.block(0,0,3,1) = body1pos_;
    state_op_variational_.block(3,0,3,1) = body2pos_;
    state_op_variational_.block(6,0,3,1) = body1vel_;
    state_op_variational_.block(9,0,3,1) = body2vel_;
    state_op_variational_.block(12,0,3,1) = xi1_op;
    state_op_variational_.block(15,0,3,1) = xi2_op;
    state_op_variational_.block(18,0,3,1) = body1omega_;
    state_op_variational_.block(21,0,3,1) = body2omega_;

    Eigen::MatrixXd lambdaop;
    lambdaop.setZero(1,1);
    lambda_op_closed_form(lambdaop, state_op_variational_, fop_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_); 
    //std::cout <<lambdaop << std::endl<< std::endl;
    lambda_op_ = lambdaop(0,0);
    if(agentID_ == 0){
        lambda_op0_ = lambdaop(0,0);
        lambda_op1_ = lambdaop(0,0);

        // lambda_bias_mag_ = 50;
        // lambda_bias_tick_ = 80000;
        // if(controlTick_-locoStart_ >=1000 && controlTick_-locoStart_ <= lambda_bias_tick_ && lambda_bias_){
        //     lambda_op0_ = lambda_bias_mag_;
        // }
        // else{
        //     lambda_op0_ = lambdaop(0,0);
        // }

    }
    else if(agentID_ == 1){
        lambda_op0_ = lambdaop(0,0);
        lambda_bias_mag_ = 58000;//48000;
        lambda_bias_tick_ = 80000;
        if(controlTick_-locoStart_ >=1000 && controlTick_-locoStart_ <= lambda_bias_tick_ && lambda_bias_){
            lambda_op0_ = lambda_bias_mag_;
        }
        else{
            lambda_op0_ = lambdaop(0,0);
        }

        lambda_op1_ = lambdaop(0,0);
        // lambda_bias_mag_ = 50;
        // lambda_bias_tick_ = 80000;
        // if(controlTick_-locoStart_ >=1000 && controlTick_-locoStart_ <= lambda_bias_tick_ && lambda_bias_){
        //     lambda_op1_ = lambda_bias_mag_;
        // }
        // else{
        //     lambda_op1_ = lambdaop(0,0);
        // }
    }
    //lambda_op_ = 0; //TEMP
}

void LocomotionPlanner::motionPlanner(){
    double kkx0, kky0, kkz0, kkx1, kky1, kkz1;
    double dkkx0, dkky0, dkkz0, dkkx1, dkky1, dkkz1;
    size_t sustaincurrentxy = 1;
    Eigen::Matrix<double, 36, 1> desired_evolve;
    desired_evolve.setZero(36,1);
    for(size_t i= 0; i< n_hor_; i++){
        kkx0 = desired_comvel0_(0)*dt_ * (i+1);
        kky0 = desired_comvel0_(1)*dt_ * (i+1);
        kkz0 = desired_comvel0_(2)*dt_ * (i+1);
        dkkx0 = desired_comvel0_(0);
        dkky0 = desired_comvel0_(1);
        dkkz0 = desired_comvel0_(2);
        kkx1 = desired_comvel1_(0)*dt_ * (i+1);
        kky1 = desired_comvel1_(1)*dt_ * (i+1);
        kkz1 = desired_comvel1_(2)*dt_ * (i+1);
        dkkx1 = desired_comvel1_(0);
        dkky1 = desired_comvel1_(1);
        dkkz1 = desired_comvel1_(2);

        desired_evolve.block(0,0,12,1) << kkx0 + state_op_(0), 
                                          kky0 + state_op_(1), 
                                          kkz0 + zheight_, 
                                          kkx1 + state_op_(3) * sustaincurrentxy + (state_op_(0) + init_x_) * (1-sustaincurrentxy), 
                                          kky1 + state_op_(4) * sustaincurrentxy + (state_op_(1) + init_y_) * (1-sustaincurrentxy),
                                          kkz1 + zheight_, 
                                          dkkx0, dkky0, dkkz0, dkkx1, dkky1, dkkz1;
        desired_evolve.block(12,0,24,1) << 1,0,0, 0,1,0, 0,0,1,    1,0,0, 0,1,0, 0,0,1,   0,0,0, 0,0,0;

        desired_state_.block(0,i,12,1) = desired_evolve.block(0,0,12,1);
        desired_state_.block(12,i,24,1) = desired_evolve.block(12,0,24,1);

        desired_input_.block(0, i, n_inputs_, 1) = fop_nominal_;
    }
    //std::cout << desired_state_.transpose() << std::endl <<std::endl;
}

void LocomotionPlanner::footstepPlanner(){
    double raibert_gain = 0.04;//sqrt(zheight_/g_);

    // hip position wrt global coordinate
    p_hip_.block( 0,0,3,1) = state_op_.block(0,0,3,1) + body1rot_ * phiprf_local_;
    p_hip_.block( 3,0,3,1) = state_op_.block(0,0,3,1) + body1rot_ * phiplf_local_;
    p_hip_.block( 6,0,3,1) = state_op_.block(0,0,3,1) + body1rot_ * phiprh_local_;
    p_hip_.block( 9,0,3,1) = state_op_.block(0,0,3,1) + body1rot_ * phiplh_local_;
    
    p_hip_.block(12,0,3,1) = state_op_.block(3,0,3,1) + body2rot_ * phiprf_local_;
    p_hip_.block(15,0,3,1) = state_op_.block(3,0,3,1) + body2rot_ * phiplf_local_;
    p_hip_.block(18,0,3,1) = state_op_.block(3,0,3,1) + body2rot_ * phiprh_local_;
    p_hip_.block(21,0,3,1) = state_op_.block(3,0,3,1) + body2rot_ * phiplh_local_;

    // desired foot position wrt global coordinate - this stage is necessary because raibert needs to be calculated based on the hip position
    p_foot_global_.block( 0,0,2,1) = p_hip_.block( 0,0,2,1) + desired_state_.block(6,0,2,1)*Tst_*0.5 + raibert_gain * (state_op_.block(6,0,2,1)-desired_state_.block(6,0,2,1));
    p_foot_global_.block( 3,0,2,1) = p_hip_.block( 3,0,2,1) + desired_state_.block(6,0,2,1)*Tst_*0.5 + raibert_gain * (state_op_.block(6,0,2,1)-desired_state_.block(6,0,2,1));
    p_foot_global_.block( 6,0,2,1) = p_hip_.block( 6,0,2,1) + desired_state_.block(6,0,2,1)*Tst_*0.5 + raibert_gain * (state_op_.block(6,0,2,1)-desired_state_.block(6,0,2,1));
    p_foot_global_.block( 9,0,2,1) = p_hip_.block( 9,0,2,1) + desired_state_.block(6,0,2,1)*Tst_*0.5 + raibert_gain * (state_op_.block(6,0,2,1)-desired_state_.block(6,0,2,1));
    
    p_foot_global_.block(12,0,2,1) = p_hip_.block(12,0,2,1) + desired_state_.block(9,0,2,1)*Tst_*0.5 + raibert_gain * (state_op_.block(9,0,2,1)-desired_state_.block(9,0,2,1));
    p_foot_global_.block(15,0,2,1) = p_hip_.block(15,0,2,1) + desired_state_.block(9,0,2,1)*Tst_*0.5 + raibert_gain * (state_op_.block(9,0,2,1)-desired_state_.block(9,0,2,1));
    p_foot_global_.block(18,0,2,1) = p_hip_.block(18,0,2,1) + desired_state_.block(9,0,2,1)*Tst_*0.5 + raibert_gain * (state_op_.block(9,0,2,1)-desired_state_.block(9,0,2,1));
    p_foot_global_.block(21,0,2,1) = p_hip_.block(21,0,2,1) + desired_state_.block(9,0,2,1)*Tst_*0.5 + raibert_gain * (state_op_.block(9,0,2,1)-desired_state_.block(9,0,2,1));
    
    //std::cout << p_foot_global_.transpose() << std::endl <<std::endl;
    
    // ===== foot hold usage without raibert ===== //
    // p_foot_.block( 0,0,3,1) = body1rot_ * phiprf_local_;
    // p_foot_.block( 3,0,3,1) = body1rot_ * phiplf_local_;
    // p_foot_.block( 6,0,3,1) = body1rot_ * phiprh_local_;
    // p_foot_.block( 9,0,3,1) = body1rot_ * phiplh_local_;

    // p_foot_.block(12,0,3,1) = body2rot_ * phiprf_local_;
    // p_foot_.block(15,0,3,1) = body2rot_ * phiplf_local_;
    // p_foot_.block(18,0,3,1) = body2rot_ * phiprh_local_;
    // p_foot_.block(21,0,3,1) = body2rot_ * phiplh_local_;
    // for(size_t i = 0; i<4; i++){
    //     p_foot_(i*3+2) -= state_op_(2);
    //     p_foot_(12+i*3+2) -= state_op_(5);
    // }

    // ===== foot hold usage with raibert ===== //
    // desired foot position wrt each robot's local coordinate (body coordinate)
    size_t zinclude = 3;
    p_foot_.block( 0,0,zinclude,1) = p_foot_global_.block( 0,0,zinclude,1) - state_op_.block(0,0,zinclude,1);
    p_foot_.block( 3,0,zinclude,1) = p_foot_global_.block( 3,0,zinclude,1) - state_op_.block(0,0,zinclude,1);
    p_foot_.block( 6,0,zinclude,1) = p_foot_global_.block( 6,0,zinclude,1) - state_op_.block(0,0,zinclude,1);
    p_foot_.block( 9,0,zinclude,1) = p_foot_global_.block( 9,0,zinclude,1) - state_op_.block(0,0,zinclude,1);
    
    p_foot_.block(12,0,zinclude,1) = p_foot_global_.block(12,0,zinclude,1) - state_op_.block(3,0,zinclude,1);
    p_foot_.block(15,0,zinclude,1) = p_foot_global_.block(15,0,zinclude,1) - state_op_.block(3,0,zinclude,1);
    p_foot_.block(18,0,zinclude,1) = p_foot_global_.block(18,0,zinclude,1) - state_op_.block(3,0,zinclude,1);
    p_foot_.block(21,0,zinclude,1) = p_foot_global_.block(21,0,zinclude,1) - state_op_.block(3,0,zinclude,1);

    //std::cout <<p_foot_.transpose() << std::endl;
}

void LocomotionPlanner::srbMPC(){
    // ================== decision variable order ============ //
    // u(t) X(t+1) lambda(t+1) u(t+1) X(t+2) lambda(t+2) . . . u(t+T-1) X(t+T) lambda(t+T)
    // ================== weights =================== //
    Eigen::Matrix<double, 6, 6> qx, qxf;
    qx.setIdentity(6,6); 
    qx = 3e5 * qx; 
    //qx(2,2) = 3e6; 
    //qx(5,5) = 3e6;
    qx(2,2) = 3e6; 
    qx(5,5) = 3e6;

    qx(1,1) = 3e7; 
    qx(2,2) = 3e7;
    Eigen::Matrix<double, 6, 6> qv, qvf;
    qv.setIdentity(6,6); 
    qv = 1e4 * qv;
    Eigen::Matrix<double, 6, 6> qr, qrf;
    qr.setIdentity(6,6);
    qr = 1e8 * qr;
    Eigen::Matrix<double, 6, 6> qw, qwf;
    qw.setIdentity(6,6);
    qw = 5e3 * qw;
    double qlambda, qlambdaf, rx, rv, rr, rw;
    qlambda = 1e4;

    //nomial stagecostgain: 0 // finalcostgain: 1 // inputgain: -2
    double stagecostgain  =  0;
    double finalcostcoeff =  1;
    double finalcostgain  = -1;
    double inputgain      = -2;

    double consensus = 1e1;

    qx = qx * pow(10, stagecostgain);
    qv = qv * pow(10, stagecostgain);
    qr = qr * pow(10, stagecostgain);
    qw = qw * pow(10, stagecostgain);
    qlambda = qlambda * pow(10, stagecostgain);

    qxf = qx * finalcostcoeff * pow(10, finalcostgain);
    qvf = qv * finalcostcoeff * pow(10, finalcostgain);
    qrf = qr * finalcostcoeff * pow(10, finalcostgain);
    qwf = qw * finalcostcoeff * pow(10, finalcostgain);
    qlambdaf = qlambda * finalcostcoeff * pow(10, finalcostgain);

    rx = pow(10, inputgain);
    rv = pow(10, inputgain);
    rr = pow(10, inputgain);
    rw = pow(10, inputgain);

    Eigen::MatrixXd cost_Q;
    Eigen::MatrixXd cost_Qf;
    Eigen::MatrixXd cost_R;
    cost_Q.setZero(n_state_ + n_holonomic_ , n_state_ + n_holonomic_);
    cost_Qf.setZero(n_state_ + n_holonomic_ , n_state_ + n_holonomic_);
    cost_R.setZero(n_inputs_ , n_inputs_);

    cost_Q.block(0,0,6,6) = qx;
    cost_Q.block(6,6,6,6) = qv;
    cost_Q.block(12,12,6,6) = qr;
    cost_Q.block(18,18,6,6) = qw;
    cost_Q(24,24) = qlambda + consensus;

    cost_Qf.block(0,0,6,6) = qxf;
    cost_Qf.block(6,6,6,6) = qvf;
    cost_Qf.block(12,12,6,6) = qrf;
    cost_Qf.block(18,18,6,6) = qwf;
    cost_Qf(24,24) = qlambdaf + consensus;

    cost_R.block(0,0,6,6) = rx * Eigen::MatrixXd::Identity(6,6);
    cost_R.block(6,6,6,6) = rv * Eigen::MatrixXd::Identity(6,6);
    cost_R.block(12,12,6,6) = rr * Eigen::MatrixXd::Identity(6,6);
    cost_R.block(18,18,6,6) = rw * Eigen::MatrixXd::Identity(6,6);

    // ======================= cost function ======================== //
    size_t idx_x = 0;
    size_t idx_u = 0;
    size_t sizeofHg = n_hor_ * n_decision_vars_;

    Eigen::MatrixXd H;
    Eigen::MatrixXd g;

    H.setZero(sizeofHg, sizeofHg);
    g.setZero(sizeofHg, 1);

    Eigen::MatrixXd rot1_desired;
    Eigen::MatrixXd rot2_desired;
    rot1_desired.setIdentity(3,3);
    rot2_desired.setIdentity(3,3);
    Eigen::VectorXd rot1_desired_vec;
    Eigen::VectorXd rot2_desired_vec;
    rot1_desired_vec.setZero(9);
    rot2_desired_vec.setZero(9);

    Eigen::VectorXd omega1_desired;
    Eigen::VectorXd omega2_desired;
    omega1_desired.setZero(3);
    omega2_desired.setZero(3);

    Eigen::MatrixXd rottemp1;
    rottemp1.setZero(3,3);
    Eigen::MatrixXd rottemp2;
    rottemp2.setZero(3,3);
    Eigen::MatrixXd temp1;
    temp1.setZero(6,1);
    Eigen::MatrixXd temp2;
    temp2.setZero(6,1);
    Eigen::Vector3d temp11, temp12;
    temp11.setZero();
    temp12.setZero();

    for(size_t i_hor = 0; i_hor< n_hor_; i_hor++){

        rot1_desired_vec = desired_state_.block(12, i_hor, 9,1);
        rot2_desired_vec = desired_state_.block(21, i_hor, 9,1);

        rot1_desired = Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::ColMajor> >(rot1_desired_vec.data());
        rot2_desired = Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::ColMajor> >(rot2_desired_vec.data());
        omega1_desired = desired_state_.block(30, i_hor, 3,1);
        omega2_desired = desired_state_.block(33, i_hor, 3,1);
        rottemp1 = rot1_desired.transpose() * rot1_op_;
        rottemp2 = rot2_desired.transpose() * rot2_op_;
        veemap(rottemp1.log(), temp11);
        veemap(rottemp2.log(), temp12);
        temp1.block(0,0,3,1) = temp11;
        temp1.block(3,0,3,1) = temp12;
        temp2.block(0,0,3,1) = rot1_op_.transpose() * rot1_desired * omega1_desired; //omega1_desired; // rot1_desired * rot1_op_.transpose() * omega1_desired; //
        temp2.block(3,0,3,1) = rot2_op_.transpose() * rot2_desired * omega2_desired; //omega2_desired; // rot2_desired * rot2_op_.transpose() * omega2_desired; //

        idx_u = i_hor * n_decision_vars_;
        idx_x = i_hor * n_decision_vars_ + n_inputs_;
        if(i_hor == n_hor_ -1){
            H.block(idx_x, idx_x, n_state_ + n_holonomic_, n_state_ + n_holonomic_) = cost_Qf;
            g.block(idx_x     , 0, 6, 1) = -qxf * desired_state_.block(0, i_hor, 6, 1);
            g.block(idx_x +  6, 0, 6, 1) = -qvf * desired_state_.block(6, i_hor, 6, 1);
            g.block(idx_x + 12, 0, 6, 1) = qrf * temp1;
            g.block(idx_x + 18, 0, 6, 1) = -qwf * temp2;
            g(idx_x + 24) = -qlambdaf * desired_lambda_ - consensus*0.5*lambda_op0_ - consensus*0.5*lambda_op1_; //qlambdaf * (0*lambda_op_ - desired_lambda_);
        }
        else{
            H.block(idx_x, idx_x, n_state_ + n_holonomic_, n_state_ + n_holonomic_) = cost_Q;
            g.block(idx_x     , 0, 6, 1) = -qx * desired_state_.block(0, i_hor, 6, 1);
            g.block(idx_x +  6, 0, 6, 1) = -qv * desired_state_.block(6, i_hor, 6, 1);
            g.block(idx_x + 12, 0, 6, 1) = qr * temp1;
            g.block(idx_x + 18, 0, 6, 1) = -qw * temp2;
            g(idx_x + 24) = -qlambda * desired_lambda_ - consensus*0.5*lambda_op0_ - consensus*0.5*lambda_op1_; //qlambda * (0*lambda_op_ - desired_lambda_);
        }
        H.block(idx_u, idx_u, n_inputs_, n_inputs_) = cost_R;
        g.block(idx_u, 0, n_inputs_, 1) = cost_R.transpose() * (fop_- desired_input_.block(0, i_hor, n_inputs_, 1));    
    }

    // =================== inequality constraints ================== //
    double NEQ = 32 + 16; 

    Eigen::MatrixXd lb;
    Eigen::MatrixXd ub;

    Eigen::MatrixXd Fu;
    Eigen::MatrixXd Fu_x_y;
    Eigen::MatrixXd zeroandones;
    
    Eigen::MatrixXd Aineq;
    Eigen::MatrixXd bineq;
    Eigen::MatrixXd fconeaugment;

    lb.setZero(8,1);
    ub.setZero(8,1);
    lb << desired_input_(2), desired_input_(5),desired_input_(8),desired_input_(11),desired_input_(14),desired_input_(17),desired_input_(20),desired_input_(23);
    ub << desired_input_(2), desired_input_(5),desired_input_(8),desired_input_(11),desired_input_(14),desired_input_(17),desired_input_(20),desired_input_(23);
    lb = -2 * lb * 1e0;
    ub =  2 * ub * 1e0;

    Fu.setZero(NEQ, n_inputs_);
    Fu_x_y.setZero(4,3);
    zeroandones.setZero(2,3);
    
    Aineq.setZero(n_hor_ * NEQ, n_hor_ * n_decision_vars_);
    bineq.setZero(n_hor_ * NEQ, 1);
    fconeaugment.setZero(32,1);

    Fu_x_y <<  1,  0, -(mu_/sqrt(2)),  
              -1,  0, -(mu_/sqrt(2)),  
               0,  1, -(mu_/sqrt(2)),   
               0, -1, -(mu_/sqrt(2));
    zeroandones << 0,0,1,0,0,-1;

    for(size_t i=0; i<8; i++){
        Fu.block(i*4,i*3, 4,3) = Fu_x_y;
        Fu.block(32+ i*2, i*3,2,3) = zeroandones;
    }

    for(size_t ii=0; ii<n_hor_; ii++){
        Aineq.block(NEQ * ii, n_decision_vars_ * ii, NEQ, n_inputs_ ) = Fu;
    }

    Eigen::MatrixXd frictioncone;
    frictioncone.setZero(4,1);
    for(size_t j=0; j<8; j++){
        frictioncone << (mu_/sqrt(2)) * fop_(2 + 3*j) - fop_(0+ 3*j),
                        (mu_/sqrt(2)) * fop_(2 + 3*j) + fop_(0+ 3*j),
                        (mu_/sqrt(2)) * fop_(2 + 3*j) - fop_(1+ 3*j),
                        (mu_/sqrt(2)) * fop_(2 + 3*j) + fop_(1+ 3*j);
        
        fconeaugment.block(4*j, 0, 4,1) = frictioncone;
    }

    Eigen::MatrixXd lu_bounds;
    lu_bounds.setZero(16,1);
    for(size_t jj=0; jj< n_hor_; jj++){
        bineq.block(NEQ * jj, 0, 32,1) = fconeaugment;
        lu_bounds <<  ub(0) - fop_(2) + desired_input_(2, jj),
                     -lb(0) + fop_(2) - desired_input_(2, jj),
                      ub(1) - fop_(5) + desired_input_(5, jj),
                     -lb(1) + fop_(5) - desired_input_(5, jj),
                      ub(2) - fop_(8) + desired_input_(8, jj),
                     -lb(2) + fop_(8) - desired_input_(8, jj),
                      ub(3) - fop_(11) + desired_input_(11, jj),
                     -lb(3) + fop_(11) - desired_input_(11, jj),
                      ub(4) - fop_(14) + desired_input_(14, jj),
                     -lb(4) + fop_(14) - desired_input_(14, jj),
                      ub(5) - fop_(17) + desired_input_(17, jj),
                     -lb(5) + fop_(17) - desired_input_(17, jj),
                      ub(6) - fop_(20) + desired_input_(20, jj),
                     -lb(6) + fop_(20) - desired_input_(20, jj),
                      ub(7) - fop_(23) + desired_input_(23, jj),
                     -lb(7) + fop_(23) - desired_input_(23, jj);
        
        bineq.block(NEQ * jj + 32, 0, 16, 1) = lu_bounds;
    }

    // ================== equality constraints ==================== //
    Eigen::MatrixXd sysvec;

    Eigen::MatrixXd Ad, Bd, Cd, Dd;
    Eigen::MatrixXd Edp, Fdp, Gdp, Hdp, Edv, Fdv, Gdv, Hdv;
    Eigen::MatrixXd Ed, Fd, Gd, Hd;

    Eigen::MatrixXd Aeq1, Aeq2, Aeq3, Aeq4, Aeq;
    Eigen::MatrixXd beq1, beq2, beq3, beq4, beq;

    sysvec.setZero(24,1);
    Ad.setZero(24,24); Bd.setZero(24,24); Cd.setZero(24, 1); Dd.setZero(24, 1);
    Edp.setZero(1, 24); Fdp.setZero(1, 24); Gdp.setZero(1, 1); Hdp.setZero(1, 1);
    Edv.setZero(1, 24); Fdv.setZero(1, 24); Gdv.setZero(1, 1); Hdv.setZero(1, 1);
    Ed.setZero(1, 24); Fd.setZero(1, 24); Gd.setZero(1, 1); Hd.setZero(1, 1);

    Aeq1.setZero(n_hor_ * n_state_, n_hor_ * n_decision_vars_);
    beq1.setZero(n_hor_ * n_state_, 1);

    Aeq2.setZero(n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_);
    beq2.setZero(n_hor_ * n_holonomic_, 1);

    Aeq3.setZero(n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_);
    beq3.setZero(n_hor_ * n_holonomic_, 1);

    Aeq4.setZero(n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_);
    beq4.setZero(n_hor_ * n_holonomic_, 1);

    system_mtx_ftn(sysvec, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_sys_state(Ad, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_sys_input(Bd, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_sys_lambda(Cd, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    //Cd = Cd * 0; //TEMP
    Dd = sysvec - Ad*state_op_variational_ - Bd * fop_ - Cd * lambda_op_;
    //std::cout <<Ad << std::endl;

    jaco_cons_state(Ed, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_cons_input(Fd, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_cons_lambda(Gd, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);    
    Hd = -Ed * state_op_variational_ - Fd * fop_ - Gd * lambda_op_;

    // equality 1
    Aeq1.block(0,0, n_state_, n_inputs_) = -Bd;
    Aeq1.block(0, n_inputs_, n_state_, n_state_) = Eigen::MatrixXd::Identity(n_state_, n_state_);
    Aeq1.block(0, n_inputs_ + n_state_, n_state_, n_holonomic_) = -Cd;
    for(size_t i=0; i<n_hor_-1; i++){
        Aeq1.block(n_state_* (i+1), n_decision_vars_ * i + n_inputs_ * 1                           , n_state_, n_state_) = -Ad;
        Aeq1.block(n_state_* (i+1), n_decision_vars_ * i + n_inputs_ * 1 + n_state_               , n_state_, n_holonomic_) = -Cd;
        Aeq1.block(n_state_* (i+1), n_decision_vars_ * i + n_inputs_ * 1 + n_state_ + n_holonomic_               , n_state_, n_inputs_) = -Bd;
        Aeq1.block(n_state_* (i+1), n_decision_vars_ * i + n_inputs_ * 2 + n_state_ + n_holonomic_, n_state_, n_state_) = Eigen::MatrixXd::Identity(n_state_, n_state_);
    }

    for(size_t ii = 0; ii< n_hor_; ii++){
        if(ii == 0){
            beq1.block(n_state_ * ii, 0, n_state_, 1) = Ad * state_op_variational_ + Dd;
        }
        else{
            beq1.block(n_state_ * ii, 0, n_state_, 1) = Dd;
        }
    }

    // equality 2
    Aeq2.block(0,0, n_holonomic_, n_inputs_) = -Fd;
    Aeq2.block(0, n_inputs_ , n_holonomic_, n_state_) = Eigen::MatrixXd::Zero(n_holonomic_, n_state_);
    Aeq2.block(0, n_inputs_ + n_state_, n_holonomic_, n_holonomic_) = -Gd;
    for(size_t i=0; i<n_hor_-1; i++){
        Aeq2.block(n_holonomic_* (i+1), n_decision_vars_ * i + n_inputs_ * 1                           , n_holonomic_, n_state_) = -Ed;
        Aeq2.block(n_holonomic_* (i+1), n_decision_vars_ * i + n_inputs_ * 1 + n_state_               , n_holonomic_, n_holonomic_) = -Gd;
        Aeq2.block(n_holonomic_* (i+1), n_decision_vars_ * i + n_inputs_ * 1 + n_state_ + n_holonomic_               , n_holonomic_, n_inputs_) = -Fd;
        Aeq2.block(n_holonomic_* (i+1), n_decision_vars_ * i + n_inputs_ * 2 + n_state_ + n_holonomic_, n_holonomic_, n_state_) = Eigen::MatrixXd::Zero(n_holonomic_, n_state_);   
    }

    for(size_t ii = 0; ii< n_hor_; ii++){
        if(ii == 0){
            beq2.block(n_holonomic_ * ii, 0, n_holonomic_, 1) = Ed * state_op_variational_ + Hd;
        }
        else{
            beq2.block(n_holonomic_ * ii, 0, n_holonomic_, 1) = Hd;
        }
    }

    // ================== equality constraints construction ==================== //
    Aeq.setZero(n_hor_ * n_state_ + n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_); // 0* is TEMP
    beq.setZero(n_hor_ * n_state_ + n_hor_ * n_holonomic_, 1); // 0* is TEMP
    
    Aeq.block(0,0, n_hor_ * n_state_ , n_hor_ * n_decision_vars_) = Aeq1;
    beq.block(0,0, n_hor_ * n_state_, 1) = beq1;

    Aeq.block(n_hor_ * n_state_, 0, n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_) = Aeq2; //TEMP
    beq.block(n_hor_ * n_state_, 0, n_hor_ * n_holonomic_, 1) = beq2; //TEMP

    // ================== MPC solver ==================== //
    double* opt_sol;
    double* kkt_multipliers;
    opt_sol = new double[n_hor_* n_decision_vars_];
    kkt_multipliers = new double[n_hor_*(n_state_ + n_holonomic_)];
    //iswiftQp_e(H, g, Aeq, beq, Aineq, bineq, opt_sol);
    iswiftQp_e_lagrange(H, g, Aeq, beq, Aineq, bineq, opt_sol, kkt_multipliers);
    for(size_t i=0; i< n_hor_ * n_decision_vars_; i++){
        opt_sol_e_(i) = opt_sol[i];
    }
    for(size_t i=0; i< n_hor_*(n_state_ + n_holonomic_); i++){
        kktmultiplier_e_(i) = kkt_multipliers[i];
    }

    delete[] opt_sol;
    delete[] kkt_multipliers;

    opt_X_ = opt_sol_e_.block(24,0,24,1);
    opt_u_ = opt_sol_e_.block(0,0,24,1) + fop_;
    opt_u_.block(0,0,3,1) = opt_u_.block(0,0,3,1) * foot_state1_(0);
    opt_u_.block(3,0,3,1) = opt_u_.block(3,0,3,1) * foot_state1_(1);
    opt_u_.block(6,0,3,1) = opt_u_.block(6,0,3,1) * foot_state1_(2);
    opt_u_.block(9,0,3,1) = opt_u_.block(9,0,3,1) * foot_state1_(3);
    opt_u_.block(12,0,3,1) = opt_u_.block(12,0,3,1) * foot_state2_(0);
    opt_u_.block(15,0,3,1) = opt_u_.block(15,0,3,1) * foot_state2_(1);
    opt_u_.block(18,0,3,1) = opt_u_.block(18,0,3,1) * foot_state2_(2);
    opt_u_.block(21,0,3,1) = opt_u_.block(21,0,3,1) * foot_state2_(3);

    if(isnan(opt_u_.norm())){
    }

    if(opt_u_.block(0,0,12,1).norm() > 1e3 || opt_u_.block(12,0,12,1).norm() > 1e3){
        opt_u_ = fop_;
    }
}

void LocomotionPlanner::srbMPC_composite(){
    // ================== decision variable order ============ //
    // u(t) X(t+1) lambda(t+1) u(t+1) X(t+2) lambda(t+2) . . . u(t+T-1) X(t+T) lambda(t+T)
    // ================== weights =================== //
    Eigen::Matrix<double, 6, 6> qx, qxf;
    qx.setIdentity(6,6); 
    qx = 3e5 * qx; 
    //qx(2,2) = 3e6; 
    //qx(5,5) = 3e6;
    qx(2,2) = 3e6; 
    qx(5,5) = 3e6;

    qx(1,1) = 3e7; 
    qx(2,2) = 3e7;
    Eigen::Matrix<double, 6, 6> qv, qvf;
    qv.setIdentity(6,6); 
    //qv = 1e4 * qv;
    qv = 1e4 * qv;
    Eigen::Matrix<double, 6, 6> qr, qrf;
    qr.setIdentity(6,6);
    //qr = 1e5 * qr; // 1e3 with using hip pose
    qr = 1e8 * qr;
    Eigen::Matrix<double, 6, 6> qw, qwf;
    qw.setIdentity(6,6);
    //qw = 5e3 * qw;
    qw = 5e3 * qw;
    double qlambda, qlambdaf, rx, rv, rr, rw;
    qlambda = 1e4;//1e4; //original: 1e0

    //nomial stagecostgain: 0 // finalcostgain: 1 // inputgain: -2
    double stagecostgain  =  0;
    double finalcostcoeff =  1;
    double finalcostgain  = -1;
    double inputgain      = -2;


    double consensus = 1e1;

    qx = qx * pow(10, stagecostgain);
    qv = qv * pow(10, stagecostgain);
    qr = qr * pow(10, stagecostgain);
    qw = qw * pow(10, stagecostgain);
    qlambda = qlambda * pow(10, stagecostgain);

    qxf = qx * finalcostcoeff * pow(10, finalcostgain);
    qvf = qv * finalcostcoeff * pow(10, finalcostgain);
    qrf = qr * finalcostcoeff * pow(10, finalcostgain);
    qwf = qw * finalcostcoeff * pow(10, finalcostgain);
    qlambdaf = qlambda * finalcostcoeff * pow(10, finalcostgain);

    rx = pow(10, inputgain);
    rv = pow(10, inputgain);
    rr = pow(10, inputgain);
    rw = pow(10, inputgain);

    Eigen::MatrixXd cost_Q;
    Eigen::MatrixXd cost_Qf;
    Eigen::MatrixXd cost_R;
    cost_Q.setZero(n_state_ + n_holonomic_ , n_state_ + n_holonomic_);
    cost_Qf.setZero(n_state_ + n_holonomic_ , n_state_ + n_holonomic_);
    cost_R.setZero(n_inputs_ , n_inputs_);

    cost_Q.block(0,0,6,6) = qx;
    cost_Q.block(6,6,6,6) = qv;
    cost_Q.block(12,12,6,6) = qr;
    cost_Q.block(18,18,6,6) = qw;
    cost_Q(24,24) = qlambda + consensus;

    cost_Qf.block(0,0,6,6) = qxf;
    cost_Qf.block(6,6,6,6) = qvf;
    cost_Qf.block(12,12,6,6) = qrf;
    cost_Qf.block(18,18,6,6) = qwf;
    cost_Qf(24,24) = qlambdaf + consensus;

    cost_R.block(0,0,6,6) = rx * Eigen::MatrixXd::Identity(6,6);
    cost_R.block(6,6,6,6) = rv * Eigen::MatrixXd::Identity(6,6);
    cost_R.block(12,12,6,6) = rr * Eigen::MatrixXd::Identity(6,6);
    cost_R.block(18,18,6,6) = rw * Eigen::MatrixXd::Identity(6,6);

    // ======================= cost function ======================== //
    size_t idx_x = 0;
    size_t idx_u = 0;
    size_t sizeofHg = n_hor_ * n_decision_vars_;

    Eigen::MatrixXd H;
    Eigen::MatrixXd g;

    H.setZero(sizeofHg, sizeofHg);
    g.setZero(sizeofHg, 1);

    Eigen::MatrixXd rot1_desired;
    Eigen::MatrixXd rot2_desired;
    rot1_desired.setIdentity(3,3);
    rot2_desired.setIdentity(3,3);
    Eigen::VectorXd rot1_desired_vec;
    Eigen::VectorXd rot2_desired_vec;
    rot1_desired_vec.setZero(9);
    rot2_desired_vec.setZero(9);

    Eigen::VectorXd omega1_desired;
    Eigen::VectorXd omega2_desired;
    omega1_desired.setZero(3);
    omega2_desired.setZero(3);

    Eigen::MatrixXd rottemp1;
    rottemp1.setZero(3,3);
    Eigen::MatrixXd rottemp2;
    rottemp2.setZero(3,3);
    Eigen::MatrixXd temp1;
    temp1.setZero(6,1);
    Eigen::MatrixXd temp2;
    temp2.setZero(6,1);
    Eigen::Vector3d temp11, temp12;
    temp11.setZero();
    temp12.setZero();

    for(size_t i_hor = 0; i_hor< n_hor_; i_hor++){

        rot1_desired_vec = desired_state_.block(12, i_hor, 9,1);
        rot2_desired_vec = desired_state_.block(21, i_hor, 9,1);

        rot1_desired = Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::ColMajor> >(rot1_desired_vec.data());
        rot2_desired = Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::ColMajor> >(rot2_desired_vec.data());
        omega1_desired = desired_state_.block(30, i_hor, 3,1);
        omega2_desired = desired_state_.block(33, i_hor, 3,1);
        rottemp1 = rot1_desired.transpose() * rot1_op_;
        rottemp2 = rot2_desired.transpose() * rot2_op_;
        veemap(rottemp1.log(), temp11);
        veemap(rottemp2.log(), temp12);
        temp1.block(0,0,3,1) = temp11;
        temp1.block(3,0,3,1) = temp12;
        temp2.block(0,0,3,1) = rot1_op_.transpose() * rot1_desired * omega1_desired; 
        temp2.block(3,0,3,1) = rot2_op_.transpose() * rot2_desired * omega2_desired; 

        idx_u = i_hor * n_decision_vars_;
        idx_x = i_hor * n_decision_vars_ + n_inputs_;
        if(i_hor == n_hor_ -1){
            H.block(idx_x, idx_x, n_state_ + n_holonomic_, n_state_ + n_holonomic_) = cost_Qf;
            g.block(idx_x     , 0, 6, 1) = -qxf * desired_state_.block(0, i_hor, 6, 1);
            g.block(idx_x +  6, 0, 6, 1) = -qvf * desired_state_.block(6, i_hor, 6, 1);
            g.block(idx_x + 12, 0, 6, 1) = qrf * temp1;
            g.block(idx_x + 18, 0, 6, 1) = -qwf * temp2;
            g(idx_x + 24) = -qlambdaf * desired_lambda_ - consensus*0.5*lambda_op0_ - consensus*0.5*lambda_op1_; 
        }
        else{
            H.block(idx_x, idx_x, n_state_ + n_holonomic_, n_state_ + n_holonomic_) = cost_Q;
            g.block(idx_x     , 0, 6, 1) = -qx * desired_state_.block(0, i_hor, 6, 1);
            g.block(idx_x +  6, 0, 6, 1) = -qv * desired_state_.block(6, i_hor, 6, 1);
            g.block(idx_x + 12, 0, 6, 1) = qr * temp1;
            g.block(idx_x + 18, 0, 6, 1) = -qw * temp2;
            g(idx_x + 24) = -qlambda * desired_lambda_ - consensus*0.5*lambda_op0_ - consensus*0.5*lambda_op1_;
        }
        H.block(idx_u, idx_u, n_inputs_, n_inputs_) = cost_R;
        g.block(idx_u, 0, n_inputs_, 1) = cost_R.transpose() * (fop_- desired_input_.block(0, i_hor, n_inputs_, 1));    
    }

    // =================== inequality constraints ================== //
    double NEQ = 32 + 16; // friction cone: 32 & z direction delu bound: 16

    Eigen::MatrixXd lb;
    Eigen::MatrixXd ub;

    Eigen::MatrixXd Fu;
    Eigen::MatrixXd Fu_x_y;
    Eigen::MatrixXd zeroandones;
    
    Eigen::MatrixXd Aineq;
    Eigen::MatrixXd bineq;
    Eigen::MatrixXd fconeaugment;

    lb.setZero(8,1);
    ub.setZero(8,1);
    lb << desired_input_(2), desired_input_(5),desired_input_(8),desired_input_(11),desired_input_(14),desired_input_(17),desired_input_(20),desired_input_(23);
    ub << desired_input_(2), desired_input_(5),desired_input_(8),desired_input_(11),desired_input_(14),desired_input_(17),desired_input_(20),desired_input_(23);
    lb = -2 * lb * 1e0;
    ub =  2 * ub * 1e0;

    Fu.setZero(NEQ, n_inputs_);
    Fu_x_y.setZero(4,3);
    zeroandones.setZero(2,3);
    
    Aineq.setZero(n_hor_ * NEQ, n_hor_ * n_decision_vars_);
    bineq.setZero(n_hor_ * NEQ, 1);
    fconeaugment.setZero(32,1);

    Fu_x_y <<  1,  0, -(mu_/sqrt(2)),  
              -1,  0, -(mu_/sqrt(2)),  
               0,  1, -(mu_/sqrt(2)),   
               0, -1, -(mu_/sqrt(2));
    zeroandones << 0,0,1,0,0,-1;

    for(size_t i=0; i<8; i++){
        Fu.block(i*4,i*3, 4,3) = Fu_x_y;
        Fu.block(32+ i*2, i*3,2,3) = zeroandones;
    }

    for(size_t ii=0; ii<n_hor_; ii++){
        Aineq.block(NEQ * ii, n_decision_vars_ * ii, NEQ, n_inputs_ ) = Fu;
    }

    Eigen::MatrixXd frictioncone;
    frictioncone.setZero(4,1);
    for(size_t j=0; j<8; j++){
        frictioncone << (mu_/sqrt(2)) * fop_(2 + 3*j) - fop_(0+ 3*j),
                        (mu_/sqrt(2)) * fop_(2 + 3*j) + fop_(0+ 3*j),
                        (mu_/sqrt(2)) * fop_(2 + 3*j) - fop_(1+ 3*j),
                        (mu_/sqrt(2)) * fop_(2 + 3*j) + fop_(1+ 3*j);
        
        fconeaugment.block(4*j, 0, 4,1) = frictioncone;
    }

    Eigen::MatrixXd lu_bounds;
    lu_bounds.setZero(16,1);
    for(size_t jj=0; jj< n_hor_; jj++){
        bineq.block(NEQ * jj, 0, 32,1) = fconeaugment;
        lu_bounds <<  ub(0) - fop_(2) + desired_input_(2, jj),
                     -lb(0) + fop_(2) - desired_input_(2, jj),
                      ub(1) - fop_(5) + desired_input_(5, jj),
                     -lb(1) + fop_(5) - desired_input_(5, jj),
                      ub(2) - fop_(8) + desired_input_(8, jj),
                     -lb(2) + fop_(8) - desired_input_(8, jj),
                      ub(3) - fop_(11) + desired_input_(11, jj),
                     -lb(3) + fop_(11) - desired_input_(11, jj),
                      ub(4) - fop_(14) + desired_input_(14, jj),
                     -lb(4) + fop_(14) - desired_input_(14, jj),
                      ub(5) - fop_(17) + desired_input_(17, jj),
                     -lb(5) + fop_(17) - desired_input_(17, jj),
                      ub(6) - fop_(20) + desired_input_(20, jj),
                     -lb(6) + fop_(20) - desired_input_(20, jj),
                      ub(7) - fop_(23) + desired_input_(23, jj),
                     -lb(7) + fop_(23) - desired_input_(23, jj);
        
        bineq.block(NEQ * jj + 32, 0, 16, 1) = lu_bounds;
    }

    // ================== equality constraints ==================== //
    Eigen::MatrixXd sysvec;

    Eigen::MatrixXd Ad, Bd, Cd, Dd;
    Eigen::MatrixXd Edp, Fdp, Gdp, Hdp, Edv, Fdv, Gdv, Hdv;
    Eigen::MatrixXd Ed, Fd, Gd, Hd;

    Eigen::MatrixXd Aeq1, Aeq2, Aeq3, Aeq4, Aeq;
    Eigen::MatrixXd beq1, beq2, beq3, beq4, beq;

    sysvec.setZero(24,1);
    Ad.setZero(24,24); Bd.setZero(24,24); Cd.setZero(24, 1); Dd.setZero(24, 1);
    Edp.setZero(1, 24); Fdp.setZero(1, 24); Gdp.setZero(1, 1); Hdp.setZero(1, 1);
    Edv.setZero(1, 24); Fdv.setZero(1, 24); Gdv.setZero(1, 1); Hdv.setZero(1, 1);
    Ed.setZero(1, 24); Fd.setZero(1, 24); Gd.setZero(1, 1); Hd.setZero(1, 1);

    Aeq1.setZero(n_hor_ * n_state_, n_hor_ * n_decision_vars_);
    beq1.setZero(n_hor_ * n_state_, 1);

    Aeq2.setZero(n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_);
    beq2.setZero(n_hor_ * n_holonomic_, 1);

    Aeq3.setZero(n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_);
    beq3.setZero(n_hor_ * n_holonomic_, 1);

    Aeq4.setZero(n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_);
    beq4.setZero(n_hor_ * n_holonomic_, 1);

    system_mtx_ftn(sysvec, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_sys_state(Ad, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_sys_input(Bd, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_sys_lambda(Cd, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    Dd = sysvec - Ad*state_op_variational_ - Bd * fop_ - Cd * lambda_op_;

    jaco_cons_state(Ed, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_cons_input(Fd, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
    jaco_cons_lambda(Gd, state_op_variational_, fop_, lambda_op_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);    
    Hd = -Ed * state_op_variational_ - Fd * fop_ - Gd * lambda_op_;

    // equality 1
    Aeq1.block(0,0, n_state_, n_inputs_) = -Bd;
    Aeq1.block(0, n_inputs_, n_state_, n_state_) = Eigen::MatrixXd::Identity(n_state_, n_state_);
    Aeq1.block(0, n_inputs_ + n_state_, n_state_, n_holonomic_) = -Cd;
    for(size_t i=0; i<n_hor_-1; i++){
        Aeq1.block(n_state_* (i+1), n_decision_vars_ * i + n_inputs_ * 1                           , n_state_, n_state_) = -Ad;
        Aeq1.block(n_state_* (i+1), n_decision_vars_ * i + n_inputs_ * 1 + n_state_               , n_state_, n_holonomic_) = -Cd;
        Aeq1.block(n_state_* (i+1), n_decision_vars_ * i + n_inputs_ * 1 + n_state_ + n_holonomic_               , n_state_, n_inputs_) = -Bd;
        Aeq1.block(n_state_* (i+1), n_decision_vars_ * i + n_inputs_ * 2 + n_state_ + n_holonomic_, n_state_, n_state_) = Eigen::MatrixXd::Identity(n_state_, n_state_);
    }

    for(size_t ii = 0; ii< n_hor_; ii++){
        if(ii == 0){
            beq1.block(n_state_ * ii, 0, n_state_, 1) = Ad * state_op_variational_ + Dd;
        }
        else{
            beq1.block(n_state_ * ii, 0, n_state_, 1) = Dd;
        }
    }

    // equality 2
    Aeq2.block(0,0, n_holonomic_, n_inputs_) = -Fd;
    Aeq2.block(0, n_inputs_ , n_holonomic_, n_state_) = Eigen::MatrixXd::Zero(n_holonomic_, n_state_);
    Aeq2.block(0, n_inputs_ + n_state_, n_holonomic_, n_holonomic_) = -Gd;
    for(size_t i=0; i<n_hor_-1; i++){
        Aeq2.block(n_holonomic_* (i+1), n_decision_vars_ * i + n_inputs_ * 1                           , n_holonomic_, n_state_) = -Ed;
        Aeq2.block(n_holonomic_* (i+1), n_decision_vars_ * i + n_inputs_ * 1 + n_state_               , n_holonomic_, n_holonomic_) = -Gd;
        Aeq2.block(n_holonomic_* (i+1), n_decision_vars_ * i + n_inputs_ * 1 + n_state_ + n_holonomic_               , n_holonomic_, n_inputs_) = -Fd;
        Aeq2.block(n_holonomic_* (i+1), n_decision_vars_ * i + n_inputs_ * 2 + n_state_ + n_holonomic_, n_holonomic_, n_state_) = Eigen::MatrixXd::Zero(n_holonomic_, n_state_);   
    }

    for(size_t ii = 0; ii< n_hor_; ii++){
        if(ii == 0){
            beq2.block(n_holonomic_ * ii, 0, n_holonomic_, 1) = Ed * state_op_variational_ + Hd;
        }
        else{
            beq2.block(n_holonomic_ * ii, 0, n_holonomic_, 1) = Hd;
        }
    }

    // ================== equality constraints construction ==================== //
    Aeq.setZero(n_hor_ * n_state_ + n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_);
    beq.setZero(n_hor_ * n_state_ + n_hor_ * n_holonomic_, 1);
    
    Aeq.block(0,0, n_hor_ * n_state_ , n_hor_ * n_decision_vars_) = Aeq1;
    beq.block(0,0, n_hor_ * n_state_, 1) = beq1;

    Aeq.block(n_hor_ * n_state_, 0, n_hor_ * n_holonomic_, n_hor_ * n_decision_vars_) = Aeq2;
    beq.block(n_hor_ * n_state_, 0, n_hor_ * n_holonomic_, 1) = beq2;

    // ================== MPC solver ==================== //
    double* opt_sol;
    double* kkt_multipliers;
    opt_sol = new double[n_hor_* n_decision_vars_];
    kkt_multipliers = new double[n_hor_*(n_state_ + n_holonomic_)];
    //iswiftQp_e(H, g, Aeq, beq, Aineq, bineq, opt_sol);
    iswiftQp_e_lagrange(H, g, Aeq, beq, Aineq, bineq, opt_sol, kkt_multipliers);
    for(size_t i=0; i< n_hor_ * n_decision_vars_; i++){
        opt_sol_e_(i) = opt_sol[i];
    }
    for(size_t i=0; i< n_hor_*(n_state_ + n_holonomic_); i++){
        kktmultiplier_e_(i) = kkt_multipliers[i];
    }

    delete[] opt_sol;
    delete[] kkt_multipliers;

    opt_X_ = opt_sol_e_.block(24,0,24,1);
    opt_u_ = opt_sol_e_.block(0,0,24,1) + fop_;
    opt_u_.block(0,0,3,1) = opt_u_.block(0,0,3,1) * foot_state1_(0);
    opt_u_.block(3,0,3,1) = opt_u_.block(3,0,3,1) * foot_state1_(1);
    opt_u_.block(6,0,3,1) = opt_u_.block(6,0,3,1) * foot_state1_(2);
    opt_u_.block(9,0,3,1) = opt_u_.block(9,0,3,1) * foot_state1_(3);
    opt_u_.block(12,0,3,1) = opt_u_.block(12,0,3,1) * foot_state2_(0);
    opt_u_.block(15,0,3,1) = opt_u_.block(15,0,3,1) * foot_state2_(1);
    opt_u_.block(18,0,3,1) = opt_u_.block(18,0,3,1) * foot_state2_(2);
    opt_u_.block(21,0,3,1) = opt_u_.block(21,0,3,1) * foot_state2_(3);

    if(isnan(opt_u_.norm())){
    }

    if(opt_u_.block(0,0,12,1).norm() > 1e3 || opt_u_.block(12,0,12,1).norm() > 1e3){
        opt_u_ = fop_;
    }
}

void LocomotionPlanner::srbMPC_distributed(size_t agentnumber){
    // ================== reduced states and inputs ============ //
    size_t reduced_state = reduced_state_;
    size_t reduced_holonomic = reduced_holonomic_;
    size_t reduced_inputs = reduced_inputs_;
    size_t reduced_decision_vars = reduced_decision_vars_;
    size_t distributed_hor = distributed_hor_;
    //srbMPC();

    // ================== Dynamics and input matrix ============ //
    Eigen::MatrixXd sysvec;
    Eigen::MatrixXd sysvec1, sysvec2;
    Eigen::MatrixXd Ad, Bd, Cd, Dd;
    Eigen::MatrixXd Ad11, Ad12, Ad21, Ad22;
    Eigen::MatrixXd Bd11, Bd12, Bd21, Bd22;
    Eigen::MatrixXd Cd1, Cd2;
    Eigen::MatrixXd Dd1, Dd2;
    Eigen::MatrixXd Ed, Fd, Gd, Hd;
    Eigen::MatrixXd Ed1, Ed2;
    Eigen::MatrixXd Fd1, Fd2;
    Eigen::MatrixXd Hd1, Hd2;

    Eigen::MatrixXd state_op_reduced0;
    Eigen::MatrixXd state_op_reduced1;

    sysvec.setZero(24,1);
    sysvec1.setZero(12,1);
    sysvec2.setZero(12,1);
    
    Ad.setZero(24,24); Bd.setZero(24,24); Cd.setZero(24, 1); Dd.setZero(24, 1);
    Ad11.setZero(12,12); Ad12.setZero(12,12); Ad21.setZero(12,12); Ad22.setZero(12,12);
    Bd11.setZero(12,12); Bd12.setZero(12,12); Bd21.setZero(12,12); Bd22.setZero(12,12);
    Cd1.setZero(12,1); Cd2.setZero(12,1); Dd1.setZero(12,1); Dd2.setZero(12,1);
    
    Ed.setZero(1, 24); Fd.setZero(1, 24); Gd.setZero(1, 1); Hd.setZero(1, 1);
    Ed1.setZero(1,12); Ed2.setZero(1,12); Fd1.setZero(1,12); Fd2.setZero(1,12);
    Hd1.setZero(1,1); Hd2.setZero(1,1);

    state_op_reduced0.setZero(12,1);
    state_op_reduced1.setZero(12,1);

    if(agentnumber == 0){
        system_mtx_ftn(sysvec, state_op_variational_, fop_, lambda_op0_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_state(Ad, state_op_variational_, fop_, lambda_op0_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_input(Bd, state_op_variational_, fop_, lambda_op0_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_lambda(Cd, state_op_variational_, fop_, lambda_op0_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        Dd = sysvec - Ad*state_op_variational_ - Bd * fop_ - Cd * lambda_op0_;

        jaco_cons_state(Ed, state_op_variational_, fop_, lambda_op0_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_cons_input(Fd, state_op_variational_, fop_, lambda_op0_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_cons_lambda(Gd, state_op_variational_, fop_, lambda_op0_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);    
        Hd = -Ed * state_op_variational_ - Fd * fop_ - Gd * lambda_op0_;
    }
    else if(agentnumber == 1){
        system_mtx_ftn(sysvec, state_op_variational_, fop_, lambda_op1_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_state(Ad, state_op_variational_, fop_, lambda_op1_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_input(Bd, state_op_variational_, fop_, lambda_op1_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_lambda(Cd, state_op_variational_, fop_, lambda_op1_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        Dd = sysvec - Ad*state_op_variational_ - Bd * fop_ - Cd * lambda_op1_;

        jaco_cons_state(Ed, state_op_variational_, fop_, lambda_op1_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_cons_input(Fd, state_op_variational_, fop_, lambda_op1_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_cons_lambda(Gd, state_op_variational_, fop_, lambda_op1_, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);    
        Hd = -Ed * state_op_variational_ - Fd * fop_ - Gd * lambda_op1_;
    }

    for(size_t i=0; i<4; i++){
    sysvec1.block(3*i, 0, 3, 1) = sysvec.block(6*i  , 0, 3, 1);
    sysvec2.block(3*i, 0, 3, 1) = sysvec.block(6*i+3, 0, 3, 1);
        for(size_t j=0; j<4; j++){
            Ad11.block(3*i  , 3*j  , 3, 3) = Ad.block(6*i  , 6*j  , 3, 3);
            Ad21.block(3*i  , 3*j  , 3, 3) = Ad.block(6*i+3, 6*j  , 3, 3);
            Ad12.block(3*i  , 3*j  , 3, 3) = Ad.block(6*i  , 6*j+3, 3, 3);
            Ad22.block(3*i  , 3*j  , 3, 3) = Ad.block(6*i+3, 6*j+3, 3, 3);
        }

        Bd11.block(3*i, 0, 3, 12) = Bd.block(6*i  , 0 , 3, 12);
        Bd21.block(3*i, 0, 3, 12) = Bd.block(6*i+3, 0 , 3, 12);
        Bd12.block(3*i, 0, 3, 12) = Bd.block(6*i  , 12, 3, 12);
        Bd22.block(3*i, 0, 3, 12) = Bd.block(6*i+3, 12, 3, 12);

        Cd1.block(3*i, 0, 3, 1) = Cd.block(6*i  , 0, 3, 1);
        Cd2.block(3*i, 0, 3, 1) = Cd.block(6*i+3, 0, 3, 1);

        Dd1.block(3*i, 0, 3, 1) = Dd.block(6*i  , 0, 3, 1);
        Dd2.block(3*i, 0, 3, 1) = Dd.block(6*i+3, 0, 3, 1);

        Ed1.block(0, 3*i, 3, 1) = Ed.block(0  , 6*i  , 3, 1);
        Ed2.block(0, 3*i, 3, 1) = Ed.block(0  , 6*i+3, 3, 1);

        Fd1.block(0, 3*i, 3, 1) = Fd.block(0  , 6*i  , 3, 1);
        Fd2.block(0, 3*i, 3, 1) = Fd.block(0  , 6*i+3, 3, 1);

        Hd1 = Hd;
        Hd2 = Hd;

        state_op_reduced0.block(3*i, 0, 3, 1) = state_op_variational_.block(6*i  , 0, 3, 1);
        state_op_reduced1.block(3*i, 0, 3, 1) = state_op_variational_.block(6*i+3, 0, 3, 1);
    }

    // ================== decision variable order ============ //
    // u(t) X(t+1) lambda(t+1) u(t+1) X(t+2) lambda(t+2) . . . u(t+T-1) X(t+T) lambda(t+T)

    // ================== weights =================== //
    Eigen::Matrix<double, 3, 3> qx, qxf;
    qx.setIdentity(3,3); 
    qx = 3e5 * qx;
    qx(1,1) = 3e7;
    qx(2,2) = 3e7;
    Eigen::Matrix<double, 3, 3> qv, qvf;
    qv.setIdentity(3,3); 
    qv = 1e4 * qv;
    Eigen::Matrix<double, 3, 3> qr, qrf;
    qr.setIdentity(3,3);
    qr = 1e10 * qr;
    Eigen::Matrix<double, 3, 3> qw, qwf;
    qw.setIdentity(3,3);
    qw = 5e3 * qw;
    double qlambda, qlambdaf, rx, rv, rr, rw;
    qlambda = 1e4;

    // ====== weights for distributed mpc
    double consensusweight = 1e1;
    double compensateweight = 1e0;
    double alpha1 = 0.5;
    double alpha2 = 1-alpha1;

    double stagecostgain  =  0; 
    double finalcostcoeff =  1;
    double finalcostgain  = -1;
    double inputgain      = -2;


    qx      = qx * pow(10, stagecostgain);
    qv      = qv * pow(10, stagecostgain);
    qr      = qr * pow(10, stagecostgain);
    qw      = qw * pow(10, stagecostgain);
    qlambda = qlambda * pow(10, stagecostgain);

    qxf      = qx * finalcostcoeff * pow(10, finalcostgain);
    qvf      = qv * finalcostcoeff * pow(10, finalcostgain);
    qrf      = qr * finalcostcoeff * pow(10, finalcostgain);
    qwf      = qw * finalcostcoeff * pow(10, finalcostgain);
    qlambdaf = qlambda * finalcostcoeff * pow(10, finalcostgain);

    rx = pow(10, inputgain);
    rv = pow(10, inputgain);
    rr = pow(10, inputgain);
    rw = pow(10, inputgain);

    Eigen::MatrixXd cost_Q;
    Eigen::MatrixXd cost_Qf;
    Eigen::MatrixXd cost_R;
    cost_Q.setZero(reduced_state + reduced_holonomic , reduced_state + reduced_holonomic);
    cost_Qf.setZero(reduced_state + reduced_holonomic , reduced_state + reduced_holonomic);
    cost_R.setZero(reduced_inputs , reduced_inputs);

    cost_Q.block(0,0,3,3) = qx;
    cost_Q.block(3,3,3,3) = qv;
    cost_Q.block(6,6,3,3) = qr;
    cost_Q.block(9,9,3,3) = qw;
    cost_Q(12,12) = qlambda + consensusweight;

    cost_Qf.block(0,0,3,3) = qxf;
    cost_Qf.block(3,3,3,3) = qvf;
    cost_Qf.block(6,6,3,3) = qrf;
    cost_Qf.block(9,9,3,3) = qwf;
    cost_Qf(12,12) = qlambdaf + consensusweight;

    cost_R.block(0,0,3,3) = rx * Eigen::MatrixXd::Identity(3,3);
    cost_R.block(3,3,3,3) = rv * Eigen::MatrixXd::Identity(3,3);
    cost_R.block(6,6,3,3) = rr * Eigen::MatrixXd::Identity(3,3);
    cost_R.block(9,9,3,3) = rw * Eigen::MatrixXd::Identity(3,3);

    // ======================= cost function ======================== //
    size_t idx_x = 0;
    size_t idx_u = 0;
    size_t sizeofHg = distributed_hor * reduced_decision_vars;

    Eigen::MatrixXd H;
    Eigen::MatrixXd g;

    H.setZero(sizeofHg, sizeofHg);
    g.setZero(sizeofHg, 1);

    Eigen::MatrixXd rot1_desired;
    Eigen::MatrixXd rot2_desired;
    rot1_desired.setIdentity(3,3);
    rot2_desired.setIdentity(3,3);
    Eigen::VectorXd rot1_desired_vec;
    Eigen::VectorXd rot2_desired_vec;
    rot1_desired_vec.setZero(9);
    rot2_desired_vec.setZero(9);

    Eigen::VectorXd omega1_desired;
    Eigen::VectorXd omega2_desired;
    omega1_desired.setZero(3);
    omega2_desired.setZero(3);

    Eigen::MatrixXd rottemp1;
    rottemp1.setZero(3,3);
    Eigen::MatrixXd rottemp2;
    rottemp2.setZero(3,3);
    Eigen::MatrixXd temp1;
    temp1.setZero(6,1);
    Eigen::MatrixXd temp2;
    temp2.setZero(6,1);
    Eigen::Vector3d temp11, temp12;
    temp11.setZero();
    temp12.setZero();

    for(size_t i_hor = 0; i_hor< distributed_hor; i_hor++){

        rot1_desired_vec = desired_state_.block(12, i_hor, 9,1);
        rot2_desired_vec = desired_state_.block(21, i_hor, 9,1);

        rot1_desired = Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::ColMajor> >(rot1_desired_vec.data());
        rot2_desired = Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::ColMajor> >(rot2_desired_vec.data());
        omega1_desired = desired_state_.block(30, i_hor, 3,1);
        omega2_desired = desired_state_.block(33, i_hor, 3,1);
        rottemp1 = rot1_desired.transpose() * rot1_op_;
        rottemp2 = rot2_desired.transpose() * rot2_op_;
        veemap(rottemp1.log(), temp11);
        veemap(rottemp2.log(), temp12);
        temp1.block(0,0,3,1) = temp11;
        temp1.block(3,0,3,1) = temp12;
        temp2.block(0,0,3,1) = rot1_op_.transpose() * rot1_desired * omega1_desired;
        temp2.block(3,0,3,1) = rot2_op_.transpose() * rot2_desired * omega2_desired;

        idx_u = i_hor * reduced_decision_vars;
        idx_x = i_hor * reduced_decision_vars + reduced_inputs;
        if(i_hor == distributed_hor -1){
            H.block(idx_x, idx_x, reduced_state + reduced_holonomic, reduced_state + reduced_holonomic) = cost_Qf;
            if(agentnumber == 0){
                g.block(idx_x     , 0, 3, 1) = -qxf * desired_state_.block(0, i_hor, 3, 1);
                g.block(idx_x +  3, 0, 3, 1) = -qvf * desired_state_.block(6, i_hor, 3, 1);
                g.block(idx_x +  6, 0, 3, 1) = qrf * temp1.block(0,0,3,1);
                g.block(idx_x +  9, 0, 3, 1) = -qwf * temp2.block(0,0,3,1);
                g(idx_x + 12) = -qlambdaf * desired_lambda_ - consensusweight*alpha1*lambda_op0_ - consensusweight*alpha2*lambda_op1_;

                g.block(idx_x, 0, 12,1) += compensateweight*Ad21.transpose() * kktmultiplier1dist_.block(i_hor*(reduced_state + reduced_holonomic),0,12,1);
                g.block(idx_x, 0, 12,1) += compensateweight*Ed2.transpose() * kktmultiplier1dist_.block(i_hor*(reduced_state + reduced_holonomic)+12,0,1,1);

                Eigen::MatrixXd lambdadynportion = Cd2.transpose() * kktmultiplier1dist_.block(i_hor*(reduced_state + reduced_holonomic),0,12,1);
                Eigen::MatrixXd lambdaholoportion = Gd * kktmultiplier1dist_.block(i_hor*(reduced_state + reduced_holonomic)+12,0,1,1);
                g(idx_x + 12) += compensateweight*lambdadynportion(0) + compensateweight*lambdaholoportion(0);
            }
            else if(agentnumber == 1){
                g.block(idx_x     , 0, 3, 1) = -qxf * desired_state_.block(3, i_hor, 3, 1);
                g.block(idx_x +  3, 0, 3, 1) = -qvf * desired_state_.block(9, i_hor, 3, 1);
                g.block(idx_x +  6, 0, 3, 1) = qrf * temp1.block(3,0,3,1);
                g.block(idx_x +  9, 0, 3, 1) = -qwf * temp2.block(3,0,3,1);
                g(idx_x + 12) = -qlambdaf * desired_lambda_ - consensusweight*alpha1*lambda_op0_ - consensusweight*alpha2*lambda_op1_;

                g.block(idx_x, 0, 12,1) += compensateweight*Ad12.transpose() * kktmultiplier0dist_.block(i_hor*(reduced_state + reduced_holonomic),0,12,1);
                g.block(idx_x, 0, 12,1) += compensateweight*Ed1.transpose() * kktmultiplier0dist_.block(i_hor*(reduced_state + reduced_holonomic)+12,0,1,1);

                Eigen::MatrixXd lambdadynportion = Cd1.transpose() * kktmultiplier0dist_.block(i_hor*(reduced_state + reduced_holonomic),0,12,1);
                Eigen::MatrixXd lambdaholoportion = Gd * kktmultiplier0dist_.block(i_hor*(reduced_state + reduced_holonomic)+12,0,1,1);
                g(idx_x + 12) += compensateweight*lambdadynportion(0) + compensateweight*lambdaholoportion(0);
            }
        }
        else{
            H.block(idx_x, idx_x, reduced_state + reduced_holonomic, reduced_state + reduced_holonomic) = cost_Q;
            if(agentnumber == 0){
                g.block(idx_x     , 0, 3, 1) = -qx * desired_state_.block(0, i_hor, 3, 1);
                g.block(idx_x +  3, 0, 3, 1) = -qv * desired_state_.block(6, i_hor, 3, 1);
                g.block(idx_x +  6, 0, 3, 1) = qr * temp1.block(0,0,3,1);
                g.block(idx_x +  9, 0, 3, 1) = -qw * temp2.block(0,0,3,1);
                g(idx_x + 12) = -qlambda * desired_lambda_ - consensusweight*alpha1*lambda_op0_ - consensusweight*alpha2*lambda_op1_;

                g.block(idx_x, 0, 12,1) += compensateweight*Ad21.transpose() * kktmultiplier1dist_.block(i_hor*(reduced_state + reduced_holonomic),0,12,1);
                g.block(idx_x, 0, 12,1) += compensateweight*Ed2.transpose() * kktmultiplier1dist_.block(i_hor*(reduced_state + reduced_holonomic)+12,0,1,1);

                Eigen::MatrixXd lambdadynportion = Cd2.transpose() * kktmultiplier1dist_.block(i_hor*(reduced_state + reduced_holonomic),0,12,1);
                Eigen::MatrixXd lambdaholoportion = Gd * kktmultiplier1dist_.block(i_hor*(reduced_state + reduced_holonomic)+12,0,1,1);
                g(idx_x + 12) += compensateweight*lambdadynportion(0) + compensateweight*lambdaholoportion(0);
            }
            else if(agentnumber == 1){
                g.block(idx_x     , 0, 3, 1) = -qx * desired_state_.block(3, i_hor, 3, 1);
                g.block(idx_x +  3, 0, 3, 1) = -qv * desired_state_.block(9, i_hor, 3, 1);
                g.block(idx_x +  6, 0, 3, 1) = qr * temp1.block(3,0,3,1);
                g.block(idx_x +  9, 0, 3, 1) = -qw * temp2.block(3,0,3,1);
                g(idx_x + 12) = -qlambda * desired_lambda_ - consensusweight*alpha1*lambda_op0_ - consensusweight*alpha2*lambda_op1_;

                g.block(idx_x, 0, 12,1) += compensateweight*Ad12.transpose() * kktmultiplier0dist_.block(i_hor*(reduced_state + reduced_holonomic),0,12,1);
                g.block(idx_x, 0, 12,1) += compensateweight*Ed1.transpose() * kktmultiplier0dist_.block(i_hor*(reduced_state + reduced_holonomic)+12,0,1,1);

                Eigen::MatrixXd lambdadynportion = Cd1.transpose() * kktmultiplier0dist_.block(i_hor*(reduced_state + reduced_holonomic),0,12,1);
                Eigen::MatrixXd lambdaholoportion = Gd * kktmultiplier0dist_.block(i_hor*(reduced_state + reduced_holonomic)+12,0,1,1);
                g(idx_x + 12) += compensateweight*lambdadynportion(0) + compensateweight*lambdaholoportion(0);
            }
        }
        H.block(idx_u, idx_u, reduced_inputs, reduced_inputs) = cost_R;
        if(agentnumber == 0){
            g.block(idx_u, 0, reduced_inputs, 1) = cost_R.transpose() * (fop_.block(0,0,12,1)- desired_input_.block(0, i_hor, reduced_inputs, 1));    

        }
        else if(agentnumber == 1){
            g.block(idx_u, 0, reduced_inputs, 1) = cost_R.transpose() * (fop_.block(12,0,12,1)- desired_input_.block(reduced_inputs, i_hor, reduced_inputs, 1));    

        }
    }

    // =================== inequality constraints ================== //
    double NEQ = 32 + 16; 
    double distributed_NEQ = NEQ * 0.5;

    Eigen::MatrixXd lb;
    Eigen::MatrixXd ub;

    Eigen::MatrixXd Fu;
    Eigen::MatrixXd Fu_x_y;
    Eigen::MatrixXd zeroandones;
    
    Eigen::MatrixXd Aineq;
    Eigen::MatrixXd bineq;
    Eigen::MatrixXd fconeaugment;

    lb.setZero(8,1);
    ub.setZero(8,1);
    lb << desired_input_(2), desired_input_(5),desired_input_(8),desired_input_(11),desired_input_(14),desired_input_(17),desired_input_(20),desired_input_(23);
    ub << desired_input_(2), desired_input_(5),desired_input_(8),desired_input_(11),desired_input_(14),desired_input_(17),desired_input_(20),desired_input_(23);
    lb = -2 * lb * 1e0;
    ub =  2 * ub * 1e0;

    Fu.setZero(distributed_NEQ, reduced_inputs);
    Fu_x_y.setZero(4,3);
    zeroandones.setZero(2,3);
    
    Aineq.setZero(distributed_hor * distributed_NEQ, distributed_hor * reduced_decision_vars);
    bineq.setZero(distributed_hor * distributed_NEQ, 1);
    fconeaugment.setZero(16,1);

    Fu_x_y <<  1,  0, -(mu_/sqrt(2)),  
              -1,  0, -(mu_/sqrt(2)),  
               0,  1, -(mu_/sqrt(2)),   
               0, -1, -(mu_/sqrt(2));
    zeroandones << 0,0,1,0,0,-1;

    for(size_t i=0; i<4; i++){
        Fu.block(i*4,i*3, 4,3) = Fu_x_y;
        Fu.block(16+ i*2, i*3,2,3) = zeroandones;
    }

    for(size_t ii=0; ii<distributed_hor; ii++){
        Aineq.block(distributed_NEQ * ii, reduced_decision_vars * ii, distributed_NEQ, reduced_inputs) = Fu;
    }

    Eigen::MatrixXd frictioncone;
    frictioncone.setZero(4,1);
    for(size_t j=0; j<4; j++){
        if(agentnumber == 0){
            frictioncone << (mu_/sqrt(2)) * fop_(2 + 3*j) - fop_(0+ 3*j),
                            (mu_/sqrt(2)) * fop_(2 + 3*j) + fop_(0+ 3*j),
                            (mu_/sqrt(2)) * fop_(2 + 3*j) - fop_(1+ 3*j),
                            (mu_/sqrt(2)) * fop_(2 + 3*j) + fop_(1+ 3*j);
        }
        else if(agentnumber == 1){
            frictioncone << (mu_/sqrt(2)) * fop_(2 + 3*j+12) - fop_(0+ 3*j+12),
                            (mu_/sqrt(2)) * fop_(2 + 3*j+12) + fop_(0+ 3*j+12),
                            (mu_/sqrt(2)) * fop_(2 + 3*j+12) - fop_(1+ 3*j+12),
                            (mu_/sqrt(2)) * fop_(2 + 3*j+12) + fop_(1+ 3*j+12);
        }
        
        fconeaugment.block(4*j, 0, 4,1) = frictioncone;
    }

    Eigen::MatrixXd lu_bounds;
    //lu_bounds.setZero(16,1);
    lu_bounds.setZero(8,1);
    for(size_t jj=0; jj< distributed_hor; jj++){
        bineq.block(distributed_NEQ * jj, 0, 16,1) = fconeaugment;
        if(agentnumber == 0){
            lu_bounds <<  ub(0) - fop_(2) + desired_input_(2, jj),
                        -lb(0) + fop_(2) - desired_input_(2, jj),
                        ub(1) - fop_(5) + desired_input_(5, jj),
                        -lb(1) + fop_(5) - desired_input_(5, jj),
                        ub(2) - fop_(8) + desired_input_(8, jj),
                        -lb(2) + fop_(8) - desired_input_(8, jj),
                        ub(3) - fop_(11) + desired_input_(11, jj),
                        -lb(3) + fop_(11) - desired_input_(11, jj);
            
            bineq.block(distributed_NEQ * jj + 16, 0, 8, 1) = lu_bounds;
        }
        else if(agentnumber == 1){
            lu_bounds <<  ub(4) - fop_(14) + desired_input_(14, jj),
                        -lb(4) + fop_(14) - desired_input_(14, jj),
                        ub(5) - fop_(17) + desired_input_(17, jj),
                        -lb(5) + fop_(17) - desired_input_(17, jj),
                        ub(6) - fop_(20) + desired_input_(20, jj),
                        -lb(6) + fop_(20) - desired_input_(20, jj),
                        ub(7) - fop_(23) + desired_input_(23, jj),
                        -lb(7) + fop_(23) - desired_input_(23, jj);
            
            bineq.block(distributed_NEQ * jj + 16, 0, 8, 1) = lu_bounds;
        }
    }

    // ================== equality constraints ==================== //
    Eigen::MatrixXd Aeq1, Aeq2, Aeq;
    Eigen::MatrixXd beq1, beq2, beq;

    Aeq1.setZero(distributed_hor * reduced_state, distributed_hor * reduced_decision_vars);
    beq1.setZero(distributed_hor * reduced_state, 1);

    Aeq2.setZero(distributed_hor * reduced_holonomic, distributed_hor * reduced_decision_vars);
    beq2.setZero(distributed_hor * reduced_holonomic, 1);

    // equality 1
    if(agentnumber == 0){
        Aeq1.block(0,0, reduced_state, reduced_inputs) = -Bd11;
        Aeq1.block(0, reduced_inputs, reduced_state, reduced_state) = Eigen::MatrixXd::Identity(reduced_state, reduced_state);
        Aeq1.block(0, reduced_inputs + reduced_state, reduced_state, reduced_holonomic) = -Cd1;
        for(size_t i=0; i<distributed_hor-1; i++){
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1                                    , reduced_state, reduced_state) = -Ad11;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state                    , reduced_state, reduced_holonomic) = -Cd1;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state + reduced_holonomic, reduced_state, reduced_inputs) = -Bd11;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 2 + reduced_state + reduced_holonomic, reduced_state, reduced_state) = Eigen::MatrixXd::Identity(reduced_state, reduced_state);
        }

        for(size_t ii = 0; ii< distributed_hor; ii++){
            if(ii == 0){
                beq1.block(reduced_state * ii, 0, reduced_state, 1) = Ad11 * state_op_reduced0 + Dd1 + Ad12 * (state_op_reduced1+(sysvec2-state_op_reduced1)*ii);
            }
            else{
                beq1.block(reduced_state * ii, 0, reduced_state, 1) = Dd1 + Ad12 * (state_op_reduced1+(sysvec2-state_op_reduced1)*ii);
            }
        }
    }
    else if(agentnumber == 1){
        Aeq1.block(0,0, reduced_state, reduced_inputs) = -Bd22;
        Aeq1.block(0, reduced_inputs, reduced_state, reduced_state) = Eigen::MatrixXd::Identity(reduced_state, reduced_state);
        Aeq1.block(0, reduced_inputs + reduced_state, reduced_state, reduced_holonomic) = -Cd2;
        for(size_t i=0; i<distributed_hor-1; i++){
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1                                    , reduced_state, reduced_state) = -Ad22;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state                    , reduced_state, reduced_holonomic) = -Cd2;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state + reduced_holonomic, reduced_state, reduced_inputs) = -Bd22;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 2 + reduced_state + reduced_holonomic, reduced_state, reduced_state) = Eigen::MatrixXd::Identity(reduced_state, reduced_state);
        }

        for(size_t ii = 0; ii< distributed_hor; ii++){
            if(ii == 0){
                beq1.block(reduced_state * ii, 0, reduced_state, 1) = Ad22 * state_op_reduced1 + Dd2 + Ad21 * (state_op_reduced0+(sysvec1-state_op_reduced0)*ii);
            }
            else{
                beq1.block(reduced_state * ii, 0, reduced_state, 1) = Dd2+ Ad21 * (state_op_reduced0+(sysvec1-state_op_reduced0)*ii);
            }
        }
    }

    // equality 2
    if(agentnumber == 0){
        Aeq2.block(0,0, reduced_holonomic, reduced_inputs) = -Fd1;
        Aeq2.block(0, reduced_inputs , reduced_holonomic, reduced_state) = Eigen::MatrixXd::Zero(reduced_holonomic, reduced_state);
        Aeq2.block(0, reduced_inputs + reduced_state, reduced_holonomic, reduced_holonomic) = -Gd;
        for(size_t i=0; i<distributed_hor-1; i++){
            Aeq2.block(reduced_holonomic* (i+1), reduced_decision_vars * i + reduced_inputs * 1                                    , reduced_holonomic, reduced_state) = -Ed1;
            Aeq2.block(reduced_holonomic* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state                    , reduced_holonomic, reduced_holonomic) = -Gd;
            Aeq2.block(reduced_holonomic* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state + reduced_holonomic, reduced_holonomic, reduced_inputs) = -Fd1;
            Aeq2.block(reduced_holonomic* (i+1), reduced_decision_vars * i + reduced_inputs * 2 + reduced_state + reduced_holonomic, reduced_holonomic, reduced_state) = Eigen::MatrixXd::Zero(reduced_holonomic, reduced_state);   
        }

        for(size_t ii = 0; ii< distributed_hor; ii++){
            if(ii == 0){
                beq2.block(reduced_holonomic * ii, 0, reduced_holonomic, 1) = Ed1 * state_op_reduced0 + Hd1 + Ed2 * (state_op_reduced1+(sysvec2-state_op_reduced1)*ii);
            }
            else{
                beq2.block(reduced_holonomic * ii, 0, reduced_holonomic, 1) = Hd1 + Ed2 * (state_op_reduced1+(sysvec2-state_op_reduced1)*ii);
            }
        }
    }
    else if(agentnumber == 1){
        Aeq2.block(0,0, reduced_holonomic, reduced_inputs) = -Fd2;
        Aeq2.block(0, reduced_inputs , reduced_holonomic, reduced_state) = Eigen::MatrixXd::Zero(reduced_holonomic, reduced_state);
        Aeq2.block(0, reduced_inputs + reduced_state, reduced_holonomic, reduced_holonomic) = -Gd;
        for(size_t i=0; i<distributed_hor-1; i++){
            Aeq2.block(reduced_holonomic* (i+1), reduced_decision_vars * i + reduced_inputs * 1                                    , reduced_holonomic, reduced_state) = -Ed2;
            Aeq2.block(reduced_holonomic* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state                    , reduced_holonomic, reduced_holonomic) = -Gd;
            Aeq2.block(reduced_holonomic* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state + reduced_holonomic, reduced_holonomic, reduced_inputs) = -Fd2;
            Aeq2.block(reduced_holonomic* (i+1), reduced_decision_vars * i + reduced_inputs * 2 + reduced_state + reduced_holonomic, reduced_holonomic, reduced_state) = Eigen::MatrixXd::Zero(reduced_holonomic, reduced_state);   
        }

        for(size_t ii = 0; ii< distributed_hor; ii++){
            if(ii == 0){
                beq2.block(reduced_holonomic * ii, 0, reduced_holonomic, 1) = Ed2 * state_op_reduced1 + Hd2 + Ed1 * (state_op_reduced0+(sysvec1-state_op_reduced0)*ii);
            }
            else{
                beq2.block(reduced_holonomic * ii, 0, reduced_holonomic, 1) = Hd2 + Ed1 * (state_op_reduced0+(sysvec1-state_op_reduced0)*ii);
            }
        }
    }

    // ================== equality constraints construction ==================== //
    Aeq.setZero(distributed_hor * reduced_state + distributed_hor * reduced_holonomic, distributed_hor * reduced_decision_vars); 
    beq.setZero(distributed_hor * reduced_state + distributed_hor * reduced_holonomic, 1); 
    
    Aeq.block(0,0, distributed_hor * reduced_state , distributed_hor * reduced_decision_vars) = Aeq1;
    beq.block(0,0, distributed_hor * reduced_state, 1) = beq1;

    Aeq.block(distributed_hor * reduced_state, 0, distributed_hor * reduced_holonomic, distributed_hor * reduced_decision_vars) = Aeq2; 
    beq.block(distributed_hor * reduced_state, 0, distributed_hor * reduced_holonomic, 1) = beq2; 

    // ================== MPC solver ==================== //
    double* opt_sol;
    double* kkt_multipliers;
    opt_sol = new double[distributed_hor * reduced_decision_vars];
    kkt_multipliers = new double[distributed_hor * (reduced_state + reduced_holonomic)];
    iswiftQp_e_lagrange(H, g, Aeq, beq, Aineq, bineq, opt_sol, kkt_multipliers);

    for(size_t i=0; i< distributed_hor * reduced_decision_vars; i++){
        dist_opt_sol_e_(i) = opt_sol[i];
    }

    if(agentnumber == 0){
        for(size_t i=0; i< distributed_hor * (reduced_state + reduced_holonomic); i++){
            kktmultiplier0dist_(i) = kkt_multipliers[i];
        }
    }
    else if(agentnumber == 1){
        for(size_t i=0; i< distributed_hor * (reduced_state + reduced_holonomic); i++){
            kktmultiplier1dist_(i) = kkt_multipliers[i];
        }
    }

    delete[] opt_sol;
    delete[] kkt_multipliers;

    double weightcoeff = 0.0;
    dist_opt_X_ = dist_opt_sol_e_.block(12,0,12,1);
    if(agentnumber == 0){
        dist_opt_u_ = (weightcoeff*dist_opt_sol0_e_.block(0,0,12,1)+(1-weightcoeff)*dist_opt_sol_e_.block(0,0,12,1)) + fop_.block(0,0,12,1);
        dist_opt_u_.block(0,0,3,1) = dist_opt_u_.block(0,0,3,1) * foot_state1_(0);
        dist_opt_u_.block(3,0,3,1) = dist_opt_u_.block(3,0,3,1) * foot_state1_(1);
        dist_opt_u_.block(6,0,3,1) = dist_opt_u_.block(6,0,3,1) * foot_state1_(2);
        dist_opt_u_.block(9,0,3,1) = dist_opt_u_.block(9,0,3,1) * foot_state1_(3);
    }
    else if(agentnumber == 1){
        dist_opt_u_ = (weightcoeff*dist_opt_sol1_e_.block(0,0,12,1)+ (1-weightcoeff)*dist_opt_sol_e_.block(0,0,12,1)) + fop_.block(12,0,12,1);
        dist_opt_u_.block(0,0,3,1) = dist_opt_u_.block(0,0,3,1) * foot_state2_(0);
        dist_opt_u_.block(3,0,3,1) = dist_opt_u_.block(3,0,3,1) * foot_state2_(1);
        dist_opt_u_.block(6,0,3,1) = dist_opt_u_.block(6,0,3,1) * foot_state2_(2);
        dist_opt_u_.block(9,0,3,1) = dist_opt_u_.block(9,0,3,1) * foot_state2_(3);
    }

    if(isnan(dist_opt_u_.norm())){
        if(agentnumber == 0){
            dist_opt_u_ = fop_.block(0,0,12,1);
        }
        else if(agentnumber == 1){
            dist_opt_u_ = fop_.block(12,0,12,1);
        }
    }

    if(dist_opt_u_.block(0,0,12,1).norm() > 1e3){ //original: 1e3
        if(agentnumber == 0){
            dist_opt_u_ = fop_.block(0,0,12,1);
        }
        else if(agentnumber == 1){
            dist_opt_u_ = fop_.block(12,0,12,1);
        }
    }
    
}

void LocomotionPlanner::srbMPC_single(size_t agentnumber){
    // ================== reduced states and inputs ============ //
    size_t reduced_state = reduced_state_;
    size_t reduced_holonomic = reduced_holonomic_;
    size_t reduced_inputs = reduced_inputs_;
    size_t reduced_decision_vars = reduced_decision_vars_ - reduced_holonomic_; 
    size_t distributed_hor = distributed_hor_;

    // ================== Dynamics and input matrix ============ //
    Eigen::MatrixXd sysvec;
    Eigen::MatrixXd sysvec1, sysvec2;
    Eigen::MatrixXd Ad, Bd, Cd, Dd;
    Eigen::MatrixXd Ad11, Ad12, Ad21, Ad22;
    Eigen::MatrixXd Bd11, Bd12, Bd21, Bd22;
    Eigen::MatrixXd Cd1, Cd2;
    Eigen::MatrixXd Dd1, Dd2;
    Eigen::MatrixXd Ed, Fd, Gd, Hd;
    Eigen::MatrixXd Ed1, Ed2;
    Eigen::MatrixXd Fd1, Fd2;
    Eigen::MatrixXd Hd1, Hd2;

    Eigen::MatrixXd state_op_reduced0;
    Eigen::MatrixXd state_op_reduced1;

    sysvec.setZero(24,1);
    sysvec1.setZero(12,1);
    sysvec2.setZero(12,1);
    
    Ad.setZero(24,24); Bd.setZero(24,24); Cd.setZero(24, 1); Dd.setZero(24, 1);
    Ad11.setZero(12,12); Ad12.setZero(12,12); Ad21.setZero(12,12); Ad22.setZero(12,12);
    Bd11.setZero(12,12); Bd12.setZero(12,12); Bd21.setZero(12,12); Bd22.setZero(12,12);
    Cd1.setZero(12,1); Cd2.setZero(12,1); Dd1.setZero(12,1); Dd2.setZero(12,1);
    
    Ed.setZero(1, 24); Fd.setZero(1, 24); Gd.setZero(1, 1); Hd.setZero(1, 1);
    Ed1.setZero(1,12); Ed2.setZero(1,12); Fd1.setZero(1,12); Fd2.setZero(1,12);
    Hd1.setZero(1,1); Hd2.setZero(1,1);

    state_op_reduced0.setZero(12,1);
    state_op_reduced1.setZero(12,1);

    if(agentnumber == 0){
        system_mtx_ftn(sysvec, state_op_variational_, fop_, 0, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_state(Ad, state_op_variational_, fop_, 0, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_input(Bd, state_op_variational_, fop_, 0, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        Dd = sysvec - Ad*state_op_variational_ - Bd * fop_;
    }
    else if(agentnumber == 1){
        system_mtx_ftn(sysvec, state_op_variational_, fop_, 0, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_state(Ad, state_op_variational_, fop_, 0, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        jaco_sys_input(Bd, state_op_variational_, fop_, 0, state_op_, fop_, p_foot_, foot_state1_, foot_state2_);
        Dd = sysvec - Ad*state_op_variational_ - Bd * fop_;
    }

    for(size_t i=0; i<4; i++){
    sysvec1.block(3*i, 0, 3, 1) = sysvec.block(6*i  , 0, 3, 1);
    sysvec2.block(3*i, 0, 3, 1) = sysvec.block(6*i+3, 0, 3, 1);
        for(size_t j=0; j<4; j++){
            Ad11.block(3*i  , 3*j  , 3, 3) = Ad.block(6*i  , 6*j  , 3, 3);
            Ad21.block(3*i  , 3*j  , 3, 3) = Ad.block(6*i+3, 6*j  , 3, 3)*0;
            Ad12.block(3*i  , 3*j  , 3, 3) = Ad.block(6*i  , 6*j+3, 3, 3)*0;
            Ad22.block(3*i  , 3*j  , 3, 3) = Ad.block(6*i+3, 6*j+3, 3, 3);
        }

        Bd11.block(3*i, 0, 3, 12) = Bd.block(6*i  , 0 , 3, 12);
        Bd21.block(3*i, 0, 3, 12) = Bd.block(6*i+3, 0 , 3, 12)*0;
        Bd12.block(3*i, 0, 3, 12) = Bd.block(6*i  , 12, 3, 12)*0;
        Bd22.block(3*i, 0, 3, 12) = Bd.block(6*i+3, 12, 3, 12);

        Dd1.block(3*i, 0, 3, 1) = Dd.block(6*i  , 0, 3, 1);
        Dd2.block(3*i, 0, 3, 1) = Dd.block(6*i+3, 0, 3, 1);

        state_op_reduced0.block(3*i, 0, 3, 1) = state_op_variational_.block(6*i  , 0, 3, 1);
        state_op_reduced1.block(3*i, 0, 3, 1) = state_op_variational_.block(6*i+3, 0, 3, 1);
    }

    // ================== decision variable order ============ //
    // u(t) X(t+1) lambda(t+1) u(t+1) X(t+2) lambda(t+2) . . . u(t+T-1) X(t+T) lambda(t+T)

    // ================== weights =================== //
    Eigen::Matrix<double, 3, 3> qx, qxf;
    qx.setIdentity(3,3); 
    qx = 3e5 * qx;
    //qx(2,2) = 3e6;
    qx(1,1) = 3e7;
    qx(2,2) = 3e7; 
    Eigen::Matrix<double, 3, 3> qv, qvf;
    qv.setIdentity(3,3); 
    qv = 1e4 * qv;
    Eigen::Matrix<double, 3, 3> qr, qrf;
    qr.setIdentity(3,3);
    qr = 1e8 * qr;
    Eigen::Matrix<double, 3, 3> qw, qwf;
    qw.setIdentity(3,3);
    qw = 5e3 * qw;
    double rx, rv, rr, rw;

    double stagecostgain  =  0; 
    double finalcostcoeff =  1;
    double finalcostgain  =  1; 
    double inputgain      = -2;

    qx      = qx * pow(10, stagecostgain);
    qv      = qv * pow(10, stagecostgain);
    qr      = qr * pow(10, stagecostgain);
    qw      = qw * pow(10, stagecostgain);

    qxf      = qx * finalcostcoeff * pow(10, finalcostgain);
    qvf      = qv * finalcostcoeff * pow(10, finalcostgain);
    qrf      = qr * finalcostcoeff * pow(10, finalcostgain);
    qwf      = qw * finalcostcoeff * pow(10, finalcostgain);

    rx = pow(10, inputgain);
    rv = pow(10, inputgain);
    rr = pow(10, inputgain);
    rw = pow(10, inputgain);

    Eigen::MatrixXd cost_Q;
    Eigen::MatrixXd cost_Qf;
    Eigen::MatrixXd cost_R;
    cost_Q.setZero(reduced_state, reduced_state);
    cost_Qf.setZero(reduced_state, reduced_state);
    cost_R.setZero(reduced_inputs , reduced_inputs);

    cost_Q.block(0,0,3,3) = qx;
    cost_Q.block(3,3,3,3) = qv;
    cost_Q.block(6,6,3,3) = qr;
    cost_Q.block(9,9,3,3) = qw;

    cost_Qf.block(0,0,3,3) = qxf;
    cost_Qf.block(3,3,3,3) = qvf;
    cost_Qf.block(6,6,3,3) = qrf;
    cost_Qf.block(9,9,3,3) = qwf;

    cost_R.block(0,0,3,3) = rx * Eigen::MatrixXd::Identity(3,3);
    cost_R.block(3,3,3,3) = rv * Eigen::MatrixXd::Identity(3,3);
    cost_R.block(6,6,3,3) = rr * Eigen::MatrixXd::Identity(3,3);
    cost_R.block(9,9,3,3) = rw * Eigen::MatrixXd::Identity(3,3);

    // ======================= cost function ======================== //
    size_t idx_x = 0;
    size_t idx_u = 0;
    size_t sizeofHg = distributed_hor * reduced_decision_vars;

    Eigen::MatrixXd H;
    Eigen::MatrixXd g;

    H.setZero(sizeofHg, sizeofHg);
    g.setZero(sizeofHg, 1);

    Eigen::MatrixXd rot1_desired;
    Eigen::MatrixXd rot2_desired;
    rot1_desired.setIdentity(3,3);
    rot2_desired.setIdentity(3,3);
    Eigen::VectorXd rot1_desired_vec;
    Eigen::VectorXd rot2_desired_vec;
    rot1_desired_vec.setZero(9);
    rot2_desired_vec.setZero(9);

    Eigen::VectorXd omega1_desired;
    Eigen::VectorXd omega2_desired;
    omega1_desired.setZero(3);
    omega2_desired.setZero(3);

    Eigen::MatrixXd rottemp1;
    rottemp1.setZero(3,3);
    Eigen::MatrixXd rottemp2;
    rottemp2.setZero(3,3);
    Eigen::MatrixXd temp1;
    temp1.setZero(6,1);
    Eigen::MatrixXd temp2;
    temp2.setZero(6,1);
    Eigen::Vector3d temp11, temp12;
    temp11.setZero();
    temp12.setZero();

    for(size_t i_hor = 0; i_hor< distributed_hor; i_hor++){

        rot1_desired_vec = desired_state_.block(12, i_hor, 9,1);
        rot2_desired_vec = desired_state_.block(21, i_hor, 9,1);

        rot1_desired = Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::ColMajor> >(rot1_desired_vec.data());
        rot2_desired = Eigen::Map<Eigen::Matrix<double, 3,3, Eigen::ColMajor> >(rot2_desired_vec.data());
        omega1_desired = desired_state_.block(30, i_hor, 3,1);
        omega2_desired = desired_state_.block(33, i_hor, 3,1);
        rottemp1 = rot1_desired.transpose() * rot1_op_;
        rottemp2 = rot2_desired.transpose() * rot2_op_;
        veemap(rottemp1.log(), temp11);
        veemap(rottemp2.log(), temp12);
        temp1.block(0,0,3,1) = temp11;
        temp1.block(3,0,3,1) = temp12;
        temp2.block(0,0,3,1) = rot1_op_.transpose() * rot1_desired * omega1_desired; 
        temp2.block(3,0,3,1) = rot2_op_.transpose() * rot2_desired * omega2_desired;

        idx_u = i_hor * reduced_decision_vars;
        idx_x = i_hor * reduced_decision_vars + reduced_inputs;
        if(i_hor == distributed_hor -1){
            H.block(idx_x, idx_x, reduced_state, reduced_state) = cost_Qf;
            if(agentnumber == 0){
                g.block(idx_x     , 0, 3, 1) = -qxf * desired_state_.block(0, i_hor, 3, 1);
                g.block(idx_x +  3, 0, 3, 1) = -qvf * desired_state_.block(6, i_hor, 3, 1);
                g.block(idx_x +  6, 0, 3, 1) = qrf * temp1.block(0,0,3,1);
                g.block(idx_x +  9, 0, 3, 1) = -qwf * temp2.block(0,0,3,1);
            }
            else if(agentnumber == 1){
                g.block(idx_x     , 0, 3, 1) = -qxf * desired_state_.block(3, i_hor, 3, 1);
                g.block(idx_x +  3, 0, 3, 1) = -qvf * desired_state_.block(9, i_hor, 3, 1);
                g.block(idx_x +  6, 0, 3, 1) = qrf * temp1.block(3,0,3,1);
                g.block(idx_x +  9, 0, 3, 1) = -qwf * temp2.block(3,0,3,1);
            }
        }
        else{
            H.block(idx_x, idx_x, reduced_state, reduced_state) = cost_Q;
            if(agentnumber == 0){
                g.block(idx_x     , 0, 3, 1) = -qx * desired_state_.block(0, i_hor, 3, 1);
                g.block(idx_x +  3, 0, 3, 1) = -qv * desired_state_.block(6, i_hor, 3, 1);
                g.block(idx_x +  6, 0, 3, 1) = qr * temp1.block(0,0,3,1);
                g.block(idx_x +  9, 0, 3, 1) = -qw * temp2.block(0,0,3,1);
            }
            else if(agentnumber == 1){
                g.block(idx_x     , 0, 3, 1) = -qx * desired_state_.block(3, i_hor, 3, 1);
                g.block(idx_x +  3, 0, 3, 1) = -qv * desired_state_.block(9, i_hor, 3, 1);
                g.block(idx_x +  6, 0, 3, 1) = qr * temp1.block(3,0,3,1);
                g.block(idx_x +  9, 0, 3, 1) = -qw * temp2.block(3,0,3,1);
            }
        }
        H.block(idx_u, idx_u, reduced_inputs, reduced_inputs) = cost_R;
        if(agentnumber == 0){
            g.block(idx_u, 0, reduced_inputs, 1) = cost_R.transpose() * (fop_.block(0,0,12,1)- desired_input_.block(0, i_hor, reduced_inputs, 1));    

        }
        else if(agentnumber == 1){
            g.block(idx_u, 0, reduced_inputs, 1) = cost_R.transpose() * (fop_.block(12,0,12,1)- desired_input_.block(reduced_inputs, i_hor, reduced_inputs, 1));    

        }
    }

    // =================== inequality constraints ================== //
    double NEQ = 32 + 16; 
    double distributed_NEQ = NEQ * 0.5;

    Eigen::MatrixXd lb;
    Eigen::MatrixXd ub;

    Eigen::MatrixXd Fu;
    Eigen::MatrixXd Fu_x_y;
    Eigen::MatrixXd zeroandones;
    
    Eigen::MatrixXd Aineq;
    Eigen::MatrixXd bineq;
    Eigen::MatrixXd fconeaugment;

    lb.setZero(8,1);
    ub.setZero(8,1);
    lb << desired_input_(2), desired_input_(5),desired_input_(8),desired_input_(11),desired_input_(14),desired_input_(17),desired_input_(20),desired_input_(23);
    ub << desired_input_(2), desired_input_(5),desired_input_(8),desired_input_(11),desired_input_(14),desired_input_(17),desired_input_(20),desired_input_(23);
    lb = -2 * lb * 1e0;
    ub =  2 * ub * 1e0;

    Fu.setZero(distributed_NEQ, reduced_inputs);
    Fu_x_y.setZero(4,3);
    zeroandones.setZero(2,3);
    
    Aineq.setZero(distributed_hor * distributed_NEQ, distributed_hor * reduced_decision_vars);
    bineq.setZero(distributed_hor * distributed_NEQ, 1);
    //fconeaugment.setZero(32,1);
    fconeaugment.setZero(16,1);

    Fu_x_y <<  1,  0, -(mu_/sqrt(2)),  
              -1,  0, -(mu_/sqrt(2)),  
               0,  1, -(mu_/sqrt(2)),   
               0, -1, -(mu_/sqrt(2));
    zeroandones << 0,0,1,0,0,-1;

    for(size_t i=0; i<4; i++){
        Fu.block(i*4,i*3, 4,3) = Fu_x_y;
        Fu.block(16+ i*2, i*3,2,3) = zeroandones;
    }

    for(size_t ii=0; ii<distributed_hor; ii++){
        Aineq.block(distributed_NEQ * ii, reduced_decision_vars * ii, distributed_NEQ, reduced_inputs) = Fu;
    }

    Eigen::MatrixXd frictioncone;
    frictioncone.setZero(4,1);
    for(size_t j=0; j<4; j++){
        if(agentnumber == 0){
            frictioncone << (mu_/sqrt(2)) * fop_(2 + 3*j) - fop_(0+ 3*j),
                            (mu_/sqrt(2)) * fop_(2 + 3*j) + fop_(0+ 3*j),
                            (mu_/sqrt(2)) * fop_(2 + 3*j) - fop_(1+ 3*j),
                            (mu_/sqrt(2)) * fop_(2 + 3*j) + fop_(1+ 3*j);
        }
        else if(agentnumber == 1){
            frictioncone << (mu_/sqrt(2)) * fop_(2 + 3*j+12) - fop_(0+ 3*j+12),
                            (mu_/sqrt(2)) * fop_(2 + 3*j+12) + fop_(0+ 3*j+12),
                            (mu_/sqrt(2)) * fop_(2 + 3*j+12) - fop_(1+ 3*j+12),
                            (mu_/sqrt(2)) * fop_(2 + 3*j+12) + fop_(1+ 3*j+12);
        }
        
        fconeaugment.block(4*j, 0, 4,1) = frictioncone;
    }

    Eigen::MatrixXd lu_bounds;
    //lu_bounds.setZero(16,1);
    lu_bounds.setZero(8,1);
    for(size_t jj=0; jj< distributed_hor; jj++){
        bineq.block(distributed_NEQ * jj, 0, 16,1) = fconeaugment;
        if(agentnumber == 0){
            lu_bounds <<  ub(0) - fop_(2) + desired_input_(2, jj),
                        -lb(0) + fop_(2) - desired_input_(2, jj),
                        ub(1) - fop_(5) + desired_input_(5, jj),
                        -lb(1) + fop_(5) - desired_input_(5, jj),
                        ub(2) - fop_(8) + desired_input_(8, jj),
                        -lb(2) + fop_(8) - desired_input_(8, jj),
                        ub(3) - fop_(11) + desired_input_(11, jj),
                        -lb(3) + fop_(11) - desired_input_(11, jj);
            
            bineq.block(distributed_NEQ * jj + 16, 0, 8, 1) = lu_bounds;
        }
        else if(agentnumber == 1){
            lu_bounds <<  ub(4) - fop_(14) + desired_input_(14, jj),
                        -lb(4) + fop_(14) - desired_input_(14, jj),
                        ub(5) - fop_(17) + desired_input_(17, jj),
                        -lb(5) + fop_(17) - desired_input_(17, jj),
                        ub(6) - fop_(20) + desired_input_(20, jj),
                        -lb(6) + fop_(20) - desired_input_(20, jj),
                        ub(7) - fop_(23) + desired_input_(23, jj),
                        -lb(7) + fop_(23) - desired_input_(23, jj);
            
            bineq.block(distributed_NEQ * jj + 16, 0, 8, 1) = lu_bounds;
        }
    }

    // ================== equality constraints ==================== //
    Eigen::MatrixXd Aeq1, Aeq2, Aeq;
    Eigen::MatrixXd beq1, beq2, beq;

    Aeq1.setZero(distributed_hor * reduced_state, distributed_hor * reduced_decision_vars);
    beq1.setZero(distributed_hor * reduced_state, 1);

    Aeq2.setZero(distributed_hor * reduced_holonomic, distributed_hor * reduced_decision_vars);
    beq2.setZero(distributed_hor * reduced_holonomic, 1);

    // equality 1
    if(agentnumber == 0){
        Aeq1.block(0,0, reduced_state, reduced_inputs) = -Bd11;
        Aeq1.block(0, reduced_inputs, reduced_state, reduced_state) = Eigen::MatrixXd::Identity(reduced_state, reduced_state);
        for(size_t i=0; i<distributed_hor-1; i++){
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1                                    , reduced_state, reduced_state) = -Ad11;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state, reduced_state, reduced_inputs) = -Bd11;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 2 + reduced_state, reduced_state, reduced_state) = Eigen::MatrixXd::Identity(reduced_state, reduced_state);
        }

        for(size_t ii = 0; ii< distributed_hor; ii++){
            if(ii == 0){
                beq1.block(reduced_state * ii, 0, reduced_state, 1) = Ad11 * state_op_reduced0 + Dd1;
            }
            else{
                beq1.block(reduced_state * ii, 0, reduced_state, 1) = Dd1;
            }
        }

    }
    else if(agentnumber == 1){
        Aeq1.block(0,0, reduced_state, reduced_inputs) = -Bd22;
        Aeq1.block(0, reduced_inputs, reduced_state, reduced_state) = Eigen::MatrixXd::Identity(reduced_state, reduced_state);
        for(size_t i=0; i<distributed_hor-1; i++){
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1                                    , reduced_state, reduced_state) = -Ad22;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 1 + reduced_state, reduced_state, reduced_inputs) = -Bd22;
            Aeq1.block(reduced_state* (i+1), reduced_decision_vars * i + reduced_inputs * 2 + reduced_state, reduced_state, reduced_state) = Eigen::MatrixXd::Identity(reduced_state, reduced_state);
        }

        for(size_t ii = 0; ii< distributed_hor; ii++){
            if(ii == 0){
                beq1.block(reduced_state * ii, 0, reduced_state, 1) = Ad22 * state_op_reduced1 + Dd2;
            }
            else{
                beq1.block(reduced_state * ii, 0, reduced_state, 1) = Dd2;
            }
        }
    }

    // ================== equality constraints construction ==================== //
    Aeq.setZero(distributed_hor * reduced_state, distributed_hor * reduced_decision_vars); // 0* is TEMP
    beq.setZero(distributed_hor * reduced_state, 1); // 0* is TEMP
    
    Aeq.block(0,0, distributed_hor * reduced_state , distributed_hor * reduced_decision_vars) = Aeq1;
    beq.block(0,0, distributed_hor * reduced_state, 1) = beq1;

    // ================== MPC solver ==================== //
    double* opt_sol;
    opt_sol = new double[distributed_hor * reduced_decision_vars];
    iswiftQp_e(H, g, Aeq, beq, Aineq, bineq, opt_sol);

    for(size_t i=0; i< distributed_hor * reduced_decision_vars; i++){
        dist_opt_sol_e_(i) = opt_sol[i];
    }

    delete[] opt_sol;

    dist_opt_X_ = dist_opt_sol_e_.block(12,0,12,1);
    if(agentnumber == 0){
        dist_opt_u_ = dist_opt_sol_e_.block(0,0,12,1) + fop_.block(0,0,12,1);
        dist_opt_u_.block(0,0,3,1) = dist_opt_u_.block(0,0,3,1) * foot_state1_(0);
        dist_opt_u_.block(3,0,3,1) = dist_opt_u_.block(3,0,3,1) * foot_state1_(1);
        dist_opt_u_.block(6,0,3,1) = dist_opt_u_.block(6,0,3,1) * foot_state1_(2);
        dist_opt_u_.block(9,0,3,1) = dist_opt_u_.block(9,0,3,1) * foot_state1_(3);
        opt_u_.block(0,0,12,1) = dist_opt_u_;
    }
    else if(agentnumber == 1){
        dist_opt_u_ = dist_opt_sol_e_.block(0,0,12,1) + fop_.block(12,0,12,1);
        dist_opt_u_.block(0,0,3,1) = dist_opt_u_.block(0,0,3,1) * foot_state2_(0);
        dist_opt_u_.block(3,0,3,1) = dist_opt_u_.block(3,0,3,1) * foot_state2_(1);
        dist_opt_u_.block(6,0,3,1) = dist_opt_u_.block(6,0,3,1) * foot_state2_(2);
        dist_opt_u_.block(9,0,3,1) = dist_opt_u_.block(9,0,3,1) * foot_state2_(3);
        opt_u_.block(12,0,12,1) = dist_opt_u_;
    }

    if(isnan(dist_opt_u_.norm())){
    }

    if(dist_opt_u_.block(0,0,12,1).norm() > 1e3){ //original: 1e3
        if(agentnumber == 0){
            dist_opt_u_ = fop_.block(0,0,12,1);
        }
        else if(agentnumber == 1){
            dist_opt_u_ = fop_.block(12,0,12,1);
        }
    }    
}

void LocomotionPlanner::compute(){
    updateStateop();
    motionPlanner();
    footstepPlanner();
    srbMPC();
    mpcdataLog();
}

void LocomotionPlanner::compute_dist(){
    updateStateop();
    motionPlanner();
    footstepPlanner();
    srbMPC_distributed(agentID_);
    mpcdataLog();
}

void LocomotionPlanner::impactDetection(){
    if(controlTick_ >= locoStart_){
        if(indxupdatefromfullorder_ == false){
            // domain change based on time
            Eigen::Matrix<double, 4, 1> foot_1001 = {1,0,0,1};
            Eigen::Matrix<double, 4, 1> foot_0110 = {0,1,1,0};
            if(locomotionTick_ == Tst_/simfreq){
                // ====== TROT for ag1 ===== //
                if((foot_state1_ - foot_1001).norm() == 0){ //trot
                    foot_state1_ << 0,1,1,0;
                }
                else if((foot_state1_ - foot_0110).norm() == 0){ //trot
                    foot_state1_ << 1,0,0,1;
                }
                else{
                    std::cout << "foot state 1 is not trotting" << std::endl;
                }

                // ====== TROT for ag2 ===== //
                if((foot_state2_ - foot_1001).norm() == 0){ //trot
                    foot_state2_ << 0,1,1,0;
                }
                else if((foot_state2_ - foot_0110).norm() == 0){ //trot
                    foot_state2_ << 1,0,0,1;
                }
                else{
                    std::cout << "foot state 2 is not trotting" << std::endl;
                }

                // ====== TROT for compositetrot ===== //
                if(compositesrbs_){
                    if(foot_state2_(0) == 1 && foot_state1_(3) == 1){
                        foot_state1_ << 0,1,0,0;
                        foot_state2_ << 0,0,1,0;
                    }
                    else if(foot_state2_(0) == 0 && foot_state1_(3) == 0){
                        foot_state1_ << 0,0,0,1;
                        foot_state2_ << 1,0,0,0;
                    }
                }

                locotickReset();
            }
        }
        locomotionTick_ ++;

        // ignite mpc solver on every dt
        if(mpcTick_ == dt_/simfreq){
        //if(mpcTick_ == 6){
            compute();

            fop_ = opt_u_;

            mpctickReset();
        }
        mpcTick_++;
    }
}

void LocomotionPlanner::impactDetection_dist(){
    if(controlTick_ >= locoStart_){
        if(indxupdatefromfullorder_ == false){
            // domain change based on time
            if(locomotionTick_ == Tst_/simfreq){
                if(foot_state1_(0) == 1 && foot_state1_(1) == 0){
                    foot_state1_ << 0,1,1,0;
                }
                else if(foot_state1_(0) == 0 && foot_state1_(1) == 1){
                    foot_state1_ << 1,0,0,1;
                }
                else{
                    std::cout << "foot state 1 is not trotting" << std::endl;
                }

                if(foot_state2_(0) == 1 && foot_state2_(1) == 0){
                    foot_state2_ << 0,1,1,0;
                }
                else if(foot_state2_(0) == 0 && foot_state2_(1) == 1){
                    foot_state2_ << 1,0,0,1;
                }
                else{
                    std::cout << "foot state 2 is not trotting" << std::endl;
                }

                locotickReset();
            }
        }
        locomotionTick_ ++;

        // ignite mpc solver on every dt
        if(mpcTick_ == dt_/simfreq){
        //if(mpcTick_ == 6){
            compute_dist();

            fop_ = opt_u_;

            mpctickReset();
        }
        mpcTick_++;
    }
}

void LocomotionPlanner::impactDetection_exp(){
    if(controlTick_ !=0){
        // ignite mpc solver on every dt
        compute();
        fop_ = opt_u_;

    }
}

void LocomotionPlanner::mpcdataLog(){
    size_t timeshift  = 0; // 10; //10sec: normal timeshift value
    if (FILE_RW){
        if(controlTick_ >= timeshift * ctrlHz){
            if(agentID_ == 0){
                file[0] << controlTick_*simfreq <<","
                        << opt_u_(0) << "," << opt_u_(1) << "," << opt_u_(2) << "," << opt_u_(3) << "," 
                        << opt_u_(4) << "," << opt_u_(5) << "," << opt_u_(6) << "," << opt_u_(7) << "," 
                        << opt_u_(8) << "," << opt_u_(9) << "," << opt_u_(10) << "," << opt_u_(11) << ","
                        << state_(0) << "," << state_(1) << "," << state_(2) << "," // actual pos of ag0
                        << state_(6) << "," << state_(7) << "," << state_(8) << "," // actual vel of ag0
                        << desired_state_(0,0) << "," << desired_state_(1,0) << "," << desired_state_(2,0) << "," // desired pos of ag0
                        << desired_state_(6,0) << "," << desired_state_(7,0) << "," << desired_state_(8,0) << "," // desired vel of ag0
                        //<<lambda_op0_<< "," << lambda_bias_tick_ << "," << lambda_bias_mag_<<std::endl;
                        <<dist_opt_sol_e_(24)<< "," << lambda_bias_tick_ << "," << lambda_bias_mag_<<std::endl;
            }
            if(agentID_ == 1){
                file[1] << controlTick_*simfreq <<","
                        << opt_u_(12) << "," << opt_u_(13) << "," << opt_u_(14) << "," << opt_u_(15) << "," 
                        << opt_u_(16) << "," << opt_u_(17) << "," << opt_u_(18) << "," << opt_u_(19) << "," 
                        << opt_u_(20) << "," << opt_u_(21) << "," << opt_u_(22) << "," << opt_u_(23) << ","
                        << state_(3) << "," << state_(4) << "," << state_(5) << "," // actual pos of ag1
                        << state_(9) << "," << state_(10) << "," << state_(11) << "," // actual vel of ag1
                        << desired_state_(3,0) << "," << desired_state_(4,0) << "," << desired_state_(5,0) << "," // desired pos of ag1
                        << desired_state_(9,0) << "," << desired_state_(10,0) << "," << desired_state_(11,0) << "," // desired vel of ag1
                        //<<lambda_op1_<< "," << lambda_bias_tick_ << "," << lambda_bias_mag_<<std::endl;
                        <<dist_opt_sol_e_(24)<< "," << lambda_bias_tick_ << "," << lambda_bias_mag_<<std::endl;
            }          
        }
    }
}
