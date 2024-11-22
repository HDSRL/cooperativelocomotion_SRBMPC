#ifndef LOCOMOTION_PLANNER_H
#define LOCOMOTION_PLANNER_H

#include "math_define.h"
#include "iswift_qp.h"
#include "srbcoopModel.h"

#include "fstream"

const size_t FILE_CNT = 2;
const std::string FILE_NAMES[FILE_CNT] = {
    "/home/seop/Documents/datalog_srb/mpc_data0.txt",
    "/home/seop/Documents/datalog_srb/mpc_data1.txt"
};

class LocomotionPlanner{
    public:
        std::fstream file[FILE_CNT];
        LocomotionPlanner();
        virtual ~LocomotionPlanner(){
            if(FILE_RW == true){
                for(size_t i=0; i<FILE_CNT; i++){
                    if(file[i].is_open()){
                        file[i].close();
                    }
                }
                std::cout << "file close done" << std::endl;
            }
            else{
                std::cout << "file rw was disabled at the beginning" <<std::endl;
            }
        }

        void locotickReset();
        void mpctickReset();
        void timeSetup(size_t standing_duration, size_t loco_start);
        //void stepSetup(double totalStepNum, double step_X, double step_Y, double body_R, double body_P, double body_Y, Eigen::MatrixXd agent_Initial);
        //void stepSetup(double agentid, double totalStepNum, double step_X, double step_Y, double body_R, double body_P, double body_Y, Eigen::MatrixXd agent_Initial);
        void agentidsetup(const size_t numberofagent, const size_t agentID);
        void gaitdomainReset(double newdomain){gaitDomain_ = newdomain;}

        void getRobotState(const Eigen::VectorXd& pos1, const Eigen::VectorXd& pos2, 
                            const Eigen::VectorXd& vel1, const Eigen::VectorXd& vel2,
                            const Eigen::MatrixXd& rot1, const Eigen::MatrixXd& rot2,
                            const Eigen::VectorXd& omega1, const Eigen::VectorXd& omega2, const size_t ctrlTick);
        void getRobotStatewindx(const Eigen::VectorXd& pos1, const Eigen::VectorXd& pos2, 
                            const Eigen::VectorXd& vel1, const Eigen::VectorXd& vel2,
                            const Eigen::MatrixXd& rot1, const Eigen::MatrixXd& rot2,
                            const Eigen::VectorXd& omega1, const Eigen::VectorXd& omega2, const size_t ctrlTick,
                            int* condIndx0, int* condIndx1);
        void updateStateop();
        
        void getSharedInfo(size_t agentnumber, const Eigen::VectorXd agentstates); // shared info between agents
        void getSharedInfo(size_t agentnumber, const Eigen::VectorXd agentstates, 
                            const Eigen::MatrixXd cycIndex1, const Eigen::MatrixXd footprintGlobal1, const Eigen::MatrixXd footprintGlobalOnes1, const::Eigen::MatrixXd comDesiredTraj1, 
                            const Eigen::MatrixXd cycIndex2, const Eigen::MatrixXd footprintGlobal2, const Eigen::MatrixXd footprintGlobalOnes2, const::Eigen::MatrixXd comDesiredTraj2);

        void motionPlanner(); //generate complan based on desired COM velocity
        void footstepPlanner(); //generate footstep based on desired COM velocity with Raibert

        void srbMPC();
        void srbMPC_composite();
        void srbMPC_distributed(size_t agentnumber);
        void srbMPC_single(size_t agentnumber);

        void impactDetection();
        void impactDetection_dist();
        void impactDetection_exp(); // freq should be set up on the top level (main ftn of the exp)
        void impactDetection(size_t &gait);
        void impactDetection(size_t &gait, bool supervised); //2 agents

        void compute();
        void compute_dist();
        void compute(size_t gait);
        void compute(size_t gait, double com_height);

        void dataLog(size_t gait);

        void mpcdataLog();

        void torqueSaturation();  

        const Eigen::Matrix<double, 24, 1>& forceFF(){return opt_u_;}
        const Eigen::Matrix<double, 12, 1>& forceFFdist(){return dist_opt_u_;}
        const Eigen::Matrix<double, 24, 1>& stateFF(){return opt_X_;}
        const Eigen::Matrix<double, 24, 1>& localfootpos(){return p_foot_;} 
        void forceopUpdate(Eigen::MatrixXd forceFF){fop_ = forceFF;};
        const Eigen::MatrixXd& getkktMultiplier0dist(){return kktmultiplier0dist_;};
        const Eigen::MatrixXd& getkktMultiplier1dist(){return kktmultiplier1dist_;};
        void kktMultiplier0distupdate(const Eigen::MatrixXd kkt0){kktmultiplier0dist_ = kkt0;};
        void kktMultiplier1distupdate(const Eigen::MatrixXd kkt1){kktmultiplier1dist_ = kkt1;};
        const Eigen::MatrixXd& getdistoptsol(){return dist_opt_sol_e_;};
        //void optsol0update(const Eigen::MatrixXd optsol){dist_opt_sol0_e_ = optsol;};
        //void optsol1update(const Eigen::MatrixXd optsol){dist_opt_sol1_e_ = optsol;};
        const double& getlambdaop0(){return lambda_op0_;};
        const double& getlambdaop1(){return lambda_op1_;};
        
        const Eigen::Matrix<double, 3, 1>& desiredcomvel(size_t agnum){
            if(agnum == 0){
                return desired_comvel0_;
            }
            else if(agnum == 1){
                return desired_comvel1_;
            }
        };
        const Eigen::MatrixXd& desiredstate(){return desired_state_;};     


    private:
        size_t controlTick_;    // Total time flow
        size_t locomotionTick_; // reset every domain
        size_t mpcTick_;

        double step_X_;
        double step_Y_;
        double body_R_;
        double body_P_;
        double body_Y_;
        Eigen::MatrixXd agent_Initial_;

        size_t gridNum_;        // grid number in domain
        size_t ts_OptTick_;     // time step number of each grid based on tick
        double phaseVariable_;  // reset every domain
        
        size_t gait_;           // kind of current gait: STAND, WALK, TROT, GALLOP
        size_t onegaitCycle_;   // how many steps in one gaitcycle: 4
        size_t gaitDomain_;     // number of current Index column in totalCycleIndex: starting from 0th column =[1,1,1,1]

        size_t standingDuration_;
        size_t locoStart_;

        size_t armConfig_;

        bool FILE_RW;

        // ================================= //
        // ============ SRB MPC ============ //
        // ================================= //
        double init_x_;
        double init_y_;

        // holonomic constraint applied position wrt COM of robot
        Eigen::Vector3d d1_;
        Eigen::Vector3d d2_;

        double mu_; // friction coefficient
        double dt_; // sim time step
        double g_; // acc due to gravity

        size_t n_hor_; // planning horizon
        size_t distributed_hor_; //planning horizon for distributed

        // MPC linear system number of states and control
        size_t n_state_; // 6 + 6 + 6 + 6: compos(3)x2 + comvel(3)x2 + rot(3)x2 + omega(3)x2
        size_t n_holonomic_; // number of lambda
        size_t n_inputs_; //12 + 12: contactforce(3)x4legs + contactforce(3)x4legs 
        size_t n_decision_vars_;

        size_t reduced_state_;
        size_t reduced_holonomic_;
        size_t reduced_inputs_;
        size_t reduced_decision_vars_;

        double Tst_; // swing time

        // =========================== A1 robot properties ========================== //
        double mass_; // mass
        Eigen::Matrix<double, 3, 3> J_; // inertia

        Eigen::Vector3d phiprf_local_; // hip position - right front from body center
        Eigen::Vector3d phiplf_local_; // hip position - left front from body center
        Eigen::Vector3d phiprh_local_; // hip position - right hind from body center
        Eigen::Vector3d phiplh_local_; // hip position - right hind from body center

        double zheight_;

        Eigen::Matrix<double, 24, 1> state_; // states expressed with variational based expansion (xi)
        // ================ operation points ==================== //
        Eigen::Matrix<double, 36,1 > state_op_;
        Eigen::Matrix<double, 24,1 > state_op_variational_;
        Eigen::Matrix<double, 3, 3>  rot1_op_; // saved rotation matrix form of the vectorized rotation matrix in state_op_
        Eigen::Matrix<double, 3, 3>  rot2_op_; // saved rotation matrix form of the vectorized rotation matrix in state_op_
        double lambda_op_;
        double lambda_op0_;
        double lambda_op1_;
        Eigen::Matrix<double, 24,1>  p_foot_global_;
        Eigen::Matrix<double, 24,1>  p_foot_nominal_;
        Eigen::Matrix<double, 24,1>  p_foot_;
        Eigen::Matrix<double, 12, 1> p_foot1_;
        Eigen::Matrix<double, 12, 1> p_foot2_;
        Eigen::Matrix<double, 24, 1> p_hip_; // total hip position from global frame
        Eigen::Matrix<double, 24, 1> fop_nominal_;
        Eigen::Matrix<double, 24, 1> fop_;

        // ========= foot state =========== //
        Eigen::Matrix<double, 4, 1> foot_state1_;
        Eigen::Matrix<double, 4, 1> foot_state2_;

        // ========= srb state =========== //
        Eigen::Vector3d body1pos_;
        Eigen::Matrix<double,3,3> body1rot_;
        Eigen::Vector3d body1vel_;
        Eigen::Vector3d body1omega_;

        Eigen::Vector3d body2pos_;
        Eigen::Matrix<double,3,3> body2rot_;
        Eigen::Vector3d body2vel_;
        Eigen::Vector3d body2omega_;

        // ========= desired state design portion ========= //
        Eigen::Vector3d desired_comvel_;
        Eigen::Vector3d desired_comvel0_;
        Eigen::Vector3d desired_comvel1_;
        Eigen::MatrixXd desired_state_;
        Eigen::MatrixXd desired_input_;
        double desired_lambda_;

        Eigen::MatrixXd opt_sol_e_;
        Eigen::Matrix<double, 24, 1> opt_u_;
        Eigen::Matrix<double, 24, 1> opt_X_; //body1pos - body2pos - body1vel - body2vel - xi1 - xi2 - omega1 - omega2
        Eigen::MatrixXd kktmultiplier_e_;

        Eigen::MatrixXd dist_opt_sol_e_;
        Eigen::MatrixXd dist_opt_sol0_e_;
        Eigen::MatrixXd dist_opt_sol1_e_;
        Eigen::Matrix<double, 12, 1> dist_opt_u_;
        Eigen::Matrix<double, 12, 1> dist_opt_X_; //bodypos - bodyvel - xi - omega
        Eigen::MatrixXd kktmultiplier0dist_;
        Eigen::MatrixXd kktmultiplier1dist_;


        //shared info between agents
        size_t agentID_;              //ID of each agent
        size_t totalagentnumber_;          //Total number of agents
        Eigen::VectorXd agentStates_; //com x1 x1dot y1 y1dot x2 x2dot y2 y2dot
        Eigen::VectorXd agentU_; //cop x1 y1 x2 y2
        
        Eigen::MatrixXd agent1cycIndex_;
        Eigen::MatrixXd agent1totalfootprintGlobal_;
        Eigen::MatrixXd agent1totalfootprintGlobalOnes_;
        Eigen::MatrixXd agent1comDesiredTrajVec_;
        
        Eigen::MatrixXd agent2cycIndex_;
        Eigen::MatrixXd agent2totalfootprintGlobal_;
        Eigen::MatrixXd agent2totalfootprintGlobalOnes_;
        Eigen::MatrixXd agent2comDesiredTrajVec_;

        //MPC supervised
        Eigen::MatrixXd trajx_e_;
        Eigen::MatrixXd traju_e_;

        bool indxupdatefromfullorder_;

        bool compositesrbs_;

        bool lambda_bias_;
        double lambda_bias_mag_;
        double lambda_bias_tick_;

};


#endif //LOCOMOTION_PLANNER_H
