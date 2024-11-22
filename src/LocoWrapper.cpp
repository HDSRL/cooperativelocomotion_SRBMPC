#include "LocoWrapper.hpp"
#include "iostream"

LocoWrapper::LocoWrapper(int argc, char *argv[]) : Parameters(argc,argv){

//    std::string filename = "/media/kavehakbarihamed/Data/A1_RaiSim_Outputs/DeePC/data_07.txt";
//    std::string filename = "/home/kaveh/A1_exp_data/DeePC_Data/Paper_Experiments/Pull_04.csv"; // empty string will produce no output file
	std::string filename0 = "/home/seop/Documents/datalog_srb/data0.txt";
    std::string filename1 = "/home/seop/Documents/datalog_srb/data1.txt";
	//std::string filename0 = "";
    //std::string filename1 = "";
    
    data0 = std::unique_ptr<DataLog>( new DataLog(filename0) ); // make_unique DNE in c++11
    data1 = std::unique_ptr<DataLog>( new DataLog(filename1) ); // make_unique DNE in c++11
    // deepc  = std::unique_ptr<DeePC>( new DeePC(argc,argv) );
    quad = new RobotModel();
    conEst = new ContactEst();
    LL = new LowLevelCtrl();
    VC = new VirtualConstraints();
    PP = new MotionPlanner();

    state = quad->getStatePointer();
    dyn = quad->getDynamicsPointer();
    kin = quad->getKinematicsPointer();
    con = conEst->getConInfoPointer();
    traj = PP->getTrajInfoPointer();
    vcon = VC->getVCPointer();
    ll = LL->getllPointer();
    
    locoTick = 0;
    maxPhase = 1.05;

    Eigen::Matrix<double, 12, 1> fDes;
    fDes << 0,0,con->ind[0],0,0,con->ind[1],0,0,con->ind[2],0,0,con->ind[3];
    fDes *= (MASS*GRAV/con->cnt);
    VC->setDesiredForce(fDes);

    double a[3] = {1.00000000,-1.95557824,0.95654368}; // 5 Hz
    double b[3] = {0.00024136,0.00048272,0.00024136};
    // double a[3] = {1.00000000,-1.99111429,0.99115360}; // 1 Hz
    // double b[3] = {0.00000983,0.00001965,0.00000983};
    populate_filter_d(trajFilt,a,b,3,3);

    HL_com.setZero(12,1);
    HL_force.setZero(12,1);
}
 
LocoWrapper::~LocoWrapper(){
    delete quad;
    delete conEst;
    delete LL;
    delete VC;
    delete PP;
    clear_filter_d(trajFilt);
}

void LocoWrapper::calcTau(const double q[18], const double dq[18], const double R[9], const int force[4], size_t gait, size_t ctrlTick){
    phaseVar = getPhase(1.0*locoTick, 0.0, 1.0*traj->domLen);   // update phase variable
    quad->updateState(q,dq,R);                                  // update state

    Eigen::Matrix<double, 12, 1> fDes;
    fDes << 0,0,con->ind[0],0,0,con->ind[1],0,0,con->ind[2],0,0,con->ind[3];
    fDes *= (MASS*GRAV/con->cnt);

    float footPos[4] = {0};  // DUMMY VARS
    if (gait!=gaitTemp || (phaseVar>=maxPhase && gait!=STAND) ){ 
        // Change domain immediately since gait changed or phase var exceeded upper bound
        conEst->forceDomChange();                                                       // force con->changeDomain=1 to plan properly
        std::cout << "Time trigger: " << phaseVar << std::endl;
        phaseVar = 0;
        PP->planTraj(state, kin, conEst, gait, phaseVar, ctrlTick, &motion_params);     // plan trajectory
        conEst->updateConState(footPos,phaseVar,force);                                 // update contact detection
        locoTick = 0;
    }else {
        // Wait for impact to change domain
        conEst->updateConState(footPos,phaseVar,force);                                 // impact detection
        if (con->changeDomain==1 && gait!=STAND && gait!=POSE){
            locoTick = 0;
            std::cout << "Contact trigger: " << phaseVar << std::endl;
            phaseVar = 0;
        }
        PP->planTraj(state, kin, conEst, gait, phaseVar, ctrlTick, &motion_params);     // plan trajectory
    }

    // Update the trajectory from MPC
    if ( gait!=STAND && useHL){
        VC->setDesiredForce(HL_force);
        PP->setDesiredCom(HL_com);
    }else{
        VC->setDesiredForce(fDes);  
    }
    
    // Update the low-level controller
    quad->updateSwingMatrices(con->ind,con->cnt);                                               // update the jacobians
    VC->updateVirtualConstraints(state, kin, traj, con, gait, phaseVar, &motion_params, ll);    // update VC's
    LL->calcTorque(state, dyn, kin, vcon, con, &ll_params);                                     // run low level controller
    if(eachagnum == 0){
        data0->writeData(state,vcon,traj,ll,ctrlTick,force);                                         // log relavent data
    }
    else if(eachagnum == 1){
        data1->writeData(state,vcon,traj,ll,ctrlTick,force);                                         // log relavent data
    }

    locoTick += (ctrlHz)/LL_Hz;     // increment locoTick
    gaitTemp = gait;                // update the previous gait used
}






