#include "MotionPlanner.hpp"

MotionPlanner::MotionPlanner(){
    x0 = 0;
    y0 = 0;
    z0 = 0.05;
    
    traj.domLen = 1*ctrlHz;
    standTime = 1*ctrlHz;

    traj.comDes.setZero();
    traj.redDes.setZero(HORIZ*NRS,1);
    traj.toeInit.setZero();
    traj.toeFinal.setZero();
    traj.toeOffset[2] = Z_TOE_OFFSET;
}

void MotionPlanner::updateStandVars(const Eigen::Matrix<double,3,1> &com, double timeToStand){
    x0 = com(0);
    y0 = com(1);
    z0 = com(2);
    traj.domLen = timeToStand;
    standTime = timeToStand;
    if(timeToStand<100){
        traj.domLen *=ctrlHz;
        standTime *= ctrlHz;
    }
};

void MotionPlanner::updatePoseType(size_t type){
    poseType = type;
}

void MotionPlanner::planTraj(const StateInfo *state, const KinematicsInfo *kin, ContactEst *con_obj, size_t gait, double phase, size_t ctrlTick, MP * params){
    const ContactInfo* con = con_obj->getConInfoPointer();  

    static Eigen::Matrix<double, 3, 1> desVel = {0,0,0};
    static Eigen::Matrix<double, 3, 1> desOmega = {0,0,0};
    static double yawLock = 0;
    if(gait==STAND){
        double s = (phase>1) ? 1 : ((phase<0) ? 0 : phase);

        double xFinal = x0-0.04*0;
        double yFinal = y0-0.04*0;
        double zFinal = params->standHeight;
        double alpha_x[8] = { x0,x0,x0,
                 x0+(xFinal-x0)/4,
                 x0+3*(xFinal-x0)/4,
                 xFinal,xFinal,xFinal};
        double alpha_y[8] = {y0,y0,y0,
                 y0+(yFinal-y0)/4,
                 y0+3*(yFinal-y0)/4,
                 yFinal,yFinal,yFinal};
        double alpha_z[8] = {z0,z0,z0,
                 z0+(zFinal-z0)/4,
                 z0+3*(zFinal-z0)/4,
                 zFinal,zFinal,zFinal};

        double traj_x[3], traj_y[3], traj_z[3];
        calcBezierAll((int)8, alpha_x, s, traj_x);
        calcBezierAll((int)8, alpha_y, s, traj_y);
        calcBezierAll((int)8, alpha_z, s, traj_z);

        // traj.comDes -> pos, vel, theta, omega
        traj.comDes.block(0,0,3,1) << traj_x[0], traj_y[0], traj_z[0];
        traj.comDes.block(3,0,3,1) << traj_x[1], traj_y[1], traj_z[1];
        traj.comDes.block(6,0,3,1) << 0, 0, 0;
        traj.comDes.block(9,0,3,1) << 0, 0, 0;

        traj.domLen = standTime;
        con_obj->setDesDomain({1,1,1,1});

        traj.toeInit = kin->toePos; 
        traj.toeFinal = kin->toePos;
        Eigen::Matrix<double, 12, 1> trajtemp;
        trajtemp.block(0,0,3,1) = traj.comDes.block(0,0,3,1);
        trajtemp.block(3,0,3,1) = traj.comDes.block(6,0,3,1);
        trajtemp.block(6,0,3,1) = traj.comDes.block(3,0,3,1);
        trajtemp.block(9,0,3,1) = traj.comDes.block(9,0,3,1);
        repmat(trajtemp,traj.redDes,HORIZ,1);
        
    }else if(gait==POSE) {
        traj.comDes.block(3,0,3,1) << 0,0,0;
        traj.comDes.block(9,0,3,1) << 0,0,0;
        double t = 1.0*ctrlTick/(1.0*ctrlHz);
        static double t_init = t;
        static Eigen::Matrix<double, 12, 1> lock = traj.comDes;
        traj.comDes = lock;
        if(poseType==POSE_X){
			double freq = 0.8*MY_PI;
            double mag = 0.04;
            traj.comDes(0) += mag*sin(freq*(t-t_init));
            traj.comDes(3) += mag*freq*cos(freq*(t-t_init));
            for (int i=0; i<HORIZ; ++i){
                traj.redDes(i*NRS+0) = lock(0) + mag*sin(freq*(t-t_init));
                traj.redDes(i*NRS+6) =  mag*freq*cos(freq*(t-t_init));
                ctrlTick+=(1000/MPC_Hz);
                t = 1.0*ctrlTick/(1.0*ctrlHz);
            }
        }
        else if(poseType==POSE_Y){
            double freq = 0.8*MY_PI;
            double mag = 0.04;
            traj.comDes(1) += mag*sin(freq*(t-t_init));
            traj.comDes(4) += mag*freq*cos(freq*(t-t_init));
            for (int i=0; i<HORIZ; ++i){
                traj.redDes(i*NRS+1) = lock(1) + mag*sin(freq*(t-t_init));
                traj.redDes(i*NRS+7) = mag*freq*cos(freq*(t-t_init));
                ctrlTick+=(1000/MPC_Hz);
                t = 1.0*ctrlTick/(1.0*ctrlHz);
            }
        }
        else if(poseType==POSE_Z){
            double freq = 0.8*MY_PI;
            double mag = 0.05;
            traj.comDes(2) += mag*cos(freq*(t-t_init))-mag;
            traj.comDes(5) += -1.0*mag*freq*sin(freq*(t-t_init));
            for (int i=0; i<HORIZ; ++i){
                traj.redDes(i*NRS+2) = lock(2) + mag*cos(freq*(t-t_init))-mag;
                traj.redDes(i*NRS+8) = -1.0*mag*freq*sin(freq*(t-t_init));
                ctrlTick+=(1000/MPC_Hz);
                t = 1.0*ctrlTick/(1.0*ctrlHz);
            }
        }
        else if(poseType==POSE_ROLL){
            double freq = 0.8*MY_PI;
			double mag = 0.3491;
            traj.comDes(6) += mag*sin(freq*(t-t_init));
            traj.comDes(9) += mag*freq*cos(freq*(t-t_init));
            for (int i=0; i<HORIZ; ++i){
                traj.redDes(i*NRS+3) = mag*sin(freq*(t-t_init));
                traj.redDes(i*NRS+9) = mag*freq*cos(freq*(t-t_init));
                ctrlTick+=(1000/MPC_Hz);
                t = 1.0*ctrlTick/(1.0*ctrlHz);
            }
        }
        else if(poseType==POSE_PITCH){
            double freq = 0.8*MY_PI;
            double mag = 0.17453;
            traj.comDes(7) += mag*sin(freq*(t-t_init));
            traj.comDes(10) += mag*freq*cos(freq*(t-t_init));
            for (int i=0; i<HORIZ; ++i){
                traj.redDes(i*NRS+4)  = mag*sin(freq*(t-t_init));
                traj.redDes(i*NRS+10) = mag*freq*cos(freq*(t-t_init));
                ctrlTick+=(1000/MPC_Hz);
                t = 1.0*ctrlTick/(1.0*ctrlHz);
            }
        }
        else if(poseType==POSE_YAW){
            double freq = 0.8*MY_PI;
            double mag = 0.13963;
            traj.comDes(8) += mag*sin(freq*(t-t_init));
            traj.comDes(11) += mag*freq*cos(freq*(t-t_init));
            for (int i=0; i<HORIZ; ++i){
                traj.redDes(i*NRS+5)  = mag*sin(freq*(t-t_init));
                traj.redDes(i*NRS+11) = mag*freq*cos(freq*(t-t_init));
                ctrlTick+=(1000/MPC_Hz);
                t = 1.0*ctrlTick/(1.0*ctrlHz);
            }
        }
        else if(poseType==POSE_COMB){
        	double freq = 0.6*MY_PI;
        	double mag = 0.3491;
        	traj.comDes(7) += mag*sin(freq*(t-t_init));
        	traj.comDes(10) += mag*freq*cos(freq*(t-t_init));
        	
            static int triggerStart = 0;
            if (triggerStart || cos(freq*(t-t_init))<0){
                traj.comDes(8) += mag*cos(freq*(t-t_init));
                traj.comDes(11) += -mag*freq*sin(freq*(t-t_init));
                triggerStart = 1;
            }
        }
    }else if(gait==TAP){
        setStepLen(0.0,0.0,0.0);
        if(con->changeDomain){ 
        	static double domLenSec = 0.5;
            con_obj->setDesDomain({1, 0, 1, 1});
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;
//            domLenSec -= (domLenSec>1.0) ? 0.25 : 0;
        }
    }else if(gait==INPLACE_WALK){
        if(con->changeDomain==1){
            traj.toeInit = kin->toePos;
            
            static int n = 0;
            Eigen::Matrix<int, 4, 4> doms;
            doms << 0,1,1,1,
                    1,0,1,1,
                    1,1,1,0,
                    1,1,0,1;
            n = (++n) % 4;
            con_obj->setDesDomain({doms(n,0),doms(n,1),doms(n,2),doms(n,3)});
            traj.domLen = 0.3*ctrlHz;
        }
    }else if(gait==INPLACE_TROT){
        if(con->changeDomain){
            if(con->des[0]==1){
                con_obj->setDesDomain({0, 1, 1, 0});}
            else{
                con_obj->setDesDomain({1, 0, 0, 1});}
            traj.domLen = 0.16*ctrlHz;
            traj.toeInit.block(0,0,2,4) = kin->hipPos.block(0,0,2,4);
            traj.toeInit.block(2,0,1,4) = kin->toePos.block(2,0,1,4);
        }
    }else if(gait==WALK){
        double dt = (1.0/LL_Hz);
        double domLenSec = 0.4;

        if(con->changeDomain==1){
            // desVel(0) += (desVel(0)<0.2) ? 0.05 : 0; 
            updateVel(desVel,desOmega,params);
            
            // ================================================ //
            // Update contact matrix and initial toe position(s)
            // ================================================ //
            static int n = 2;
            Eigen::Matrix<int, 4, 4> doms;
            doms << 0,1,1,1,
                    1,1,0,1,
                    1,1,1,0,
                    1,0,1,1;
            n = (++n) % 4;
            con_obj->setDesDomain({doms(n,0),doms(n,1),doms(n,2),doms(n,3)});
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;

            // ================================================ //
            // Marc Raibert foothold selection (similar)
            // ================================================ //
            Eigen::Matrix<double, 3, 1> stepLenTemp;
            std::vector<double> KP = {0.04,0.01,0.0};
            setStepLen(0.0,0.0,0.0);
            stepLenTemp = toBody(state->comFiltered,state->R)-desVel;
            stepLenTemp(0) *= KP[0]; stepLenTemp(1) *= KP[1]; stepLenTemp(2) *= KP[2];
            stepLenTemp += (domLenSec*desVel/2);
            stepLenTemp(0)+=0.05;
            toWorld(traj.stepLen,stepLenTemp,state->R);
        }
    }else if(gait==TROT){
        double domLenSec = 0.2;
        
        static int startTrot = 0;
        if(con->changeDomain==1){
        	if (startTrot>10){
                updateVel(desVel, desOmega, params);
            }
            startTrot+=1;
            
            // ================================================ //
            // Update contact matrix and initial toe position(s)
            // ================================================ //
            if(con->des[0]==1){
                con_obj->setDesDomain({0, 1, 1, 0});}
            else{
                con_obj->setDesDomain({1, 0, 0, 1});}
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;

            // ================================================ //
            // Marc Raibert foothold selection (similar)
            // ================================================ //
            Eigen::Matrix<double, 3, 1> stepLenTemp;
            std::vector<double> KP = {0.04,0.005,0.0};
            setStepLen(0.0,0.0,0.0);
            stepLenTemp = toBody(state->comFiltered,state->R)-desVel;
            stepLenTemp(0) *= KP[0]; stepLenTemp(1) *= KP[1]; stepLenTemp(2) *= KP[2];
            stepLenTemp += (domLenSec*desVel/2);
            toWorld(traj.stepLen,stepLenTemp,state->R);
        }
    }else if(gait==PACE){
        double dt = (1.0/LL_Hz);
        double domLenSec = 0.1;

        if(con->changeDomain==1){
            updateVel(desVel, desOmega, params);
            
            // ================================================ //
            // Update contact matrix and initial toe position(s)
            // ================================================ //
            if(con->des[0]==1){
                con_obj->setDesDomain({0, 1, 0, 1});}
            else{
                con_obj->setDesDomain({1, 0, 1, 0});}
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;

            // ================================================ //
            // Marc Raibert foothold selection (similar)
            // ================================================ //
            Eigen::Matrix<double, 3, 1> stepLenTemp;
            std::vector<double> KP = {0.04,0.02,0.0};
            setStepLen(0.0,0.0,0.0);
            stepLenTemp = toBody(state->comFiltered,state->R)-desVel;
            stepLenTemp(0) *= KP[0]; stepLenTemp(1) *= KP[1]; stepLenTemp(2) *= KP[2];
            stepLenTemp += (domLenSec*desVel/2);
            toWorld(traj.stepLen,stepLenTemp,state->R);
        }
    }else if(gait==RAND){
        double dt = (1.0/LL_Hz);
        double domLenSec = 0.16;

        static int timecnt = 0;
        double t = 1.0*ctrlTick/(1.0*ctrlHz);
        static double t_init = t;
        double maxvelx = 1.5;
        double maxvely = 0.5;
        double freq = 0.1*2*MY_PI;
        double mag = maxvelx;
        if (timecnt % 10 == 0){
            desVel(0) = 2*maxvelx*(static_cast<double>(rand())/static_cast<double>(RAND_MAX))-maxvelx+0.2;
            desVel(1) = 2*maxvely*(static_cast<double>(rand())/static_cast<double>(RAND_MAX))-maxvely;
        }
        timecnt++;

        if(con->changeDomain==1){
            
            // ================================================ //
            // Update contact matrix and initial toe position(s)
            // ================================================ //
            if(con->des[0]==1){
                con_obj->setDesDomain({0, 1, 1, 0});}
            else{
                con_obj->setDesDomain({1, 0, 0, 1});}
            traj.domLen = domLenSec*ctrlHz;
            traj.toeInit = kin->toePos;

            // ================================================ //
            // Marc Raibert foothold selection (similar)
            // ================================================ //
            Eigen::Matrix<double, 3, 1> stepLenTemp;
            std::vector<double> KP = {0.04,0.01,0.0};
            setStepLen(0.0,0.0,0.0);
            stepLenTemp = toBody(state->comFiltered,state->R)-desVel;
            stepLenTemp(0) *= KP[0]; stepLenTemp(1) *= KP[1]; stepLenTemp(2) *= KP[2];
            stepLenTemp += (domLenSec*toBody(state->comFiltered,state->R)/2);
            toWorld(traj.stepLen,stepLenTemp,state->R);
        }
    }

    if(gait!=STAND && gait!=POSE && gait!=TAP){
        double dt = (1.0/LL_Hz);
        Eigen::Matrix<double, 3, 1> desVelWorld = toWorld(desVel,state->R);
        Eigen::Matrix<double, 3, 1> desOmegaWorld = toWorld(desOmega,state->R);

        traj.comDes.block(0,0,3,1) = state->q.block(0,0,3,1) + desVelWorld*dt;
        traj.comDes(2) = params->standHeight; // fixed standing height
        traj.comDes.block(3,0,3,1) = desVelWorld;
        // traj.comDes(8) = state->q(5) + desOmegaWorld(2)*dt;
        // traj.comDes(11) = desOmegaWorld(2); 

        for(int i=0; i<HORIZ; ++i){
            traj.redDes.block(i*NRS,0,3,1) = state->q.block(0,0,3,1) + (i+1)*desVelWorld*MPC_dt;
            traj.redDes(i*NRS+2) = params->standHeight; 
            traj.redDes.block(i*NRS+6,0,3,1) = desVelWorld;
            // traj.redDes(i*NRS+8) = state->q(5) + (i+1)*desOmegaWorld(2)*MPC_dt;
            // traj.redDes(i*NRS+11) = desOmegaWorld(2);
        }
    }else if(gait==STAND){
        for(int i=0; i<HORIZ; ++i){
            traj.redDes.block(i*NRS,0,3,1) = traj.comDes.block(0,0,3,1);
            traj.redDes(i*NRS+2) = params->standHeight; 
            traj.redDes.block(i*NRS+6,0,3,1) = traj.comDes.block(3,0,3,1);
        }
    }

}

void MotionPlanner::updateVel(Eigen::Matrix<double,3,1> &desVel, Eigen::Matrix<double, 3, 1> &desOmega, MP *params){
    float rate = 0.1;
    int fwd_sgn = (params->fwdSpeed>0) ? 1 : (params->fwdSpeed<0) ? -1 : 0;
    int lat_sgn = (params->latSpeed>0) ? 1 : (params->latSpeed<0) ? -1 : 0;
    desVel(0) += (fwd_sgn*desVel(0)<fwd_sgn*params->fwdSpeed) ? rate*fwd_sgn : 0;
    desVel(1) += (lat_sgn*desVel(1)<lat_sgn*params->latSpeed) ? rate*lat_sgn : 0;

    rate = 0.1;
    int yaw_sgn = (params->yawSpeed>0) ? 1 : (params->yawSpeed<0) ? -1 : 0;
    desOmega(2) += (yaw_sgn*desOmega(2)<yaw_sgn*params->yawSpeed) ? rate*yaw_sgn : 0;
};
