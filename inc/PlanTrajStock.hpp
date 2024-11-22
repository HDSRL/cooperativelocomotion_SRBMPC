#include "global_loco_structs.hpp"
#include "MPC_global_params.hpp"
#include "Transforms.hpp"
#include "iostream"


inline void updateVel(Eigen::Matrix<double,3,1> &desVel, Eigen::Matrix<double, 3, 1> &desOmega){
    double fwdSpeed = 0.5;
    double latSpeed = 0.0;
    double yawSpeed = 0.0;
    
    float rate = 0.1;
    int fwd_sgn = (fwdSpeed>0) ? 1 : (fwdSpeed<0) ? -1 : 0;
    int lat_sgn = (latSpeed>0) ? 1 : (latSpeed<0) ? -1 : 0;
    desVel(0) += (fwd_sgn*desVel(0)<fwd_sgn*fwdSpeed) ? rate*fwd_sgn : 0;
    desVel(1) += (lat_sgn*desVel(1)<lat_sgn*latSpeed) ? rate*lat_sgn : 0;

    rate = 0.1;
    int yaw_sgn = (yawSpeed>0) ? 1 : (yawSpeed<0) ? -1 : 0;
    desOmega(2) += (yaw_sgn*desOmega(2)<yaw_sgn*yawSpeed) ? rate*yaw_sgn : 0;
}

int ConDetection(int old[4], int force[4], double phase){
    int len = 5; // CHANGE NUM ROWS IN hist AS WELL AS THIS
    int thresh = 5;
    int current[4] = {0};
    static int hist[5][4] = {{0}};
    static int ind = 0;
    
    for (int i=1; i<len; ++i){
        for (int j=0; j<4; ++j){
            hist[i][j] = hist[i-1][j];
        }
    }
    for (int j=0; j<4; ++j){
        hist[0][j] = (force[j]>thresh) ? 1 : 0;
    }

    int sum = 0;
    for (int j=0; j<4; ++j){
        for (int i=0; i<len; ++i){
            sum += hist[i][j];
        }
        current[j] = (sum==len) ? 1 : 0; 
        sum = 0;
    }
    for (int i=0; i<4; ++i){
        if (current[i]==1 && old[i]==0){
            return 1;
        }
    }
    return 0;
}


void PlanTraj(int des[4],int changeDomain,Eigen::Matrix3d R,double qa[18],double dqa[18],Eigen::MatrixXd &redDes){
	static Eigen::Matrix<double, 3, 1> desVel = {0,0,0};
    static Eigen::Matrix<double, 3, 1> desOmega = {0,0,0};
        
	static int startTrot = 0;
	if(changeDomain==1){
		if (startTrot>10){
			updateVel(desVel, desOmega);
		}
		startTrot+=1;

        // Update desired contacts
		if(des[0]==1){
			des[0] = 0; des[1] = 1; des[2] = 1; des[3] = 0;}
		else{
			des[0] = 1; des[1] = 0; des[2] = 0; des[3] = 1;}

	}
    
    Eigen::Matrix<double, 3, 1> desVelWorld = toWorld(desVel,R);
    Eigen::Map<Eigen::Matrix<double,18,1>> q(qa,18,1);
    double height = 0.26;

	for(int i=0; i<HORIZ; ++i){
		redDes.block(i*NRS,0,3,1) = q.block(0,0,3,1) + (i+1)*desVelWorld*MPC_dt;
		redDes(i*NRS+2) = height; 
		redDes.block(i*NRS+6,0,3,1) = desVelWorld;
	}
}
