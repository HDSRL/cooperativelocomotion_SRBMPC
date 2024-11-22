#ifndef SHARED_DATA
#define SHARED_DATA

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"
#include "global_loco_opts.h"
#include "MPC_global_params.hpp"

#include "mutex"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#define SET_DATA 1
#define GET_DATA 0
#define HL_DATA 1
#define LL_DATA 0
boost::mutex mtx;
boost::mutex mtx_HL;
boost::mutex mtx_LL;

struct sharedData
{
	// Provided by LL
	double domLen = 200;
	double phaseVar = 0;
	int ind[4] = {1,1,1,1};
	double q[18] = {0};
	double dq[18] = {0};
	Eigen::Matrix<double, NRS*HORIZ, 1> redDes = Eigen::Matrix<double, NRS*HORIZ, 1>::Zero(); 
	Eigen::Matrix<double, 12, 1> QP_force = Eigen::Matrix<double, 12, 1>::Zero();
	Eigen::Matrix<double, 12, HORIZ> comHist = Eigen::Matrix<double, 12, HORIZ>::Zero();
	int MPC_cnt = 0; 

	// Provided by HL
	Eigen::Matrix<double,  12, HORIZ> comDes = Eigen::Matrix<double, 12, HORIZ>::Zero();
	Eigen::Matrix<double,  12, HORIZ> fDes   = Eigen::Matrix<double, 12, HORIZ>::Zero();
	int cnt_reset = 0;
};

//sharedData data[2];

struct sharedDataMulti
{
	// Provided by LL
	double domLen = 200;
	double phaseVar = 0;
	int ind[4] = {1,1,1,1};
	double q[18] = {0};
	double dq[18] = {0};
	Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
	//Eigen::Matrix<double, NRS*HORIZ, 1> redDes = Eigen::Matrix<double, NRS*HORIZ, 1>::Zero(); 
	//Eigen::Matrix<double, 12, 1> QP_force = Eigen::Matrix<double, 12, 1>::Zero();
	//Eigen::Matrix<double, 12, HORIZ> comHist = Eigen::Matrix<double, 12, HORIZ>::Zero();
	int MPC_cnt = 0; 

	// Provided by HL
	Eigen::Matrix<double,  12, 1> comDes = Eigen::Matrix<double, 12, 1>::Zero();
	Eigen::Matrix<double,  12, 1> fDes   = Eigen::Matrix<double, 12, 1>::Zero();
	int cnt_reset = 0;

	int ctrlTickreal = 0;

	// ================================================= //
	// ================================================= //
	// ================================================= //

	// Provided by LL - opposite
	double domLen_opposite = 200;
	double phaseVar_opposite = 0;
	int ind_opposite[4] = {1,1,1,1};
	double q_opposite[18] = {0};
	double dq_opposite[18] = {0};
	Eigen::Matrix3d R0_opposite = Eigen::Matrix3d::Identity();
	//Eigen::Matrix<double, NRS*HORIZ, 1> redDes_opposite = Eigen::Matrix<double, NRS*HORIZ, 1>::Zero(); 
	//Eigen::Matrix<double, 12, 1> QP_force_opposite = Eigen::Matrix<double, 12, 1>::Zero();
	//Eigen::Matrix<double, 12, HORIZ> comHist_opposite = Eigen::Matrix<double, 12, HORIZ>::Zero();
	int MPC_cnt_opposite = 0; 

	// Provided by HL - opposite
	Eigen::Matrix<double,  12, 1> comDes_opposite = Eigen::Matrix<double, 12, 1>::Zero();
	Eigen::Matrix<double,  12, 1> fDes_opposite   = Eigen::Matrix<double, 12, 1>::Zero();
	int cnt_reset_opposite = 0;

	int ctrlTickreal_opposite = 0;
};

sharedDataMulti data[2], HLdata[2], LLdata[2];


void updateData(int setget, int highlow, sharedData *newData){
	// // set=1,  get=0
	// // high=1, low=0
	// boost::lock_guard<boost::mutex> guard(mtx);
	// int reset_old = data.cnt_reset;
	// int reset_new = newData->cnt_reset;
	// if (setget==SET_DATA){
	// 	if (highlow==HL_DATA){ // set high level data
	// 		data.fDes = newData->fDes;
	// 		data.comDes = newData->comDes;
	// 		data.cnt_reset = newData->cnt_reset;
	// 	}else{ // set low level data
	// 		Eigen::Matrix<double, 12, HORIZ> fDes_temp = data.fDes;
	// 		Eigen::Matrix<double, 12, HORIZ> comDes_temp = data.comDes;
	// 		memcpy(&data,newData,sizeof(sharedData));
	// 		data.fDes = fDes_temp;
	// 		data.comDes = comDes_temp;
	// 		if (reset_old==-1 && reset_new==-2){
	// 			data.cnt_reset = 0;
	// 		}else if(reset_old==-1 && reset_new!=-2){
	// 			data.cnt_reset = reset_old;
	// 		}
	// 	}
	// }else {
	// 	if (highlow==HL_DATA){ // get data for high level
	// 		Eigen::Matrix<double, 12, HORIZ> fDes_temp = newData->fDes;
	// 		Eigen::Matrix<double, 12, HORIZ> comDes_temp = newData->comDes;
	// 		memcpy(newData,&data,sizeof(sharedData));
	// 		newData->fDes = fDes_temp;
	// 		newData->comDes = comDes_temp;
	// 	}else{ // get data for low level
	// 		newData->fDes = data.fDes;
	// 		newData->comDes = data.comDes;
	// 		newData->cnt_reset = data.cnt_reset;
	// 	}
	// }
};

// void updateData_multi(int setget, int highlow, int agnum, sharedDataMulti *newData){
// 	// set=1,  get=0
// 	// high=1, low=0
// 	//boost::lock_guard<boost::mutex> guard(mtx);
// 	int reset_old = data[agnum].cnt_reset;
// 	int reset_new = newData->cnt_reset;
// 	if(setget==SET_DATA){
// 		if(highlow==HL_DATA){ // set high level data
// 			boost::lock_guard<boost::mutex> guard(mtx_HL);
// 			data[agnum].fDes = newData->fDes;
// 			data[agnum].comDes = newData->comDes;
// 			data[agnum].cnt_reset = newData->cnt_reset;
// 			data[1-agnum].fDes_opposite = newData->fDes;
// 			data[1-agnum].comDes_opposite = newData->comDes;
// 			data[1-agnum].cnt_reset_opposite = newData->cnt_reset;
// 		}
// 		else if(highlow==LL_DATA){ // set low level data
// 			// Eigen::Matrix<double, 12, 1> fDes_temp = data[agnum].fDes;
// 			// Eigen::Matrix<double, 12, 1> comDes_temp = data[agnum].comDes;
// 			// memcpy(&data[agnum],newData,sizeof(sharedDataMulti));
// 			// data[agnum].fDes = fDes_temp;
// 			// data[agnum].comDes = comDes_temp;

// 			//data[agnum].q = newData->q;
// 			//data[agnum].dq = newData->dq;
// 			//data[agnum].ind = newData->ind;
// 			boost::lock_guard<boost::mutex> guard(mtx_LL);
// 			memcpy(data[agnum].q, newData->q, 18*sizeof(double));
// 			memcpy(data[agnum].dq, newData->dq, 18*sizeof(double));
// 			memcpy(data[agnum].ind, newData->ind, 4*sizeof(double));
// 			data[agnum].R0 = newData->R0;
// 			data[agnum].phaseVar = newData->phaseVar;
// 			data[agnum].domLen = newData->domLen;

// 			//data[1-agnum].q_opposite = newData->q;
// 			//data[1-agnum].dq_opposite = newData->dq;
// 			//data[1-agnum].ind_opposite = newData->ind;
// 			memcpy(data[1-agnum].q_opposite, newData->q, 18*sizeof(double));
// 			memcpy(data[1-agnum].dq_opposite, newData->dq, 18*sizeof(double));
// 			memcpy(data[1-agnum].ind_opposite, newData->ind, 4*sizeof(double));
// 			data[1-agnum].R0_opposite = newData->R0;
// 			data[1-agnum].phaseVar_opposite = newData->phaseVar;
// 			data[1-agnum].domLen_opposite = newData->domLen;

// 			if (reset_old==-1 && reset_new==-2){
// 				data[agnum].cnt_reset = 0;
// 			}
// 			else if(reset_old==-1 && reset_new!=-2){
// 				data[agnum].cnt_reset = reset_old;
// 			}
// 		}
// 	}
// 	else if(setget==GET_DATA){
// 		if(highlow==HL_DATA){ // get data for high level
// 			boost::lock_guard<boost::mutex> guard(mtx_LL);
// 			Eigen::Matrix<double, 12, 1> fDes_temp = newData->fDes;
// 			Eigen::Matrix<double, 12, 1> comDes_temp = newData->comDes;
// 			memcpy(newData,&data[agnum],sizeof(sharedDataMulti));
// 			newData->fDes = fDes_temp;
// 			newData->comDes = comDes_temp;
// 		}
// 		else if(highlow==LL_DATA){ // get data for low level
// 			boost::lock_guard<boost::mutex> guard(mtx_HL);
// 			newData->fDes = data[agnum].fDes;
// 			newData->comDes = data[agnum].comDes;
// 			newData->cnt_reset = data[agnum].cnt_reset;
// 		}
// 	}
// };

void updateData_multi(int setget, int highlow, int agnum, sharedDataMulti *newData){
	// set=1,  get=0
	// high=1, low=0
	//boost::lock_guard<boost::mutex> guard(mtx);
	//int reset_old = HLdata[agnum].cnt_reset;
	//int reset_new = newData->cnt_reset;
	if(setget==SET_DATA){
		if(highlow==HL_DATA){ // set high level data
			boost::lock_guard<boost::mutex> guard(mtx_HL);
			HLdata[agnum].fDes = newData->fDes;
			HLdata[agnum].comDes = newData->comDes;
			HLdata[agnum].cnt_reset = newData->cnt_reset;
			HLdata[1-agnum].fDes_opposite = newData->fDes;
			HLdata[1-agnum].comDes_opposite = newData->comDes;
			HLdata[1-agnum].cnt_reset_opposite = newData->cnt_reset;
		}
		else if(highlow==LL_DATA){ // set low level data
			// Eigen::Matrix<double, 12, 1> fDes_temp = data[agnum].fDes;
			// Eigen::Matrix<double, 12, 1> comDes_temp = data[agnum].comDes;
			// memcpy(&data[agnum],newData,sizeof(sharedDataMulti));
			// data[agnum].fDes = fDes_temp;
			// data[agnum].comDes = comDes_temp;

			//data[agnum].q = newData->q;
			//data[agnum].dq = newData->dq;
			//data[agnum].ind = newData->ind;
			boost::lock_guard<boost::mutex> guard(mtx_LL);
			memcpy(LLdata[agnum].q, newData->q, 18*sizeof(double));
			memcpy(LLdata[agnum].dq, newData->dq, 18*sizeof(double));
			memcpy(LLdata[agnum].ind, newData->ind, 4*sizeof(double));
			LLdata[agnum].R0 = newData->R0;
			LLdata[agnum].phaseVar = newData->phaseVar;
			LLdata[agnum].domLen = newData->domLen;

			//data[1-agnum].q_opposite = newData->q;
			//data[1-agnum].dq_opposite = newData->dq;
			//data[1-agnum].ind_opposite = newData->ind;
			memcpy(LLdata[1-agnum].q_opposite, newData->q, 18*sizeof(double));
			memcpy(LLdata[1-agnum].dq_opposite, newData->dq, 18*sizeof(double));
			memcpy(LLdata[1-agnum].ind_opposite, newData->ind, 4*sizeof(double));
			LLdata[1-agnum].R0_opposite = newData->R0;
			LLdata[1-agnum].phaseVar_opposite = newData->phaseVar;
			LLdata[1-agnum].domLen_opposite = newData->domLen;
		}
	}
	else if(setget==GET_DATA){
		if(highlow==HL_DATA){ // get data for high level
			boost::lock_guard<boost::mutex> guard(mtx_LL);
			Eigen::Matrix<double, 12, 1> fDes_temp = newData->fDes;
			Eigen::Matrix<double, 12, 1> comDes_temp = newData->comDes;
			memcpy(newData,&LLdata[agnum],sizeof(sharedDataMulti));
			newData->fDes = fDes_temp;
			newData->comDes = comDes_temp;
		}
		else if(highlow==LL_DATA){ // get data for low level
			boost::lock_guard<boost::mutex> guard(mtx_HL);
			newData->fDes = HLdata[agnum].fDes;
			newData->comDes = HLdata[agnum].comDes;
			newData->cnt_reset = HLdata[agnum].cnt_reset;
		}
	}
};



#endif
