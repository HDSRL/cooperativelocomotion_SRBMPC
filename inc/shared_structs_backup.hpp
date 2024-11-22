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

struct sharedData
{
	// Provided by LL
	double domLen = 200;
	double phaseVar = 0;
	int ind[4] = {1};
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

sharedData data[2];

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

void updateData_multi(int setget, int highlow, int agnum, sharedData *newData){
	// set=1,  get=0
	// high=1, low=0
	boost::lock_guard<boost::mutex> guard(mtx);
	int reset_old = data[agnum].cnt_reset;
	int reset_new = newData->cnt_reset;
	if (setget==SET_DATA){
		if (highlow==HL_DATA){ // set high level data
			data[agnum].fDes = newData->fDes;
			data[agnum].comDes = newData->comDes;
			data[agnum].cnt_reset = newData->cnt_reset;
		}else{ // set low level data
			Eigen::Matrix<double, 12, HORIZ> fDes_temp = data[agnum].fDes;
			Eigen::Matrix<double, 12, HORIZ> comDes_temp = data[agnum].comDes;
			memcpy(&data[agnum],newData,sizeof(sharedData));
			data[agnum].fDes = fDes_temp;
			data[agnum].comDes = comDes_temp;
			if (reset_old==-1 && reset_new==-2){
				data[agnum].cnt_reset = 0;
			}else if(reset_old==-1 && reset_new!=-2){
				data[agnum].cnt_reset = reset_old;
			}
		}
	}else {
		if (highlow==HL_DATA){ // get data for high level
			Eigen::Matrix<double, 12, HORIZ> fDes_temp = newData->fDes;
			Eigen::Matrix<double, 12, HORIZ> comDes_temp = newData->comDes;
			memcpy(newData,&data[agnum],sizeof(sharedData));
			newData->fDes = fDes_temp;
			newData->comDes = comDes_temp;
		}else{ // get data for low level
			newData->fDes = data[agnum].fDes;
			newData->comDes = data[agnum].comDes;
			newData->cnt_reset = data[agnum].cnt_reset;
		}
	}
};


#endif
