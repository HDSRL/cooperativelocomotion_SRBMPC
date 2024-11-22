#ifndef __KINOVAKINEMATICS_GENERAL_H__
#define __KINOVAKINEMATICS_GENERAL_H__

#include "math.h"

//void R_I_B(double* R_IB, const double* q, const double* xyzrpy);
void R_I_B(double* R_IB, const double* in1, const double* in2);
//void R_I_L1(double* R_IL1, const double* q, const double* xyzrpy);
void R_I_L1(double* R_IL1, const double* in1, const double* in2);
//void R_I_L2(double* R_IL2, const double* q, const double* xyzrpy);
void R_I_L2(double* R_IL2, const double* in1, const double* in2);
//void R_I_L3(double* R_IL3, const double* q, const double* xyzrpy);
void R_I_L3(double* R_IL3, const double* in1, const double* in2);
//void R_I_L4(double* R_IL4, const double* q, const double* xyzrpy);
void R_I_L4(double* R_IL4, const double* in1, const double* in2);


//void COMpos_armLR(double* compos, const double* q, const double* xyzrpy);
void COMpos_armLR(double* b_compos, const double* in1, const double* in3);
//void COMvel_armLR(double* comvel, const double* q, const double* qdot, const double* xyzrpy);
void COMvel_armLR(double* b_comvel, const double* in1, const double* in2, const double* in3);

//void FK_armLR(double *output, const double* q, const double* xyzrpy);
void FK_armLR(double *output, const double* in1, const double* in3);

//void Jv_armLR(double *J_v, const double* q, const double* xyzrpy);
void Jv_armLR(double *J_v, const double* in1, const double* in3);

//void dJv_armLR(double *dJ_v, const double* q, const double* qdot, const double* xyzrpy);
void dJv_armLR(double *dJ_v, const double* in1, const double* in2, const double* in3);


#endif