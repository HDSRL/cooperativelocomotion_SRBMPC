#ifndef __KINOVADYNAMICSGENERAL_H__
#define __KINOVADYNAMICSGENERAL_H__

#include "math.h"
#include "cstring"


void armLRD_mtx(double* Inertiamtx, const double* q, const double* xyzrpy); //too big matlab to deal with -> divide and conquer
void JvcomB(double* JvcomB, const double* in1, const double* in3);
void JvcomL1(double* JvcomL1, const double* in1, const double* in3);
void JvcomL2(double* JvcomL2, const double* in1, const double* in3);
void JvcomL3(double* JvcomL3, const double* in1, const double* in3);
void JvcomL4(double* JvcomL4, const double* in1, const double* in3);
void JwcomB(double* JwcomB, const double* in1, const double* in3);
void JwcomL1(double* JwcomL1, const double* in1, const double* in3);
void JwcomL2(double* JwcomL2, const double* in1, const double* in3);
void JwcomL3(double* JwcomL3, const double* in1, const double* in3);
void JwcomL4(double* JwcomL4, const double* in1, const double* in3);


//void armLRH_mtx(double* hmtx, const double* q, const double* qdot, const double* xyzrpy);
void armLRH_mtx(double* b_hmtx, const double* in1, const double* in2, const double* in3);

#endif