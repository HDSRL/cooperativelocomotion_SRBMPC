#ifndef __KINOVAKINEMATICS_H__
#define __KINOVAKINEMATICS_H__

//deprecated because of fixed base position of the arm
//keep this header and cpp for this header as legacy

#include "math.h"

void COMpos_arm(double* compos, const double* q);
void COMvel_arm(double* comvel, const double* q, const double* qdot);

void FK_arm(double *output, const double* q);

void Jv_arm(double *J_v, const double* q);

void dJv_arm(double *dJ_v, const double* q, const double* qdot);

#endif