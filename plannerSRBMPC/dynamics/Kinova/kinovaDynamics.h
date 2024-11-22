#ifndef __KINOVADYNAMICS_H__
#define __KINOVADYNAMICS_H__

//deprecated because of fixed base position of the arm
//keep this header and cpp for this header as legacy

#include "math.h"


void armD_mtx(double* Inertiamtx, const double* q);

void armH_mtx(double* hmtx, const double* q, const double* qdot);

#endif