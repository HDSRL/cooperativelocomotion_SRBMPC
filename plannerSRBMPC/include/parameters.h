#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "iostream"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Sparse"
#include "eigen3/unsupported/Eigen/MatrixFunctions"

// =============================================================================================== //
// ============================================= quad model ====================================== //
// =============================================================================================== //

#define QUAD_DOF 18 //6(free body) + 12(3dof for each leg)
#define QUAD_CTRL 12 // 3(x,y,z) * 4(legs)

#define ARM_STATUS 0
#define DUAL_ARM 0

#if ARM_STATUS == 1
    #if DUAL_ARM ==0
        ////////////////////////////// Use this part to turn on the armx1 ////////////////////////
        #define ARM_DOF 4 //if arm is on then: 4 || if not: 0  //4(link dof) + 6(2dof for each finger)
        #define FINGER_DOF 6 // if arm is on then:6 || if not: 0
        #define ARM_CTRL 3 //if arm is on then: 3 || if not: 0 // 3(x,y,z)
        ////////////////////////////////////////////////////////////////////////////////////////////////
    #endif
    #if DUAL_ARM == 1
        ////////////////////////////// Use this part to turn on the armx2 ////////////////////////
        #define ARM_DOF 8 //if arm is on then: 4x2 || if not: 0  //4(link dof) + 6(2dof for each finger)
        #define FINGER_DOF 12 // if arm is on then:6x2 || if not: 0
        #define ARM_CTRL 6 //if arm is on then: 3x2 || if not: 0 // 3(x,y,z)
        ////////////////////////////////////////////////////////////////////////////////////////////////
    #endif
#else
    ////////////////////////////// Use this part to turn off the arm ////////////////////////
    #define ARM_DOF 0 //if arm is on then: 4 || if not: 0  //4(link dof) + 6(2dof for each finger)
    #define FINGER_DOF 0 // if arm is on then:6 || if not: 0
    #define ARM_CTRL 0 //if arm is on then: 3 || if not: 0 // 3(x,y,z)
    ////////////////////////////////////////////////////////////////////////////////////////////////
#endif





// ========================================================================================================== //
// ============================================= low-lvl + LIP planner ====================================== //
// ========================================================================================================== //
/////////////////////////////////////////////////////////////
// define variables
#define TOTALSTEPNUM    50                  //changable// variable M in matlab
#define NDOMAIN         4                   //agent sharing//constant // numer of grids per domain
#define TSOPTTICK       80                  //agent sharing//changable// grid time lengh: 40(ms)

#define EVENTBASE       1                   //agent sharing//changable// event base MPC on(1) / off(0)
#define SUPERVISED      0                   //no needs to change//
#define DISTRIBUTED     0                   //no needs to change//

#define STEPLENGTHX     0.1                 //no needs to change//
#define STEPLENGTHY     0.0                 //no needs to change//
#define STEPHEIGHT      0.15//0.20//0.12    //agent sharing//changable 0.2 looks better?

#define BODYROLL        0.0                 //no needs to change//
#define BODYPITCH       0.0                 //no needs to change//
#define BODYYAW         0.0                 //no needs to change//
//#define COMROWNUM       TOTALSTEPNUM * 4    //constant// totalstepnumber * 4
//#define FOOTTRAJCOLNUM  TOTALSTEPNUM - 2    //agent sharing//constant// totalstepnumber -2

// offline MPC: M*Ndomain
// online MPC for single agent: 2*Ndomain
// online MPC for double agent: 1*Ndomain
#if EVENTBASE == 0
    #define CONTROLHORIZON TOTALSTEPNUM*NDOMAIN
#else
    #if SUPERVISED == 1
        #define CONTROLHORIZON 1*NDOMAIN
    #endif
    #if SUPERVISED == 0
        #define CONTROLHORIZON 2*NDOMAIN
    #endif
#endif
/////////////////////////////////////////////////////////////

#define simfreq_raisim 0.001        // maybe constant
#define simfreq 0.001               //maybe changable
#define ctrlHz  1000                //depends on simfreq

#define STAND   0
#define WALK    1
#define AMBLE   2
#define TROT    3
#define GALLOP  4


#endif