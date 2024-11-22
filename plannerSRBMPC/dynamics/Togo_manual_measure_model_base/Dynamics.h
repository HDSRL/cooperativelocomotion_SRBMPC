#ifndef __DYNAMICS_H__
#define __DYNAMICS_H__

#define hzd_real  double
#define MAX(X,Y)  ((X) < (Y) ? (Y) : (X))  // Maximum of two expressions  
#define MIN(X,Y)  ((X) > (Y) ? (Y) : (X))  // Minimum of two expressions

#include "math.h"

inline double Power(double x, double y) { return pow(x, y); }
inline double Sqrt(double x) { return sqrt(x); }

inline double Abs(double x) { return fabs(x); }

inline double Exp(double x) { return exp(x); }
inline double Log(double x) { return log(x); }

inline double Sin(double x) { return sin(x); }
inline double Cos(double x) { return cos(x); }
inline double Tan(double x) { return tan(x); }

inline double ArcSin(double x) { return asin(x); }
inline double ArcCos(double x) { return acos(x); }
inline double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
inline double ArcTan(double x, double y) { return atan2(y,x); }

inline double Sinh(double x) { return sinh(x); }
inline double Cosh(double x) { return cosh(x); }
inline double Tanh(double x) { return tanh(x); }

const double E	= 2.71828182845904523536029;
const double Pi = 3.14159265358979323846264;
const double Degree = 0.01745329251994329576924;

/* Compute Mass Inertia Matrix D */
void HZD_D_mat(hzd_real *pr, const hzd_real *q);

/* Compute Corioilis Terms (Only Gravitational Included now)*/
void HZD_H_mat(hzd_real *pr, const hzd_real *q, const hzd_real *dq);

/* Compute Forward Kinematics of the toe points */
void FK_toe0(hzd_real *p_output1, const hzd_real *q);
void FK_toe1(hzd_real *p_output1, const hzd_real *q);
void FK_toe2(hzd_real *p_output1, const hzd_real *q);
void FK_toe3(hzd_real *p_output1, const hzd_real *q);

/* Compute Center of Mass position and velocity */
void COM_posFunc(hzd_real* p_output1, const hzd_real* q);
void COM_velFunc(hzd_real* p_output1, const hzd_real* q, const hzd_real* dq);

/* Compute Jacobian of the contact points*/

/*==============Toe0============================*/
void HZD_Jh_toe0(hzd_real *pr, const hzd_real *q);

/*==============Toe1============================*/
void HZD_Jh_toe1(hzd_real *pr, const hzd_real *q);

/*==============Toe2============================*/
void HZD_Jh_toe2(hzd_real *pr, const hzd_real *q);

/*==============Toe3============================*/
void HZD_Jh_toe3(hzd_real *pr, const hzd_real *q);

//void HZD_contact_jacobian(hzd_dmat *J, hzd_dmat *J0, hzd_dmat *J1, hzd_dmat *J2, hzd_dmat *J3, hzd_contact *contact);


/* Compute Input Distribution Matrix*/
void HZD_B_mat(hzd_real *p_output1, const hzd_real *q);


/* Compute Gradient of the Jacobian Matrix*/
void HZD_dJh_toe0(hzd_real *pr, const hzd_real *q, const hzd_real *dq);

void HZD_dJh_toe1(hzd_real *pr, const hzd_real *q, const hzd_real *dq);

void HZD_dJh_toe2(hzd_real *pr, const hzd_real *q, const hzd_real *dq);

void HZD_dJh_toe3(hzd_real *pr, const hzd_real *q, const hzd_real *dq);

//void HZD_contact_jacabian_grad(hzd_real *Jvdot, hzd_real *dJ0, hzd_real *dJ1, hzd_real *dJ2, hzd_real *dJ3, hzd_contact *contact);


/* Flagship Function for Obtaining Dynamics */
//void HZD_dynamics(hzd_dmat *D, hzd_real *H, hzd_dmat *B, hzd_dmat *Jv, hzd_real *Jvdot, hzd_real *q, hzd_real *qd, hzd_contact *contact);

#endif