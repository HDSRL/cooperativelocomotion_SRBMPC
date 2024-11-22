#ifndef SRBCOOPMODEL_H
#define SRBCOOPMODEL_H

#include "math.h"
#include "math_define.h"
#include "cstring"

void system_mtx_ftn(double* systemdynamics,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_sys_state(double* jaco_sys_state_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_sys_input(double* jaco_sys_input_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_sys_lambda(double* jaco_sys_lambda_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);


void jaco_consp_state(double* jaco_consp_state_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_consp_input(double* jaco_consp_input_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_consp_lambda(double* jaco_consp_lambda_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);


void jaco_consv_state(double* jaco_consv_state_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_consv_input(double* jaco_consv_input_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_consv_lambda(double* jaco_consv_lambda_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);


void jaco_cons_state(double* jaco_cons_state_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_cons_input(double* jaco_cons_input_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);

void jaco_cons_lambda(double* jaco_cons_lambda_output,
                    double* state, double* input, double lambda, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);  
                    

void lambda_op_closed_form(double* lambda_op_closed_form_output,
                    double* state, double* input, double* state_op, double* input_op, 
                    double* contactfoot_pose, double* fstate1, double* fstate2);  



void system_mtx_ftn(Eigen::MatrixXd& systemdynamics_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);


void jaco_sys_state(Eigen::MatrixXd& jaco_sys_state_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op,
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

void jaco_sys_input(Eigen::MatrixXd& jaco_sys_input_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

void jaco_sys_lambda(Eigen::MatrixXd& jaco_sys_lambda_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);


void jaco_consp_state(Eigen::MatrixXd& jaco_consp_state_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

void jaco_consp_input(Eigen::MatrixXd& jaco_consp_input_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

void jaco_consp_lambda(Eigen::MatrixXd& jaco_consp_lambda_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);


void jaco_consv_state(Eigen::MatrixXd& jaco_consv_state_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

void jaco_consv_input(Eigen::MatrixXd& jaco_consv_input_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

void jaco_consv_lambda(Eigen::MatrixXd& jaco_consv_lambda_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);


void jaco_cons_state(Eigen::MatrixXd& jaco_cons_state_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

void jaco_cons_input(Eigen::MatrixXd& jaco_cons_input_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

void jaco_cons_lambda(Eigen::MatrixXd& jaco_cons_lambda_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, double lambda, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2); 


void lambda_op_closed_form(Eigen::MatrixXd& lambda_op_closed_form_output_e,
                    Eigen::MatrixXd state, Eigen::MatrixXd input, Eigen::MatrixXd state_op, Eigen::MatrixXd input_op, 
                    Eigen::MatrixXd contactfoot_pose, Eigen::MatrixXd fstate1, Eigen::MatrixXd fstate2);

#endif