#ifndef MPC_GLOBAL_PARAMS
#define MPC_GLOBAL_PARAMS


#ifndef DFLOAT
    using osqp_mat = Eigen::MatrixXd;
    using osqp_arr = Eigen::ArrayXd;
#else
    using osqp_mat = Eigen::MatrixXf;
    using osqp_arr = Eigen::ArrayXf;
#endif

#define NRS 12
#define NRI 12
#define HORIZ 5

#define MPC_Hz 200
#define MY_INF 1e30
#define MPC_dt (double)1.0/MPC_Hz
#define RAND 8

#define MASS 12.45
#define GRAV 9.81


#endif