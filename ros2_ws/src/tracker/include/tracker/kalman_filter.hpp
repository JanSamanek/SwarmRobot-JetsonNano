#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(const Eigen::Vector3d& init_pos, double dt, 
                   const Eigen::Vector3d& init_acc = Eigen::Vector3d::Zero(), 
                   double std_acc = 3.0, double pos_std_meas = 0.02);

    Eigen::Vector3d predict(const Eigen::Vector3d& acc_input = Eigen::Vector3d::Zero());
    Eigen::Vector3d update(const Eigen::Vector3d& measured_pos);

private:
    double dt_;  
    Eigen::Vector3d u_;  
    Eigen::VectorXd x_; 

    Eigen::MatrixXd A_;  
    Eigen::MatrixXd B_;  
    Eigen::MatrixXd H_;  
    Eigen::MatrixXd Q_;  
    Eigen::MatrixXd R_;  
    Eigen::MatrixXd P_; 
};

#endif // KALMAN_FILTER_H
