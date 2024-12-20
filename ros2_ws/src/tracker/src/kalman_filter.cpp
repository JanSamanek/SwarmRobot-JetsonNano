#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter(const Eigen::Vector3d& init_pos, double dt, 
                               const Eigen::Vector3d& init_acc, double std_acc, 
                               double pos_std_meas)
    : dt_(dt), u_(init_acc), x_(6) 
    {

    x_ << init_pos(0), init_pos(1), init_pos(2), 0.0, 0.0, 0.0;

    // State Transition Matrix A
    A_ = Eigen::MatrixXd::Identity(6, 6);
    A_(0, 3) = dt_;
    A_(1, 4) = dt_;
    A_(2, 5) = dt_;

    // Control Input Matrix B
    B_ = Eigen::MatrixXd::Zero(6, 3);
    B_(0, 0) = 0.5 * dt_ * dt_;
    B_(1, 1) = 0.5 * dt_ * dt_;
    B_(2, 2) = 0.5 * dt_ * dt_;
    B_(3, 0) = dt_;
    B_(4, 1) = dt_;
    B_(5, 2) = dt_;

    // Measurement Mapping Matrix H
    H_ = Eigen::MatrixXd::Zero(3, 6);
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    H_(2, 2) = 1.0;

    // Process Noise Covariance Q
    Q_ = Eigen::MatrixXd::Zero(6, 6);
    Q_(0, 0) = Q_(1, 1) = Q_(2, 2) = (dt_ * dt_ * dt_ * dt_) / 4.0;
    Q_(0, 3) = Q_(1, 4) = Q_(2, 5) = (dt_ * dt_ * dt_) / 2.0;
    Q_(3, 3) = Q_(4, 4) = Q_(5, 5) = (dt_ * dt_);
    Q_ *= std_acc * std_acc;

    // Measurement Noise Covariance R
    R_ = Eigen::MatrixXd::Identity(3, 3) * pos_std_meas * pos_std_meas;

    // Initial Covariance Matrix P
    P_ = Eigen::MatrixXd::Identity(6, 6);
}

Eigen::Vector3d KalmanFilter::predict(const Eigen::Vector3d& acc_input) {
    u_ = acc_input;

    x_ = A_ * x_ + B_ * u_;
    P_ = A_ * P_ * A_.transpose() + Q_;

    return x_.head<3>();  
}

Eigen::Vector3d KalmanFilter::update(const Eigen::Vector3d& measured_pos) {
    
    Eigen::Vector3d z = measured_pos;

    Eigen::Vector3d y = z - H_ * x_;
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = P_ - K * H_ * P_;

    return x_.head<3>(); 
}
