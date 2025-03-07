#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
public:
  PIDController(double p, double i, double d)
    : p_gain_(p), i_gain_(i), d_gain_(d), prev_error_(0.0), integral_(0.0)
  {}

  double compute(double error);

private:
  double p_gain_, i_gain_, d_gain_;
  double prev_error_, integral_;
};

#endif // PID_CONTROLLER_H