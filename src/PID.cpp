#include "PID.h"
#include <math.h>

using namespace std;

PID::PID() {
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {

  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
}

void PID::UpdateError(double cte) {

  // calculate diferential error
  d_error_ = cte - p_error_;  // p_error_ contains previous cte

  // Update proportional error.
  p_error_ = cte;

   // update integral error.
  i_error_ += cte;
}

double PID::TotalError() {
  return -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
}

