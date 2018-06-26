#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  // current cte
  double p_error_;

  // sum of all previous ctes
  double i_error_;

  // difference between previous cte and current one
  double d_error_;


  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Returns the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
