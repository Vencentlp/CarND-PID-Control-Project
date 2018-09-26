#ifndef PID_H
#define PID_H
#include <vector>
#include <iostream>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  bool Initialized;
  
  unsigned long step;
  unsigned short MaxSettleStep;
  unsigned short MaxEvalStep;
  bool twiddle;
  double sum_error;
  
  
  
  std::vector<double> P;
  std::vector<double> dp;
  
  double error1;
  double error2;
  double best_err;
  
  int ChangeCoee;
  
  //double error1;
  //double error2;

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
  * Calculate the total PID error.
  */
  double TotalError();
  
  void UpdateParam(double &P, double &dp, unsigned long &step, double cte);
};

  

#endif /* PID_H */
