#ifndef PID_H
#define PID_H
#include <vector>

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
  
  bool Initialized = false;
  
  unsigned long step;
  unsigned short MaxStep;
  
  
  
  std::vector<double> P;
  std::vector<double> dp;
  
  double error1;
  double error2;

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
  void Init(double Kp, double Ki, double Kd, double cte);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double cte);
  
  void UpdateParam(double &P, double &dp, unsigned long &step, double cte, double &error1, double &error2,double best_err);
};

  

#endif /* PID_H */
