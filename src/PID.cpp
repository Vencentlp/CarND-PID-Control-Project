#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double cte) 
{
    P = {Kp, Ki,Kd};
    dp = {1, 1, 1};
    p_error = cte;
    Initialized = true;
    step = 0;
    error1 = 0;
    error2 = 0;
}

void PID::UpdateError(double cte) 
{
    
    d_error = cte - p_error;
    p_error = cte;
    i_error = i_error + cte; 
    
    double tol = 0.00001;    
    double best_err = 1000.0;
    
    if (dp[0] > tol)
    {
        UpdateParam(P[0], dp[0], step, cte, error1, error2, best_err);
    }
    else if(dp[0] > tol)
    {
        UpdateParam(P[1], dp[1], step, cte, error1, error2, best_err);
    }
    else if(dp[2] > tol)
    {
        UpdateParam(P[2], dp[2], step, cte, error1, error2, best_err);
    }
    
    step++;
    step = step % (4 * MaxStep);
     
   
    
    
    
}


void PID::UpdateParam(double &P, double &dp, unsigned long &step, double cte, double &error1, double &error2,double best_err)
{
    P += dp;
            
    if ((step >= (MaxStep-1)) & (step <= (2*MaxStep-1)))
    {
        error1 += cte*cte;        
    }
        
    if (step >= (2*MaxStep-1))
    {
        if (error1/MaxStep < best_err)
        {
            best_err = error1;
            dp *= 1.1;
            step = 0;
            error1 =0;
        }
        else
        {
            P -= 2*dp;
            if ((step >= (3*MaxStep -1)) & (step <= (4*MaxStep-1)))
            {
                 error2 += cte*cte;
            }
            if (step >= (4*MaxStep-1))
            {
                if (error2/MaxStep < best_err)
                {
                    best_err = error2;
                    dp *=1.1;
                }
                else
                {
                    P += dp;
                    dp *= 0.9;
                }
                error1 = 0;
                error2 = 0;
            }
            
        }
    }
    else
    {
        
        
        if ((step >= 3*MaxStep) & (step <= 4*MaxStep))
        {
            error2 += cte*cte;
        }
        if (step == 4*MaxStep & error2 < best_err)
        {
            best_err = error2;
            dp *=1.1;
            error2 = 0;
        }
        else
        {
            P += dp;
            dp *= 0.9;
        }
    }
}


double PID::TotalError(double cte) 
{
    double pid_error = -P[0]*p_error - P[1]*i_error - P[2]*d_error;

    
    

    
    return pid_error;
}

