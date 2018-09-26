#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
    P = {Kp, Ki, Kd};
    dp = {1.90574e-6, 5.17606e-10, 3.45089e-5};
    //p_error = cte;
    Initialized = false;
    step = 0;
    //error1 = 0;
    //error2 = 0;
    MaxSettleStep = 100;
    MaxEvalStep = 2000;
    i_error = 0;
    error1 = 0;
    error2 = 0;
    best_err = 10;
    twiddle = false;
    ChangeCoee = 0;
    sum_error = 0;
    
}

void PID::UpdateError(double cte) 
{
    
    if (!Initialized)
    {
        p_error = cte;
        Initialized = true;
    }
    d_error = cte - p_error;
    p_error = cte;
    i_error = i_error + cte; 
    
    double tol = 0.00001;    
    //double best_err = 1000.0;
    
    
    if (twiddle)
    {
        step++;
        if (dp[0] +dp[1] +dp[2] > tol)
        {
            if (ChangeCoee == 0 )
            {UpdateParam(P[0], dp[0], step, cte);}
            if(ChangeCoee == 1)
            {UpdateParam(P[1], dp[1], step, cte);}
            if(ChangeCoee == 2)
            {UpdateParam(P[2], dp[2], step, cte);}
            
        }
        
        if (step == 0)
        {
            ChangeCoee ++;
            ChangeCoee = ChangeCoee % 3;
        }
        
        
        
        step = step % (2* (MaxEvalStep + MaxSettleStep));
        
        
         
        std::cout<<"P "<<P[0]<<" I "<<P[1]<<" D "<<P[2]<<std::endl;
        std::cout<<"dP "<<dp[0]<<" dPI "<<dp[1]<<" dPD "<<dp[2]<<std::endl;
    }
    
    
}


void PID::UpdateParam(double &P, double &dp, unsigned long &step, double cte)
{
    
      
    
    
    unsigned long stepchek1 = MaxSettleStep;
    unsigned long stepchek2 = MaxSettleStep + MaxEvalStep;
    unsigned long stepchek3 = MaxSettleStep*2 + MaxEvalStep;
    unsigned long stepchek4 = MaxSettleStep*2 + MaxEvalStep*2;
    std::cout<< " stepceck1 "<<stepchek1<< " stepchek2 "<< stepchek2<<" stepcheck3 "<<stepchek3<< " stepchek4 " << stepchek4<<std::endl;
    
    if (step == 1) {P += dp; std::cout<<"First Add"<<std::endl;}
    if ((step > stepchek1) & (step <= stepchek2))
    {
        
        sum_error += cte*cte; 
        error1 =sum_error/(step - stepchek1);
       // error1 = error1/(step-stepchek1);
        //std::cout<<" error1 :step"<<step<<std::endl;
        
    }
    
   if(step == stepchek2){sum_error = 0;}
    
    
    //best_err = error1/MaxEvalStep;
    

    if (error1 < best_err & step == stepchek2)
    {
        dp *= 1.1;
        best_err = error1;
        step = 0;
        error1 =0; 
    
    }
        
    else
    {
        if (step == stepchek2) {P -= 2*dp;} 
            
        if ((step > stepchek3) & (step <= stepchek4))
        {
                sum_error += cte*cte;
                error2 =sum_error/(step - stepchek3);
                //error2 = error2/(step - stepchek3);
                //std::cout<<" error2 :step"<<step - stepchek3<<std::endl;
            }
            
        
                
        if (step == stepchek4)
        {
            if (error2 < best_err)
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
            step = 0;
            sum_error = 0;
        }
                
    }
    std::cout<<" Best error  "<<best_err<<"  error1  "<<error1<<"  error2  "<<error2<<std::endl;
            
}
    
    



double PID::TotalError() 
{
    double pid_error = -P[0]*p_error - P[1]*i_error - P[2]*d_error;

    
    

    
    return pid_error;
}

