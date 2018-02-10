#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    // used for twiddle
    twiddle_mode = true;
    param_index = 0;
    err = 0;
    best_err = std::numeric_limits<double>::max();
    n = 100;
    step = 0; 
    reset = false; // reset simulator when we run 200 time step
    dp = {1, 1, 1};
    reduce_flag = false;

    if(twiddle_mode){
    	this->Kp = 0;
    	this->Ki = 0;
    	this->Kd = 0;   	
    }
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
   	p_error = cte;
   	i_error += cte;

   	reset = false;

   	if(twiddle_mode){
   		if(step >= n){
   			err += pow(cte, 2);
   		}

   		if(step == 2 * n){
   			cout << "step: " << step << endl;
        	cout << "err: " << err << endl;
        	cout << "best err: " << best_err << endl;

   			if(reduce_flag){
   				if(err < best_err){
   					best_err = err;
   					dp[param_index] *= 1.1;
   				}else{
   					UpdateParam(param_index, dp[param_index]);
   					dp[param_index] *= 0.9;
   				}
   				param_index = (param_index + 1) % 3;
   				reduce_flag = false;
   			}else{
	   			if(err < best_err){
	   				best_err = err;
	   				dp[param_index] *= 1.1;
	   				param_index = (param_index + 1) % 3;
	   			}else{
	   				UpdateParam(param_index, -2 * dp[param_index]);
	   				reduce_flag = true;
	   			}
   			}

   			// reset the simulator
   			err = 0;
   			reset = true;
   			step = 0;
        	cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << endl;         	
   		}


   		step++;
   	}

}

double PID::TotalError() {
	return -Kp * p_error - Kd * d_error - Ki * i_error;
}

void PID::UpdateParam(int param_index, double delta){
    if (param_index == 0) {
        Kp += delta;
    }
    else if (param_index == 1) {
        Kd += delta;
    }
    else if (param_index == 2) {
        Ki += delta;
    }
}
