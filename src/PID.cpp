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

    // ref https://discussions.udacity.com/t/how-to-twiddle-in-pid-controller-project/246971/3
    // used for twiddle
    twiddle_mode = false;
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

   	// ref https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/1397890f-71c5-4d83-aae7-0a031eedd1f2/concepts/87d07dd3-70f3-46a0-b35c-c8528792d90f
   	if(twiddle_mode){

   		if(step == 0 && !reduce_flag){
   			UpdateParam(param_index, dp[param_index]);
   		}

   		if(step >= n){
   			err += pow(cte, 2);
   		}

   		if(step == (2 * n - 1)){
   			cout << "################################"<< endl;
	        cout << "step: " << step << endl;
	        cout << "err: " << err << endl;
	        cout << "best err: " << best_err << endl;

	        cout << "Before Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << endl;  
	        cout << "Before dp:" << dp[0] <<","<<dp[1]<<","<<dp[2] << endl;

   			if(reduce_flag){
   				if(err < best_err){
   					best_err = err;
   					dp[param_index] *= 1.1;
   					cout << "op1"  << endl;
   				}else{
   					UpdateParam(param_index, dp[param_index]);
   					dp[param_index] *= 0.9;
   					cout << "op2"  << endl;
   				}
   				param_index = (param_index + 1) % 3;
   				reduce_flag = false;
   			}else{
	   			if(err < best_err){
	   				best_err = err;
	   				dp[param_index] *= 1.1;
	   				param_index = (param_index + 1) % 3;
	   				cout << "op3"  << endl;
	   			}else{
	   				UpdateParam(param_index, -2 * dp[param_index]);
	   				reduce_flag = true;
	   				cout << "op4"  << endl;
	   			}
   			}

	        cout << "After Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << endl;  
	        cout << "After dp:" << dp[0] <<","<<dp[1]<<","<<dp[2] << endl;

   			// reset the simulator
   			err = 0;
   			reset = true;
   			step = -1;       	
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
