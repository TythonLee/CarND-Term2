#include "PID.h"
#include <iostream>
#include <cmath>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	
	PID::p_error = 0.0;
	PID::i_error = 0.0;
	PID::d_error = 0.0;
	PID::total_square_error = 0.0;
	PID::best_error = 100000000.0;// std::numeric_limits<double>::max();
	PID::n = 1;

	PID::p = { Kp,Ki,Kd };
	PID::dp = { 0.2*Kp,0.2*Ki,0.2*Kd };
	PID::n_threshold = 100;
	PID::param_index = 0;
	PID::tried_adding = false;
	PID::tried_subtracting = false;
}

void PID::UpdateError(double cte) {
	//cout << n - 1 << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"   << endl;
	this->d_error = cte - this->p_error;
	this->p_error = cte;
	this->i_error += cte;
	/*
	if ( n % n_threshold==0 ){
		total_square_error += cte * cte;
		cout << "Times: ---" << n/100 << "---" << endl;
		
		if (total_square_error < best_error) {
			best_error = total_square_error;
			dp[param_index] *= 1.1;
			// next parameter
			param_index = (param_index + 1) % 3;
			tried_adding = tried_subtracting = false;
		}
		if (!tried_adding && !tried_subtracting) {
			// try adding dp[i] to params[i]
			//AddToParameterAtIndex(param_index, dp[param_index]);
			p[param_index] += dp[param_index];
			tried_adding = true;
		}
		else if (tried_adding && !tried_subtracting) {
			// try subtracting dp[i] from params[i]
			//AddToParameterAtIndex(param_index, -2 * dp[param_index]);
			//cout << "Subtract pid values II ..." << endl;
			p[param_index] -= 2 * dp[param_index];
			tried_subtracting = true;
		}
		else {
			// set it back, reduce dp[i], move on to next parameter
			//AddToParameterAtIndex(param_index, dp[param_index]);
			//cout << "Add Params II ... " << endl;
			p[param_index] += dp[param_index];
			dp[param_index] *= 0.9;
			// next parameter
			param_index = (param_index + 1) % 3;
			tried_adding = tried_subtracting = false;
		}
		total_square_error = 0;
		Kp = p[0];
		Ki = p[1];
		Kd = p[2];

	}
	n++;
	*/
	
}

double PID::TotalError() {
	return this->Kp * this->p_error
		+ this->Ki * this->i_error
		+ this->Kd * this->d_error;
}

