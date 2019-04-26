#include"pid.h"

void pid::set_gain(double kp, double ki, double kd){
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void pid::reset(){
	err = 0;
	time = 0;
	integral = 0;
}

#define INTEGRAL_LIMIT 10
double pid::calculate(double target, double current, double t){
	double terr, d1, d2 = 0;
	double d;
	double out;
	double dt = (double)(t - time); //time diff in sec.
	time = t;

	d1 = target - current;
	d2 = d1 + 2*pi;
	if((fabs(d2) -  fabs(d1)) > 0.0001) terr = d1;
	else terr = d2;

	if(fabs(terr) > eps){
		integral = integral + terr*dt;

		if(integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
		if(integral < -1*INTEGRAL_LIMIT) integral = -1*INTEGRAL_LIMIT;
	}
	if(dt > 0){
		d = (terr - err) / dt;
	} else {
		d = 0;
	}

	out = terr*kp + integral*ki + kd*d; 

	err = terr;

	return out;
}

