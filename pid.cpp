#include"pid.h"

void pid::set_gain(double kp, double ki, double kd){
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void pid::reset(){
	time = 0;
	integral = 0;
	d.reset();
}

void pid::set_sampling_period(double period){
	d.setSamplingPeriod(period);
	i.setSamplingPeriod(period);
}

#define INTEGRAL_LIMIT 10
double pid::calculate(double target, double current){
	double err, d1, d2 = 0;
	double out;

	d1 = target - current;
	d2 = d1 + 2*pi;
	if((fabs(d2) -  fabs(d1)) > 0.0001) err = d1;
	else err = d2;

	out = err*kp +
	      ki*i.integrate(err) +
	      kd*d.differentiate(err); 

	return out;
}

