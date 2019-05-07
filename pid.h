#ifndef PID
#define PID

#include"controller.h"
#include"Differentiator.hpp"
#include <Integrator.hpp>

class pid : public controller
{
	public:
		pid() : controller(C_PID)
		      , kp(0)
		      , ki(0)
		      , kd(0)
		      , time(0)
		      , integral(0)
		      , err(0)
		{ 
		};

		void set_gain(double kp, double ki, double kd);
		void set_sampling_period(double period);
		virtual void reset();
		virtual double calculate(double target, double current);

	private:
		double kp, ki, kd;
		double time, integral, err;
		Differentiator<double> d;
		Integrator<double> i;
};

#endif
