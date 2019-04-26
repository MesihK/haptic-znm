#ifndef CONTROLLER
#define CONTROLLER

#include <math.h>
#define pi 3.141592653
#define eps (pi/1800)

class controller 
{
	public:
		enum controller_type : int{
			C_PID=1,
			C_NONE=255
		};

		controller(controller_type type=C_NONE) : type(type) {};
		virtual void reset() = 0;
		virtual double calculate(double target, double current, double elapsedTime) = 0;
		controller_type type;
};
#endif
