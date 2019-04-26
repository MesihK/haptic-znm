#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <math.h>

#define pi 3.141592653
#define eps (pi/1800)
#define INTEGRAL_LIMIT 10
typedef struct {
	int enc;
	float err;
	float integral;
	float time;
	float kp;
	float ki;
	float kd;
} pid_param;


class delta
{
	public:
		delta(double e, double f, double re, double rf,
		      double ENC1, double ENC2, double ENC3,
		      bool reverseM1, bool reverseM2, bool reverseM3);
		~delta();

		//set poision of haptic.>> dac1 dac2 dac3 is output to hardware 
		void set_pos(double elapsedTime,
			     double x, double y, double z, 
			     double *dac1, double *dac2, double *dac3 );

		//set angle of haptic.>> dac1 dac2 dac3 is output to hardware 
		void set_deg(double elapsedTime,
			     double d1, double d2, double d3,
			     double *dac1, double *dac2, double *dac3 );

		//get current position
		void get_pos(double *x, double *y, double *z);

		//get current angles.
		void get_deg(double *d1, double *d2, double *d3);

		//get target angles.
		void get_tgt_deg(double *d1, double *d2, double *d3);

		//set pid gains.
		void set_pid_gains(double kp1, double ki1, double kd1,
				   double kp2, double ki2, double kd2,
				   double kp3, double ki3, double kd3);

		//set delta parameters
		void set_delta_param(double e, double f, double re, double rf,
				     double ENC1, double ENC2, double ENC3);

		//set encoder values, this read from hardware
		void set_mtr_enc(int e1, int e2, int e3);

		//pid reset
		void reset();

	private:
		double e, f, re, rf, ENC1, ENC2, ENC3;
		double enc1, enc2, enc3;
		bool reverseM1, reverseM2, reverseM3;

		double tgt_d1, tgt_d2, tgt_d3;

		pid_param pid1, pid2, pid3;
		int forward(double theta1, double theta2,
			    double theta3, double *x0, double *y0,
			    double *z0);
		int inverse(double x0, double y0, double z0,
			    double *theta1, double *theta2, double *theta3);
		int calcAngleYZ(double x0, double y0, double z0, double *theta);
		float puls2rad(uint16_t pulse, pid_param pid);
		float deg2rad(float angle);
		float rad2deg(float rad);
		float puls2ang(uint16_t pulse, pid_param pid);
		float pid(float target, float curr, pid_param *param, float t);
};

