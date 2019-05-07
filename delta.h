#ifndef DELTA
#define DELTA

#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include "controller.h"


class delta
{
	public:
		delta(double e, double f, double re, double rf,
		      double ENC1, double ENC2, double ENC3,
		      bool reverseM1, bool reverseM2, bool reverseM3,
		      controller &c1, controller &c2, controller &c3);
		~delta();

		//set poision of haptic.>> dac1 dac2 dac3 is output to hardware 
		void set_pos(double x, double y, double z, 
			     double *dac1, double *dac2, double *dac3 );

		//set angle of haptic.>> dac1 dac2 dac3 is output to hardware 
		void set_deg(double d1, double d2, double d3,
			     double *dac1, double *dac2, double *dac3 );

		//get current position
		void get_pos(double *x, double *y, double *z);

		//get current angles.
		void get_deg(double *d1, double *d2, double *d3);

		//get target angles.
		void get_tgt_deg(double *d1, double *d2, double *d3);

		//set delta parameters
		void set_delta_param(double e, double f, double re, double rf,
				     double ENC1, double ENC2, double ENC3);

		//set encoder values, this read from hardware
		void set_mtr_enc(int e1, int e2, int e3);

		//reset
		void reset();

	private:
		double e, f, re, rf, ENC1, ENC2, ENC3;
		double enc1, enc2, enc3;
		bool reverseM1, reverseM2, reverseM3;
		controller &c1, &c2, &c3;

		double tgt_d1, tgt_d2, tgt_d3;

		int forward(double theta1, double theta2,
			    double theta3, double *x0, double *y0,
			    double *z0);
		int inverse(double x0, double y0, double z0,
			    double *theta1, double *theta2, double *theta3);
		int calcAngleYZ(double x0, double y0, double z0, double *theta);
		float puls2rad(uint16_t pulse, int max_enc);
		float deg2rad(float angle);
		float rad2deg(float rad);
		float puls2ang(uint16_t pulse, int max_enc);
};

#endif
