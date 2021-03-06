#include <controlbase.h>
#include <stdio.h>
#include <fcntl.h>
#include <stropts.h>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <string.h>
#include <mutex>
#include <QByteArray>
#include "mtr_hw.h"
#include "delta.h"
#include "pid.h"

// delta robot geometry
const double e = 210.0;     // end effector
const double f = 105.0;     // base
const double re = 135.0;
const double rf = 60.0;

//motor encoder max values
const int ENC1=4096-512;
const int ENC2=4096-512;
const int ENC3=4096-512;

class deltaEx : public ControlBase
{
	public:
		// ----- User Functions -----
		// This functions need to be implemented by the user.
		int initialize();
		int start();
		int doloop();
		int stop();
		int terminate();

	private:
		// ----- Log Variables -----
		double sine;
		double tim1;
		double tim2;
		double tim3;

		double target1;
		double target2;
		double target3;

		double v1;
		double v2;
		double v3;

		double t_err1;
		double t_err2;
		double t_err3;

		double x,y,z;

		// ----- Control Parameters -----
		double kp1;
		double kp2;
		double kp3;

		double kd1;
		double kd2;
		double kd3;

		double ki1;
		double ki2;
		double ki3;

		double curr1;
		double curr2;
		double curr3;


		// ----- Variables -----
		mtr_hw *hw;
		delta *robot;
		pid p1, p2, p3;
		int state = 0;
};

/**
 * This function is called when the control program is loaded to zenom.
 * Use this function to register control parameters, to register log variables
 * and to initialize control parameters.
 *
 * @return Return non-zero to indicate an error.
 */
int deltaEx::initialize()
{
	robot = new delta(e, f, re, rf, ENC1, ENC2, ENC3, true, true, true, p1, p2, p3);
	hw = new mtr_hw(ENC1, ENC2, ENC3, "/dev/ttyUSB0", B921600); 
	// ----- Initializes log and control variables -----
	// ----- Register the log variables -----
	registerLogVariable(&tim1, "tim1");
	registerLogVariable(&tim2, "tim2");
	registerLogVariable(&tim3, "tim3");

	registerLogVariable(&target1, "target1");
	registerLogVariable(&target2, "target2");
	registerLogVariable(&target3, "target3");

	registerLogVariable(&v1, "V1");
	registerLogVariable(&v2, "V2");
	registerLogVariable(&v3, "V3");

	registerLogVariable(&x, "X");
	registerLogVariable(&y, "Y");
	registerLogVariable(&z, "Z");

	registerLogVariable(&t_err1, "t_err1");
	registerLogVariable(&t_err2, "t_err2");
	registerLogVariable(&t_err3, "t_err3");
	// ----- Register the control paramateres -----
	registerControlVariable(&kp1, "kp1");
	registerControlVariable(&kp2, "kp2");
	registerControlVariable(&kp3, "kp3");

	registerControlVariable(&kd1, "kd1");
	registerControlVariable(&kd2, "kd2");
	registerControlVariable(&kd3, "kd3");

	registerControlVariable(&ki1, "ki1");
	registerControlVariable(&ki2, "ki2");
	registerControlVariable(&ki3, "ki3");

	registerControlVariable(&curr1, "curr1");
	registerControlVariable(&curr2, "curr2");
	registerControlVariable(&curr3, "curr3");

	// ----- Prints message in screen -----
	std::cout
		<< "This is a control program for delta robot"
		<< std::endl << std::endl;

	return 0;
}

int deltaEx::start()
{
	hw->start();

	p1.set_sampling_period(period());
	p2.set_sampling_period(period());
	p3.set_sampling_period(period());
	return 0; 
}

/**
 * This function is called periodically (as specified by the control frequency).
 * The useful functions that you can call used in doloop() are listed below.
 *
 * frequency()          returns frequency of simulation.
 * period()             returns period of simulation.
 * duration()           returns duration of simulation.
 * simTicks()           returns elapsed simulation ticks.
 * simTimeInNano()      returns elapsed simulation time in nano seconds.
 * simTimeInMiliSec()   returns elapsed simulation time in miliseconds.
 * simTimeInSec()       returns elapsed simulation time in seconds.
 * overruns()           returns the count of overruns.
 *
 * @return If you return 0, the control will continue to execute. If you return
 * nonzero, the control will abort.
 */
int deltaEx::doloop()
{

	p1.set_gain(kp1, ki1, kd1);
	p2.set_gain(kp2, ki2, kd2);
	p3.set_gain(kp3, ki3, kd3);

	double dt = 1.5;
	state = elapsedTime()/dt;
	double state_time = (elapsedTime()-dt*state)/dt;

	if(state == 0){
		//move up little bit
		x = 0;
		y = 0;
		z = state_time*5;
	} else if(state == 1){
		//cartesian test
		x = 15*state_time;
		y = 0;
		z = 5;
	} else if(state == 2){
		x = 15;
		y = 15*state_time;
		z = 5;
	} else if(state == 3){
		x = 15*(1 - state_time);
		y = 15;
		z = 5;
	} else if(state == 4){
		x = 0;
		y = 15*(1 - state_time);
		z = 5;
	} else if(state == 5){
		x = 0;
		y = -15*state_time;
		z = 5;
	} else if(state == 6){
		x = -15*state_time;
		y = -15;
		z = 5;
	} else if(state == 7){
		x = -15;
		y = -15*(1-state_time);
		z = 5;
	} else if(state == 8){
		x = -15*(1-state_time);
		y = 0;
		z = 5;
	} else if(state == 9){
		//sinusoidal test
		x = 0;
		y = 12*state_time;
		z = 5;
	} else if(state == 10){
		x = 12*sin(state_time*pi);
		y = 12*cos(state_time*pi);
		z = 5+state_time*3;;
	} else if(state == 11){
		x = 12*sin((state_time+1)*pi);
		y = 12*cos((state_time+1)*pi);
		z = 8+state_time*3;
	} else if(state == 12){
		x = 12*sin((state_time+2)*pi);
		y = 12*cos((state_time+2)*pi);
		z = 11-(state_time)*3;
	} else if(state == 13){
		x = 12*sin((state_time+3)*pi);
		y = 12*cos((state_time+3)*pi);
		z = 8-(state_time)*3;
	} else if(state == 14){
		//sinusoidal end
		x = 0;
		y = 12*(1-state_time);
		z = 5;
	} else if(state == 15){
		//go down
		x = 0;
		y = 0;
		z = (1-state_time)*5;
	}

	int e1, e2, e3;
	hw->get_enc(&e1, &e2, &e3);
	robot->set_mtr_enc(e1, e2, e3);
	robot->set_pos(x, y, z,
		       &v1, &v2, &v3);
	hw->set(v1, v2, v3, curr1, curr2, curr3);

	robot->get_deg(&tim1, &tim2, &tim3);
	robot->get_tgt_deg(&target1, &target2, &target3);

	t_err1 = target1 - tim1;
	t_err2 = target2 - tim2;
	t_err3 = target3 - tim3;

	//to remove clutter
	if(tim1 >= 350) tim1 = 20;
	if(tim2 >= 350) tim2 = 20;
	if(tim3 >= 350) tim3 = 20;

	if(v1 > 15) v1 = 15;
	if(v1 < -15) v1 = -15;

	if(v2 > 15) v2 = 15;
	if(v2 < -15) v2 = -15;

	if(v3 > 15) v3 = 15;
	if(v3 < -15) v3 = -15;
		       
	return 0;
}


/**
 * Called when a timed run ends or the STOP button is pushed from zenom.
 *
 * @return Return non-zero to indicate an error.
 */
int deltaEx::stop()
{
	hw->stop();
	robot->reset();
	p1.reset();
	p2.reset();
	p3.reset();
	return 0;
}


/**
 * This function is called when the control is unloaded. It happens when
 * the user loads a new control program or exits.
 *
 * @return Return non-zero to indicate an error.
 */
int deltaEx::terminate()
{
	hw->stop();
	robot->reset();
	p1.reset();
	p2.reset();
	p3.reset();
	return 0;
}


/**
 * The main function starts the control program
 */
int main( int argc, char *argv[] )
{
	deltaEx c;
	c.run( argc, argv );

	return 0;
}


